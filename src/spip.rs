//! Serial Peripheral Interface Peripheral (SPIP).
//!
//! Implements the general purpose SPI Peripheral Interface that enables the connection of SPI-based peripheral devices.

use crate::{cdcg, interrupt::typelevel::Interrupt, pac, peripherals::SPIP};
use core::{convert::Infallible, future::poll_fn, marker::PhantomData, task::Poll};
use embassy_hal_internal::{into_ref, Peripheral, PeripheralRef};
use embassy_sync::waitqueue::AtomicWaker;
use embedded_hal::spi::{Mode, Phase, Polarity, MODE_0};

/// Pin that can be used as SPIP Mosi.
pub type MosiPin = crate::peripherals::PK12;
/// Pin that can be used as SPIP Miso.
pub type MisoPin = crate::peripherals::PM12;
/// Pin that can be used as SPIP Sclk.
pub type SclkPin = crate::peripherals::PL12;

/// Pin that has been tied to the SPIP IO matrix, and cannot be used as GPIO when SPIP is used.
pub type LegacyPin = crate::peripherals::PL10;

const MAX_FREQUENCY: u32 = 12_500_000;

/// SPIP configuration.
#[non_exhaustive]
#[derive(Clone)]
pub struct Config {
    /// SPI mode.
    pub mode: Mode,

    /// Bus frequency at which SCLK will operate.
    ///
    /// Maximum supported frequency is 12.5MHz.
    pub frequency: u32,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            mode: MODE_0,
            frequency: 1_000_000,
        }
    }
}

#[allow(private_bounds)]
mod sealed {
    pub trait SealedInstance {}
}

/// A marker trait implemented by the SPIP peripherals.
pub trait Instance: Peripheral<P = Self> + sealed::SealedInstance + 'static + Send {
    /// The interrupt used by this instance.
    type Interrupt: crate::interrupt::typelevel::Interrupt;

    /// The waker used by this instance.
    fn waker() -> &'static AtomicWaker;

    /// The register belonging to this instance.
    fn regs() -> &'static crate::pac::spip::RegisterBlock;
}

impl sealed::SealedInstance for SPIP {}
impl Instance for SPIP {
    type Interrupt = crate::interrupt::typelevel::SPIP;

    fn regs() -> &'static pac::spip::RegisterBlock {
        let ptr = crate::pac::Spip::ptr();

        // Safety:
        // the pac ptr functions return pointers to memory that is used for registers for the 'static lifetime
        // and the created reference is shared.
        unsafe { &*ptr }
    }

    fn waker() -> &'static AtomicWaker {
        static WAKER: AtomicWaker = AtomicWaker::new();
        &WAKER
    }
}

/// The interrupt handler for the SPIP driver.
pub struct InterruptHandler<T> {
    _phantom: PhantomData<T>,
}

impl<T: Instance> crate::interrupt::typelevel::Handler<T::Interrupt> for InterruptHandler<T> {
    unsafe fn on_interrupt() {
        T::regs().spip_ctl1().modify(|_, w| w.eir().clear_bit());
        T::waker().wake();
    }
}

/// Driver for the SPI (Master) Peripheral.
pub struct Spip<'d, T: Instance, U = u8> {
    _peri: PeripheralRef<'d, T>,
    _mod: PhantomData<U>,
}

trait SpipPrimitive: Default + Copy + 'static {}

impl SpipPrimitive for u8 {}
impl SpipPrimitive for u16 {}

#[allow(private_bounds)]
impl<T: Instance, U: SpipPrimitive> Spip<'_, T, U> {
    fn init(
        _irqs: impl crate::interrupt::typelevel::Binding<T::Interrupt, InterruptHandler<T>>,
        config: Config,
        mod_: bool,
    ) {
        // Note(cs): other peripherals might also be modifying swsrst* and devalt0 at the same time.
        critical_section::with(|_| {
            let sysconfig = unsafe { crate::pac::Sysconfig::steal() };

            // Reset all device registers.
            sysconfig.swrst_ctl2().write(|w| w.spip_rst().set_bit());
            sysconfig.swrst_trg().write(|w| unsafe { w.bits(0xC183) });
            while sysconfig.swrst_ctl2().read().spip_rst().bit_is_set() {}

            // Configure SPI pins to peripheral, safe because we took ownership.
            sysconfig
                .devalt0()
                .modify(|_, w| w.spip_sl().set_bit().gpio_no_spip().clear_bit());

            sysconfig.pupd_en1().modify(|_, w| w.spip_pd_en().clear_bit());
        });

        // Note(safety): SPIP can only be constructed after clocks have been initialized.
        let srcclk = unsafe { cdcg::get_clocks() }.apb2_clk;

        assert!(config.frequency <= MAX_FREQUENCY);

        // Best-effort divisor selection, rounded up.
        let div = srcclk.div_ceil(config.frequency).clamp(2, 256);
        let scdv6_0 = (div / 2) - 1;

        let r = T::regs();
        r.spip_ctl1().modify(|_, w| {
            w.mod_().bit(mod_);
            w.scidl().bit(config.mode.polarity == Polarity::IdleHigh);
            w.scm().bit(config.mode.phase == Phase::CaptureOnSecondTransition);
            unsafe { w.scdv6_0().bits(scdv6_0 as u8) };
            w
        });

        // Safety: _irqs ensures an interrupt handler is bound
        unsafe {
            T::Interrupt::enable();
        }

        r.spip_ctl1().modify(|_, w| w.spien().set_bit());
    }

    async fn transfer_word(&mut self, data: U) -> U {
        let r = T::regs();

        let stat = r.spip_stat().read();
        assert!(stat.bsy().bit_is_clear() && stat.rbf().bit_is_clear());

        r.spip_ctl1().modify(|_, w| w.eir().set_bit());

        let ptr = r.spip_data().as_ptr();
        // The manual states that the read from the register must correspond with the size as
        // specified in the MOD register.
        let ptr = ptr as *mut U;

        // Starts the transaction.
        unsafe { ptr.write_volatile(data) };

        poll_fn(move |cx| {
            T::waker().register(cx.waker());
            if r.spip_stat().read().rbf().bit_is_set() {
                // Reading the data clears the rbf-bit.
                let data = unsafe { ptr.read_volatile() };
                Poll::Ready(data)
            } else {
                Poll::Pending
            }
        })
        .await
    }

    /// Get the effective bus frequency.
    pub fn frequency(&self) -> u32 {
        // Note(safety): SPIP can only be constructed after clocks have been initialized.
        let srcclk = unsafe { cdcg::get_clocks() }.apb2_clk;
        let scdv6_0 = T::regs().spip_ctl1().read().scdv6_0().bits() as u32;
        let div = (scdv6_0 + 1) * 2;
        srcclk / div
    }
}

impl<'d, T: Instance> Spip<'d, T, u8> {
    /// Enables the peripheral in 8-bit word mode with the applicable bus pins.
    pub fn new_8bit(
        peri: impl Peripheral<P = T> + 'd,
        mosi: impl Peripheral<P = MosiPin> + 'd,
        miso: impl Peripheral<P = MisoPin> + 'd,
        sclk: impl Peripheral<P = SclkPin> + 'd,
        legacy: impl Peripheral<P = LegacyPin> + 'd,
        irqs: impl crate::interrupt::typelevel::Binding<T::Interrupt, InterruptHandler<T>>,
        config: Config,
    ) -> Self {
        into_ref!(peri);

        // We only tie the pins to our lifetime, discard.
        let _ = (mosi, miso, sclk, legacy);

        Self::init(irqs, config, false);

        Self {
            _peri: peri,
            _mod: Default::default(),
        }
    }
}

impl<'d, T: Instance> Spip<'d, T, u16> {
    /// Enables the peripheral in 16-bit word mode with the applicable bus pins.
    pub fn new_16bit(
        peri: impl Peripheral<P = T> + 'd,
        mosi: impl Peripheral<P = MosiPin> + 'd,
        miso: impl Peripheral<P = MisoPin> + 'd,
        sclk: impl Peripheral<P = SclkPin> + 'd,
        legacy: impl Peripheral<P = LegacyPin> + 'd,
        irqs: impl crate::interrupt::typelevel::Binding<T::Interrupt, InterruptHandler<T>>,
        config: Config,
    ) -> Self {
        into_ref!(peri);

        // We only tie the pins to our lifetime, discard.
        let _ = (mosi, miso, sclk, legacy);

        Self::init(irqs, config, true);

        Self {
            _peri: peri,
            _mod: Default::default(),
        }
    }
}

impl<T: Instance, U> Drop for Spip<'_, T, U> {
    fn drop(&mut self) {
        T::regs().spip_ctl1().modify(|_, w| w.spien().clear_bit());
    }
}

impl<T: Instance, U> embedded_hal_async::spi::ErrorType for Spip<'_, T, U> {
    type Error = Infallible;
}

impl<T: Instance, U: SpipPrimitive> embedded_hal_async::spi::SpiBus<U> for Spip<'_, T, U> {
    async fn read(&mut self, words: &mut [U]) -> Result<(), Self::Error> {
        for r in words {
            *r = self.transfer_word(U::default()).await;
        }
        Ok(())
    }

    async fn write(&mut self, words: &[U]) -> Result<(), Self::Error> {
        for w in words {
            self.transfer_word(*w).await;
        }
        Ok(())
    }

    async fn transfer(&mut self, read: &mut [U], write: &[U]) -> Result<(), Self::Error> {
        for (r, w) in read.iter_mut().zip(write.iter()) {
            *r = self.transfer_word(*w).await;
        }
        Ok(())
    }

    async fn transfer_in_place(&mut self, words: &mut [U]) -> Result<(), Self::Error> {
        for rw in words {
            *rw = self.transfer_word(*rw).await;
        }
        Ok(())
    }

    async fn flush(&mut self) -> Result<(), Self::Error> {
        Ok(()) // No-op
    }
}
