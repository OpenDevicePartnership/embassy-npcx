use core::{convert::Infallible, future::poll_fn, marker::PhantomData, task::Poll};

use crate::{
    pac::{self, interrupt},
    peripherals::SPIP,
};
use embassy_hal_internal::{into_ref, Peripheral, PeripheralRef};
use embassy_sync::waitqueue::AtomicWaker;
use embedded_hal::spi::{Mode, Phase, Polarity, MODE_0};

pub type MosiPin = crate::peripherals::PK12;
pub type MisoPin = crate::peripherals::PM12;
pub type SclkPin = crate::peripherals::PL12;

static WAKER: AtomicWaker = AtomicWaker::new();

/// SPIP configuration.
#[non_exhaustive]
#[derive(Clone)]
pub struct Config {
    /// SPI mode
    pub mode: Mode,

    // TODO
    pub frequency: (),
}

impl Default for Config {
    fn default() -> Self {
        Self {
            mode: MODE_0,
            frequency: (),
        }
    }
}

#[allow(private_bounds)]
mod sealed {
    pub trait SealedInstance {
        fn regs() -> &'static crate::pac::spip::RegisterBlock;
    }
}

pub trait Instance: Peripheral<P = Self> + sealed::SealedInstance + 'static + Send {}

impl sealed::SealedInstance for SPIP {
    fn regs() -> &'static pac::spip::RegisterBlock {
        let ptr = crate::pac::Spip::ptr();

        // Safety:
        // the pac ptr functions return pointers to memory that is used for registers for the 'static lifetime
        // and the created reference is shared.
        unsafe { &*ptr }
    }
}
impl Instance for SPIP {}

/// Driver for the SPI (Master) Peripheral.
pub struct Spip<'d, T: Instance, U> {
    _peri: PeripheralRef<'d, T>,
    _mod: PhantomData<U>,
}

pub trait WordConvertible: Copy + 'static {
    fn to_base(self) -> u16;
    fn from_base(_: u16) -> Self;
}

impl WordConvertible for u8 {
    fn to_base(self) -> u16 {
        self as u16
    }

    fn from_base(x: u16) -> Self {
        x as u8 // Throw away extra byte.
    }
}

impl WordConvertible for u16 {
    fn to_base(self) -> u16 {
        self
    }

    fn from_base(x: u16) -> Self {
        x
    }
}

impl<'d, T: Instance, U> Spip<'d, T, U> {
    fn init(config: Config, mod_: bool) {
        // Configure SPI pins to peripheral, safe because we took ownership.
        // Note(cs): other peripherals might also be modifying devalt0 at the same time.
        critical_section::with(|_| {
            let sysconfig = unsafe { crate::pac::Sysconfig::steal() };
            sysconfig.devalt0().modify(|_, w| w.spip_sl().set_bit());
        });

        T::regs().spip_ctl1().modify(|_, w| {
            w.spien().set_bit();
            w.mod_().bit(mod_);
            w.scidl().bit(config.mode.polarity == Polarity::IdleHigh);
            w.scm().bit(config.mode.phase == Phase::CaptureOnSecondTransition);
            unsafe { w.scdv6_0().bits(2) }; // TODO clocking freq
            w
        });
    }

    async fn transfer_word(&mut self, data: u16) -> u16 {
        let r = T::regs();
        r.spip_ctl1().modify(|_, w| w.eir().set_bit());
        r.spip_data().write(|w| unsafe { w.bits(data) });

        poll_fn(|cx| {
            WAKER.register(cx.waker());

            if r.spip_stat().read().rbf().bit_is_set() {
                // Reading the data clears the rbf-bit.
                let data = r.spip_data().read().bits();
                Poll::Ready(data)
            } else {
                Poll::Pending
            }
        })
        .await
    }
}

impl<'d, T: Instance> Spip<'d, T, u8> {
    pub fn new_8bit(
        peri: impl Peripheral<P = T> + 'd,
        mosi: impl Peripheral<P = MosiPin> + 'd,
        miso: impl Peripheral<P = MisoPin> + 'd,
        sclk: impl Peripheral<P = SclkPin> + 'd,
        config: Config,
    ) -> Self {
        into_ref!(peri);

        // We only tie the pins to our lifetime, discard.
        let _ = (mosi, miso, sclk);

        Self::init(config, false);

        Self {
            _peri: peri,
            _mod: Default::default(),
        }
    }
}

impl<'d, T: Instance> Spip<'d, T, u16> {
    pub fn new_16bit(
        peri: impl Peripheral<P = T> + 'd,
        mosi: impl Peripheral<P = MosiPin> + 'd,
        miso: impl Peripheral<P = MisoPin> + 'd,
        sclk: impl Peripheral<P = SclkPin> + 'd,
        config: Config,
    ) -> Self {
        into_ref!(peri);

        // We only tie the pins to our lifetime, discard.
        let _ = (mosi, miso, sclk);

        Self::init(config, true);

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

impl<T: Instance, U: WordConvertible> embedded_hal_async::spi::SpiBus<U> for Spip<'_, T, U> {
    async fn read(&mut self, words: &mut [U]) -> Result<(), Self::Error> {
        for r in words {
            *r = U::from_base(self.transfer_word(0x0000).await);
        }
        Ok(())
    }

    async fn write(&mut self, words: &[U]) -> Result<(), Self::Error> {
        for w in words {
            self.transfer_word((*w).to_base()).await;
        }
        Ok(())
    }

    async fn transfer(&mut self, read: &mut [U], write: &[U]) -> Result<(), Self::Error> {
        for (r, w) in read.iter_mut().zip(write.iter()) {
            *r = U::from_base(self.transfer_word((*w).to_base()).await);
        }
        Ok(())
    }

    async fn transfer_in_place(&mut self, words: &mut [U]) -> Result<(), Self::Error> {
        for rw in words {
            *rw = U::from_base(self.transfer_word((*rw).to_base()).await);
        }
        Ok(())
    }

    async fn flush(&mut self) -> Result<(), Self::Error> {
        Ok(()) // No-op
    }
}

#[pac::interrupt]
fn SPIP() {
    let spip = unsafe { pac::Spip::steal() };
    spip.spip_ctl1().modify(|_, w| w.eir().clear_bit());
    WAKER.wake();
}
