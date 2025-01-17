use core::{convert::Infallible, future::poll_fn, marker::PhantomData, task::Poll};

use crate::pac::{self, interrupt};
use embassy_hal_internal::{into_ref, Peripheral, PeripheralRef};
use embassy_sync::waitqueue::AtomicWaker;
use embedded_hal::spi::{Mode, Phase, Polarity};

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

/// Driver for the SPI (Master) Peripheral.
pub struct Spip<'d, T> {
    peri: PeripheralRef<'d, pac::Spip>,
    _mod: PhantomData<T>,
}

impl<'d, T> Spip<'d, T> {
    fn init(peri: &mut PeripheralRef<'d, pac::Spip>, config: Config, mod_: bool) {
        // Configure SPI pins to peripheral, safe because we took ownership.
        // Note(cs): other peripherals might also be modifying devalt0 at the same time.
        critical_section::with(|_| {
            let sysconfig = unsafe { crate::pac::Sysconfig::steal() };
            sysconfig.devalt0().modify(|_, w| w.spip_sl().set_bit());
        });

        peri.spip_ctl1().modify(|_, w| {
            w.spien().set_bit();
            w.mod_().bit(mod_);
            w.scidl().bit(config.mode.polarity == Polarity::IdleHigh);
            w.scm().bit(config.mode.phase == Phase::CaptureOnSecondTransition);
            unsafe { w.scdv6_0().bits(2) }; // TODO clocking freq
            w
        });
    }

    async fn transfer_word(&mut self, data: u16) -> u16 {
        self.peri.spip_ctl1().modify(|_, w| w.eir().set_bit());
        self.peri.spip_data().write(|w| unsafe { w.bits(data) });

        poll_fn(|cx| {
            WAKER.register(cx.waker());

            if self.peri.spip_stat().read().rbf().bit_is_set() {
                // Reading the data clears the rbf-bit.
                let data = self.peri.spip_data().read().bits();
                Poll::Ready(data)
            } else {
                Poll::Pending
            }
        })
        .await
    }
}

impl<'d> Spip<'d, u8> {
    pub fn new_8bit(
        peri: impl Peripheral<P = pac::Spip> + 'd,
        mosi: impl Peripheral<P = MosiPin> + 'd,
        miso: impl Peripheral<P = MisoPin> + 'd,
        sclk: impl Peripheral<P = SclkPin> + 'd,
        config: Config,
    ) -> Self {
        into_ref!(peri);

        // We only tie the pins to our lifetime, discard.
        let _ = (mosi, miso, sclk);

        Self::init(&mut peri, config, false);

        Self {
            peri,
            _mod: Default::default(),
        }
    }
}

impl<'d> Spip<'d, u16> {
    pub fn new_16bit(
        peri: impl Peripheral<P = pac::Spip> + 'd,
        mosi: impl Peripheral<P = MosiPin> + 'd,
        miso: impl Peripheral<P = MisoPin> + 'd,
        sclk: impl Peripheral<P = SclkPin> + 'd,
        config: Config,
    ) -> Self {
        into_ref!(peri);

        // We only tie the pins to our lifetime, discard.
        let _ = (mosi, miso, sclk);

        Self::init(&mut peri, config, true);

        Self {
            peri,
            _mod: Default::default(),
        }
    }
}

impl<T> Drop for Spip<'_, T> {
    fn drop(&mut self) {
        self.peri.spip_ctl1().modify(|_, w| w.spien().clear_bit());
    }
}

impl<T> embedded_hal_async::spi::ErrorType for Spip<'_, T> {
    type Error = Infallible;
}

impl<T: Copy + 'static + From<u16> + Into<u16>> embedded_hal_async::spi::SpiBus<T> for Spip<'_, T> {
    async fn read(&mut self, words: &mut [T]) -> Result<(), Self::Error> {
        for r in words {
            *r = self.transfer_word(0x0000).await.into();
        }
        Ok(())
    }

    async fn write(&mut self, words: &[T]) -> Result<(), Self::Error> {
        for w in words {
            self.transfer_word((*w).into()).await;
        }
        Ok(())
    }

    async fn transfer(&mut self, read: &mut [T], write: &[T]) -> Result<(), Self::Error> {
        for (r, w) in read.iter_mut().zip(write.iter()) {
            *r = self.transfer_word((*w).into()).await.into();
        }
        Ok(())
    }

    async fn transfer_in_place(&mut self, words: &mut [T]) -> Result<(), Self::Error> {
        for rw in words {
            *rw = self.transfer_word((*rw).into()).await.into();
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
