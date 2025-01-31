use core::future::Future;
use core::marker::PhantomData;

use embassy_futures::select::select;
use embassy_hal_internal::{into_ref, Peripheral, PeripheralRef};
use embassy_sync::blocking_mutex::raw::RawMutex;
use embassy_sync::waitqueue::AtomicWaker;
use embassy_sync::watch;
pub use embedded_hal_async::i2c::{I2c, Operation};

use crate::cdcg::get_clocks;
use crate::gpio::Pin;
use crate::interrupt::typelevel::Interrupt;

// Size of the peripherals fifo
const FIFO_SIZE: u8 = 32;
// Number of bytes between refilling/emptying of the fifo
const GROUP_SIZE: u8 = 24;

/// SMBUS Broadcast address
pub const GENERALCALL_ADDRESS: u8 = 0;
/// SMBUS ARP address
pub const ARP_ADDRESS: u8 = 0b1100001;

pub struct InterruptHandler<T> {
    _phantom: PhantomData<T>,
}

impl<T: Instance> crate::interrupt::typelevel::Handler<T::Interrupt> for InterruptHandler<T> {
    unsafe fn on_interrupt() {
        T::waker().wake();
        critical_section::with(|_| {
            T::regs().smbn_ctl1().modify(|_, w| w.inten().clear_bit());
        });
    }
}

struct WaitForCondition<F> {
    regs: &'static crate::pac::smb0::RegisterBlock,
    waker: &'static AtomicWaker,
    f: F,
}

impl<O, F: Fn() -> Option<O>> Future for WaitForCondition<F> {
    type Output = O;

    fn poll(self: core::pin::Pin<&mut Self>, cx: &mut core::task::Context<'_>) -> core::task::Poll<Self::Output> {
        if let Some(out) = (self.f)() {
            core::task::Poll::Ready(out)
        } else {
            self.waker.register(cx.waker());
            critical_section::with(|_| {
                self.regs.smbn_ctl1().modify(|_, w| w.inten().set_bit());
            });
            core::task::Poll::Pending
        }
    }
}

struct ReadCompletion<'a> {
    regs: &'static crate::pac::smb0::RegisterBlock,
    group: &'a mut [u8],
}

impl ReadCompletion<'_> {
    fn complete(self) {
        if self.group.is_empty() {
            self.regs.smbn_st().write(|w| w.stastr().set_bit());
            self.regs.smbn_ctl1().modify(|_, w| w.stastre().clear_bit());
        } else {
            for b in self.group {
                *b = self.regs.smbn_sda().read().bits();
            }
        }
    }
}

#[derive(Default, Debug, Copy, Clone, Eq, PartialEq, Ord, PartialOrd, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Speed {
    #[default]
    Standard,
    Fast,
    FastPlus,
}

#[derive(Default, Debug, Copy, Clone, Eq, PartialEq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Config {
    pub speed: Speed,
    pub pullup: bool,
}

#[derive(Debug, Copy, Clone, Eq, PartialEq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error {
    LostArbitration,
    BusError,
    AddressNack,
    DataNack,
}

impl embedded_hal_async::i2c::Error for Error {
    fn kind(&self) -> embedded_hal::i2c::ErrorKind {
        match self {
            Error::LostArbitration => embedded_hal::i2c::ErrorKind::ArbitrationLoss,
            Error::BusError => embedded_hal::i2c::ErrorKind::Bus,
            Error::DataNack => {
                embedded_hal::i2c::ErrorKind::NoAcknowledge(embedded_hal::i2c::NoAcknowledgeSource::Data)
            }
            Error::AddressNack => {
                embedded_hal::i2c::ErrorKind::NoAcknowledge(embedded_hal::i2c::NoAcknowledgeSource::Address)
            }
        }
    }
}

#[derive(Debug, Copy, Clone, Eq, PartialEq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ListenError {
    InvalidAddressList,
}

#[derive(Debug, Eq, PartialEq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ListenCommand<'a> {
    /// Part of a master's write has been received
    PartialWrite(&'a [u8]),
    /// The final part of a master's write has been received
    WriteFinished(&'a [u8]),
    /// Prepare a buffer for a read operation. Note, not
    /// all bytes in the buffer may be read. Actions
    /// that should happen once bytes have been read should
    /// be done in ReadFinished.
    PrepareRead(&'a mut [u8]),
    /// A read has been finished. Reports how many bytes were
    /// read in total as part of the read.
    ReadFinished(usize),
    /// A bus error occured. No action is needed, but gives
    /// the handler a chance to report this. Always occurs with
    /// address 0. The corresponding finished command for the
    /// transaction will still be provided after the bus error.
    BusError,
}

struct AnySMB {}

impl<T: Instance> From<T> for AnySMB {
    fn from(_smb: T) -> Self {
        AnySMB {}
    }
}

// Allow use of PeripheralRef to do lifetime management
impl Peripheral for AnySMB {
    type P = AnySMB;

    unsafe fn clone_unchecked(&self) -> Self::P {
        AnySMB {}
    }
}

pub struct I2CController<'a> {
    _dev: PeripheralRef<'a, AnySMB>,
    regs: &'static crate::pac::smb0::RegisterBlock,
    waker: &'static AtomicWaker,
}

trait IteratorExt: ExactSizeIterator + Sized {
    fn mark_last(mut self) -> impl Iterator<Item = (Self::Item, bool)> {
        core::iter::from_fn(move || self.next().map(|v| (v, self.len() == 0)))
    }
}

impl<T: ExactSizeIterator + Sized> IteratorExt for T {}

impl<'p> I2CController<'p> {
    fn wait_for<O, F: Fn() -> Option<O>>(&mut self, f: F) -> impl Future<Output = O> {
        WaitForCondition {
            f,
            regs: self.regs,
            waker: self.waker,
        }
    }

    async fn wait_read(&mut self) -> Result<(), Error> {
        let r = self
            .wait_for(|| {
                let r = self.regs.smbn_st().read();
                let f = self.regs.smbn_rxf_sts().read();
                if r.ber().bit_is_set() || r.sdast().bit_is_set() || f.rx_thst().bit_is_set() {
                    Some(r)
                } else {
                    None
                }
            })
            .await;
        if r.ber().bit_is_set() {
            return self.handle_ber();
        }
        Ok(())
    }

    // Select register bank
    fn bank_sel(&mut self, bnk: bool) {
        self.regs
            .smbn_ctl3()
            .modify(|_, w| w.scl_lvl().set_bit().sda_lvl().set_bit().bnk_sel().bit(bnk));
    }

    /// Initialize speed settings
    ///
    /// Should be called only with bank 0 selected and peripheral disabled
    fn speed_init(&mut self, speed: Speed, clk: u32) {
        struct StandardMode {
            sclfrq: u16,
            hldt: u8,
        }

        struct FastMode {
            scllt: u8,
            sclht: u8,
            hldt: u8,
        }

        const STANDARDMODE: [(u32, StandardMode); 7] = [
            (15_000_000, StandardMode { sclfrq: 38, hldt: 15 }),
            (25_000_000, StandardMode { sclfrq: 63, hldt: 15 }),
            (30_000_000, StandardMode { sclfrq: 77, hldt: 17 }),
            (40_000_000, StandardMode { sclfrq: 101, hldt: 17 }),
            (48_000_000, StandardMode { sclfrq: 121, hldt: 17 }),
            (50_000_000, StandardMode { sclfrq: 126, hldt: 17 }),
            (60_000_000, StandardMode { sclfrq: 152, hldt: 17 }),
        ];

        const FASTMODE: [(u32, FastMode); 8] = [
            (
                15_000_000,
                FastMode {
                    scllt: 12,
                    sclht: 9,
                    hldt: 7,
                },
            ),
            (
                20_000_000,
                FastMode {
                    scllt: 16,
                    sclht: 11,
                    hldt: 7,
                },
            ),
            (
                24_000_000,
                FastMode {
                    scllt: 20,
                    sclht: 13,
                    hldt: 8,
                },
            ),
            (
                30_000_000,
                FastMode {
                    scllt: 24,
                    sclht: 16,
                    hldt: 10,
                },
            ),
            (
                40_000_000,
                FastMode {
                    scllt: 32,
                    sclht: 21,
                    hldt: 13,
                },
            ),
            (
                48_000_000,
                FastMode {
                    scllt: 39,
                    sclht: 25,
                    hldt: 16,
                },
            ),
            (
                50_000_000,
                FastMode {
                    scllt: 40,
                    sclht: 26,
                    hldt: 17,
                },
            ),
            (
                60_000_000,
                FastMode {
                    scllt: 48,
                    sclht: 31,
                    hldt: 20,
                },
            ),
        ];

        const FASTMODEPLUS: [(u32, FastMode); 7] = [
            (
                15_000_000,
                FastMode {
                    scllt: 7,
                    sclht: 5,
                    hldt: 7,
                },
            ),
            (
                24_000_000,
                FastMode {
                    scllt: 8,
                    sclht: 5,
                    hldt: 7,
                },
            ),
            (
                30_000_000,
                FastMode {
                    scllt: 10,
                    sclht: 7,
                    hldt: 7,
                },
            ),
            (
                40_000_000,
                FastMode {
                    scllt: 13,
                    sclht: 10,
                    hldt: 7,
                },
            ),
            (
                48_000_000,
                FastMode {
                    scllt: 15,
                    sclht: 11,
                    hldt: 7,
                },
            ),
            (
                50_000_000,
                FastMode {
                    scllt: 16,
                    sclht: 11,
                    hldt: 8,
                },
            ),
            (
                60_000_000,
                FastMode {
                    scllt: 19,
                    sclht: 13,
                    hldt: 9,
                },
            ),
        ];

        match speed {
            Speed::Standard => {
                let mut i = 0;
                while STANDARDMODE[i].0 < clk {
                    i += 1;
                }

                self.regs.smbn_ctl3().modify(|_, w| unsafe {
                    w._400k_mode()
                        .clear_bit()
                        .sclfrq8_7()
                        .bits(((STANDARDMODE[i].1.sclfrq & 0x180) >> 7) as u8)
                });
                self.regs
                    .smbn_ctl4()
                    .modify(|_, w| unsafe { w.hldt().bits(STANDARDMODE[i].1.hldt) });
                self.regs
                    .smbn_ctl2()
                    .modify(|_, w| unsafe { w.sclfrq6_0().bits((STANDARDMODE[i].1.sclfrq & 0x7F) as u8) });
            }
            Speed::Fast => {
                let mut i = 0;
                while FASTMODE[i].0 < clk {
                    i += 1;
                }

                self.regs
                    .smbn_ctl3()
                    .modify(|_, w| unsafe { w._400k_mode().set_bit().sclfrq8_7().bits(0) });
                self.regs
                    .smbn_ctl4()
                    .modify(|_, w| unsafe { w.hldt().bits(FASTMODE[i].1.hldt) });
                self.regs.smbn_ctl2().modify(|_, w| unsafe { w.sclfrq6_0().bits(0) });
                self.regs.smbn_scllt().write(|w| unsafe { w.bits(FASTMODE[i].1.scllt) });
                self.regs.smbn_sclht().write(|w| unsafe { w.bits(FASTMODE[i].1.sclht) });
            }
            Speed::FastPlus => {
                let mut i = 0;
                while FASTMODEPLUS[i].0 < clk {
                    i += 1;
                }

                self.regs
                    .smbn_ctl3()
                    .modify(|_, w| unsafe { w._400k_mode().set_bit().sclfrq8_7().bits(0) });
                self.regs
                    .smbn_ctl4()
                    .modify(|_, w| unsafe { w.hldt().bits(FASTMODEPLUS[i].1.hldt) });
                self.regs.smbn_ctl2().modify(|_, w| unsafe { w.sclfrq6_0().bits(0) });
                self.regs
                    .smbn_scllt()
                    .write(|w| unsafe { w.bits(FASTMODEPLUS[i].1.scllt) });
                self.regs
                    .smbn_sclht()
                    .write(|w| unsafe { w.bits(FASTMODEPLUS[i].1.sclht) });
            }
        }
    }

    pub fn new<T: Instance + 'p, Scl: Pin, Sda: Pin, Mode>(
        peri: impl Peripheral<P = T> + 'p,
        scl: impl Peripheral<P = Scl> + 'p,
        sda: impl Peripheral<P = Sda> + 'p,
        _irqs: impl crate::interrupt::typelevel::Binding<T::Interrupt, InterruptHandler<T>>,
        _mode: Mode,
        config: Config,
    ) -> Self
    where
        (T, Scl, Sda, Mode): ValidI2CConfig,
    {
        // Safety: _irqs ensures an interrupt handler is bound
        unsafe {
            T::Interrupt::enable();
        }

        into_ref!(peri);
        into_ref!(scl);
        into_ref!(sda);

        critical_section::with(|cs| {
            // Safety: We are disabling low voltage mode and exclusively own the peripherals
            unsafe {
                scl.set_low_voltage(cs, false);
            }
            unsafe {
                sda.set_low_voltage(cs, false);
            }
            // Safety: We have exclusive ownership over the peripherals.
            unsafe { <(T, Scl, Sda, Mode) as sealed::SealedValidI2CConfig>::setup_pins(cs) };
            unsafe { <(T, Scl, Sda, Mode) as sealed::SealedValidI2CConfig>::setup_pullup(cs, config.pullup) };
        });

        let mut dev = Self {
            _dev: peri.map_into(),
            regs: T::regs(),
            waker: T::waker(),
        };

        dev.regs.smbn_ctl3().write(|w| {
            w.scl_lvl()
                .set_bit()
                .sda_lvl()
                .set_bit()
                .bnk_sel()
                .clear_bit()
                .slp_start()
                .clear_bit()
                .arpmen()
                .clear_bit()
        });
        dev.regs.smbn_ctl4().modify(|_, w| w.lvl_we().clear_bit());
        // Safety: We have the peripheral, so init was called.
        dev.speed_init(config.speed, unsafe { T::clockfreq() });
        dev.regs.smbn_fif_ctl().modify(|_, w| w.fifo_en().set_bit());
        dev.regs.smbn_ctl2().modify(|_, w| w.enable().set_bit());
        dev.bank_sel(true);

        dev
    }

    fn handle_ber<T>(&mut self) -> Result<T, Error> {
        // This should be enough for arbitration errors. However, the documentation is somewhat unclear on more
        // serious problems.
        self.regs.smbn_fif_cts().write(|w| w.clr_fifo().set_bit());
        self.regs.smbn_st().write(|w| w.ber().set_bit());

        Err(Error::BusError)
    }

    fn handle_negack(&mut self) {
        self.regs.smbn_fif_cts().write(|w| w.clr_fifo().set_bit());
        self.regs.smbn_ctl1().modify(|_, w| w.stop().set_bit());
        self.regs.smbn_st().write(|w| w.negack().set_bit());
    }

    async fn bulk_write(&mut self, data: &[u8]) -> Result<(), Error> {
        // Set up transmission of first 32 bytes
        let mut data = data.iter().copied();
        for b in (&mut data).take((FIFO_SIZE).into()) {
            self.regs.smbn_sda().write(|w| unsafe { w.bits(b) });
        }

        while data.len() > 0 {
            // Wait for 8 bytes remaining in fifo
            self.regs
                .smbn_txf_ctl()
                .modify(|_, w| unsafe { w.thr_txie().set_bit().tx_thr().bits(FIFO_SIZE - GROUP_SIZE) });
            let r = self
                .wait_for(|| {
                    let r = self.regs.smbn_st().read();
                    let f = self.regs.smbn_txf_sts().read();
                    if r.ber().bit_is_set()
                        || r.negack().bit_is_set()
                        || r.sdast().bit_is_set()
                        || f.tx_thst().bit_is_set()
                    {
                        Some(r)
                    } else {
                        None
                    }
                })
                .await;
            if r.ber().bit_is_set() {
                return self.handle_ber();
            }
            if r.negack().bit_is_set() {
                self.handle_negack();
                return Err(Error::DataNack);
            }

            // Add new data
            for b in (&mut data).take(GROUP_SIZE.into()) {
                self.regs.smbn_sda().write(|w| unsafe { w.bits(b) });
            }

            // Reset threshold trigger
            self.regs.smbn_txf_sts().write(|w| w.tx_thst().set_bit());
        }

        // Wait for completion
        self.regs.smbn_txf_ctl().modify(|_, w| w.thr_txie().clear_bit());
        let r = self
            .wait_for(|| {
                let r = self.regs.smbn_st().read();
                if r.ber().bit_is_set() || r.negack().bit_is_set() || r.sdast().bit_is_set() {
                    Some(r)
                } else {
                    None
                }
            })
            .await;
        if r.ber().bit_is_set() {
            return self.handle_ber();
        }
        if r.negack().bit_is_set() {
            self.handle_negack();
            return Err(Error::DataNack);
        }

        Ok(())
    }

    // Should not be called with zero-length data
    async fn bulk_read<'a>(
        &mut self,
        mut data: &'a mut [u8],
        negack_last: bool,
        start: impl '_ + FnOnce(),
    ) -> Result<ReadCompletion<'a>, Error> {
        debug_assert_ne!(data.len(), 0);
        let mut group;

        // Setup transfer
        if data.len() <= FIFO_SIZE.into() {
            group = data;
            data = &mut [];
            // Add nack at end if required
            self.regs.smbn_rxf_ctl().write(|w| unsafe {
                w.last_pec()
                    .bit(negack_last)
                    .thr_rxie()
                    .clear_bit()
                    .rx_thr()
                    .bits(group.len() as u8)
            });
        } else if data.len() <= (2 * FIFO_SIZE).into() {
            (data, group) = data.split_at_mut(FIFO_SIZE as _);
            // Ensure we stall before the final group
            self.regs
                .smbn_rxf_ctl()
                .write(|w| unsafe { w.last_pec().clear_bit().thr_rxie().clear_bit().rx_thr().bits(FIFO_SIZE) });
        } else {
            (data, group) = data.split_at_mut(GROUP_SIZE as _);
            // Can do bulk transfer without stalling
            self.regs
                .smbn_rxf_ctl()
                .write(|w| unsafe { w.last_pec().clear_bit().thr_rxie().set_bit().rx_thr().bits(GROUP_SIZE) });
        }

        start();

        // Do bulk
        while !data.is_empty() {
            self.wait_read().await?;

            let next_group;

            if data.len() <= FIFO_SIZE.into() {
                next_group = data;
                data = &mut [];
                // Add nack at end
                self.regs.smbn_rxf_ctl().write(|w| unsafe {
                    w.last_pec()
                        .bit(negack_last)
                        .thr_rxie()
                        .clear_bit()
                        .rx_thr()
                        .bits(group.len() as u8)
                });
            } else if data.len() <= (2 * FIFO_SIZE).into() {
                (next_group, data) = data.split_at_mut(FIFO_SIZE as _);
                // Ensure we stall before the final group
                self.regs
                    .smbn_rxf_ctl()
                    .write(|w| unsafe { w.last_pec().clear_bit().thr_rxie().clear_bit().rx_thr().bits(FIFO_SIZE) });
            } else {
                (next_group, data) = data.split_at_mut(GROUP_SIZE as _);
                // Continue bulk transfer
            }

            // Next group is setup, so we can safely read data
            for b in group {
                *b = self.regs.smbn_sda().read().bits();
            }

            // And clear rx threshold status
            self.regs.smbn_rxf_sts().write(|w| w.rx_thst().set_bit());

            group = next_group;
        }

        // Finish last block
        self.wait_read().await?;

        // Return completion
        Ok(ReadCompletion { regs: self.regs, group })
    }

    pub async fn transaction(&mut self, address: u8, operations: &mut [Operation<'_>]) -> Result<(), Error> {
        enum PrevOpType<'a> {
            None,
            Read(ReadCompletion<'a>),
            Write,
        }

        let mut prevop = PrevOpType::None;

        for (op, last) in operations.iter_mut().mark_last() {
            match op {
                Operation::Write(data) => {
                    if !matches!(prevop, PrevOpType::Write) {
                        // Handle (repeated) start and address bytes
                        self.regs.smbn_ctl1().modify(|_, w| w.start().set_bit());
                        if let PrevOpType::Read(completion) = prevop {
                            completion.complete();
                        }

                        // Wait for completion
                        let r = self
                            .wait_for(|| {
                                let r = self.regs.smbn_st().read();
                                if r.ber().bit_is_set() || r.sdast().bit_is_set() {
                                    Some(r)
                                } else {
                                    None
                                }
                            })
                            .await;
                        if r.ber().bit_is_set() {
                            let _ = self.handle_ber::<()>();
                            return Err(Error::LostArbitration);
                        }

                        self.regs.smbn_sda().write(|w| unsafe { w.bits(address << 1) });

                        // Wait for completion
                        let r = self
                            .wait_for(|| {
                                let r = self.regs.smbn_st().read();
                                if r.ber().bit_is_set() || r.negack().bit_is_set() || r.sdast().bit_is_set() {
                                    Some(r)
                                } else {
                                    None
                                }
                            })
                            .await;
                        if r.ber().bit_is_set() {
                            let _ = self.handle_ber::<()>();
                            return Err(Error::LostArbitration);
                        }
                        if r.negack().bit_is_set() {
                            self.handle_negack();
                            return Err(Error::AddressNack);
                        }
                    }

                    self.bulk_write(data).await?;

                    prevop = PrevOpType::Write;
                }
                Operation::Read(data) => {
                    if data.is_empty() && matches!(prevop, PrevOpType::Read(_)) {
                        // Ignore chained zero sized reads
                    } else if let PrevOpType::Read(completion) = prevop {
                        prevop = PrevOpType::Read(self.bulk_read(data, last, || completion.complete()).await?);
                    } else {
                        // Send start and address
                        self.regs
                            .smbn_ctl1()
                            .modify(|_, w| w.stastre().set_bit().start().set_bit());
                        // Wait for completion
                        let r = self
                            .wait_for(|| {
                                let r = self.regs.smbn_st().read();
                                if r.ber().bit_is_set() || r.sdast().bit_is_set() {
                                    Some(r)
                                } else {
                                    None
                                }
                            })
                            .await;
                        if r.ber().bit_is_set() {
                            let _ = self.handle_ber::<()>();
                            return Err(Error::LostArbitration);
                        }

                        self.regs.smbn_sda().write(|w| unsafe { w.bits((address << 1) | 1) });

                        // Wait for completion
                        let r = self
                            .wait_for(|| {
                                let r = self.regs.smbn_st().read();
                                if r.ber().bit_is_set()
                                    || r.negack().bit_is_set()
                                    || r.sdast().bit_is_set()
                                    || r.stastr().bit_is_set()
                                {
                                    Some(r)
                                } else {
                                    None
                                }
                            })
                            .await;
                        if r.ber().bit_is_set() {
                            let _ = self.handle_ber::<()>();
                            return Err(Error::LostArbitration);
                        }
                        if r.negack().bit_is_set() {
                            self.handle_negack();
                            return Err(Error::AddressNack);
                        }

                        if data.is_empty() {
                            // This causes proper completion of the zero sized read on the next operation.
                            // See also the implementation of readcompletion.
                            prevop = PrevOpType::Read(ReadCompletion {
                                regs: self.regs,
                                group: &mut [],
                            });
                        } else {
                            let regs = self.regs;
                            prevop = PrevOpType::Read(
                                self.bulk_read(data, last, || {
                                    regs.smbn_st().write(|w| w.stastr().set_bit());
                                    regs.smbn_ctl1().modify(|_, w| w.stastre().clear_bit());
                                })
                                .await?,
                            );
                        }
                    }
                }
            }
        }

        // Generate stop bit and complete final read if any
        self.regs.smbn_ctl1().modify(|_, w| w.stop().set_bit());

        if let PrevOpType::Read(completion) = prevop {
            completion.complete();
        }

        // And reset fifos
        self.regs.smbn_fif_cts().write(|w| w.clr_fifo().set_bit());

        Ok(())
    }

    // Functions below ensure you don't need the embedded_hal_async trait

    pub async fn read(&mut self, address: u8, read: &mut [u8]) -> Result<(), Error> {
        self.transaction(address, &mut [Operation::Read(read)]).await
    }

    pub async fn write(&mut self, address: u8, write: &[u8]) -> Result<(), Error> {
        self.transaction(address, &mut [Operation::Write(write)]).await
    }

    pub async fn write_read(&mut self, address: u8, write: &[u8], read: &mut [u8]) -> Result<(), Error> {
        self.transaction(address, &mut [Operation::Write(write), Operation::Read(read)])
            .await
    }

    fn configure_addresses(&mut self, addresses: &[u8]) -> Result<(), ListenError> {
        let mut cnt = 0;
        for (i, addr) in addresses.iter().copied().enumerate() {
            if addr & 0x80 != 0 {
                // Not 7 bit
                return Err(ListenError::InvalidAddressList);
            }
            if addresses[..i].contains(&addr) {
                // Disallow duplicate addresses
                return Err(ListenError::InvalidAddressList);
            }
            // Don't count the addresses with special matching hardware
            if addr != GENERALCALL_ADDRESS && addr != ARP_ADDRESS {
                cnt += 1;
            }
        }
        // Check we don't exceed capacity
        if cnt > 8 {
            return Err(ListenError::InvalidAddressList);
        }

        // Store the addresses in the address registers
        let mut addr_reg_index = 0;
        self.bank_sel(false);
        for addr in addresses.iter().copied() {
            if addr == ARP_ADDRESS {
                self.regs.smbn_ctl3().modify(|_, w| w.arpmen().set_bit());
            } else if addr == GENERALCALL_ADDRESS {
                self.regs.smbn_ctl1().modify(|_, w| w.gcmen().set_bit());
            } else {
                // Unfortunately, the address registers are irregularly spaced in memory, so this match is needed
                match addr_reg_index {
                    0 => self
                        .regs
                        .smbn_addr1()
                        .write(|w| unsafe { w.addr().bits(addr).saen().set_bit() }),
                    1 => self
                        .regs
                        .smbn_addr2()
                        .write(|w| unsafe { w.addr().bits(addr).saen().set_bit() }),
                    2 => self
                        .regs
                        .smbn_addr3()
                        .write(|w| unsafe { w.addr().bits(addr).saen().set_bit() }),
                    3 => self
                        .regs
                        .smbn_addr4()
                        .write(|w| unsafe { w.addr().bits(addr).saen().set_bit() }),
                    4 => self
                        .regs
                        .smbn_addr5()
                        .write(|w| unsafe { w.addr().bits(addr).saen().set_bit() }),
                    5 => self
                        .regs
                        .smbn_addr6()
                        .write(|w| unsafe { w.addr().bits(addr).saen().set_bit() }),
                    6 => self
                        .regs
                        .smbn_addr7()
                        .write(|w| unsafe { w.addr().bits(addr).saen().set_bit() }),
                    7 => self
                        .regs
                        .smbn_addr8()
                        .write(|w| unsafe { w.addr().bits(addr).saen().set_bit() }),
                    _ => unreachable!("Incorrectly checked address list"),
                };
                addr_reg_index += 1;
            }
        }
        self.bank_sel(true);

        Ok(())
    }

    fn disable_addresses(&mut self) {
        self.bank_sel(false);
        self.regs.smbn_ctl1().modify(|_, w| w.gcmen().clear_bit());
        self.regs.smbn_ctl3().modify(|_, w| w.arpmen().clear_bit());
        self.regs.smbn_addr1().write(|w| unsafe { w.bits(0) });
        self.regs.smbn_addr2().write(|w| unsafe { w.bits(0) });
        self.regs.smbn_addr3().write(|w| unsafe { w.bits(0) });
        self.regs.smbn_addr4().write(|w| unsafe { w.bits(0) });
        self.regs.smbn_addr5().write(|w| unsafe { w.bits(0) });
        self.regs.smbn_addr6().write(|w| unsafe { w.bits(0) });
        self.regs.smbn_addr7().write(|w| unsafe { w.bits(0) });
        self.regs.smbn_addr8().write(|w| unsafe { w.bits(0) });
        self.bank_sel(true);
    }

    fn decode_addr(&mut self) -> Option<u8> {
        let r = self.regs.smbn_cst().read();
        if r.gcmatch().bit_is_set() {
            Some(GENERALCALL_ADDRESS)
        } else if r.arpmatch().bit_is_set() {
            Some(ARP_ADDRESS)
        } else {
            let r = self.regs.smbn_cst2().read();
            if r.matcha1f().bit_is_set() {
                Some(self.regs.smbn_addr1().read().addr().bits())
            } else if r.matcha2f().bit_is_set() {
                Some(self.regs.smbn_addr2().read().addr().bits())
            } else if r.matcha3f().bit_is_set() {
                self.bank_sel(false);
                let a = self.regs.smbn_addr3().read();
                self.bank_sel(true);
                Some(a.addr().bits())
            } else if r.matcha4f().bit_is_set() {
                self.bank_sel(false);
                let a = self.regs.smbn_addr4().read();
                self.bank_sel(true);
                Some(a.addr().bits())
            } else if r.matcha5f().bit_is_set() {
                self.bank_sel(false);
                let a = self.regs.smbn_addr5().read();
                self.bank_sel(true);
                Some(a.addr().bits())
            } else if r.matcha6f().bit_is_set() {
                self.bank_sel(false);
                let a = self.regs.smbn_addr6().read();
                self.bank_sel(true);
                Some(a.addr().bits())
            } else if r.matcha7f().bit_is_set() {
                self.bank_sel(false);
                let a = self.regs.smbn_addr7().read();
                self.bank_sel(true);
                Some(a.addr().bits())
            } else if self.regs.smbn_cst3().read().matcha8f().bit_is_set() {
                self.bank_sel(false);
                let a = self.regs.smbn_addr7().read();
                self.bank_sel(true);
                Some(a.addr().bits())
            } else {
                None
            }
        }
    }

    async fn handle_listen_transaction(&mut self, mut handler: impl FnMut(u8, ListenCommand)) {
        let Some(addr) = self.decode_addr() else {
            // Spurious nmatch, clear it and return
            self.regs.smbn_st().write(|w| w.nmatch().set_bit());
            return;
        };

        let mut buffer = [0u8; GROUP_SIZE as _];

        if self.regs.smbn_st().read().xmit().bit_is_set() {
            // Read from master
            self.regs.smbn_st().write(|w| w.nmatch().set_bit());

            let mut bytes_read = 0usize;

            let st = loop {
                handler(addr, ListenCommand::PrepareRead(&mut buffer));

                for b in buffer {
                    self.regs.smbn_sda().write(|w| unsafe { w.bits(b) });
                }
                self.regs
                    .smbn_txf_ctl()
                    .modify(|_, w| unsafe { w.tx_thr().bits(FIFO_SIZE - GROUP_SIZE).thr_txie().set_bit() });
                self.regs.smbn_txf_sts().write(|w| w.tx_thst().set_bit());
                bytes_read += GROUP_SIZE as usize;

                let st = self
                    .wait_for(|| {
                        let st = self.regs.smbn_st().read();
                        let txf_sts = self.regs.smbn_txf_sts().read();
                        if txf_sts.tx_thst().bit_is_set()
                            || st.ber().bit_is_set()
                            || st.negack().bit_is_set()
                            || st.sdast().bit_is_set()
                        {
                            Some(st)
                        } else {
                            None
                        }
                    })
                    .await;

                if st.ber().bit_is_set() || st.negack().bit_is_set() {
                    break st;
                }
            };

            if st.ber().bit_is_set() {
                handler(0, ListenCommand::BusError);
            }

            bytes_read -= self.regs.smbn_txf_sts().read().tx_bytes().bits() as usize;

            handler(addr, ListenCommand::ReadFinished(bytes_read));

            // Reset so we can wait for the next transaction
            self.regs.smbn_txf_ctl().modify(|_, w| w.thr_txie().clear_bit());
            self.regs.smbn_fif_cts().write(|w| w.clr_fifo().set_bit());
            // Clearing the ber bit is enough to handle the bus error.
            self.regs.smbn_st().write(|w| w.ber().set_bit().negack().set_bit());
        } else {
            // Write from master

            // Start receiving bytes
            // Note that we wait till we have 1 byte more than the group size to ensure slvrstr is always triggered when relevant
            self.regs.smbn_rxf_ctl().write(|w| unsafe {
                w.last_pec()
                    .clear_bit()
                    .thr_rxie()
                    .set_bit()
                    .rx_thr()
                    .bits(GROUP_SIZE + 1)
            });
            self.regs.smbn_st().write(|w| w.nmatch().set_bit());

            let st = loop {
                let (st, fif_cts) = self
                    .wait_for(|| {
                        let st = self.regs.smbn_st().read();
                        let rxf_sts = self.regs.smbn_rxf_sts().read();
                        let fif_cts = self.regs.smbn_fif_cts().read();
                        if st.ber().bit_is_set()
                            || st.slvstp().bit_is_set()
                            || st.sdast().bit_is_set()
                            || rxf_sts.rx_thst().bit_is_set()
                            || fif_cts.slvrstr().bit_is_set()
                        {
                            Some((st, fif_cts))
                        } else {
                            None
                        }
                    })
                    .await;

                if st.ber().bit_is_set() || st.slvstp().bit_is_set() || fif_cts.slvrstr().bit_is_set() {
                    break st;
                }

                for b in buffer.iter_mut() {
                    *b = self.regs.smbn_sda().read().bits();
                }

                self.regs.smbn_rxf_sts().write(|w| w.rx_thst().set_bit());

                handler(addr, ListenCommand::PartialWrite(&buffer));
            };

            if st.ber().bit_is_set() {
                handler(0, ListenCommand::BusError);
            }

            // Read remaining bytes
            let rest = self.regs.smbn_rxf_sts().read().rx_bytes().bits();
            if rest > GROUP_SIZE {
                for b in buffer.iter_mut() {
                    *b = self.regs.smbn_sda().read().bits();
                }

                handler(addr, ListenCommand::WriteFinished(&buffer));
            }

            for b in buffer.iter_mut().take(rest.into()) {
                *b = self.regs.smbn_sda().read().bits();
            }

            handler(addr, ListenCommand::WriteFinished(&buffer[..rest.into()]));

            // Reset and prepare for next transaction
            // Note that we leave slvstp for the caller to deal with
            self.regs.smbn_rxf_ctl().write(|w| w.thr_rxie().clear_bit());
            self.regs
                .smbn_fif_cts()
                .write(|w| w.clr_fifo().set_bit().slvrstr().set_bit());
            // Clearing the Ber bit is enough to handle the bus error
            self.regs.smbn_st().write(|w| w.ber().set_bit());
        }
    }

    /// Listen for i2c interactions targeting the specified addresses. The handler will be called
    /// to handle the various transactions. The listening can be stopped by setting the provided
    /// watched value to true
    pub async fn listen<const N: usize>(
        &mut self,
        addresses: &[u8],
        mut handler: impl FnMut(u8, ListenCommand),
        stop: &mut watch::Receiver<'_, impl RawMutex, bool, N>,
    ) -> Result<(), ListenError> {
        self.regs.smbn_ctl1().modify(|_, w| w.nminte().set_bit());
        if let Err(e) = self.configure_addresses(addresses) {
            self.regs.smbn_ctl1().modify(|_, w| w.nminte().clear_bit());
            return Err(e);
        }

        loop {
            match select(
                self.wait_for(|| {
                    if self.regs.smbn_st().read().nmatch().bit_is_set() {
                        Some(())
                    } else {
                        None
                    }
                }),
                stop.get_and(|v| *v),
            )
            .await
            {
                embassy_futures::select::Either::First(_) => {
                    // just continue the loop
                }
                embassy_futures::select::Either::Second(_) => {
                    // cleanup the wait for interrupt
                    self.regs.smbn_ctl1().modify(|_, w| w.inten().clear_bit());

                    // disable listening
                    self.disable_addresses();

                    // Only stop if there is no pending transaction, otherwise handle those first
                    if self.regs.smbn_st().read().nmatch().bit_is_clear() {
                        self.regs.smbn_ctl1().modify(|_, w| w.nminte().clear_bit());
                        return Ok(());
                    }
                }
            }

            loop {
                self.handle_listen_transaction(&mut handler).await;

                // wait for slvstp or a new match
                let st = self
                    .wait_for(|| {
                        let st = self.regs.smbn_st().read();
                        if st.nmatch().bit_is_set() || st.slvstp().bit_is_set() {
                            Some(st)
                        } else {
                            None
                        }
                    })
                    .await;

                if st.slvstp().bit_is_set() {
                    break;
                }
            }

            // clear the slave stop
            self.regs.smbn_st().write(|w| w.slvstp().set_bit());
        }
    }
}

impl embedded_hal_async::i2c::ErrorType for I2CController<'_> {
    type Error = Error;
}

impl I2c for I2CController<'_> {
    async fn transaction(&mut self, address: u8, operations: &mut [Operation<'_>]) -> Result<(), Self::Error> {
        (*self).transaction(address, operations).await
    }
}

mod sealed {
    use embassy_sync::waitqueue::AtomicWaker;

    pub trait SealedValidI2CConfig {
        unsafe fn setup_pins(cs: critical_section::CriticalSection);
        unsafe fn setup_pullup(_cs: critical_section::CriticalSection, enable: bool);
    }

    pub trait SealedInstance {
        fn waker() -> &'static AtomicWaker;
        fn regs() -> &'static crate::pac::smb0::RegisterBlock;
        /// Safety: should only be called after clock init
        unsafe fn clockfreq() -> u32;
    }
}

pub trait ValidI2CConfig: sealed::SealedValidI2CConfig {}

pub trait Instance: sealed::SealedInstance + embassy_hal_internal::Peripheral<P = Self> {
    type Interrupt: crate::interrupt::typelevel::Interrupt;
}

macro_rules! impl_instance {
    ($instance:ident, $pac:ident, $clock:ident) => {
        impl sealed::SealedInstance for crate::peripherals::$instance {
            fn waker() -> &'static AtomicWaker {
                static WAKER: AtomicWaker = AtomicWaker::new();
                &WAKER
            }

            fn regs() -> &'static crate::pac::smb0::RegisterBlock {
                // Safety: not owned, memory is always present
                unsafe { &*crate::pac::$pac::PTR }
            }

            unsafe fn clockfreq() -> u32 {
                // Safety: We require clock init to be called before this is called
                unsafe { get_clocks() }.$clock
            }
        }

        impl Instance for crate::peripherals::$instance {
            type Interrupt = crate::interrupt::typelevel::$instance;
        }
    };
}

macro_rules! impl_config {
    ($instance:ident, $scl:ident, $sda:ident, $pin_config:expr, $pullup_config:expr) => {
        impl<T> sealed::SealedValidI2CConfig
            for (
                crate::peripherals::$instance,
                crate::peripherals::$scl,
                crate::peripherals::$sda,
                T,
            )
        {
            unsafe fn setup_pins(_cs: critical_section::CriticalSection) {
                fn internal_set(f: impl FnOnce(crate::pac::Sysconfig, crate::pac::Sysglue)) {
                    f(unsafe { crate::pac::Sysconfig::steal() }, unsafe {
                        crate::pac::Sysglue::steal()
                    });
                }
                internal_set($pin_config);
            }

            unsafe fn setup_pullup(_cs: critical_section::CriticalSection, enable: bool) {
                fn internal_set(f: impl FnOnce(crate::pac::Sysconfig, bool), enable: bool) {
                    f(unsafe { crate::pac::Sysconfig::steal() }, enable);
                }
                internal_set($pullup_config, enable);
            }
        }

        impl<T> ValidI2CConfig
            for (
                crate::peripherals::$instance,
                crate::peripherals::$scl,
                crate::peripherals::$sda,
                T,
            )
        {
        }
    };
}

macro_rules! impl_config_lpc {
    ($instance:ident, $scl:ident, $sda:ident, $pin_config:expr, $pullup_config:expr) => {
        impl sealed::SealedValidI2CConfig
            for (
                crate::peripherals::$instance,
                crate::peripherals::$scl,
                crate::peripherals::$sda,
                crate::Lpc,
            )
        {
            unsafe fn setup_pins(_cs: critical_section::CriticalSection) {
                fn internal_set(f: impl FnOnce(crate::pac::Sysconfig, crate::pac::Sysglue)) {
                    f(unsafe { crate::pac::Sysconfig::steal() }, unsafe {
                        crate::pac::Sysglue::steal()
                    });
                }
                internal_set($pin_config);
            }

            unsafe fn setup_pullup(_cs: critical_section::CriticalSection, enable: bool) {
                fn internal_set(f: impl FnOnce(crate::pac::Sysconfig, bool), enable: bool) {
                    f(unsafe { crate::pac::Sysconfig::steal() }, enable);
                }
                internal_set($pullup_config, enable);
            }
        }

        impl ValidI2CConfig
            for (
                crate::peripherals::$instance,
                crate::peripherals::$scl,
                crate::peripherals::$sda,
                crate::Lpc,
            )
        {
        }
    };
}

impl_instance!(SMB0, Smb0, apb3_clk);
impl_instance!(SMB1, Smb1, apb3_clk);
impl_instance!(SMB2, Smb2, apb2_clk);
impl_instance!(SMB3, Smb3, apb2_clk);
impl_instance!(SMB4, Smb4, apb3_clk);
impl_instance!(SMB5, Smb5, apb3_clk);
impl_instance!(SMB6, Smb6, apb3_clk);
impl_instance!(SMB7, Smb7, apb3_clk);

impl_config!(
    SMB0,
    PC12,
    PB12,
    |config, _| {
        config.devalt2().modify(|_, w| w.i2c0_0_sl().set_bit());
    },
    |config, enable| {
        config.pupd_en0().modify(|_, w| w.i2c0_0_pue().bit(enable));
    }
);

impl_config!(
    SMB1,
    PK08,
    PK07,
    |config, _| {
        config.devalt2().modify(|_, w| w.i2c1_0_sl().set_bit());
    },
    |config, enable| {
        config.pupd_en0().modify(|_, w| w.i2c1_0_pue().bit(enable));
    }
);

impl_config!(
    SMB2,
    PL08,
    PK09,
    |config, _| {
        config.devalt2().modify(|_, w| w.i2c2_0_sl().set_bit());
    },
    |config, enable| {
        config.pupd_en0().modify(|_, w| w.i2c2_0_pue().bit(enable));
    }
);

impl_config!(
    SMB3,
    PF08,
    PF09,
    |config, _| {
        config.devalt2().modify(|_, w| w.i2c3_0_sl().set_bit());
        config.devaltm().modify(|_, w| w.ad22_sl().clear_bit());
    },
    |config, enable| {
        config.pupd_en0().modify(|_, w| w.i2c3_0_pue().bit(enable));
    }
);

impl_config!(
    SMB4,
    PJ06,
    PJ09,
    |config, glue| {
        glue.smb_sel().modify(|_, w| w.smb4_sl().clear_bit());
        // safety: we are in a critical section
        unsafe { crate::pac::Miwu0::steal() }
            .wkpcln5()
            .write(|w| w.input3().set_bit());
        config.devalta().modify(|_, w| w._32k_out_sl().clear_bit());
        config.devalt2().modify(|_, w| w.i2c4_0_sl().set_bit());
        config
            .devaltb()
            .modify(|_, w| w.rxd_sl().clear_bit().txd_sl().clear_bit());
        config
            .devaltj()
            .modify(|_, w| w.cr_sin2_sl().clear_bit().cr_sout2_sl().clear_bit());
    },
    |config, enable| {
        config.pupd_en0().modify(|_, w| w.i2c4_0_pue().bit(enable));
    }
);

impl_config!(
    SMB4,
    PF05,
    PF06,
    |config, glue| {
        glue.smb_sel().modify(|_, w| w.smb4_sl().set_bit());
        // safety: we are in a critical section
        unsafe { crate::pac::Miwu0::steal() }
            .wkpcln5()
            .write(|w| w.input3().set_bit());
        config.devalt6().modify(|_, w| w.i2c4_1_sl().set_bit());
    },
    |config, enable| {
        config.pupd_en1().modify(|_, w| w.i2c4_1_pue().bit(enable));
    }
);

impl_config!(
    SMB5,
    PD05,
    PD04,
    |config, glue| {
        glue.smb_sel().modify(|_, w| w.smb5_sl().clear_bit());
        // safety: we are in a critical section
        unsafe { crate::pac::Miwu2::steal() }
            .wkpcln7()
            .write(|w| w.input0().set_bit());
        config.devalt2().modify(|_, w| w.i2c5_0_sl().set_bit());
        config
            .devaltb()
            .modify(|_, w| w.cts_sl().clear_bit().rts_sl().clear_bit());
    },
    |config, enable| {
        config.pupd_en0().modify(|_, w| w.i2c5_0_pue().bit(enable));
    }
);

impl_config_lpc!(
    SMB5,
    PE08,
    PE09,
    |config, glue| {
        glue.smb_sel().modify(|_, w| w.smb5_sl().set_bit());
        // safety: we are in a critical section
        unsafe { crate::pac::Miwu2::steal() }
            .wkpcln7()
            .write(|w| w.input0().set_bit());
        config.devalt6().modify(|_, w| w.i2c5_1_sl().set_bit());
        config.devaltn().modify(|_, w| w.i3c1_sl().clear_bit());
    },
    |config, enable| {
        config.pupd_en1().modify(|_, w| w.i2c5_1_pue().bit(enable));
    }
);

impl_config!(
    SMB6,
    PH10,
    PH09,
    |config, glue| {
        glue.smb_sel().modify(|_, w| w.smb6_sl().clear_bit());
        // safety: we are in a critical section
        unsafe { crate::pac::Miwu2::steal() }
            .wkpcln7()
            .write(|w| w.input1().set_bit());
        config.devalt4().modify(|_, w| w.pwm1_sl().clear_bit());
        config.devalt2().modify(|_, w| w.i2c6_0_sl().set_bit());
        config.devaltm().modify(|_, w| w.ad22_sl().clear_bit());
    },
    |config, enable| {
        config.pupd_en0().modify(|_, w| w.i2c6_0_pue().bit(enable));
    }
);

impl_config!(
    SMB6,
    PL06,
    PL07,
    |config, glue| {
        glue.smb_sel().modify(|_, w| w.smb6_sl().set_bit());
        // safety: we are in a critical section
        unsafe { crate::pac::Miwu2::steal() }
            .wkpcln7()
            .write(|w| w.input1().set_bit());
        config.devalt6().modify(|_, w| w.i2c6_1_sl().set_bit());
        config.devaltn().modify(|_, w| w.i3c1_sl().clear_bit());
    },
    |config, enable| {
        config.pupd_en1().modify(|_, w| w.i2c6_1_pue().bit(enable));
    }
);

impl_config!(
    SMB7,
    PJ10,
    PK10,
    |config, glue| {
        glue.smb_sel().modify(|_, w| w.smb7_sl().clear_bit());
        // safety: we are in a critical section
        unsafe { crate::pac::Miwu2::steal() }
            .wkpcln7()
            .write(|w| w.input2().set_bit());
        config.devalt2().modify(|_, w| w.i2c7_0_sl().set_bit());
        config
            .devaltb()
            .modify(|_, w| w.dsr_sl().clear_bit().dcd_sl().clear_bit());
    },
    |config, enable| {
        config.pupd_en0().modify(|_, w| w.i2c7_0_pue().bit(enable));
    }
);

impl_config!(
    SMB7,
    PH08,
    PJ07,
    |config, glue| {
        glue.smb_sel().modify(|_, w| w.smb7_sl().set_bit());
        // safety: we are in a critical section
        unsafe { crate::pac::Miwu2::steal() }
            .wkpcln7()
            .write(|w| w.input2().set_bit());
        config
            .devalt4()
            .modify(|_, w| w.pwm5_sl().clear_bit().pwm6_sl().clear_bit());
        config.devaltk().modify(|_, w| w.i2c7_1_sl().set_bit());
        // Note: this messes with debug interfaces, maybe there is a better way?
        unsafe { crate::pac::Dev::steal() }
            .dbgctrl2()
            .modify(|_, w| w.ccdev_sel().bits(6));
    },
    |config, enable| {
        config.pupd_en1().modify(|_, w| w.i2c7_1_pue().bit(enable));
    }
);
