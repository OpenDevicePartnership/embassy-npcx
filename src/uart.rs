#![allow(private_interfaces)]

//! Core Universal Asynchronous Receiver-Transmitter (CR_UART).
//!
//! Implements the full-duplex receiver transmitter integration with 16-byte FIFO buffers for receive and transmit.
//! Does not (yet) support DMA transactions.

use core::convert::Infallible;
use core::future::poll_fn;
use core::marker::PhantomData;
use core::sync::atomic::{AtomicU8, Ordering};
use core::task::Poll;

use embassy_hal_internal::{impl_peripheral, Peri};
use embassy_sync::waitqueue::AtomicWaker;

use crate::interrupt::typelevel::Interrupt;

/// Configuration for the number of stopbits
#[derive(Debug, Copy, Clone, Eq, PartialEq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum StopBits {
    /// One stop bit
    STOP1,
    /// Two stop bits
    STOP2,
}

/// Configuration for the parity used
#[derive(Debug, Copy, Clone, Eq, PartialEq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Parity {
    /// Use odd parity
    ParityOdd = 0b00,
    /// Use even parity
    ParityEven = 0b01,
    /// Force parity bit to be 1
    ParityMark = 0b10,
    /// Force parity bit to be 0
    ParitySpace = 0b11,
}

/// Base configuration for the Core Uart peripheral.
///
/// Applicable for both the "Separate Mode" and the "Common Mode".
#[derive(Debug, Copy, Clone, Eq, PartialEq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub struct Config {
    /// The target baudrate
    pub baudrate: u32,
    /// The number of stop bits
    pub stop_bits: StopBits,
    /// If None, no parity bit is added. If Some, a parity bit is added according to the value
    pub parity: Option<Parity>,
    /// When true the input is inverted. High is seen as low and low is seen as high
    pub input_inverted: bool,
    /// When true the output is inverted. High is seen as low and low is seen as high
    pub output_inverted: bool,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            baudrate: 115_200,
            stop_bits: StopBits::STOP1,
            parity: None,
            input_inverted: false,
            output_inverted: false,
        }
    }
}

/// "Common Mode" configuration for the Core Uart peripheral.
#[derive(Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub struct CommonModeConfig {
    /// The base config just like for normal uart
    pub base: Config,
    /// When true the pin is in push-pull mode. Otherwise it's open-drain
    pub push_pull: bool,
    /// When true an 'echo' of all transmitted data is received by the receiver side
    pub feedback: bool,
}

/// The error type for the uart driver
#[derive(Debug, Copy, Clone, Eq, PartialEq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error {
    /// A line break was detected
    Break,
    /// Data wasn't processed fast enough and an overrun happened
    DataOverrun,
    /// Something went wrong with the framing of the incoming data
    Framing,
    /// Invalid parity detected
    Parity,
}

impl core::fmt::Display for Error {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Error::Break => write!(f, "Line break detected"),
            Error::DataOverrun => write!(f, "Data overrun"),
            Error::Framing => write!(f, "Framing error"),
            Error::Parity => write!(f, "Parity error"),
        }
    }
}
impl core::error::Error for Error {}

struct AnyUart {
    regs: &'static crate::pac::cr_uart1::RegisterBlock,
    rx_waker: &'static AtomicWaker,
    tx_waker: &'static AtomicWaker,
    state: &'static State,
}

impl<T: Instance> From<T> for AnyUart {
    fn from(_uart: T) -> Self {
        AnyUart {
            regs: T::regs(),
            rx_waker: T::rx_waker(),
            tx_waker: T::tx_waker(),
            state: T::state(),
        }
    }
}

// Allow use of Peri to do lifetime management
impl_peripheral!(AnyUart);

/// Core Universal Asynchronous Receiver-Transmitter (CR_UART) driver.
pub struct Uart<'a, T> {
    peripheral: PhantomData<T>,
    rx: UartRx<'a>,
    tx: UartTx<'a>,
}

/// Potential UART clock configuration.
///
/// The baudrate is computed as follows:
/// ```"not rust"
/// BR = SRC / (16 x div x p)
/// (div x p) = SRC / (16 * BR)
/// ```
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
struct ClockConfiguration {
    // Divisor value
    div: u16,
    // Prescaler value times two
    p2: u8,
}

impl ClockConfiguration {
    pub fn from_registers(r: &crate::pac::cr_uart1::RegisterBlock) -> Self {
        let upsrn = r.upsrn().read();

        let udiv10_l = r.ubaudn().read().bits() as u16;
        let udiv10_h = upsrn.udiv10_8().bits() as u16;
        let udiv10 = udiv10_h << 8 | udiv10_l;
        let upsc = upsrn.upsc().bits();

        Self {
            div: udiv10 + 1,
            p2: upsc + 1,
        }
    }

    pub fn generate_valid(srcclk: u32, desired_baudrate: u32) -> impl Iterator<Item = Self> {
        let dstclk2 = (srcclk * 2) / 16 / desired_baudrate;

        (2..=32)
            .flat_map(move |p2| {
                let div_a = dstclk2.div_ceil(p2 as u32);
                let div_b = dstclk2 / p2 as u32;
                // Note(cast to u16): if overflow will pin to 0xffff, which will be filtered later on.
                [
                    ClockConfiguration { div: div_a as u16, p2 },
                    ClockConfiguration { div: div_b as u16, p2 },
                ]
                .into_iter()
            })
            .filter(|clkcfg| clkcfg.div > 0 && clkcfg.div <= 0x800)
    }

    // Compute the UDIV10_0 register field value.
    pub fn udiv10(&self) -> u16 {
        self.div - 1
    }

    /// Compute UPSC register field value.
    pub fn upsc(&self) -> u8 {
        self.p2 - 1 // 1,1.5,2..=16 => 1..=31
    }

    // Compute the effective baudrate given a source clock.
    pub fn baudrate(&self, srcclk: u32) -> u32 {
        // BR = SRC / (16 x div x p)
        srcclk / ((16 * self.div as u32 * self.p2 as u32) / 2)
    }
}

impl<'a, T: Instance + 'a> Uart<'a, T> {
    /// Configure the base registers for the peripheral and enables it.
    fn configure_enable(config: Config) {
        // Safety: _irqs ensures an interrupt handler is bound
        unsafe {
            T::Interrupt::enable();
        }

        let r = T::regs();

        r.ucntln().modify(|_, w| {
            w.cr_sin_inv()
                .bit(config.input_inverted)
                .cr_sout_inv()
                .bit(config.output_inverted)
        });

        r.ufrsn().write(|w| {
            match config.parity {
                Some(parity) => {
                    w.pen().set_bit();
                    unsafe { w.psel().bits(parity as u8) };
                }
                None => {
                    w.pen().clear_bit();
                }
            };

            w.stp().bit(config.stop_bits == StopBits::STOP2)
        });

        // Set TX FIFO watermark level to 1.
        r.uftctln().modify(|_, w| unsafe { w.tempty_level_sel().bits(0x01) });

        // Safety: UART can only be initialized after the clocks have been initialized.
        let srcclk = unsafe { crate::cdcg::get_clocks() }.apb4_clk;

        let clkcfg = ClockConfiguration::generate_valid(srcclk, config.baudrate)
            // Minimize baudrate error.
            .min_by_key(move |cfg| cfg.baudrate(srcclk).abs_diff(config.baudrate))
            .expect("Failed to find clock configuration for requested baudrate");

        #[cfg(feature = "defmt")]
        {
            let eff = clkcfg.baudrate(srcclk);
            defmt::debug!(
                "uart: {}, target: {}, eff:{}, diff:{}",
                clkcfg,
                config.baudrate,
                eff,
                eff as i64 - config.baudrate as i64
            );
        }

        r.ubaudn().write(|w| unsafe { w.bits((clkcfg.udiv10() & 0xff) as u8) });
        // Setting the prescaler to non-zero also enables the peripheral.
        r.upsrn().write(|w| unsafe {
            w.upsc()
                .bits(clkcfg.upsc())
                .udiv10_8()
                .bits(((clkcfg.udiv10() & 0x700) >> 8) as u8)
        });
    }

    /// Configure the base registers and general common mode registers for the peripheral, and enables it.
    fn configure_common_mode_enable(config: CommonModeConfig) {
        T::regs().ucntln().modify(|_, w| w.com_fdbk_en().bit(config.feedback));
        Self::configure_enable(config.base);
    }

    fn instantiate_rx_tx(peri: Peri<'a, T>) -> Self {
        // Safety: we ensure that UartRx and UartTx can work at the same time.
        Self {
            rx: UartRx::<'a>::new(unsafe { peri.clone_unchecked().into() }),
            tx: UartTx::<'a>::new(peri.into()),
            peripheral: Default::default(),
        }
    }

    /// Compute the effective baudrate.
    pub fn baudrate(&self) -> u32 {
        let clkcfg = ClockConfiguration::from_registers(T::regs());
        let srcclk = unsafe { crate::cdcg::get_clocks() }.apb4_clk;
        clkcfg.baudrate(srcclk)
    }

    /// Enables the peripheral in "Separate Mode" with applicable input and output pins.
    pub fn new<Sin: InputPin, Sout: OutputPin>(
        peri: Peri<'a, T>,
        _sin: Peri<'a, Sin>,
        _sout: Peri<'a, Sout>,
        _irqs: impl crate::interrupt::typelevel::Binding<T::Interrupt, InterruptHandler<T>>,
        config: Config,
    ) -> Self {
        // Note(cs): other peripherals might also be modifying swsrst* at the same time.
        critical_section::with(|cs| {
            // Safety: We have exclusive ownership over the peripherals.
            unsafe {
                T::reset(cs);
                Sin::setup(cs);
                Sout::setup(cs)
            };
        });

        Self::configure_enable(config);
        Self::instantiate_rx_tx(peri)
    }

    /// Enables the peripheral in "Separate Mode" with applicable input pin.
    pub fn new_rx<Sin: InputPin>(
        peri: Peri<'a, T>,
        _sin: Peri<'a, Sin>,
        _irqs: impl crate::interrupt::typelevel::Binding<T::Interrupt, InterruptHandler<T>>,
        config: Config,
    ) -> UartRx<'a> {
        // Note(cs): other peripherals might also be modifying swsrst* at the same time.
        critical_section::with(|cs| {
            // Safety: We have exclusive ownership over the peripherals.
            unsafe {
                T::reset(cs);
                Sin::setup(cs)
            };
        });

        Self::configure_enable(config);

        UartRx::new(peri.into())
    }

    /// Enables the peripheral in "Separate Mode" with applicable output pin.
    pub fn new_tx<Sout: OutputPin>(
        peri: Peri<'a, T>,
        _sout: Peri<'a, Sout>,
        _irqs: impl crate::interrupt::typelevel::Binding<T::Interrupt, InterruptHandler<T>>,
        config: Config,
    ) -> UartTx<'a> {
        // Note(cs): other peripherals might also be modifying swsrst* at the same time.
        critical_section::with(|cs| {
            // Safety: We have exclusive ownership over the peripherals.
            unsafe {
                T::reset(cs);
                Sout::setup(cs)
            };
        });

        Self::configure_enable(config);

        UartTx::new(peri.into())
    }

    /// Enables the peripheral in "Common Mode" by using an applicable input pin as both input and output.
    pub fn new_common<Scom: CommonPin>(
        peri: Peri<'a, T>,
        _scom: Peri<'a, Scom>,
        _irqs: impl crate::interrupt::typelevel::Binding<T::Interrupt, InterruptHandler<T>>,
        config: CommonModeConfig,
    ) -> Self {
        // Note(cs): other peripherals might also be modifying swsrst* at the same time.
        critical_section::with(|cs| {
            // Safety: We have exclusive ownership over the peripherals.
            unsafe {
                T::reset(cs);
                Scom::setup(cs);
            };
        });

        Scom::configure_for_common::<T>(config.push_pull);
        Self::configure_common_mode_enable(config);
        Self::instantiate_rx_tx(peri)
    }

    /// Split the uart into a receiver and sender.
    /// This can make doing rx and tx in different tasks much easier.
    pub fn split(self) -> (UartRx<'a>, UartTx<'a>) {
        (self.rx, self.tx)
    }
}

/// A receive-only uart
pub struct UartRx<'a> {
    dev: Peri<'a, AnyUart>,
}

/// A send-only uart
pub struct UartTx<'a> {
    dev: Peri<'a, AnyUart>,
}

impl<'a> UartRx<'a> {
    fn new(dev: Peri<'a, AnyUart>) -> Self {
        dev.state.rx_tx_refcount.fetch_add(1, Ordering::AcqRel);
        Self { dev }
    }
}

fn drop_rx_tx(dev: &Peri<'_, AnyUart>) {
    if dev.state.rx_tx_refcount.fetch_sub(1, Ordering::AcqRel) == 1 {
        // We need to clean up.

        // Setting the prescaler to 0 disables the clock and disables the peripheral.
        dev.regs.upsrn().write(|w| unsafe { w.upsc().bits(0b0_0000) });
    }
}

impl Drop for UartRx<'_> {
    fn drop(&mut self) {
        drop_rx_tx(&self.dev);
    }
}

impl<'a> UartTx<'a> {
    fn new(dev: Peri<'a, AnyUart>) -> Self {
        dev.state.rx_tx_refcount.fetch_add(1, Ordering::AcqRel);
        Self { dev }
    }
}

impl Drop for UartTx<'_> {
    fn drop(&mut self) {
        drop_rx_tx(&self.dev);
    }
}

/// The interrupt handler for the uart driver
pub struct InterruptHandler<T> {
    _phantom: PhantomData<T>,
}

impl<T: Instance> crate::interrupt::typelevel::Handler<T::Interrupt> for InterruptHandler<T> {
    unsafe fn on_interrupt() {
        let r = T::regs();

        // If async task is awaiting a byte to be available, and a byte is available or an error occurred.
        if r.ufrctln().read().rfifo_nempty_en().bit_is_set() {
            let rsts = r.ufrstsn().read();
            if rsts.rfifo_nempty_sts().bit_is_set() || rsts.err().bit_is_set() {
                r.ufrctln()
                    .modify(|_, w| w.rfifo_nempty_en().clear_bit().err_en().clear_bit());
                T::rx_waker().wake();
            }
        }

        // If async task is awaiting for space in TX fifo, and space became available.
        if r.uftctln().read().tempty_level_en().bit_is_set() && r.uftstsn().read().tempty_level().bits() != 0 {
            r.uftctln().modify(|_, w| w.tempty_level_en().clear_bit());
            T::tx_waker().wake();
        }

        // If async task is awaiting for transmission to be completed.
        if r.uftctln().read().nxmip_en().bit_is_set() && r.uftstsn().read().nxmip().bit_is_set() {
            r.uftctln().modify(|_, w| w.nxmip_en().clear_bit());
            T::tx_waker().wake();
        }
    }
}

impl embedded_io_async::Error for Error {
    fn kind(&self) -> embedded_io_async::ErrorKind {
        use embedded_io_async::ErrorKind;
        match self {
            Error::Break => ErrorKind::InvalidData,
            Error::DataOverrun => ErrorKind::OutOfMemory,
            Error::Framing => ErrorKind::InvalidData,
            Error::Parity => ErrorKind::InvalidData,
        }
    }
}

impl embedded_io_async::ErrorType for UartRx<'_> {
    type Error = Error;
}

impl embedded_io_async::ErrorType for UartTx<'_> {
    type Error = Infallible;
}

impl embedded_io_async::Read for UartRx<'_> {
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        let r = self.dev.regs;

        // If we have no bytes pending, await until we have at least a single byte pending or error.
        let rsts = r.ufrstsn().read();
        if rsts.rfifo_nempty_sts().bit_is_clear() && rsts.err().bit_is_clear() {
            // Note(cs): register is also modified in interrupt handler.
            critical_section::with(|_| {
                r.ufrctln()
                    .modify(|_, w| w.rfifo_nempty_en().set_bit().err_en().set_bit());
            });

            poll_fn(|cx| {
                self.dev.rx_waker.register(cx.waker());

                let rsts = r.ufrstsn().read();
                if rsts.rfifo_nempty_sts().bit_is_set() || rsts.err().bit_is_set() {
                    Poll::Ready(())
                } else {
                    Poll::Pending
                }
            })
            .await;
        }

        // Note: clears ustat bits when read.
        let ustat = r.ustatn().read();
        if ustat.bkd().bit_is_set() {
            return Err(Error::Break);
        } else if ustat.doe().bit_is_set() {
            return Err(Error::DataOverrun);
        } else if ustat.fe().bit_is_set() {
            return Err(Error::Framing);
        } else if ustat.pe().bit_is_set() {
            return Err(Error::Parity);
        }

        // When we have no error and pending bytes from the fifo, copy them to the output buffer and return.
        let mut c = 0;
        for b in buf {
            if r.ufrstsn().read().rfifo_nempty_sts().bit_is_clear() {
                break;
            }
            *b = r.urbufn().read().bits();
            c += 1;
        }

        Ok(c)
    }
}

impl embedded_io_async::ReadReady for UartRx<'_> {
    fn read_ready(&mut self) -> Result<bool, Self::Error> {
        Ok(self.dev.regs.ufrstsn().read().rfifo_nempty_sts().bit_is_set())
    }
}

impl embedded_io_async::Write for UartTx<'_> {
    async fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        let r = self.dev.regs;

        // Await until space in the FIFO buffer.
        if r.uftstsn().read().tempty_level().bits() == 0 {
            // Note(cs): register is also modified in interrupt handler.
            critical_section::with(|_| {
                // Note: in configure we set watermark level to at least 1 byte.
                r.uftctln().modify(|_, w| w.tempty_level_en().set_bit());
            });

            poll_fn(|cx| {
                self.dev.tx_waker.register(cx.waker());
                if r.uftstsn().read().tempty_level().bits() != 0 {
                    Poll::Ready(())
                } else {
                    Poll::Pending
                }
            })
            .await;
        }

        // Write until FIFO is full.
        let mut c = 0;
        for b in buf {
            if r.uftstsn().read().tempty_level().bits() == 0 {
                break;
            }
            r.utbufn().write(|w| unsafe { w.bits(*b) });
            c += 1;
        }

        Ok(c)
    }

    async fn flush(&mut self) -> Result<(), Self::Error> {
        let r = self.dev.regs;

        // Await until the driver has transmitted all bits.
        if r.uftstsn().read().nxmip().bit_is_clear() {
            // Note(cs): register is also modified in interrupt handler.
            critical_section::with(|_| {
                r.uftctln().modify(|_, w| w.nxmip_en().set_bit());
            });

            poll_fn(|cx| {
                self.dev.tx_waker.register(cx.waker());
                if r.uftstsn().read().nxmip().bit_is_set() {
                    Poll::Ready(())
                } else {
                    Poll::Pending
                }
            })
            .await;
        }

        Ok(())
    }
}

impl embedded_io_async::WriteReady for UartTx<'_> {
    fn write_ready(&mut self) -> Result<bool, Self::Error> {
        Ok(self.dev.regs.uftstsn().read().tempty_level().bits() != 0)
    }
}

impl<T: Instance> embedded_io_async::ErrorType for Uart<'_, T> {
    type Error = Error;
}

impl<T: Instance> embedded_io_async::Read for Uart<'_, T> {
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        self.rx.read(buf).await
    }
}

impl<T: Instance> embedded_io_async::ReadReady for Uart<'_, T> {
    fn read_ready(&mut self) -> Result<bool, Self::Error> {
        self.rx.read_ready()
    }
}

impl<T: Instance> embedded_io_async::Write for Uart<'_, T> {
    async fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        // Panic safety: TX operations are infallible so it is safe to unwrap
        Ok(self.tx.write(buf).await.unwrap())
    }

    async fn flush(&mut self) -> Result<(), Self::Error> {
        // Panic safety: TX operations are infallible so it is safe to unwrap
        self.tx.flush().await.unwrap();
        Ok(())
    }
}

impl<T: Instance> embedded_io_async::WriteReady for Uart<'_, T> {
    fn write_ready(&mut self) -> Result<bool, Self::Error> {
        // Panic safety: TX operations are infallible so it is safe to unwrap
        Ok(self.tx.write_ready().unwrap())
    }
}

struct State {
    rx_tx_refcount: AtomicU8,
}

impl State {
    const fn new() -> Self {
        Self {
            rx_tx_refcount: AtomicU8::new(0),
        }
    }
}

mod sealed {
    use embassy_hal_internal::PeripheralType;
    use embassy_sync::waitqueue::AtomicWaker;

    pub trait SealedInstance {
        fn rx_waker() -> &'static AtomicWaker;
        fn tx_waker() -> &'static AtomicWaker;
        fn regs() -> &'static crate::pac::cr_uart1::RegisterBlock;
        fn state() -> &'static crate::uart::State;

        unsafe fn reset(cs: critical_section::CriticalSection);
    }

    pub trait SealedPin: PeripheralType {
        unsafe fn setup(cs: critical_section::CriticalSection);
    }

    pub trait SealedInputPin: SealedPin {}
    pub trait SealedOutputPin: SealedPin {}
    pub trait SealedCommonPin: SealedPin {
        fn configure_for_common<T: SealedInstance>(push_pull: bool);
    }
}

/// A marker trait implemented by all uart peripherals
pub trait Instance: sealed::SealedInstance + embassy_hal_internal::PeripheralType {
    /// The interrupt used by this instance
    type Interrupt: crate::interrupt::typelevel::Interrupt;
}

/// A marker trait implemented by all pins that can function as an input pin for the uart
pub trait InputPin: sealed::SealedInputPin {
    /// The uart this pin can be used for
    type Instance: Instance;
}
/// A marker trait implemented by all pins that can function as an output pin for the uart
pub trait OutputPin: sealed::SealedOutputPin {
    /// The uart this pin can be used for
    type Instance: Instance;
}

/// A marker trait implemented by all pins that can function as a common pin for the uart
pub trait CommonPin: sealed::SealedCommonPin {
    /// The uart this pin can be used for
    type Instance: Instance;
}

macro_rules! impl_instance {
    ($instance:ident, $pac:ident, $interrupt:ident, $reset_config:expr) => {
        impl sealed::SealedInstance for crate::peripherals::$instance {
            fn rx_waker() -> &'static AtomicWaker {
                static WAKER: AtomicWaker = AtomicWaker::new();
                &WAKER
            }

            fn tx_waker() -> &'static AtomicWaker {
                static WAKER: AtomicWaker = AtomicWaker::new();
                &WAKER
            }

            fn state() -> &'static State {
                static STATE: State = State::new();
                &STATE
            }

            fn regs() -> &'static crate::pac::cr_uart1::RegisterBlock {
                // Safety: not owned, memory is always present
                unsafe { &*crate::pac::$pac::PTR }
            }

            unsafe fn reset(_cs: critical_section::CriticalSection) {
                fn internal_set(f: impl FnOnce(crate::pac::Sysconfig, crate::pac::Sysglue)) {
                    f(unsafe { crate::pac::Sysconfig::steal() }, unsafe {
                        crate::pac::Sysglue::steal()
                    });
                }
                internal_set($reset_config);
            }
        }

        impl Instance for crate::peripherals::$instance {
            type Interrupt = crate::interrupt::typelevel::$interrupt;
        }
    };
}

macro_rules! impl_pin {
    ($instance:ident, $pin:ident, $pin_config:expr) => {
        impl sealed::SealedPin for crate::peripherals::$pin {
            unsafe fn setup(_cs: critical_section::CriticalSection) {
                fn internal_set(f: impl FnOnce(crate::pac::Sysconfig, crate::pac::Sysglue)) {
                    f(unsafe { crate::pac::Sysconfig::steal() }, unsafe {
                        crate::pac::Sysglue::steal()
                    });
                }
                internal_set($pin_config);
            }
        }
    };
}

macro_rules! impl_pin_input {
    ($instance:ident, $pin:ident, $pin_config:expr) => {
        impl_pin!($instance, $pin, $pin_config);
        impl sealed::SealedInputPin for crate::peripherals::$pin {}
        impl sealed::SealedCommonPin for crate::peripherals::$pin {
            fn configure_for_common<T: sealed::SealedInstance>(push_pull: bool) {
                T::regs()
                    .ucntln()
                    .modify(|_, w| w.cr_sin_com().set_bit().cr_sin_pp().bit(push_pull));
            }
        }
        impl InputPin for crate::peripherals::$pin {
            type Instance = crate::peripherals::$instance;
        }
        impl CommonPin for crate::peripherals::$pin {
            type Instance = crate::peripherals::$instance;
        }
    };
}

macro_rules! impl_pin_output {
    ($instance:ident, $pin:ident, $pin_config:expr) => {
        impl_pin!($instance, $pin, $pin_config);
        impl sealed::SealedOutputPin for crate::peripherals::$pin {}
        impl sealed::SealedCommonPin for crate::peripherals::$pin {
            fn configure_for_common<T: sealed::SealedInstance>(push_pull: bool) {
                T::regs()
                    .ucntln()
                    .modify(|_, w| w.cr_sout_com().set_bit().cr_sout_pp().bit(push_pull));
            }
        }
        impl OutputPin for crate::peripherals::$pin {
            type Instance = crate::peripherals::$instance;
        }
        impl CommonPin for crate::peripherals::$pin {
            type Instance = crate::peripherals::$instance;
        }
    };
}

fn trigger_reset(sysconfig: &crate::pac::Sysconfig) {
    sysconfig.swrst_trg().write(|w| unsafe { w.bits(0xC183) });
}

impl_instance!(CR_UART1, CrUart1, CR_UART1_MDMA1, |sysconfig, _| {
    sysconfig.swrst_ctl2().write(|w| w.crurt1_rst().set_bit());
    trigger_reset(&sysconfig);
    while sysconfig.swrst_ctl2().read().crurt1_rst().bit_is_set() {}
});
impl_instance!(CR_UART2, CrUart2, CR_UART2_MDMA2, |sysconfig, _| {
    sysconfig.swrst_ctl2().write(|w| w.crurt2_rst().set_bit());
    trigger_reset(&sysconfig);
    while sysconfig.swrst_ctl2().read().crurt2_rst().bit_is_set() {}
});
impl_instance!(CR_UART3, CrUart3, CR_UART3_MDMA3, |sysconfig, _| {
    sysconfig.swrst_ctl3().write(|w| w.crurt3_rst().set_bit());
    trigger_reset(&sysconfig);
    while sysconfig.swrst_ctl3().read().crurt3_rst().bit_is_set() {}
});
impl_instance!(CR_UART4, CrUart4, CR_UART4_MDMA4, |sysconfig, _| {
    sysconfig.swrst_ctl3().write(|w| w.crurt4_rst().set_bit());
    trigger_reset(&sysconfig);
    while sysconfig.swrst_ctl3().read().crurt4_rst().bit_is_set() {}
});

impl_pin_input!(CR_UART1, PC10, |config, _| {
    config.devaltj().modify(|_, w| w.cr_sin1_sl1().set_bit());
});
impl_pin_input!(CR_UART1, PG04, |config, _| {
    config.devaltj().modify(|_, w| w.cr_sin1_sl2().set_bit());
});
impl_pin_output!(CR_UART1, PC09, |config, _| {
    config.devaltj().modify(|_, w| w.cr_sout1_sl1().set_bit());
});
impl_pin_output!(CR_UART1, PH04, |config, _| {
    config.devaltj().modify(|_, w| w.cr_sout1_sl2().set_bit());
});

impl_pin_input!(CR_UART2, PJ06, |config, _| {
    config.devaltj().modify(|_, w| w.cr_sin2_sl().set_bit());
});
impl_pin_output!(CR_UART2, PJ09, |config, _| {
    config.devaltj().modify(|_, w| w.cr_sout2_sl().set_bit());
});

impl_pin_input!(CR_UART3, PA09, |config, _| {
    config.devaltj().modify(|_, w| w.cr_sin3_sl().set_bit());
});
impl_pin_output!(CR_UART3, PH03, |config, _| {
    config.devaltj().modify(|_, w| w.cr_sout3_sl().set_bit());
});

impl_pin_input!(CR_UART4, PD08, |config, _| {
    config.devalte().modify(|_, w| w.cr_sin4_sl().set_bit());
});
impl_pin_output!(CR_UART4, PK02, |config, _| {
    config.devalte().modify(|_, w| w.cr_sout4_sl().set_bit());
});
