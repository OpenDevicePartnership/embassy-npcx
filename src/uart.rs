//! Core Universal Asynchronous Receiver-Transmitter (CR_UART).
//!
//! Implements the full-duplex receiver transmitter integration with 16-byte FIFO buffers for receive and transmit.
//! Does not (yet) support DMA transactions.

use crate::interrupt::typelevel::Interrupt;
use core::future::Future;
use core::marker::PhantomData;
use embassy_hal_internal::{into_ref, Peripheral, PeripheralRef};
use embassy_sync::waitqueue::AtomicWaker;

#[derive(Debug, Copy, Clone, Eq, PartialEq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum StopBits {
    STOP1,
    STOP2,
}

#[derive(Debug, Copy, Clone, Eq, PartialEq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Parity {
    ParityOdd = 0b00,
    ParityEven = 0b01,
    ParityMark = 0b10,
    ParitySpace = 0b11,
}

/// Base configuration for the Core Uart peripheral.
///
/// Applicable for both the "Separate Mode" and the "Common Mode".
#[derive(Debug, Copy, Clone, Eq, PartialEq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub struct Config {
    pub baudrate: u32,
    pub stop_bits: StopBits,
    pub parity: Option<Parity>,
    pub input_inverted: bool,
    pub output_inverted: bool,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            baudrate: 9_600,
            stop_bits: StopBits::STOP1,
            parity: None,
            input_inverted: false,
            output_inverted: false,
        }
    }
}

/// "Common Mode" configuration for the Core Uart peripheral.
#[non_exhaustive]
pub struct CommonModeConfig {
    pub base: Config,
    pub push_pull: bool,
    pub feedback: bool,
}

impl Default for CommonModeConfig {
    fn default() -> Self {
        Self {
            base: Config::default(),
            push_pull: false,
            feedback: false,
        }
    }
}

struct AnyUart {
    regs: &'static crate::pac::cr_uart1::RegisterBlock,
    waker: &'static AtomicWaker,
}

impl<T: Instance> From<T> for AnyUart {
    fn from(_uart: T) -> Self {
        AnyUart {
            regs: T::regs(),
            waker: T::waker(),
        }
    }
}

// Allow use of PeripheralRef to do lifetime management
impl Peripheral for AnyUart {
    type P = AnyUart;

    unsafe fn clone_unchecked(&self) -> Self::P {
        AnyUart {
            regs: self.regs,
            waker: self.waker,
        }
    }
}

/// Core Universal Asynchronous Receiver-Transmitter (CR_UART) driver.
pub struct Uart<'a> {
    dev: PeripheralRef<'a, AnyUart>,
}

/// Potential UART clock configuration.
///
/// The baudrate is computed as follows:
/// ```"not rust"
/// BR = SRC / (16 x div x p)
/// (div x p) = SRC / (16 * BR)
/// ```
struct ClockConfiguration {
    // Divisor value
    div: u16,
    // Prescaler value times two
    p2: u8,
}

impl ClockConfiguration {
    pub fn generate_valid(srcclk: u32, desired_baudrate: u32) -> impl Iterator<Item = Self> {
        let dstclk2 = (srcclk * 2) / 16 / desired_baudrate;

        (2..=32).filter_map(move |p2| {
            let div = dstclk2 / p2 as u32;
            if div > 0 && div <= 0x800 {
                Some(ClockConfiguration { div: div as u16, p2 })
            } else {
                None
            }
        })
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

impl<'a> Uart<'a> {
    fn regs(&self) -> &'static crate::pac::cr_uart1::RegisterBlock {
        self.dev.regs
    }

    /// Configure the base registers for the peripheral and enables it.
    fn configure_enable(&mut self, config: Config) {
        let r = self.regs();

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

        // Safety: UART can only be initialized after the clocks have been initialized.
        let srcclk = unsafe { crate::cdcg::get_clocks() }.apb4_clk;

        let clkcfg = ClockConfiguration::generate_valid(srcclk, config.baudrate)
            // Minimize baudrate error.
            .min_by_key(move |cfg| cfg.baudrate(srcclk).abs_diff(config.baudrate))
            .expect("Failed to find clock configuration for requested baudrate");

        self.regs()
            .ubaudn()
            .write(|w| unsafe { w.bits((clkcfg.udiv10() & 0xff) as u8) });
        // Setting the prescaler to non-zero also enables the peripheral.
        self.regs().upsrn().write(|w| unsafe {
            w.upsc()
                .bits(clkcfg.upsc())
                .udiv10_8()
                .bits(((clkcfg.udiv10() & 0x700) >> 8) as u8)
        });
    }

    /// Configure the base registers and general common mode registers for the peripheral, and enables it.
    fn configure_common_mode_enable(&mut self, config: CommonModeConfig) {
        self.regs().ucntln().modify(|_, w| w.com_fdbk_en().bit(config.feedback));
        self.configure_enable(config.base);
    }

    /// Enables the peripheral in "Separate Mode" with applicable input and output pins.
    pub fn new<T: Instance + 'a, Sin: InputPin, Sout: OutputPin>(
        peri: impl Peripheral<P = T> + 'a,
        _sin: impl Peripheral<P = Sin> + 'a,
        _sout: impl Peripheral<P = Sout> + 'a,
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

        // Safety: _irqs ensures an interrupt handler is bound
        unsafe {
            T::Interrupt::enable();
        }

        into_ref!(peri);

        let mut dev = Self { dev: peri.map_into() };
        dev.configure_enable(config);
        dev
    }

    /// Enables the peripheral in "Common Mode" by using an applicable input pin as both input and output.
    pub fn new_common_input<T: Instance + 'a, Sin: InputPin>(
        peri: impl Peripheral<P = T> + 'a,
        _sin: impl Peripheral<P = Sin> + 'a,
        _irqs: impl crate::interrupt::typelevel::Binding<T::Interrupt, InterruptHandler<T>>,
        config: CommonModeConfig,
    ) -> Self {
        // Note(cs): other peripherals might also be modifying swsrst* at the same time.
        critical_section::with(|cs| {
            // Safety: We have exclusive ownership over the peripherals.
            unsafe {
                T::reset(cs);
                Sin::setup(cs)
            };
        });

        into_ref!(peri);

        let mut dev = Self { dev: peri.map_into() };
        dev.regs()
            .ucntln()
            .modify(|_, w| w.cr_sin_com().set_bit().cr_sin_pp().bit(config.push_pull));
        dev.configure_common_mode_enable(config);
        dev
    }

    /// Enables the peripheral in "Common Mode" by using an applicable output pin as both input and output.
    pub fn new_common_output<T: Instance + 'a, Sout: OutputPin>(
        peri: impl Peripheral<P = T> + 'a,
        _sout: impl Peripheral<P = Sout> + 'a,
        _irqs: impl crate::interrupt::typelevel::Binding<T::Interrupt, InterruptHandler<T>>,
        config: CommonModeConfig,
    ) -> Self {
        // Note(cs): other peripherals might also be modifying swsrst* at the same time.
        critical_section::with(|cs| {
            // Safety: We have exclusive ownership over the peripherals.
            unsafe {
                T::reset(cs);
                Sout::setup(cs)
            };
        });

        into_ref!(peri);

        let mut dev = Self { dev: peri.map_into() };
        dev.regs()
            .ucntln()
            .modify(|_, w| w.cr_sout_com().set_bit().cr_sout_pp().bit(config.push_pull));
        dev.configure_common_mode_enable(config);
        dev
    }
}

impl<'a> Drop for Uart<'a> {
    fn drop(&mut self) {
        // Setting the prescaler to 0 disables the clock and disables the peripheral.
        self.regs().upsrn().write(|w| unsafe { w.upsc().bits(0b0_0000) });
    }
}

pub struct InterruptHandler<T> {
    _phantom: PhantomData<T>,
}

impl<T: Instance> crate::interrupt::typelevel::Handler<T::Interrupt> for InterruptHandler<T> {
    unsafe fn on_interrupt() {
        T::waker().wake();
        critical_section::with(|_| {
            // T::regs().ucntln().modify(|_, w| w.inten().clear_bit());
        });
    }
}

mod sealed {
    use embassy_sync::waitqueue::AtomicWaker;

    pub trait SealedInstance {
        fn waker() -> &'static AtomicWaker;
        fn regs() -> &'static crate::pac::cr_uart1::RegisterBlock;

        unsafe fn reset(cs: critical_section::CriticalSection);
    }

    pub trait SealedConfigurablePin {
        unsafe fn setup(cs: critical_section::CriticalSection);
    }

    pub trait SealedInputPin: SealedConfigurablePin {}
    pub trait SealedOutputPin: SealedConfigurablePin {}
}

pub trait Instance: sealed::SealedInstance + embassy_hal_internal::Peripheral<P = Self> {
    type Interrupt: crate::interrupt::typelevel::Interrupt;
}

pub trait InputPin: sealed::SealedInputPin {
    type Instance: Instance;
}
pub trait OutputPin: sealed::SealedOutputPin {
    type Instance: Instance;
}

macro_rules! impl_instance {
    ($instance:ident, $pac:ident, $interrupt:ident, $reset_config:expr) => {
        impl sealed::SealedInstance for crate::peripherals::$instance {
            fn waker() -> &'static AtomicWaker {
                static WAKER: AtomicWaker = AtomicWaker::new();
                &WAKER
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
        impl sealed::SealedConfigurablePin for crate::peripherals::$pin {
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
        impl InputPin for crate::peripherals::$pin {
            type Instance = crate::peripherals::$instance;
        }
    };
}

macro_rules! impl_pin_output {
    ($instance:ident, $pin:ident, $pin_config:expr) => {
        impl_pin!($instance, $pin, $pin_config);
        impl sealed::SealedOutputPin for crate::peripherals::$pin {}
        impl OutputPin for crate::peripherals::$pin {
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
