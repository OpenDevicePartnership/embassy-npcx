use crate::interrupt::typelevel::Interrupt;
use core::future::Future;
use core::marker::PhantomData;
use embassy_hal_internal::{into_ref, Peripheral, PeripheralRef};
use embassy_sync::waitqueue::AtomicWaker;

#[derive(Debug, Copy, Clone, Eq, PartialEq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub struct Config {
    pub frequency: u32,
    pub input_inverted: bool,
    pub output_inverted: bool,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            frequency: 9_600,
            input_inverted: false,
            output_inverted: false,
        }
    }
}

#[non_exhaustive]
pub struct CommonConfig {
    pub base: Config,
    pub input_push_pull: bool,
    pub output_push_pull: bool,
}

impl Default for CommonConfig {
    fn default() -> Self {
        Self {
            base: Config::default(),
            input_push_pull: false,
            output_push_pull: false,
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

pub struct Uart<'a> {
    dev: PeripheralRef<'a, AnyUart>,
}

impl<'a> Uart<'a> {
    fn regs(&self) -> &'static crate::pac::cr_uart1::RegisterBlock {
        self.dev.regs
    }

    pub fn new<T: Instance + 'a, Sin: InputPin, Sout: OutputPin>(
        peri: impl Peripheral<P = T> + 'a,
        sin: impl Peripheral<P = Sin> + 'a,
        sout: impl Peripheral<P = Sout> + 'a,
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

        // We take ownership over the pins, but do nothing with them after this.
        let (_, _) = (sin, sout);

        // Safety: _irqs ensures an interrupt handler is bound
        unsafe {
            T::Interrupt::enable();
        }

        into_ref!(peri);

        let mut dev = Self { dev: peri.map_into() };

        todo!();
    }

    pub fn new_common_input<T: Instance + 'a, Sin: InputPin>(
        peri: impl Peripheral<P = T> + 'a,
        sin: impl Peripheral<P = Sin> + 'a,
        _irqs: impl crate::interrupt::typelevel::Binding<T::Interrupt, InterruptHandler<T>>,
        config: Config,
    ) -> Self {
        todo!()
    }

    pub fn new_common_output<T: Instance + 'a, Sout: OutputPin>(
        peri: impl Peripheral<P = T> + 'a,
        sout: impl Peripheral<P = Sout> + 'a,
        _irqs: impl crate::interrupt::typelevel::Binding<T::Interrupt, InterruptHandler<T>>,
        config: Config,
    ) -> Self {
        todo!()
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
