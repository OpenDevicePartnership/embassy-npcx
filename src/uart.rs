use core::future::Future;
use core::marker::PhantomData;

use embassy_hal_internal::{into_ref, Peripheral, PeripheralRef};
use embassy_sync::waitqueue::AtomicWaker;

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
    ($instance:ident, $pac:ident, $interrupt:ident) => {
        impl sealed::SealedInstance for crate::peripherals::$instance {
            fn waker() -> &'static AtomicWaker {
                static WAKER: AtomicWaker = AtomicWaker::new();
                &WAKER
            }

            fn regs() -> &'static crate::pac::cr_uart1::RegisterBlock {
                // Safety: not owned, memory is always present
                unsafe { &*crate::pac::$pac::PTR }
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

impl_instance!(CR_UART1, CrUart1, CR_UART1_MDMA1);
impl_instance!(CR_UART2, CrUart2, CR_UART2_MDMA2);
impl_instance!(CR_UART3, CrUart3, CR_UART3_MDMA3);
impl_instance!(CR_UART4, CrUart4, CR_UART4_MDMA4);

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

// TODO common mode
