use core::marker::PhantomData;

use embassy_hal_internal::{into_ref, Peripheral};

use crate::interrupt::typelevel::Interrupt;

pub struct Espi<'p> {
    phantom: PhantomData<&'p ()>,
    regs: &'static crate::pac::espi::RegisterBlock,
}

impl<'p> Espi<'p> {
    pub fn new<T: Instance + 'p>(
        _peri: impl Peripheral<P = T> + 'p,
        _irqs: impl crate::interrupt::typelevel::Binding<T::Interrupt, InterruptHandler<T>>,
        _mode: crate::ESpi,
    ) -> Self {
        into_ref!(_peri);

        // Safety: _irqs ensures an interrupt handler is bound
        unsafe {
            T::Interrupt::enable();
        }

        Self {
            phantom: PhantomData,
            regs: T::regs(),
        }
    }
}

pub struct InterruptHandler<T> {
    _phantom: PhantomData<T>,
}

impl<T: Instance> crate::interrupt::typelevel::Handler<T::Interrupt> for InterruptHandler<T> {
    unsafe fn on_interrupt() {}
}

mod sealed {
    pub trait SealedInstance {
        fn regs() -> &'static crate::pac::espi::RegisterBlock;
    }
}

pub trait Instance: sealed::SealedInstance + embassy_hal_internal::Peripheral<P = Self> {
    type Interrupt: crate::interrupt::typelevel::Interrupt;
}

impl sealed::SealedInstance for crate::peripherals::ESPI {
    fn regs() -> &'static crate::pac::espi::RegisterBlock {
        unsafe { &*crate::pac::Espi::PTR }
    }
}
impl Instance for crate::peripherals::ESPI {
    type Interrupt = crate::interrupt::typelevel::ESPI_SHI;
}
