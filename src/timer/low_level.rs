//! Partial low-level implementation for the MFT16 timer.

use crate::{interrupt::typelevel::Interrupt, timer::MultiFunctionInstance};
use core::{future::poll_fn, marker::PhantomData, task::Poll};
use embassy_hal_internal::{into_ref, Peripheral, PeripheralRef};

#[derive(Default, Debug, Copy, Clone, Eq, PartialEq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
/// Clock source for MFT16 timers.
pub enum ClockSource {
    /// Do not run this counter.
    #[default]
    NoClock = 0b000,
    /// Prescaled APB1 clock. (The counter is frozen in sleep mode)
    PrescaledAPB1Clock = 0b001,
    /// Trigger on external event on TBn.
    ExternalEvent = 0b010,
    /// Pulse Accumulate mode where the clock signal is gated on external input TBn.
    PulseAccumulate = 0b011,
    /// 32KHz clock.
    SlowSpeedClock = 0b100,
}

#[derive(Default, Debug, Copy, Clone, Eq, PartialEq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
/// Clock source when using Pulse Accumulate mode for MFT16 timers.
pub enum PulseAccumulateClockSelect {
    /// Prescaled APB1 clock. (The counter is frozen in sleep mode)
    #[default]
    PrescaledAPB1Clock = 0b0,
    /// LFCLK at 32KHz.
    LowFrequencyClock = 0b1,
}

#[derive(Default, Debug, Copy, Clone, Eq, PartialEq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
/// Configuration for a MFT16 clock.
pub struct ClockConfig {
    /// Clock source for counter 1.
    pub counter1_src: ClockSource,
    /// Clock source for counter 2.
    pub counter2_src: ClockSource,
    /// Prescaler when using PrescaledAPB1Clock.
    pub clkps: u8,
    /// Clock source when using PulseAccumulate.
    pub pls_acc_clk: PulseAccumulateClockSelect,
    /// Enable timers in low power mode.
    pub low_pwr: bool,
}

#[derive(Default, Debug, Copy, Clone, Eq, PartialEq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
/// Mode for a MFT16 peripheral.
pub enum Mode {
    /// PWM and Counter mode.
    Mode1 = 0b000,
    /// Dual-Input Capture mode.
    Mode2 = 0b001,
    /// Dual-Independent Timer mode.
    #[default]
    Mode3 = 0b010,
    /// Input Capture and Timer mode.
    Mode4 = 0b011,
    /// Dual-Independent Input Capture mode.
    Mode5 = 0b100,
    /// Duty Cycle Capture mode.
    Mode6 = 0b101,
}

#[derive(Default, Debug, Copy, Clone, Eq, PartialEq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
/// Config for a MFT16 peripheral.
pub struct Config {
    /// Clock configuration.
    pub clk: ClockConfig,
    /// Mode that determines the way the two counters are used and which events are generated.
    pub mode: Mode,
}

#[allow(missing_docs)]
#[derive(Debug, Copy, Clone, Eq, PartialEq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
/// One of two counters contained in the MFT16 peripheral.
pub enum Counter {
    Counter1,
    Counter2,
}

#[allow(missing_docs)]
#[derive(Debug, Copy, Clone, Eq, PartialEq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
/// One of the various WakeUpEvents, for which the semantics depend on the selected Mode.
pub enum WakeUpEvent {
    A = 0,
    B,
    C,
    D,
    E,
    F,
}

impl WakeUpEvent {
    const fn mask(&self) -> u8 {
        1 << (*self as u8)
    }
}

#[derive(Default, Debug, Copy, Clone, Eq, PartialEq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
/// Struct to encapsulate a selection of the various WakeUpEvents.
pub struct WakeUpEvents(u8);

impl WakeUpEvents {
    /// Get whether `event` is set in the current selection.
    pub fn get(&self, event: WakeUpEvent) -> bool {
        self.0 & event.mask() != 0x0
    }

    /// Set whether `event` is set in the current selection, depending on the `value`.
    pub fn set(&mut self, event: WakeUpEvent, value: bool) {
        if value {
            self.0 |= event.mask();
        } else {
            self.0 &= !event.mask();
        }
    }
}

impl From<WakeUpEvent> for WakeUpEvents {
    fn from(value: WakeUpEvent) -> Self {
        let mut res = WakeUpEvents::default();
        res.set(value, true);
        res
    }
}

/// Partial timer driver for one of three 16-bit MultiFunctionTimer(MFT16).
///
/// In practice this driver is mainly usable in Dual-Independent Timer mode.
/// Control of clock inputs and the various other modes has not yet been implemented.
pub struct MultiFunctionTimer<'d, T: MultiFunctionInstance> {
    _instance: PeripheralRef<'d, T>,
}

impl<'d, T: MultiFunctionInstance> MultiFunctionTimer<'d, T> {
    /// Instantiate the MFT16 driver for this peripheral.
    pub fn new(
        instance: impl Peripheral<P = T> + 'd,
        _irqs: impl crate::interrupt::typelevel::Binding<T::Interrupt, InterruptHandler<T>>,
    ) -> Self {
        // Safety: _irqs ensures an interrupt handler is bound
        unsafe {
            T::Interrupt::enable();
        }

        into_ref!(instance);
        Self { _instance: instance }
    }

    /// Enable the MFT16 driver for this peripheral with a specific configuration, and starts running the timer.
    pub fn enable(&mut self, cfg: Config) {
        let r = T::regs();

        // Disable the clocksources before configuring.
        self.disable();

        r.tn_mctrl().write(|w| unsafe { w.mdsel().bits(cfg.mode as u8) });

        self.reset(Counter::Counter1);
        self.reset(Counter::Counter2);

        // Configure comparison mode
        r.tn_cpcfg().write(|w| w.eqaen().set_bit().eqben().set_bit());

        r.tn_prsc().write(|w| unsafe { w.bits(cfg.clk.clkps) });

        // Starts the clock.
        r.tn_ckc().write(|w| unsafe {
            w.low_pwr()
                .bit(cfg.clk.low_pwr)
                .pls_acc_clk()
                .bit(cfg.clk.pls_acc_clk as u8 == 0b1)
                .c1csel()
                .bits(cfg.clk.counter1_src as u8)
                .c2csel()
                .bits(cfg.clk.counter2_src as u8)
        });
    }

    /// Disable running the timer.
    pub fn disable(&mut self) {
        T::regs()
            .tn_ckc()
            .write(|w| unsafe { w.c1csel().bits(0b000).c2csel().bits(0b000) });
    }

    /// Reset the selected counter for this timer.
    pub fn reset(&mut self, counter: Counter) {
        self.set_counter(counter, 0x0000);
    }

    /// Get the value for the selected counter.
    pub fn counter(&self, counter: Counter) -> u16 {
        match counter {
            Counter::Counter1 => T::regs().tn_cnt1().read().bits(),
            Counter::Counter2 => T::regs().tn_cnt2().read().bits(),
        }
    }

    /// Set the value for the selected counter.
    pub fn set_counter(&mut self, counter: Counter, value: u16) {
        match counter {
            Counter::Counter1 => T::regs().tn_cnt1().write(|w| unsafe { w.bits(value) }),
            Counter::Counter2 => T::regs().tn_cnt2().write(|w| unsafe { w.bits(value) }),
        };
    }

    /// Set the comparator value for the selected counter.
    pub fn set_compare(&mut self, counter: Counter, value: u16) {
        match counter {
            Counter::Counter1 => T::regs().tn_cpa().write(|w| unsafe { w.bits(value) }),
            Counter::Counter2 => T::regs().tn_cpb().write(|w| unsafe { w.bits(value) }),
        };
    }

    /// Set the value to which the value will be reloaded when underflowing for the selected counter.
    pub fn set_reload_capture(&mut self, counter: Counter, value: u16) {
        match counter {
            Counter::Counter1 => T::regs().tn_cra().write(|w| unsafe { w.bits(value) }),
            Counter::Counter2 => T::regs().tn_crb().write(|w| unsafe { w.bits(value) }),
        };
    }

    /// Await for at least a single wake up event to be pending.
    ///
    /// Will reset the pending event, deconfigure the interrupt, and will return all events that ended up triggering.
    pub async fn wait_for_single(&mut self, events: impl Into<WakeUpEvents>) -> WakeUpEvents {
        let events = events.into();

        if events == WakeUpEvents::default() {
            return events; // Do not need to wait for any event.
        }

        // Reset enabled interrupt wake events to our set.
        // Note(cs): interrupt handler changes this register as well.
        critical_section::with(|_| {
            T::regs()
                .tn_ien()
                .modify(|r, w| unsafe { w.bits((r.bits() & 0b1100_0000) | events.0) });
        });

        // Await until at least one of our events is pending.
        let pending = poll_fn(|cx| {
            T::waker().register(cx.waker());

            let pending = T::regs().tn_ectrl().read().bits();
            if pending & events.0 != 0x00 {
                // Note: interrupt was de-configured in interrupt handler.
                Poll::Ready(pending)
            } else {
                Poll::Pending
            }
        })
        .await;

        // Clear pending bits.
        T::regs().tn_eclr().write(|w| unsafe { w.bits(pending) });

        WakeUpEvents(pending)
    }
}

impl<T: MultiFunctionInstance> Drop for MultiFunctionTimer<'_, T> {
    fn drop(&mut self) {
        self.disable();
    }
}

/// The interrupt handler for the MFT16 driver.
pub struct InterruptHandler<T> {
    _phantom: PhantomData<T>,
}

impl<T: MultiFunctionInstance> crate::interrupt::typelevel::Handler<T::Interrupt> for InterruptHandler<T> {
    unsafe fn on_interrupt() {
        T::waker().wake();
        // Deconfigure all wake-up events, but do not clear them.
        T::regs()
            .tn_ien()
            .modify(|r, w| unsafe { w.bits(r.bits() & 0b1100_0000) });
    }
}
