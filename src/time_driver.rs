use crate::{interrupt::typelevel::Interrupt, pac};
use core::{
    cell::{Cell, RefCell},
    sync::atomic::{compiler_fence, AtomicU32, Ordering},
};
use critical_section::CriticalSection;
use embassy_sync::blocking_mutex::{raw::CriticalSectionRawMutex, Mutex};
use embassy_time_driver::{Driver, TICK_HZ};
use embassy_time_queue_utils::Queue;

macro_rules! impl_instance {
    ($instance:ident, $interrupt:ident) => {
        const fn regs() -> &'static crate::pac::mft16_1::RegisterBlock {
            unsafe { &*pac::$instance::PTR }
        }

        unsafe fn enable_interrupt() {
            crate::interrupt::typelevel::$interrupt::enable();
        }

        #[allow(unused)]
        use crate::interrupt;

        #[cfg(feature = "rt")]
        #[pac::interrupt]
        fn $interrupt() {
            DRIVER.on_interrupt()
        }
    };
}

cfg_if::cfg_if! {
    if #[cfg(feature = "time-driver-mft16-1")] {
        impl_instance!(Mft16_1, MFT16_1);
    } else if #[cfg(feature = "time-driver-mft16-2")] {
        impl_instance!(Mft16_2, MFT16_2);
    } else if #[cfg(feature = "time-driver-mft16-3")] {
        impl_instance!(Mft16_3, MFT16_3);
    }
}

// Clock timekeeping works with something we call "periods", which are time intervals
// of 2^15 ticks. The Clock counter value is 16 bits, so one "overflow cycle" is 2 periods.
//
// A `period` count is maintained in parallel to the Timer hardware `counter`, like this:
// - `period` and `counter` start at 0
// - `period` is incremented on overflow (at counter value 0)
// - `period` is incremented "midway" between overflows (at counter value 0x8000)
//
// Therefore, when `period` is even, counter is in 0..0x7FFF. When odd, counter is in 0x8000..0xFFFF
// This allows for now() to return the correct value even if it races an overflow.
//
// To get `now()`, `period` is read first, then `counter` is read. If the counter value matches
// the expected range for the `period` parity, we're done. If it doesn't, this means that
// a new period start has raced us between reading `period` and `counter`, so we assume the `counter` value
// corresponds to the next period.
//
// `period` is a 32bit integer, so It overflows on 2^32 * 2^15 / 32768 seconds of uptime, which is 136 years.
fn calc_now(period: u32, counter: u16) -> u64 {
    // We have a down-counting counter, thus we need to invert.
    let counter = 0xffff - counter;
    ((period as u64) << 15) + ((counter as u32 ^ ((period & 1) << 15)) as u64)
}

struct AlarmState {
    timestamp: Cell<u64>,
}

unsafe impl Send for AlarmState {}

impl AlarmState {
    const fn new() -> Self {
        Self {
            timestamp: Cell::new(u64::MAX),
        }
    }
}

pub(crate) struct MultiFunctionTimerDriver {
    /// Number of 2^15 periods elapsed since boot.
    period: AtomicU32,
    alarm: Mutex<CriticalSectionRawMutex, AlarmState>,
    queue: Mutex<CriticalSectionRawMutex, RefCell<Queue>>,
}

embassy_time_driver::time_driver_impl!(static DRIVER: MultiFunctionTimerDriver = MultiFunctionTimerDriver {
    period: AtomicU32::new(0),
    alarm: Mutex::const_new(CriticalSectionRawMutex::new(), AlarmState::new()),
    queue: Mutex::new(RefCell::new(Queue::new()))
});

impl MultiFunctionTimerDriver {
    fn init(&'static self, _cs: critical_section::CriticalSection) {
        let r = regs();

        unsafe { enable_interrupt() };

        // Disable the clocks.
        r.tn_ckc()
            .write(|w| unsafe { w.c1csel().bits(0b000).c2csel().bits(0b000) });

        // Configure Mode 3: Dual-Independent Timers.
        r.tn_mctrl().write(|w| unsafe { w.mdsel().bits(0b010) });

        // Reset the counters to the same value.
        r.tn_cnt1().write(|w| unsafe { w.bits(0xffff) });
        r.tn_cnt2().write(|w| unsafe { w.bits(0xffff) });

        // Set both counters to be overflowed (reload-captured) to the same value.
        r.tn_cra().write(|w| unsafe { w.bits(0xffff) });
        r.tn_crb().write(|w| unsafe { w.bits(0xffff) });

        // Set Counter 1 to compare-match at the half-way point.
        // Note: we'll use Counter 2 as alarm and thus will not need to configure it now.
        r.tn_cpa().write(|w| unsafe { w.bits(0x7fff) });

        // Set Counter comparison to equality.
        r.tn_cpcfg().write(|w| w.eqaen().set_bit().eqben().set_bit());

        // Enable overflow and half overflow detection interrupts.
        r.tn_ien().write(|w| w.taien().set_bit().teien().set_bit());

        // Set low power clock mode before starting the clocks.
        r.tn_ckc().write(|w| w.low_pwr().bit(true));

        if TICK_HZ == 32768 {
            // Starts the clocks sources from LFCLK.
            r.tn_ckc()
                .modify(|_, w| unsafe { w.c1csel().bits(0b100).c2csel().bits(0b100) });
        } else {
            // Use prescaled APB1 clock to drive counters.

            // Note(unsafe): time driver is initialized after the clocks have been initialized.
            let clocks = unsafe { crate::cdcg::get_clocks() };

            if clocks.apb1_clk % TICK_HZ as u32 != 0 {
                panic!(
                    "APB1CLK ({}) is not a multiple of desired time_driver tickrate ({})",
                    clocks.apb1_clk, TICK_HZ
                );
            }

            let prescaler = clocks.apb1_clk / TICK_HZ as u32;
            if prescaler > 256 {
                panic!(
                    "Cannot derive a prescaling for APB1CLK ({}) and desired time_driver tickrate ({})",
                    clocks.apb1_clk, TICK_HZ
                );
            }
            let clkps = (prescaler - 1) as u8;
            r.tn_prsc().write(|w| unsafe { w.bits(clkps) });

            // Starts the clocks sources from prescaled APB1.
            r.tn_ckc()
                .modify(|_, w| unsafe { w.c1csel().bits(0b001).c2csel().bits(0b001) });
        }
    }

    #[allow(unused)]
    fn on_interrupt(&self) {
        let r = regs();
        let pending = r.tn_ectrl().read();

        critical_section::with(|cs| {
            // Clear pending interrupts.
            r.tn_eclr().write(|w| unsafe { w.bits(pending.bits()) });

            // Counter 1 - reloaded (overflow)
            if pending.tapnd().bit_is_set() {
                self.next_period(cs);
            }

            // Counter 1 - compare matched (half overflow)
            if pending.tepnd().bit_is_set() {
                self.next_period(cs);
            }

            // Counter 2 - compare matched (alarm)
            if pending.tfpnd().bit_is_set() {
                self.trigger_alarm(cs);
            }
        });
    }

    fn next_period(&self, cs: CriticalSection) {
        // We only modify the period from the timer interrupt, so we know this can't race.
        let period = self.period.fetch_add(1, Ordering::Relaxed) + 1;
        let t = (period as u64) << 15;

        let alarm = self.alarm.borrow(cs);
        let at = alarm.timestamp.get();

        if at < t + 0xc000 {
            // Enable alarm interrupt, the compare-match value has already been set earlier.
            regs().tn_ien().modify(|_, w| w.tfien().set_bit());
        }
    }

    fn trigger_alarm(&self, cs: CriticalSection) {
        let mut next = self.queue.borrow(cs).borrow_mut().next_expiration(self.now());
        while !self.set_alarm(cs, next) {
            next = self.queue.borrow(cs).borrow_mut().next_expiration(self.now());
        }
    }

    fn set_alarm(&self, cs: CriticalSection, timestamp: u64) -> bool {
        let r = regs();
        self.alarm.borrow(cs).timestamp.set(timestamp);
        let t = self.now();

        if timestamp <= t {
            // If alarm timestamp has passed the alarm will not fire.
            // Disarm the alarm and return `false` to indicate that.
            regs().tn_ien().modify(|_, w| w.tfien().clear_bit());

            self.alarm.borrow(cs).timestamp.set(u64::MAX);

            return false;
        }

        // Write the alarm compare value regardless of whether we're going to enable it now or not.
        // This way, when we enable it later, the right value is already set.
        // Cast it to u16 grabbing the lsb's, and subtract because our counter counts down.
        let counter = 0xffff - (timestamp as u16);
        r.tn_cpb().write(|w| unsafe { w.bits(counter) });

        // Enable it if it'll happen soon. Otherwise, `next_period` will enable it.
        let diff = timestamp - t;
        regs().tn_ien().modify(|_, w| w.tfien().bit(diff < 0xc000));

        // Reevaluate if the alarm timestamp is still in the future
        let t = self.now();
        if timestamp <= t {
            // If alarm timestamp has passed since we set it, we have a race condition and
            // the alarm may or may not have fired.
            // Disarm the alarm and return `false` to indicate that.
            // It is the caller's responsibility to handle this ambiguity.
            regs().tn_ien().modify(|_, w| w.tfien().clear_bit());

            self.alarm.borrow(cs).timestamp.set(u64::MAX);

            return false;
        }

        // We're confident the alarm will ring in the future.
        true
    }
}

impl Driver for MultiFunctionTimerDriver {
    fn now(&self) -> u64 {
        let period = self.period.load(Ordering::Relaxed);
        compiler_fence(Ordering::Acquire); // Ensure that period is loaded before counter.
        let counter = regs().tn_cnt1().read().bits();
        calc_now(period, counter)
    }

    fn schedule_wake(&self, at: u64, waker: &core::task::Waker) {
        critical_section::with(|cs| {
            let mut queue = self.queue.borrow(cs).borrow_mut();

            if queue.schedule_wake(at, waker) {
                let mut next = queue.next_expiration(self.now());
                while !self.set_alarm(cs, next) {
                    next = queue.next_expiration(self.now());
                }
            }
        })
    }
}

pub(crate) fn init(cs: CriticalSection) {
    DRIVER.init(cs)
}
