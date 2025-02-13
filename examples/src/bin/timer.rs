#![no_main]
#![no_std]

use embassy_executor::Spawner;
use embassy_npcx::gpio::{Level, Output, OutputOnly};
use embassy_npcx::timer::low_level::{self, ClockSource, MultiFunctionTimer, WakeUpEvent};
use embassy_npcx::{self as hal, bind_interrupts, Config};
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    MFT16_2 => hal::timer::low_level::InterruptHandler<hal::peripherals::MFT16_2>;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let (p, _) = embassy_npcx::init_espi(Config::default());

    let mut timer = MultiFunctionTimer::new(p.MFT16_2, Irqs);

    let mut config = low_level::Config::default();
    config.clk.counter1_src = ClockSource::SlowSpeedClock;

    // 32KHz 16383 count => 0.5Hz timer.
    timer.set_reload_capture(low_level::Counter::Counter1, 0x3fff);
    timer.set_compare(low_level::Counter::Counter1, 0x001);

    timer.enable(config);

    let mut led = Output::<'_, OutputOnly>::new(p.PJ07, Level::High);
    loop {
        timer.wait_for_single(WakeUpEvent::E).await;
        defmt::info!("Ping");
        led.set_low();
        timer.wait_for_single(WakeUpEvent::E).await;
        defmt::info!("Ping");
        led.set_high();
    }
}
