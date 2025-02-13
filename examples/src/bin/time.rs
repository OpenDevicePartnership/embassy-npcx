//! Run with feature `time_lfclk` or `time_apb1`.

#![no_main]
#![no_std]

use embassy_executor::Spawner;
use embassy_npcx::gpio::{Level, Output, OutputOnly};
use embassy_npcx::Config;
use embassy_time::Timer;
use {defmt_rtt as _, panic_probe as _};

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let mut config = Config::default();
    config.cdcg.mult_m = 0xc00; // MCLK = 48KibiHz, such that APB1clk is 6MibiHz.
    let (p, _) = embassy_npcx::init_espi(config);

    let mut led = Output::<'_, OutputOnly>::new(p.PJ07, Level::High);
    loop {
        led.toggle();
        Timer::after_millis(500).await;
        defmt::info!("Ping");
    }
}
