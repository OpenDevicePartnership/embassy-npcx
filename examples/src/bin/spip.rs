#![no_main]
#![no_std]

use cortex_m::asm::delay;
use defmt_rtt as _;
use embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice as MutexSpiDevice;
use embassy_executor::Spawner;
use embassy_npcx::{
    self as hal, bind_interrupts,
    gpio::{Level, OutputOnly, OutputOpenDrain},
    peripherals::SPIP,
    spip::Spip,
    Config,
};
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::mutex::Mutex;
use panic_probe as _;

bind_interrupts!(struct Irqs {
    SPIP => hal::spip::InterruptHandler<SPIP>;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let config = Config::default();
    let (p, _) = embassy_npcx::init_espi(config);

    let config: hal::spip::Config = Default::default();

    let spip = Spip::new_8bit(p.SPIP, p.PK12, p.PM12, p.PL12, p.PL10, Irqs, config);
    let spip = Mutex::<NoopRawMutex, Spip<SPIP, u8>>::new(spip);

    // Note: flash0 is unusable on the devboard.

    let cs1 = OutputOpenDrain::<'_, OutputOnly>::new(p.PK11, Level::High);
    let mut flash1 = MutexSpiDevice::new(&spip, cs1);

    use embedded_hal_async::spi::SpiDevice;
    loop {
        let mut buf = [0; 13];
        buf[0] = 0x4B;
        flash1.transfer_in_place(&mut buf).await.unwrap();
        defmt::info!("flash1 unique ID {:x}", buf[4..]);
        delay(5_000_000);
    }
}
