#![no_main]
#![no_std]

use embassy_executor::Spawner;
use embassy_npcx::{bind_interrupts, i2c, peripherals, Config};
use embedded_hal::i2c::Operation;
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(pub struct Irqs {
    SMB5 => embassy_npcx::i2c::InterruptHandler<peripherals::SMB5>;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let (p, mode) = embassy_npcx::init_lpc(Config::default());

    defmt::info!("Creating i2c");
    let mut i2c = i2c::I2CController::new(
        p.SMB5,
        p.PE08,
        p.PE09,
        Irqs,
        mode,
        i2c::Config {
            pullup: true,
            speed: i2c::Speed::Standard,
            ..i2c::Config::default()
        },
    );

    let mut output = [0u8];

    defmt::info!("Getting WHOAMI");
    i2c.transaction(
        0x6a,
        &mut [
            Operation::Write(&[0x0f]),
            Operation::Write(&[]),
            Operation::Read(&mut []),
            Operation::Read(&mut output),
        ],
    )
    .await
    .unwrap();

    defmt::info!("Read: {:02x}", output[0]);

    cortex_m::asm::bkpt();
}
