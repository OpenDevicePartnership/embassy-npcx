#![no_main]
#![no_std]

use embassy_executor::Spawner;
use embassy_npcx::cdcg::VoscClockMode;
use embassy_npcx::i2c::{self, I2CController, ListenCommand};
use embassy_npcx::{bind_interrupts, peripherals, Config};
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::signal::Signal;
use embassy_sync::watch::Watch;
use embedded_hal::i2c::Operation;
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(pub struct Irqs {
    SMB0 => embassy_npcx::i2c::InterruptHandler<peripherals::SMB0>;
    SMB5 => embassy_npcx::i2c::InterruptHandler<peripherals::SMB5>;
});

static SIG: Signal<ThreadModeRawMutex, ()> = Signal::new();

#[embassy_executor::task]
async fn device(mut i2c: I2CController<'static>) {
    let mut flash_data = [0u8; 256];
    let mut cur_addr = 0u8;
    let mut read_addr = 0u8;
    let mut in_transaction = false;

    let stopwatch: Watch<ThreadModeRawMutex, bool, 1> = Watch::new_with(false);
    let mut stop = stopwatch.receiver().unwrap();

    SIG.signal(());
    i2c.listen(
        &[30],
        move |_, command| match command {
            ListenCommand::PartialWrite(items) => {
                let data = if !in_transaction {
                    if let Some((addr, data)) = items.split_first() {
                        in_transaction = true;
                        cur_addr = *addr;
                        data
                    } else {
                        &[]
                    }
                } else {
                    items
                };

                for b in data.iter().copied() {
                    flash_data[cur_addr as usize] = b;
                    cur_addr = cur_addr.wrapping_add(1);
                }
            }
            ListenCommand::WriteFinished(items) => {
                let data = if !in_transaction {
                    if let Some((addr, data)) = items.split_first() {
                        cur_addr = *addr;
                        data
                    } else {
                        &[]
                    }
                } else {
                    items
                };

                for b in data.iter().copied() {
                    flash_data[cur_addr as usize] = b;
                    cur_addr = cur_addr.wrapping_add(1);
                }

                in_transaction = false;
            }
            ListenCommand::PrepareRead(items) => {
                if !in_transaction {
                    read_addr = cur_addr;
                    in_transaction = true;
                }
                for b in items.iter_mut() {
                    *b = flash_data[read_addr as usize];
                    read_addr = read_addr.wrapping_add(1);
                }
            }
            ListenCommand::ReadFinished(read) => {
                in_transaction = false;
                cur_addr = cur_addr.wrapping_add(read as u8);
            }
            ListenCommand::BusError => {
                defmt::info!("A bus error occured");
            }
        },
        &mut stop,
    )
    .await
    .unwrap();
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    cortex_m::asm::delay(1_000_000);
    let mut config = Config::default();
    config.cdcg.mult_m = 0xBEC;
    config.cdcg.vosc_mode = VoscClockMode::ExtendedFrequency;
    let (p, mode) = embassy_npcx::init_lpc(config);

    defmt::info!("Creating i2c");
    let i2cdevice = i2c::I2CController::new(
        p.SMB0,
        p.PC12,
        p.PB12,
        Irqs,
        mode,
        i2c::Config {
            speed: i2c::Speed::Standard,
            ..i2c::Config::default()
        },
    );

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

    spawner.must_spawn(device(i2cdevice));
    SIG.wait().await;

    let mut output = [0u8; 128];
    let mut target = [0u8; 256];
    for i in 0u8..=255 {
        target[i as usize] = i;
    }

    i2c.transaction(30, &mut [Operation::Write(&[0]), Operation::Write(&target)])
        .await
        .unwrap();

    i2c.transaction(30, &mut [Operation::Write(&[0x20]), Operation::Read(&mut output)])
        .await
        .unwrap();

    defmt::info!("Read: {:?}", output);
}
