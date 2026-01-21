#![no_main]
#![no_std]

use defmt::info;
use embassy_executor::Spawner;
use embassy_npcx::cdcg::VoscClockMode;
use embassy_npcx::i2c::{self, I2CController};
use embassy_npcx::{bind_interrupts, peripherals, Config};
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::signal::Signal;
use embedded_hal::i2c::Operation;
use embedded_hal_i2c::{
    AnyAddress, AsyncI2cTarget, AsyncReadTransaction, AsyncWriteTransaction, TransactionExpectEither, WriteResult,
};
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(pub struct Irqs {
    SMB0 => embassy_npcx::i2c::InterruptHandler<peripherals::SMB0>;
    SMB5 => embassy_npcx::i2c::InterruptHandler<peripherals::SMB5>;
});

static SIG: Signal<ThreadModeRawMutex, ()> = Signal::new();

const BUFLEN: usize = 512;
const SLAVE_ADDR: AnyAddress = AnyAddress::Seven(30);

#[embassy_executor::task]
async fn device(mut i2c: I2CController<'static>) {
    i2c.configure_addresses(&[30]).await.unwrap();

    SIG.signal(());

    // Implement a simple i2c RAM, demonstrating the features
    // of the new interface.

    let mut buf = [0u8; BUFLEN];
    let mut cur_addr = 0usize;

    let mut expect_read = false;

    loop {
        let mut addr = [0u8; 2];
        let result: TransactionExpectEither<_, _> = if expect_read && cur_addr < BUFLEN {
            i2c.listen_expect_read(SLAVE_ADDR, buf.get(cur_addr..).unwrap_or_default())
                .await
                .unwrap()
                .into()
        } else {
            i2c.listen_expect_write(SLAVE_ADDR, &mut addr).await.unwrap().into()
        };

        use TransactionExpectEither::*;
        match result {
            Deselect => {
                expect_read = false;
                info!("Deselection detected");
            }
            Read { handler, .. } => {
                if cur_addr >= BUFLEN {
                    // No valid address, so can't facilitate a read, nack it.
                    info!("Rejected read transaction, no valid start address");
                    drop(handler);
                } else {
                    // Provide the data for the read, and then let go of the bus after.
                    let size = handler.handle_complete(&buf[cur_addr..], 0xFF).await.unwrap();
                    info!(
                        "Read transaction starting at addr {}, provided {} bytes",
                        cur_addr, size
                    );
                    cur_addr = cur_addr.saturating_add(size).min(BUFLEN);
                }
            }
            ExpectedCompleteRead { size } => {
                info!(
                    "Expected read transaction starting at addr {}, provided {} bytes",
                    cur_addr, size
                );
                cur_addr = cur_addr.saturating_add(size).min(BUFLEN);
            }
            ExpectedPartialRead { handler } => {
                let size =
                    buf.get(cur_addr..).unwrap_or_default().len() + handler.handle_complete(&[], 0xFF).await.unwrap();
                info!(
                    "Expected partial read transaction starting at addr {}, provided {} bytes",
                    cur_addr, size
                );
                cur_addr = cur_addr.saturating_add(size).min(BUFLEN);
            }
            Write { handler, .. } => {
                info!("Write request");
                let mut addr = [0u8; 2];
                match handler.handle_part(&mut addr).await.unwrap() {
                    WriteResult::Partial(handler) => {
                        let new_addr: usize = u16::from_le_bytes(addr).into();
                        if new_addr < BUFLEN {
                            cur_addr = new_addr;
                            info!("Received addr {}", cur_addr);
                            expect_read = true;

                            let size_written = handler.handle_complete(&mut buf[cur_addr..]).await.unwrap();
                            cur_addr += size_written;
                            info!("Received write of {} bytes to ram", size_written);
                        } else {
                            // Invalid address, nack it
                            drop(handler);
                        }
                    }
                    WriteResult::Complete(size) => {
                        info!("Incomplete address write of size {} received, ignoring", size);
                    }
                };
            }
            ExpectedCompleteWrite { size } => {
                info!("Expected incomplete address write of size {} received, ignoring", size);
            }
            ExpectedPartialWrite { handler } => {
                info!("Expected partial write");
                let new_addr: usize = u16::from_le_bytes(addr).into();
                if new_addr < BUFLEN {
                    cur_addr = new_addr;
                    info!("Received addr {}", cur_addr);
                    expect_read = true;

                    let size_written = handler.handle_complete(&mut buf[cur_addr..]).await.unwrap();
                    cur_addr += size_written;
                    info!("Received write of {} bytes to ram", size_written);
                } else {
                    // Invalid address, nack it
                    drop(handler);
                }
            }
        }
    }
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

    spawner.spawn(device(i2cdevice).unwrap());
    SIG.wait().await;

    let mut buf = [0; 8];
    i2c.transaction(
        30,
        &mut [Operation::Write(&[
            16, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15,
        ])],
    )
    .await
    .unwrap();
    i2c.transaction(30, &mut [Operation::Write(&[20, 0]), Operation::Read(&mut buf)])
        .await
        .unwrap();
    defmt::info!("Read {}", buf);
}
