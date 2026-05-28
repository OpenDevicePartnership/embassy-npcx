#![no_main]
#![no_std]

use embassy_executor::Spawner;
use embassy_npcx::{bind_interrupts, peripherals, uart, Config};
use embedded_io_async::{Read, Write};
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(pub struct Irqs {
    CR_UART1_MDMA1 => embassy_npcx::uart::InterruptHandler<peripherals::CR_UART1>;
});

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    defmt::info!("Connected PC10 and PC09 together with a jumper wire to see output.");
    defmt::warn!("You may get DataOverrun errors depending on the MCU clock speed.");

    let (p, _mode) = embassy_npcx::init_lpc(Config::default());

    let mut config = uart::Config::default();
    config.baudrate = 115200;
    config.parity = Some(uart::Parity::ParityEven);
    config.stop_bits = uart::StopBits::STOP2;

    let uart = uart::Uart::new(p.CR_UART1, p.PC10, p.PC09, Irqs, config);

    let (rx, mut tx) = uart.split();

    spawner.spawn(reader(rx).unwrap());

    let buf = "Hello NPCX! Greetings from the Open Device Partnership!\r\n".as_bytes();
    loop {
        tx.write_all(&buf).await.unwrap();
        tx.flush().await.unwrap();
    }
}

fn find_newline(buf: &[u8]) -> Option<usize> {
    for (i, b) in buf.into_iter().enumerate() {
        if *b == b'\n' {
            return Some(i);
        }
    }
    None
}

#[embassy_executor::task]
async fn reader(mut rx: uart::UartRx<'static>) {
    let mut buf = [0u8; 128];
    let mut slice = &mut buf[..];
    loop {
        match rx.read(slice).await {
            Ok(n) => {
                let contains_newline = slice[0..n].contains(&b'\n');
                slice = &mut slice[n..];

                if slice.len() < 16 || contains_newline {
                    let buf_remaining = slice.len();
                    if let Some(newline_i) = find_newline(&buf) {
                        if let Ok(s) = core::str::from_utf8(&buf[0..newline_i]) {
                            defmt::info!("Read: {}", s);
                        } else {
                            defmt::error!("Got non-UTF8 bytes");
                        }

                        let buf_used = buf.len() - newline_i - buf_remaining;
                        // Copy back of buffer into front of buffer.
                        for i in 0..buf_used {
                            buf[i] = buf[i + newline_i];
                        }
                    } else {
                        defmt::warn!("Threw everything away");
                    }

                    //Throw everything away.
                    slice = &mut buf;
                }
            }
            Err(uart::Error::Break) => {
                defmt::info!("Break");
            }
            Err(e) => {
                defmt::error!("Read error: {}", e);
            }
        }
    }
}
