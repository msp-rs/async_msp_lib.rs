extern crate alloc;
extern crate async_msp_lib;
extern crate multiwii_serial_protocol;

extern crate serialport;

use serialport::{available_ports, open};

extern crate packed_struct;
extern crate packed_struct_codegen;

use async_std::fs::OpenOptions;
use async_std::io::BufWriter;
use async_std::io::prelude::*;



#[async_std::main]
async fn main() {
    let serialport = open(&available_ports().expect("No serial port")[0].port_name)
        .expect("Failed to open serial port");

    // green-thread 1: read into input channel from serial(reading from serial is blocking)
    let inav = inav_msp_lib::INavMsp::new();

    inav.start(serialport);

    let summary = inav.flash_summary().await;
    println!("done {:?}", summary);

    let mut flash_data_file = inav.open_flash_data().await;

    let f = OpenOptions::new()
        .write(true)
        .create(true)
        .truncate(true)
        .open("/tmp/trash.bin")
        .await.unwrap();

    let mut buf_writer = BufWriter::new(f);
    loop {
        let chunk = flash_data_file.read_chunk().await.unwrap();
        if chunk.len() == 0 {
            buf_writer.flush().await.unwrap();
            break;
        }
        buf_writer.write(&chunk[..]).await.unwrap();
    }
}
