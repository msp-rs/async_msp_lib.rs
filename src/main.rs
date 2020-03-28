extern crate alloc;
extern crate multiwii_serial_protocol;

extern crate serialport;

use serialport::{available_ports, open};

use async_std::fs::OpenOptions;
use async_std::io::prelude::*;
use async_std::io::BufWriter;
use async_std::sync::{channel, Mutex, Arc};
use async_std::{io, task};

use async_std::future;
use std::time::Duration;

extern crate packed_struct;
use packed_struct::prelude::*;
#[macro_use]
extern crate packed_struct_codegen;


// TODO: move this to multiwii_serial_protocol.rs library
// TODO: and figure out why we can't call unpack on structs from multiwii library
#[derive(PackedStruct, Debug, Copy, Clone)]
#[packed_struct(bytes = "6", endian = "lsb", bit_numbering = "msb0")]
pub struct MspDataFlashRead {
  pub read_address: u32,
  pub read_length: u16,
}

#[derive(PackedStruct, Debug, Copy, Clone)]
#[packed_struct(bytes = "1", endian = "lsb", bit_numbering = "msb0")]
pub struct MspDataFlashSummaryReply {
    #[packed_field(bits = "6")]
    pub supported: bool,
    #[packed_field(bits = "7")]
    pub ready: bool,
    pub sectors: u32,
    pub total_size_bytes: u32,
    pub used_size_bytes: u32,
}


// #[derive(PackedStruct, Debug, Copy, Clone)]
// #[packed_struct(bytes = "1", endian = "lsb", bit_numbering = "msb0")]
// pub struct MspDataFlashReply {
//     pub reply_address: u32,
//     pub data: u32,
// }

#[async_std::main]
async fn main() {
    let mut serialport = open(&available_ports().expect("No serial port")[0].port_name)
        .expect("Failed to open serial port");

    // Clone the port
    let mut clone = serialport.try_clone().expect("Failed to clone");

    let (msp_send, msp_recv) = channel(1);

    // green-thread 1: read into input channel from serial(reading from serial is blocking)
    let parser = multiwii_serial_protocol::MspParser::new();
    let parser_locked = Arc::new(Mutex::new(parser));
    let parser_locked_clone = parser_locked.clone();
    task::spawn(async move {
        loop {
            let mut serial_buf: Vec<u8> = vec![0; 1000];
            // TODO: i think we should use epoll to know when to read the serial
            match serialport.read(serial_buf.as_mut_slice()) {
                Ok(bytes) => {
                    for n in 0..bytes {
                        match (*parser_locked.lock().await).parse(serial_buf[n]) {
                            Ok(Some(p)) => msp_send.send(p).await,
                            Err(e) => println!("bad crc {:?}", e),
                            Ok(None) => ()
                        }
                    }
                }
                Err(ref e) if e.kind() == io::ErrorKind::TimedOut => task::yield_now().await,
                Err(e) => eprintln!("{:?}", e),
            }
        }
    });

    let (serial_writer_send, serial_writer_recv) = channel::<multiwii_serial_protocol::MspPacket>(1);

    task::spawn(async move {
        loop {
            match serial_writer_recv.recv().await {
                None => break,
                Some(packet) => {
                    let size = packet.packet_size_bytes();
                    let mut output = vec![0; size];

                    packet.serialize_v2(&mut output).expect("Failed to serialize");
                    clone
                        .write(&output)
                        .expect("Failed to write to serial port");
                }
            }
        }
    });

    let serial_writer_send_clone = serial_writer_send.clone();


    // TODO: await for summary
    let packet = multiwii_serial_protocol::MspPacket {
        cmd: multiwii_serial_protocol::MspCommandCode::MSP_DATAFLASH_SUMMARY as u16,
        direction: multiwii_serial_protocol::MspPacketDirection::ToFlightController,
        data: alloc::borrow::Cow::Owned(vec![]),
    };

    serial_writer_send.send(packet).await;
    let summary_packet = msp_recv.recv().await.unwrap();

    if summary_packet.cmd != multiwii_serial_protocol::MspCommandCode::MSP_DATAFLASH_SUMMARY as u16 {
        println!("failed to receive summary");
        return;
    }

    let summary = MspDataFlashSummaryReply::unpack_from_slice(&summary_packet.data).unwrap();
    let used_size = summary.used_size_bytes;

    // green-thread 3: read from the write channel and write to serial(writing to serial is blocking)
    let request_next_packet = task::spawn(async move {
        let f = OpenOptions::new()
            .write(true)
            .create(true)
            .truncate(true)
            .open("/tmp/trash.bin").await?;

        let mut buf_writer = BufWriter::new(f);

        let mut next_address = 0u32;
        loop {
            let timeout_res = future::timeout(Duration::from_millis(30), msp_recv.recv()).await;

            // resend the packet
            if timeout_res.is_ok() {
                match timeout_res.unwrap() {
                    None => break,
                    Some(packet) => {
                        if packet.cmd != multiwii_serial_protocol::MspCommandCode::MSP_DATAFLASH_READ as u16 {
                            // panic because we recevied unexpected packet
                        }

                        // extract the read address from the packet
                        let mut s = [0; 4];
                        s.copy_from_slice(&packet.data[..4]);
                        let packet_address = u32::from_le_bytes(s);

                        // TODO: we could buffer the blocks writer
                        //     : we could send like 5 chunks ahead
                        //     : and wait for the blocks replay
                        //     : we save dictionary of the address blocks
                        //     : once the block is received we remove it from the map
                        //     : and pop the next block into the write channel

                        // Verify that the address of the memory returned matches what the caller asked for
                        if packet_address != next_address {
                            continue
                        }

                        // remove the last address bytes and send to remaning payload to file stream(stdout)
                        let packet_payload = &packet.data[4..];

                        // TOOD: open a new channel for the file write

                        buf_writer.write(packet_payload).await?;
                        next_address += packet_payload.len() as u32;

                        if next_address >= used_size {
                            buf_writer.flush().await?;
                            println!("done");
                            break;
                        }
                    }
                }
            } else {
                println!("reset parser");
                (*parser_locked_clone.lock().await).reset();
            }

            let payload = MspDataFlashRead {
		            read_address: next_address,
                read_length: 0x1000,
	          };
            let packed = payload.pack();

            let packet = multiwii_serial_protocol::MspPacket {
		            cmd: multiwii_serial_protocol::MspCommandCode::MSP_DATAFLASH_READ as u16,
		            direction: multiwii_serial_protocol::MspPacketDirection::ToFlightController,
		            data: alloc::borrow::Cow::Owned(packed.to_vec()),
	          };

            println!("getting packet packet {:?} of {:?}", next_address, used_size);
            serial_writer_send_clone.send(packet).await;
        }

        Ok::<(), std::io::Error>(())
    });

    request_next_packet.await.expect("failed to read blackbox");

    // TODO: use arg parse of some sort
}
