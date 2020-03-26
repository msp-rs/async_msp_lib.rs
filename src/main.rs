extern crate alloc;
extern crate multiwii_serial_protocol;

extern crate serialport;

use serialport::{available_ports, open};

use async_std::fs::File;
use async_std::io::prelude::*;
use async_std::io::BufWriter;
use async_std::sync::channel;
use async_std::{io,task};

extern crate packed_struct;
use packed_struct::prelude::*;
#[macro_use]
extern crate packed_struct_codegen;


// TODO: move this to multiwii_serial_protocol.rs library
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
    let (serial_reader_send, serial_reader_recv) = channel(4096);

    let mut serialport = open(&available_ports().expect("No serial port")[0].port_name)
        .expect("Failed to open serial port");

    // Clone the port
    let mut clone = serialport.try_clone().expect("Failed to clone");

    // green-thread 1: read into input channel from serial(reading from serial is blocking)
    task::spawn(async move {
        loop {
            let mut serial_buf: Vec<u8> = vec![0; 1];
            match serialport.read(serial_buf.as_mut_slice()) {
                Ok(bytes) => { // where does the bytes appeared
                    if bytes == 1 { // what if we received more then one? will we miss it? 
                        serial_reader_send.send(serial_buf[0]).await;
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
                    packet.serialize(&mut output).unwrap();
                    clone
                        .write(&output)
                        .expect("Failed to write to serial port");
                }
            }
        }
    });

    let (msp_send, msp_recv) = channel(1);

    // green-thread 2: parse the packets from channel
    //               : on new packet write into output channel the next request packet
    task::spawn(async move {
        let mut parser = multiwii_serial_protocol::MspParser::new();
        loop {
            // TODO: i think we should use epoll to know when to read the serial
            match serial_reader_recv.recv().await {
                None => break,
                Some(b) => {
                    match parser.parse(b) {
                        Ok(Some(p)) => {
                            msp_send.send(p).await;
                        },
                        Err(_) => {
                            println!("bad crc");
                            break;
                        },
                        Ok(None) => () //println!("not yet {:?}", &[b]),
                    }
                },
            }
        }
    });

    let serial_writer_send_clone = serial_writer_send.clone();
    // green-thread 3: read from the write channel and write to serial(writing to serial is blocking)
    let request_next_packet = task::spawn(async move {
        let mut used_size = 0u32;
        let mut next_address = 0u32;
        // let f = File::open("/tmp/trash.bin").await;
        // let mut writer = BufWriter::new(f);

        loop {
            match msp_recv.recv().await {
                None => break,
                Some(packet) => {
                    // TODO: create strust using https://github.com/hashmismatch/packed_struct.rs
                    if packet.cmd == multiwii_serial_protocol::MspCommandCode::MSP_DATAFLASH_SUMMARY as u8 {
                        let summary = MspDataFlashSummaryReply::unpack_from_slice(&packet.data).unwrap();
                        used_size = summary.used_size_bytes;
                        println!("{:?}", used_size);

                        let payload = MspDataFlashRead {
		                        read_address: 11111,  // next_address
                            read_length: 0xFA, // TODO: why are we geting crc error if parsing more the 254 bytes
	                      };
                        let packed = payload.pack();
                        // println!("CCCCCCCCCCCCCCCCCCC {:?}", packed.to_vec());
                        // println!("DDDDDDDDDDDDDDDDDDD {:?}", vec![0x01, 0x00, 0x00, 0x00,   0x01, 0x00]);

                        let packet = multiwii_serial_protocol::MspPacket {
		                        cmd: multiwii_serial_protocol::MspCommandCode::MSP_DATAFLASH_READ as u8,
		                        direction: multiwii_serial_protocol::MspPacketDirection::ToFlightController,
		                        data: alloc::borrow::Cow::Owned(packed.to_vec()),
                            // data: alloc::borrow::Cow::Owned(vec![]),
                            // data: alloc::borrow::Cow::Owned(vec![0,0,0,0   ,96, 0]),
                            // data: alloc::borrow::Cow::Owned(vec![0x01, 0x00, 0x00, 0x00,  0x02, 0x00]),
	                      };

                        serial_writer_send_clone.send(packet).await;
                    }

                    if packet.cmd == multiwii_serial_protocol::MspCommandCode::MSP_DATAFLASH_READ as u8 {

                        // extract the read address from the packet
                        let mut s = [0; 4];
                        &mut s[..].copy_from_slice(&packet.data[..4]);
                        let packet_address = u32::from_le_bytes(s);

                        let packet_payload;
                        if packet_address == next_address {
                            // remove the last address bytes and send to remaning payload to file stream(stdout)
                            packet_payload = &packet.data[4..];

                            // println!("got shitty packet {:?}", packet_address);
                            // TOOD: open a new channel for the file write

                            next_address += packet_payload.len() as u32;
                            if used_size < next_address {
                                // TODO: we are done we reached the end
                                break;
                            }

                            // print!("{:?}", packet_payload);
                            io::stdout().write(packet_payload).await;
                        }



                        let payload = MspDataFlashRead {
		                        read_address: next_address,
                            read_length: 0xFA, // TODO: why are we geting crc error if parsing more the 254 bytes
	                      };
                        let packed = payload.pack();

                        let packet = multiwii_serial_protocol::MspPacket {
		                        cmd: multiwii_serial_protocol::MspCommandCode::MSP_DATAFLASH_READ as u8,
		                        direction: multiwii_serial_protocol::MspPacketDirection::ToFlightController,
		                        data: alloc::borrow::Cow::Owned(packed.to_vec()),
	                      };

                        serial_writer_send_clone.send(packet).await;

                        // TODO: write file after send
                    }
                }
            }
        }
    });

    let packet = multiwii_serial_protocol::MspPacket {
		    cmd: multiwii_serial_protocol::MspCommandCode::MSP_DATAFLASH_SUMMARY as u8,
		    direction: multiwii_serial_protocol::MspPacketDirection::ToFlightController,
		    data: alloc::borrow::Cow::Owned(vec![])
	  };

    serial_writer_send.send(packet).await;

    request_next_packet.await;

    // TODO: use arg parse of some sort
}
