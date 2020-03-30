extern crate alloc;
extern crate multiwii_serial_protocol;
extern crate serialport;
// extern crate serial;

use multiwii_serial_protocol::{MspCommandCode, MspPacket, MspPacketDirection, MspParser};
use serialport::SerialPort;

// use serial::prelude::*;
use std::io::prelude::*;

use async_std::sync::{channel, Arc, Mutex};
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

// TODO: extract this code to rust module(different file)

// #[derive(PackedStruct, Debug, Copy, Clone)]
// #[packed_struct(bytes = "1", endian = "lsb", bit_numbering = "msb0")]
// pub struct MspDataFlashReply {
//     pub reply_address: u32,
//     pub data: u32,
// }


pub struct INavMsp {
  _parser_locked: Arc<Mutex<MspParser>>,
  _recv: async_std::sync::Receiver<MspPacket>,
  _serial: Box<dyn SerialPort>,
}

impl INavMsp {
  /// Create a new parserSerialPort
  pub fn new(mut serial_port: Box<dyn SerialPort>) -> INavMsp {
        let parser = MspParser::new();
        let parser_locked = Arc::new(Mutex::new(parser));
        let parser_locked_clone = parser_locked.clone();

        // Clone the port
        let serial_port_write = serial_port.try_clone().unwrap();

        let (msp_reader_send, msp_reader_recv) = channel(1);

        // task 1: read into input channel from serial(reading from serial is blocking)
        // let read_task =
        task::spawn(async move {
            loop {
                let mut serial_buf: Vec<u8> = vec![0; 1000];
                match serial_port.read(serial_buf.as_mut_slice()) {
                    Ok(bytes) => {
                        for n in 0..bytes {
                            match (*parser_locked.lock().await).parse(serial_buf[n]) {
                                Ok(Some(p)) => {
                                    msp_reader_send.send(p).await
                                },
                                Err(e) => eprintln!("bad crc {:?}", e),
                                Ok(None) => ()
                            }
                        }
                    }
                    Err(ref e) if e.kind() == io::ErrorKind::TimedOut => task::yield_now().await,
                    Err(e) => eprintln!("{:?}", e),
                }
            }
        });

		    return INavMsp {
            _parser_locked: parser_locked_clone,
            _recv: msp_reader_recv,
			      _serial: serial_port_write,
		    };
	}

    fn _send_packet(serial: &mut Box<dyn SerialPort>, packet: &MspPacket) {
        let size = packet.packet_size_bytes();
        let mut output = vec![0; size];

        packet
            .serialize_v2(&mut output)
            .expect("Failed to serialize");

        serial
            .write(&output)
            .expect("Failed to write to serial port");
    }

    // Are we waiting for the header of a brand new packet?
    pub async fn fetch_blackbox(&mut self) -> Result<async_std::sync::Receiver<Vec<u8>>, &str> {
        let mut serial_clone = (&mut self._serial).try_clone().unwrap();
        let parser = (&mut self._parser_locked).clone();
        // await for summary
        let summary = INavMsp::_flash_summary(&mut serial_clone, &mut self._recv).await?;
        let used_size = summary.used_size_bytes;

        let (chunk_send, chunk_recv) = channel::<Vec<u8>>(1);
        let msp_recv = (&mut self._recv).clone();

        task::spawn(async move {
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

                            // Verify that the address of the memory returned matches what the caller asked for
                            if packet_address != next_address {
                                continue
                            }

                            // remove the last address bytes and send to remaning payload to file stream(stdout)
                            let packet_payload = &packet.data[4..];

                            chunk_send.send(packet_payload.to_vec()).await;
                            next_address += packet_payload.len() as u32;

                            if next_address >= used_size {
                                break;
                            }
                        }
                    }
                } else {
                    (*parser.lock().await).reset();
                }

                let payload = MspDataFlashRead {
		                read_address: next_address,
                    read_length: 0x1000,
	              };
                let packed = payload.pack();

                let packet = multiwii_serial_protocol::MspPacket {
		                cmd: multiwii_serial_protocol::MspCommandCode::MSP_DATAFLASH_READ as u16,
		                direction: multiwii_serial_protocol::MspPacketDirection::ToFlightController,
		                data: packed.to_vec(),
	              };

                INavMsp::_send_packet(&mut serial_clone, &packet);
            }

            Ok::<(), std::io::Error>(())
        });

        return Ok(chunk_recv);
	  }

    pub async fn flash_summary(&mut self) -> Result<MspDataFlashSummaryReply, &'static str> {
        return INavMsp::_flash_summary(&mut self._serial, &mut self._recv).await;
    }

    async fn _flash_summary(
        serial: &mut Box<dyn SerialPort>,
        recv: &mut async_std::sync::Receiver<MspPacket>,
    ) -> Result<MspDataFlashSummaryReply, &'static str> {
        let packet = MspPacket {
            cmd: MspCommandCode::MSP_DATAFLASH_SUMMARY as u16,
            direction: MspPacketDirection::ToFlightController,
            data: vec![],
        };

        INavMsp::_send_packet(serial, &packet);

        let summary_packet = recv.recv().await.unwrap();

        if summary_packet.cmd == MspCommandCode::MSP_DATAFLASH_SUMMARY as u16 {
            let summary = MspDataFlashSummaryReply::unpack_from_slice(&summary_packet.data).unwrap();
            return Ok(summary);
        }

        return Err("failed to receive summary");
	  }
}
