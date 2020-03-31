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

use std::sync::atomic::{AtomicBool, Ordering};


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


pub struct INavMsp<'a> {
  _serial: &'a Box<dyn SerialPort>,
}

impl<'a> INavMsp<'a> {
  // Create a new parserSerialPort
    pub fn new(serial_port: &mut Box<dyn SerialPort>) -> INavMsp {
        return INavMsp {
            _serial: serial_port,
        };
	  }

    fn _process_input(
        serial: &Box<dyn SerialPort>,
        parser_locked: Arc<Mutex<MspParser>>,
    ) -> (async_std::sync::Receiver<MspPacket>, Arc<AtomicBool>) {
        let mut serial_clone = serial.try_clone().unwrap();
        let (msp_reader_send, msp_reader_recv) = channel(1);

        let should_stop = Arc::new(AtomicBool::new(false));
        let should_stop_clone = should_stop.clone();

        // task 1: read into input channel from serial(reading from serial is blocking)
        task::spawn(async move {
            while should_stop.load(Ordering::Relaxed) == false {
                let mut serial_buf: Vec<u8> = vec![0; 1000];
                match serial_clone.read(serial_buf.as_mut_slice()) {
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

        return (msp_reader_recv, should_stop_clone);
	  }

    // TODO: test the diffrence between mut infront othe parameter name vs after
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

    // TODO: stop the reading thread when done receiving 
    // Are we waiting for the header of a brand new packet?
    pub async fn fetch_blackbox(&mut self) -> Result<async_std::sync::Receiver<Vec<u8>>, &str> {
        // let serial = &self._serial;
        let parser = MspParser::new();
        let parser_locked = Arc::new(Mutex::new(parser));

        let (msp_recv, should_stop) = INavMsp::_process_input(&self._serial, parser_locked.clone());

        // await for summary
        let mut serial_clone_1 = (&self._serial).try_clone().unwrap();
        let summary = INavMsp::_flash_summary(&mut serial_clone_1, &msp_recv).await?;
        let used_size = summary.used_size_bytes;

        let (chunk_send, chunk_recv) = channel::<Vec<u8>>(1);
        let mut serial_clone = (&self._serial).try_clone().unwrap();
        // let msp_recv = (&mut recv).clone();

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
                                should_stop.store(true, Ordering::Relaxed);
                                break;
                            }
                        }
                    }
                } else {
                    (*parser_locked.lock().await).reset();
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
        let parser = MspParser::new();
        let parser_locked = Arc::new(Mutex::new(parser));
        let mut serial = (&self._serial).try_clone().unwrap();

        let (msp_recv, should_stop) = INavMsp::_process_input(&mut (&self._serial), parser_locked);

        let result = INavMsp::_flash_summary(&mut serial, &msp_recv).await;
        should_stop.store(true, Ordering::Relaxed);

        return result;
    }

    async fn _flash_summary(
        serial: &mut Box<dyn SerialPort>,
        recv: &async_std::sync::Receiver<MspPacket>,
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
