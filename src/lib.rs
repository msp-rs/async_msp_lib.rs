extern crate alloc;
extern crate multiwii_serial_protocol;
extern crate serialport;
// extern crate serial;

use multiwii_serial_protocol::{MspCommandCode, MspPacket, MspPacketDirection, MspParser};
use serialport::SerialPort;

// use serial::prelude::*;
use std::io::prelude::*;

use async_std::sync::{channel, Arc, Mutex, Sender, Receiver};
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

pub struct MspDataFlashReply {
  pub read_address: u32,
  pub payload: Vec<u8>,
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

  _msp_reader_send: Sender<MspPacket>,
  _msp_reader_recv: Receiver<MspPacket>,
  _msp_writer_send: Sender<MspPacket>,
  _msp_writer_recv: Receiver<MspPacket>,

  _summary_recv: Receiver<MspDataFlashSummaryReply>,
  _summary_send: Sender<MspDataFlashSummaryReply>,
  _chunk_recv: Receiver<MspDataFlashReply>,
  _chunk_send: Sender<MspDataFlashReply>,
}

impl INavMsp {
  // Create a new parserSerialPort
    pub fn new() -> INavMsp {
        let (msp_reader_send, msp_reader_recv) = channel::<MspPacket>(1);
        let (msp_writer_send, msp_writer_recv) = channel::<MspPacket>(1);

        let (summary_send, summary_recv) = channel::<MspDataFlashSummaryReply>(1);
        let (chunk_send, chunk_recv) = channel::<MspDataFlashReply>(1);

        let parser = MspParser::new();
        let parser_locked = Arc::new(Mutex::new(parser));

        return INavMsp {
            _parser_locked: parser_locked,
            _msp_reader_send: msp_reader_send,
            _msp_reader_recv: msp_reader_recv,
            _msp_writer_send: msp_writer_send,
            _msp_writer_recv: msp_writer_recv,

            _summary_send: summary_send,
            _summary_recv: summary_recv,
            _chunk_send: chunk_send,
            _chunk_recv: chunk_recv,
        };
	  }

    pub fn start(&self, serial: Box<dyn SerialPort>) {
        let serial_clone = serial.try_clone().unwrap();

        INavMsp::_process_input(serial, self._parser_locked.clone(), self._msp_reader_send.clone());
        INavMsp::_process_output(serial_clone, self._msp_writer_recv.clone());
        INavMsp::_process_route(
            self._msp_reader_recv.clone(),
            self._summary_send.clone(),
            self._chunk_send.clone(),
        );
    }

    fn _process_route(
        msp_reader_recv: Receiver<MspPacket>,
        summary_send: Sender<MspDataFlashSummaryReply>,
        chunk_send: Sender<MspDataFlashReply>,
    ) {
        task::spawn(async move {
            loop {
                let packet = match msp_reader_recv.recv().await {
                    None => break,
                    Some(packet) => packet,
                };

                if packet.cmd == MspCommandCode::MSP_DATAFLASH_SUMMARY as u16 {
                    let summary = MspDataFlashSummaryReply::unpack_from_slice(&packet.data).unwrap();
                    summary_send.send(summary).await;
                }

                if packet.cmd == MspCommandCode::MSP_DATAFLASH_READ as u16 {
                    // extract the read address from the packet
                    let mut s = [0; 4];
                    s.copy_from_slice(&packet.data[..4]);
                    let packet_address = u32::from_le_bytes(s);

                    // remove the last address bytes and send to remaning payload to file stream(stdout)
                    let packet_payload = &packet.data[4..];

                    let chunk = MspDataFlashReply {
                        read_address: packet_address,
                        payload: packet_payload.to_vec(),
                    };
                    chunk_send.send(chunk).await;
                }
            }
        });
    }

    // TODO: return joinhandler, so we can stop the tasks on drop
    fn _process_input(
        mut serial: Box<dyn SerialPort>,
        parser_locked: Arc<Mutex<MspParser>>,
        msp_reader_send: Sender<MspPacket>
    ) -> Arc<AtomicBool> {
        let should_stop = Arc::new(AtomicBool::new(false));
        let should_stop_clone = should_stop.clone();

        // task 1: read into input channel from serial(reading from serial is blocking)
        task::spawn(async move {
            while should_stop.load(Ordering::Relaxed) == false {
                let mut serial_buf: Vec<u8> = vec![0; 1000];
                match serial.read(serial_buf.as_mut_slice()) {
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
        return should_stop_clone;
	  }

    // TODO: return joinhandler, so we can stop the tasks on drop
    fn _process_output(
        mut serial: Box<dyn SerialPort>,
        msp_writer_recv: Receiver<MspPacket>,
    ) {
        task::spawn(async move {
            loop {
                let packet = match msp_writer_recv.recv().await {
                    None => break,
                    Some(packet) => packet,
                };

                let size = packet.packet_size_bytes();
                let mut output = vec![0; size];

                packet
                    .serialize_v2(&mut output)
                    .expect("Failed to serialize");

                serial
                    .write(&output)
                    .expect("Failed to write to serial port");
            }
        });
	  }

    // TODO: start the reading thread and writing thread on on _start_proccess
    // TODO: we then register channel for each message type prior to start reading
    // TODO: stop the reading thread when done receiving
    // Are we waiting for the header of a brand new packet?
    // This should return file stream and not channel... lol lololo ... lolol...
    // we should return interface that implements async_std::io::Read trait
    pub async fn fetch_blackbox(&self) {
        // await for summary
        let summary = self.flash_summary().await;
        let used_size = summary.used_size_bytes;

        let mut next_address = 0u32;
        loop {
            let timeout_res = future::timeout(Duration::from_millis(30), self._chunk_recv.recv()).await;

            // resend the packet
            if timeout_res.is_ok() {
                match timeout_res.unwrap() {
                    None => break,
                    Some(packet) => {
                        // Verify that the address of the memory returned matches what the caller asked for
                        if packet.read_address != next_address {
                            continue
                        }

                        next_address += packet.payload.len() as u32;

                        if next_address >= used_size {
                            // should_stop.store(true, Ordering::Relaxed);
                            break;
                        }
                    }
                }
            } else {
                (*self._parser_locked.lock().await).reset();
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

            self._msp_writer_send.send(packet).await;
        }
	  }

    pub async fn flash_summary(&self) -> MspDataFlashSummaryReply {
        let packet = MspPacket {
            cmd: MspCommandCode::MSP_DATAFLASH_SUMMARY as u16,
            direction: MspPacketDirection::ToFlightController,
            data: vec![],
        };

        self._msp_writer_send.send(packet).await;

        // TODO: set timeout on recv future::timeout(Duration::from_millis(30), msp_recv.recv()).await;
        // condier using Result like Err() and Ok() here
        return self._summary_recv.recv().await.unwrap();
	  }
}
