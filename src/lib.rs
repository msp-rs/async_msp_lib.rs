extern crate alloc;
extern crate multiwii_serial_protocol;
extern crate serialport;
extern crate packed_struct;
#[macro_use]
extern crate packed_struct_codegen;

use multiwii_serial_protocol::{MspCommandCode, MspPacket, MspPacketDirection};
use serialport::SerialPort;
use packed_struct::prelude::*;

use async_std::sync::{channel, Sender, Receiver};
use async_std::{io, task};
use async_std::future;

use std::time::Duration;

mod core;


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

#[derive(PackedStruct, Debug, Copy, Clone)]
#[packed_struct(bytes = "4", endian = "lsb", bit_numbering = "msb0")]
pub struct MspModeRange {
    pub box_id: u8,
    pub aux_channel_index: u8,
    pub start_step: u8,
    pub end_step: u8,
}

#[derive(PackedStruct, Debug, Copy, Clone)]
#[packed_struct(bytes = "5", endian = "lsb", bit_numbering = "msb0")]
pub struct MspSetModeRange {
    pub index: u8,
    #[packed_field(size_bytes="4")]
    pub mode_range: MspModeRange,
}

// const MAX_MODE_ACTIVATION_CONDITION_COUNT: u8 = 20u8;

#[derive(PackedStruct, Debug, Copy, Clone)]
#[packed_struct(endian = "lsb", bit_numbering = "msb0")]
pub struct MspModeRangesReplay {
    #[packed_field(element_size_bytes="4")]
    mode_ranges: [MspModeRange; 20], // 20 is defined as MAX_MODE_ACTIVATION_CONDITION_COUNT
}


#[derive(Debug)]
pub struct ModeRange {
    pub index: u8,
    pub box_id: u8,
    pub aux_channel_index: u8,
    pub start_step: u8,
    pub end_step: u8,
}

#[derive(PackedStruct, Debug, Copy, Clone)]
#[packed_struct(bytes = "8", endian = "lsb", bit_numbering = "msb0")]
pub struct MspMotorMixer {
    pub throttle: u16,
    pub roll: u16,
    pub pitch: u16,
    pub yaw: u16,
}

#[derive(PackedStruct, Debug, Copy, Clone)]
#[packed_struct(bytes = "9", endian = "lsb", bit_numbering = "msb0")]
pub struct MspSetMotorMixer {
    pub index: u8,
    #[packed_field(size_bytes="8")]
    pub motor_mixer: MspMotorMixer,
}

// const MAX_MODE_ACTIVATION_CONDITION_COUNT: u8 = 20u8;

#[derive(PackedStruct, Debug, Copy, Clone)]
#[packed_struct(endian = "lsb", bit_numbering = "msb0")]
pub struct MspMotorMixersReplay {
    #[packed_field(element_size_bytes="8")]
    mode_ranges: [MspMotorMixer; 12], // 12 is defined as MAX_SUPPORTED_MOTORS
}

#[derive(Debug)]
pub struct MotorMixer {
    pub index: u8,
    pub throttle: u16,
    pub roll: u16,
    pub pitch: u16,
    pub yaw: u16,
}

// TODO: extract this code to rust module(different file)

pub struct FlashDataFile {
    core: core::Core,
    chunk_recv: Receiver<MspDataFlashReply>,
    used_size: u32,
    next_address: u32,
    // requested_address: u32,
    received_address: u32,
}

// TODO: we should return interface that implements async_std::io::Read trait
// TODO: why not return move the payload vec instead of the io result??
impl FlashDataFile {
    pub async fn read_chunk(&mut self) -> io::Result<Vec<u8>> {
        if self.received_address >= self.used_size {
            return Err(io::Error::new(io::ErrorKind::ConnectionReset, "use after close"));
        }

        loop {
            if self.next_address > self.received_address || self.next_address == 0 {
                let payload = MspDataFlashRead {
                    read_address: self.next_address,
                    read_length: 0x1000,
                };
                let packed = payload.pack();

                let packet = multiwii_serial_protocol::MspPacket {
                    cmd: multiwii_serial_protocol::MspCommandCode::MSP_DATAFLASH_READ as u16,
                    direction: multiwii_serial_protocol::MspPacketDirection::ToFlightController,
                    data: packed.to_vec(),
                };

                self.core.write(packet).await;
            }

            let timeout_res = future::timeout(Duration::from_millis(50), self.chunk_recv.recv()).await;

            // resend the packet
            if timeout_res.is_ok() {
                match timeout_res.unwrap() {
                    None => return Err(io::Error::new(io::ErrorKind::ConnectionAborted, "device disconnected")),
                    Some(packet) => {

                        if packet.read_address >= self.next_address {
                            self.received_address = packet.read_address;
                            self.next_address = packet.read_address + packet.payload.len() as u32;
                        } else {
                            continue;
                        }

                        println!("{:?}/{:?}", packet.read_address, self.used_size);

                        if self.received_address >= self.used_size {
                            return Ok(vec![]);
                        }

                        return Ok(packet.payload);
                    }
                }
            } else {
                self.core.reset_parser().await;
            }
        }
    }
}

pub struct INavMsp {
    core: core::Core,

    mode_ranges_recv: Receiver<MspModeRangesReplay>,
    mode_ranges_send: Sender<MspModeRangesReplay>,
    set_mode_range_ack_recv: Receiver<()>,
    set_mode_range_ack_send: Sender<()>,
    motor_mixers_recv: Receiver<MspMotorMixersReplay>,
    motor_mixers_send: Sender<MspMotorMixersReplay>,
    set_motor_mixer_ack_recv: Receiver<()>,
    set_motor_mixer_ack_send: Sender<()>,
    summary_recv: Receiver<MspDataFlashSummaryReply>,
    summary_send: Sender<MspDataFlashSummaryReply>,
    chunk_recv: Receiver<MspDataFlashReply>,
    chunk_send: Sender<MspDataFlashReply>,
}

impl INavMsp {
    // Create a new parserSerialPort
    pub fn new() -> INavMsp {
        let core = core::Core::new();

        let (mode_ranges_send, mode_ranges_recv) = channel::<MspModeRangesReplay>(100);
        let (set_mode_range_ack_send, set_mode_range_ack_recv) = channel::<()>(100);
        let (motor_mixers_send, motor_mixers_recv) = channel::<MspMotorMixersReplay>(100);
        let (set_motor_mixer_ack_send, set_motor_mixer_ack_recv) = channel::<()>(100);
        let (summary_send, summary_recv) = channel::<MspDataFlashSummaryReply>(100);
        let (chunk_send, chunk_recv) = channel::<MspDataFlashReply>(4096);

        return INavMsp {
            core: core,

            mode_ranges_send: mode_ranges_send,
            mode_ranges_recv: mode_ranges_recv,
            set_mode_range_ack_recv: set_mode_range_ack_recv,
            set_mode_range_ack_send: set_mode_range_ack_send,
            motor_mixers_send: motor_mixers_send,
            motor_mixers_recv: motor_mixers_recv,
            set_motor_mixer_ack_recv: set_motor_mixer_ack_recv,
            set_motor_mixer_ack_send: set_motor_mixer_ack_send,
            summary_send: summary_send,
            summary_recv: summary_recv,
            chunk_send: chunk_send,
            chunk_recv: chunk_recv,
        };
	  }

    // TODO: If serial-port rs supports standard read write interface we should use this instead of seril explocitly
    pub fn start(&self, serial: Box<dyn SerialPort>) {
        &self.core.start(serial);

        INavMsp::process_route(
            self.core.clone(),
            self.mode_ranges_send.clone(),
            self.set_mode_range_ack_send.clone(),
            self.motor_mixers_send.clone(),
            self.set_motor_mixer_ack_send.clone(),
            self.summary_send.clone(),
            self.chunk_send.clone(),
        );
    }

    fn process_route(
        core: core::Core,
        mode_ranges_send: Sender<MspModeRangesReplay>,
        set_mode_range_ack_send: Sender<()>,
        motor_mixers_send: Sender<MspMotorMixersReplay>,
        set_motor_mixer_ack_send: Sender<()>,
        summary_send: Sender<MspDataFlashSummaryReply>,
        chunk_send: Sender<MspDataFlashReply>,
    ) {
        task::spawn(async move {
            loop {
                let packet = match core.read().await {
                    None => break,
                    Some(packet) => packet,
                };

                if packet.direction != MspPacketDirection::FromFlightController {
                    continue;
                }

                if packet.cmd == MspCommandCode::MSP_MODE_RANGES as u16 {
                    let ranges = MspModeRangesReplay::unpack_from_slice(&packet.data).unwrap();
                    mode_ranges_send.send(ranges).await;
                }

                if packet.cmd == MspCommandCode::MSP_SET_MODE_RANGE as u16 {
                    // packet data should be empty, so just signal ack is received
                    set_mode_range_ack_send.send(()).await;
                }

                if packet.cmd == MspCommandCode::MSP2_MOTOR_MIXER as u16 {
                    let mixers = MspMotorMixersReplay::unpack_from_slice(&packet.data).unwrap();
                    motor_mixers_send.send(mixers).await;
                }

                if packet.cmd == MspCommandCode::MSP2_SET_MOTOR_MIXER as u16 {
                    // packet data should be empty, so just signal ack is received
                    set_motor_mixer_ack_send.send(()).await;
                }

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

                // TODO: create debug flag for additional print on demand
                // println!("{:?}", packet);
            }
        });
    }

    // TODO: because this is a serial protocol, we cannot allow two reads of the file at the same time.
    //       so throw error, if this function is called while another file is open already
    /// altought slower then read_flash_data, its a safe data, the reads are happening serialy
    pub async fn open_flash_data(&self) -> FlashDataFile {
        // await for summary
        let summary = self.flash_summary().await;
        let used_size = summary.unwrap().used_size_bytes;

        return FlashDataFile {
            core: self.core.clone(),
            chunk_recv: self.chunk_recv.clone(),
            used_size: used_size,
            next_address: 0u32,
            received_address: 0u32,
        };
	  }

    // TODO: use https://docs.rs/async-std/1.5.0/async_std/io/struct.Cursor.html to write unordered file stream,
    // TODO: move blackbox to sibling module
    // buffer all reads into channel and when function is called receive from that channel
    // so once the file is open, the reading loop starts, and new chunks are pushed into channel and fetched once the read function called on the returned file descriptor
    /// coccurent unordered data flash reading, this method assumes data is organazised into equal(except last one) chunks of data
    pub async fn read_flash_data(&self, chunk_size: usize, callback: fn(chunk: usize, total: usize)) -> io::Result<Vec<u8>> {
        // await for summary
        let summary = self.flash_summary().await;
        let used_size = summary.unwrap().used_size_bytes as usize;

        // let chunk_size = 0x800u32;
        // let chunk_size = 0x1000u32; // no point going more then this size because inav won't return bigger chunk
        // let chunk_size = 0x4000u32;
        // let block_size = 4096; // number of chunks to read

        // TODO, fix bug: round up here
        let blocks_count = used_size / chunk_size; // number of chunks to read

        let mut expected_address = vec![];


        for x in (0..blocks_count * chunk_size).step_by(chunk_size as usize) {
            &expected_address.push(x);
        }

        println!("expected_address: {:?}", &expected_address.len());

        let mut accumulated_payload = vec![vec![]; blocks_count as usize];
        loop {
            for addr in &expected_address {
                let payload = MspDataFlashRead {
                    read_address: *addr as u32,
                    read_length: chunk_size as u16,
                };
                let packed = payload.pack();

                let packet = multiwii_serial_protocol::MspPacket {
                    cmd: multiwii_serial_protocol::MspCommandCode::MSP_DATAFLASH_READ as u16,
                    direction: multiwii_serial_protocol::MspPacketDirection::ToFlightController,
                    data: packed.to_vec(),
                };

                self.core.write(packet).await;
            }

            loop {
                // TODO: maybe let the caller handle the timeout?
                let timeout_res = future::timeout(Duration::from_millis(500), self.chunk_recv.recv()).await;

                if timeout_res.is_ok() {
                    match timeout_res.unwrap() {
                        None => return Err(io::Error::new(io::ErrorKind::ConnectionAborted, "device disconnected")),
                        Some(packet) => {
                            let idx = match expected_address.binary_search(&(packet.read_address as usize)) {
                                Ok(idx) => idx,
                                Err(_) => continue,
                            };

                            // last element can be less then chunk size
                            // assert_eq!(*&chunk_size as usize, packet.payload.len());

                            let insert_location = &(packet.read_address as usize) / &chunk_size;
                            (&mut accumulated_payload)[insert_location as usize] = packet.payload;
                            expected_address.remove(idx);

                            callback(used_size - expected_address.len() * chunk_size, used_size);

                            if expected_address.is_empty() {
                                let buff = accumulated_payload.iter().cloned().flatten().collect::<Vec<u8>>();
                                // println!("return {:?}", buff.len());
                                return Ok(buff);
                            }
                        }
                    }
                } else {
                    self.core.reset_parser().await;
                    // println!("timeout, address left {:?}", expected_address);
                    break;
                }
            }
        }
	  }

    pub async fn flash_summary(&self) -> io::Result<MspDataFlashSummaryReply> {
        let packet = MspPacket {
            cmd: MspCommandCode::MSP_DATAFLASH_SUMMARY as u16,
            direction: MspPacketDirection::ToFlightController,
            data: vec![],
        };

        self.core.write(packet).await;

        let timeout_res = future::timeout(Duration::from_millis(500), self.summary_recv.recv()).await;
        if timeout_res.is_ok() {
            return Ok(timeout_res.unwrap().unwrap());
        }

        return Err(io::Error::new(io::ErrorKind::TimedOut, "timedout waiting for summary response"));
	  }

    pub async fn set_mode_range(&self, mode: ModeRange) -> io::Result<()> {

        let payload = MspSetModeRange {
            index: mode.index,
            mode_range: MspModeRange {
                box_id: mode.box_id,
                aux_channel_index: mode.aux_channel_index,
                start_step: mode.start_step,
                end_step: mode.end_step,
            }
        };

        let packet = MspPacket {
            cmd: MspCommandCode::MSP_SET_MODE_RANGE as u16,
            direction: MspPacketDirection::ToFlightController,
            data: payload.pack().to_vec(),
        };

        self.core.write(packet).await;

        // TODO: we are not sure this ack is for our request, because there is no id for the request
        let timeout_res = future::timeout(Duration::from_millis(500), self.set_mode_range_ack_recv.recv()).await;
        if timeout_res.is_ok() {
            return Ok(timeout_res.unwrap().unwrap());
        }

        return Err(io::Error::new(io::ErrorKind::TimedOut, "timedout waiting for set mode range response"));
	  }

    pub async fn get_mode_ranges(&self) -> io::Result<Vec<ModeRange>> {
        let packet = MspPacket {
            cmd: MspCommandCode::MSP_MODE_RANGES as u16,
            direction: MspPacketDirection::ToFlightController,
            data: vec![],
        };

        self.core.write(packet).await;

        // TODO: we are not sure this ack is for our request, because there is no id for the request
        // TODO: what if we are reading packet that was sent long time ago
        // TODO: also currently if no one is reading the channges, we may hang

        let timeout_res = future::timeout(Duration::from_millis(5000), self.mode_ranges_recv.recv()).await;
        if !timeout_res.is_ok() {
            return Err(io::Error::new(io::ErrorKind::TimedOut, "timedout waiting for get mode ranges response"));
        }

        let ranges_replay = timeout_res.unwrap().unwrap();
        let mut valid_ranges = vec![];

        // TODO: not all 20 ranges will be active, return only the active ranges
        ranges_replay.mode_ranges.iter().enumerate().fold(&mut valid_ranges, |acc, (i, r)| {
            if r.start_step != 0 && r.end_step != 0 {
                acc.push(ModeRange {
                    index: i as u8,
                    box_id: r.box_id,
                    aux_channel_index: r.aux_channel_index,
                    start_step: r.start_step,
                    end_step: r.end_step,
                });
            }

            return acc;
        });

        return Ok(valid_ranges);
	  }

    pub async fn set_motor_mixer(&self, mmix: MotorMixer) -> io::Result<()> {

        let payload = MspSetMotorMixer {
            index: mmix.index,
            motor_mixer: MspMotorMixer {
                throttle: mmix.throttle,
                roll: mmix.roll,
                pitch: mmix.pitch,
                yaw: mmix.yaw,
            }
        };

        let packet = MspPacket {
            cmd: MspCommandCode::MSP2_SET_MOTOR_MIXER as u16,
            direction: MspPacketDirection::ToFlightController,
            data: payload.pack().to_vec(),
        };

        self.core.write(packet).await;

        // TODO: we are not sure this ack is for our request, because there is no id for the request
        let timeout_res = future::timeout(Duration::from_millis(500), self.set_motor_mixer_ack_recv.recv()).await;
        if timeout_res.is_ok() {
            return Ok(timeout_res.unwrap().unwrap());
        }

        return Err(io::Error::new(io::ErrorKind::TimedOut, "timedout waiting for set mode range response"));
	  }

    pub async fn get_motor_mixers(&self) -> io::Result<Vec<MotorMixer>> {
        let packet = MspPacket {
            cmd: MspCommandCode::MSP2_MOTOR_MIXER as u16,
            direction: MspPacketDirection::ToFlightController,
            data: vec![],
        };

        self.core.write(packet).await;

        // TODO: we are not sure this ack is for our request, because there is no id for the request, unlike mavlink packet
        // TODO: what if we are reading packet that was sent long time ago

        let timeout_res = future::timeout(Duration::from_millis(5000), self.motor_mixers_recv.recv()).await;
        if !timeout_res.is_ok() {
            return Err(io::Error::new(io::ErrorKind::TimedOut, "timedout waiting for get motor mixers response"));
        }

        let mmix_replay = timeout_res.unwrap().unwrap();

        let mut valid_mmix = vec![];

        // TODO: not all 20 ranges will be active, return only the active ranges
        mmix_replay.mode_ranges.iter().enumerate().fold(&mut valid_mmix, |acc, (i, m)| {
            if m.throttle != 0 {
                acc.push(MotorMixer {
                    index: i as u8,
                    throttle: m.throttle,
                    roll: m.roll,
                    pitch: m.pitch,
                    yaw: m.yaw,
                });
            }

            return acc;
        });

        return Ok(valid_mmix);
	  }

}
