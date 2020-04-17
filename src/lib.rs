extern crate alloc;
extern crate multiwii_serial_protocol;
extern crate serialport;
extern crate packed_struct;

use multiwii_serial_protocol::{MspCommandCode, MspPacket, MspPacketDirection};
use multiwii_serial_protocol::structs::*;
use serialport::SerialPort;
use packed_struct::prelude::*;

use async_std::sync::{channel, Sender, Receiver};
use async_std::{io, task};
use async_std::future;
use std::time::Duration;

mod core;


pub struct MspDataFlashReplyWithData {
    pub read_address: u32,
    pub payload: Vec<u8>,
}

#[derive(Debug)]
pub struct ModeRange {
    pub index: u8,
    pub box_id: u8,
    pub aux_channel_index: u8,
    pub start_step: u8,
    pub end_step: u8,
}


#[derive(Debug)]
pub struct MotorMixer {
    pub index: u8,
    pub throttle: u16,
    pub roll: u16,
    pub pitch: u16,
    pub yaw: u16,
}

pub struct FlashDataFile {
    core: core::Core,
    chunk_recv: Receiver<Vec<u8>>,
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
                    Some(payload) => {
                        let packet = INavMsp::parse_chunk(payload);

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

// TODO: we should pass by reference in channel, or Vec<u8> reference
pub struct INavMsp {
    core: core::Core,

    mode_ranges: (Sender<Vec<u8>>, Receiver<Vec<u8>>),
    set_mode_range_ack: (Sender<()>, Receiver<()>),

    motor_mixers: (Sender<Vec<u8>>,Receiver<Vec<u8>>),
    set_motor_mixer_ack: (Sender<()>, Receiver<()>),

    osd_configs: (Sender<Vec<u8>>, Receiver<Vec<u8>>),
    set_osd_config_ack: (Sender<()>,Receiver<()>),

    serial_settings: (Sender<Vec<u8>>, Receiver<Vec<u8>>),
    set_serial_settings_ack: (Sender<()>, Receiver<()>),

    features: (Sender<Vec<u8>>, Receiver<Vec<u8>>),
    set_features_ack: (Sender<()>, Receiver<()>),

    servo_mix_rules: (Sender<Vec<u8>>, Receiver<Vec<u8>>),
    set_servo_mix_rules_ack: (Sender<()>, Receiver<()>),

    summary: (Sender<Vec<u8>>, Receiver<Vec<u8>>),

    chunk: (Sender<Vec<u8>>, Receiver<Vec<u8>>),
}

impl INavMsp {
    // Create a new parserSerialPort
    pub fn new() -> INavMsp {
        let core = core::Core::new();

        return INavMsp {
            core: core,

            mode_ranges: channel::<Vec<u8>>(100),
            set_mode_range_ack: channel::<()>(100),

            motor_mixers: channel::<Vec<u8>>(100),
            set_motor_mixer_ack: channel::<()>(100),

            osd_configs: channel::<Vec<u8>>(100),
            set_osd_config_ack: channel::<()>(100),

            serial_settings: channel::<Vec<u8>>(100),
            set_serial_settings_ack: channel::<()>(100),

            features: channel::<Vec<u8>>(100),
            set_features_ack: channel::<()>(100),

            servo_mix_rules: channel::<Vec<u8>>(100),
            set_servo_mix_rules_ack: channel::<()>(100),

            summary: channel::<Vec<u8>>(100),

            chunk: channel::<Vec<u8>>(4096),
        };
	  }

    // TODO: If serial-port rs supports standard read write interface we should use this instead of seril explocitly
    pub fn start(&self, serial: Box<dyn SerialPort>) {
        &self.core.start(serial);

        INavMsp::process_route(
            self.core.clone(),

            self.mode_ranges.0.clone(),
            self.set_mode_range_ack.0.clone(),

            self.motor_mixers.0.clone(),
            self.set_motor_mixer_ack.0.clone(),

            self.osd_configs.0.clone(),
            self.set_osd_config_ack.0.clone(),

            self.serial_settings.0.clone(),
            self.set_serial_settings_ack.0.clone(),

            self.features.0.clone(),
            self.set_features_ack.0.clone(),

            self.servo_mix_rules.0.clone(),
            self.set_servo_mix_rules_ack.0.clone(),

            self.summary.0.clone(),
            self.chunk.0.clone(),
        );
    }

    fn process_route(
        core: core::Core,

        mode_ranges_send: Sender<Vec<u8>>,
        set_mode_range_ack_send: Sender<()>,

        motor_mixers_send: Sender<Vec<u8>>,
        set_motor_mixer_ack_send: Sender<()>,

        osd_configs_send: Sender<Vec<u8>>,
        set_osd_config_ack_send: Sender<()>,

        serial_settings_send: Sender<Vec<u8>>,
        set_serial_settings_ack_send: Sender<()>,

        features_send: Sender<Vec<u8>>,
        set_features_ack_send: Sender<()>,

        servo_mix_rules_send: Sender<Vec<u8>>,
        set_servo_mix_rules_ack_send: Sender<()>,

        summary_send: Sender<Vec<u8>>,

        chunk_send: Sender<Vec<u8>>,
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

                let cmd = MspCommandCode::from_primitive(packet.cmd);

                match cmd {
                    Some(MspCommandCode::MSP_MODE_RANGES) => mode_ranges_send.send(packet.data).await,
                    Some(MspCommandCode::MSP_SET_MODE_RANGE) => set_mode_range_ack_send.send(()).await,

                    Some(MspCommandCode::MSP2_MOTOR_MIXER) => motor_mixers_send.send(packet.data).await,
                    Some(MspCommandCode::MSP2_SET_MOTOR_MIXER) => set_motor_mixer_ack_send.send(()).await,

                    Some(MspCommandCode::MSP_OSD_CONFIG) => osd_configs_send.send(packet.data).await,
                    Some(MspCommandCode::MSP_SET_OSD_CONFIG) => set_osd_config_ack_send.send(()).await,

                    Some(MspCommandCode::MSP2_SERIAL_CONFIG) => serial_settings_send.send(packet.data).await,
                    Some(MspCommandCode::MSP2_SET_SERIAL_CONFIG) => set_serial_settings_ack_send.send(()).await,

                    Some(MspCommandCode::MSP_FEATURE) => features_send.send(packet.data).await,
                    Some(MspCommandCode::MSP_SET_FEATURE) => set_features_ack_send.send(()).await,

                    Some(MspCommandCode::MSP_SERVO_MIX_RULES) => servo_mix_rules_send.send(packet.data).await,
                    Some(MspCommandCode::MSP_SET_SERVO_MIX_RULE) => set_servo_mix_rules_ack_send.send(()).await,

                    Some(MspCommandCode::MSP_DATAFLASH_SUMMARY) => summary_send.send(packet.data).await,

                    Some(MspCommandCode::MSP_DATAFLASH_READ) => chunk_send.send(packet.data).await,

                    _ => (),
                }
                // TODO: create debug(--verbose) flag for additional print on demand
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
            chunk_recv: self.chunk.1.clone(),
            used_size: used_size,
            next_address: 0u32,
            received_address: 0u32,
        };
	  }

    pub fn parse_chunk(payload: Vec<u8>) -> MspDataFlashReplyWithData {
        // extract the read address from the packet
        let flash_reply = MspDataFlashReply::unpack_from_slice(&payload[..4]).unwrap();

        // remove the last address bytes and send to remaning payload to file stream(stdout)
        let packet_payload = &payload[4..];

        return MspDataFlashReplyWithData {
            read_address: flash_reply.read_address,
            payload: packet_payload.to_vec(),
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
                let timeout_res = future::timeout(Duration::from_millis(500), self.chunk.1.recv()).await;

                if !timeout_res.is_ok() {
                    self.core.reset_parser().await;
                    // println!("timeout, address left {:?}", expected_address);
                    break;
                }

                match timeout_res.unwrap() {
                    None => return Err(io::Error::new(io::ErrorKind::ConnectionAborted, "device disconnected")),
                    Some(payload) => {
                        let packet = INavMsp::parse_chunk(payload);

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
                            return Ok(buff);
                        }
                    }
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

        let timeout_res = future::timeout(Duration::from_millis(500), self.summary.1.recv()).await;

        if !timeout_res.is_ok() {
            return Err(io::Error::new(io::ErrorKind::TimedOut, "timedout waiting for summary response"));
        }

        let payload = timeout_res.unwrap().unwrap();
        let summary = MspDataFlashSummaryReply::unpack_from_slice(&payload).unwrap();

        return Ok(summary);
	  }

    pub async fn set_mode_range(&self, mode: ModeRange) -> io::Result<()> {

        let payload = MspSetModeRange {
            index: mode.index,
            mode_range: MspModeRange {
                box_id: mode.box_id,
                aux_channel_index: MspRcChannel::from_primitive(mode.aux_channel_index).unwrap(),
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

        let timeout_res = future::timeout(Duration::from_millis(500), self.set_mode_range_ack.1.recv()).await;
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

        let timeout_res = future::timeout(Duration::from_millis(5000), self.mode_ranges.1.recv()).await;
        if !timeout_res.is_ok() {
            return Err(io::Error::new(io::ErrorKind::TimedOut, "timedout waiting for get mode ranges response"));
        }

        let payload = timeout_res.unwrap().unwrap();

        let mut ranges = vec![];
        let len = MspModeRange::packed_bytes();
        for i in (0..payload.len()).step_by(len) {
            let r = MspModeRange::unpack_from_slice(&payload[i..i+len]).unwrap();

            ranges.push(ModeRange {
                index: (i / len) as u8,
                box_id: r.box_id,
                aux_channel_index: r.aux_channel_index.to_primitive(),
                start_step: r.start_step,
                end_step: r.end_step,
            });
        }

        return Ok(ranges);
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

        let timeout_res = future::timeout(Duration::from_millis(500), self.set_motor_mixer_ack.1.recv()).await;
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

        let timeout_res = future::timeout(Duration::from_millis(5000), self.motor_mixers.1.recv()).await;
        if !timeout_res.is_ok() {
            return Err(io::Error::new(io::ErrorKind::TimedOut, "timedout waiting for get motor mixers response"));
        }

        let payload = timeout_res.unwrap().unwrap();

        let mut mmixers = vec![];
        let len = MspMotorMixer::packed_bytes();

        for i in (0..payload.len()).step_by(len) {
            let m = MspMotorMixer::unpack_from_slice(&payload[i..i+len]).unwrap();
            if m.throttle != 0 {
                mmixers.push(MotorMixer {
                    index: (i / len) as u8,
                    throttle: m.throttle,
                    roll: m.roll,
                    pitch: m.pitch,
                    yaw: m.yaw,
                });
            }
        }

        return Ok(mmixers);
	  }

    pub async fn set_osd_config_item(&self, id: u8, item: OsdItemPosition) -> io::Result<()> {
        let payload = MspSetOsdLayout {
            item_index: id,
            item: item,
        };

        let packet = MspPacket {
            cmd: MspCommandCode::MSP_SET_OSD_CONFIG as u16,
            direction: MspPacketDirection::ToFlightController,
            data: payload.pack().to_vec(),
        };

        self.core.write(packet).await;

        let timeout_res = future::timeout(Duration::from_millis(500), self.set_osd_config_ack.1.recv()).await;
        if timeout_res.is_ok() {
            return Ok(timeout_res.unwrap().unwrap());
        }

        return Err(io::Error::new(io::ErrorKind::TimedOut, "timedout waiting for set osd layout response"));

    }

    pub async fn set_osd_config(&self, config: OsdConfig) -> io::Result<()> {
        // if -1 will set different kinds of configurations else the laytout id
        // but when fetching it always returns everything with correct osd_support
        // so it seems to set everything we need to call it twice, once with -1 and then with the real value
        let payload = MspSetGetOsdConfig {
            item_index: 0xffu8,
            config: config,
        };

        let packet = MspPacket {
            cmd: MspCommandCode::MSP_SET_OSD_CONFIG as u16,
            direction: MspPacketDirection::ToFlightController,
            data: payload.pack().to_vec(),
        };

        self.core.write(packet).await;

        let timeout_res = future::timeout(Duration::from_millis(500), self.set_osd_config_ack.1.recv()).await;
        if timeout_res.is_ok() {
            return Ok(timeout_res.unwrap().unwrap());
        }

        return Err(io::Error::new(io::ErrorKind::TimedOut, "timedout waiting for set osd layout response"));
	  }

    pub async fn get_osd_settings(&self) -> io::Result<OsdSettings> {
        let packet = MspPacket {
            cmd: MspCommandCode::MSP_OSD_CONFIG as u16,
            direction: MspPacketDirection::ToFlightController,
            data: vec![],
        };

        self.core.write(packet).await;

        let timeout_res = future::timeout(Duration::from_millis(5000), self.osd_configs.1.recv()).await;
        if !timeout_res.is_ok() {
            return Err(io::Error::new(io::ErrorKind::TimedOut, "timedout waiting for get motor mixers response"));
        }

        let payload = timeout_res.unwrap().unwrap();

        let header_len = MspSetGetOsdConfig::packed_bytes();
        let osd_set_get_reply = MspSetGetOsdConfig::unpack_from_slice(&payload[..header_len]).unwrap();

        let mut item_positions = vec![];
        let len = OsdItemPosition::packed_bytes();
        for i in (header_len..payload.len()).step_by(len) {
            let item_pos = OsdItemPosition::unpack_from_slice(&payload[i..i+len]).unwrap();
            item_positions.push(item_pos);
        }

        return Ok(OsdSettings {
            osd_support: osd_set_get_reply.item_index,
            config: osd_set_get_reply.config,
            item_positions: item_positions,
        });
	  }

    pub async fn set_serial_settings(&self, serials: Vec<SerialSetting>) -> io::Result<()> {
        let payload = serials.iter().flat_map(|s| s.pack().to_vec()).collect();
        let packet = MspPacket {
            cmd: MspCommandCode::MSP2_SET_SERIAL_CONFIG as u16,
            direction: MspPacketDirection::ToFlightController,
            data: payload,
        };

        self.core.write(packet).await;

        let timeout_res = future::timeout(Duration::from_millis(500), self.set_serial_settings_ack.1.recv()).await;
        if timeout_res.is_ok() {
            return Ok(timeout_res.unwrap().unwrap());
        }

        return Err(io::Error::new(io::ErrorKind::TimedOut, "timedout waiting for set serial settings response"));
	  }

    pub async fn get_serial_settings(&self) -> io::Result<Vec<SerialSetting>> {
        let packet = MspPacket {
            cmd: MspCommandCode::MSP2_SERIAL_CONFIG as u16,
            direction: MspPacketDirection::ToFlightController,
            data: vec![],
        };

        self.core.write(packet).await;

        let timeout_res = future::timeout(Duration::from_millis(5000), self.serial_settings.1.recv()).await;
        if !timeout_res.is_ok() {
            return Err(io::Error::new(io::ErrorKind::TimedOut, "timedout waiting for get serial settings response"));
        }

        let payload = timeout_res.unwrap().unwrap();

        let mut serials = vec![];
        let len = SerialSetting::packed_bytes();

        for i in (0..payload.len()).step_by(len) {
            let serial_setting = SerialSetting::unpack_from_slice(&payload[i..i+len]).unwrap();
            if serial_setting.index != 0 {
                serials.push(serial_setting);
            }
        }

        return Ok(serials);
	  }

    pub async fn set_features(&self, features: MspFeatures) -> io::Result<()> {
        // let payload = serials.iter().flat_map(|s| s.pack().to_vec()).collect();
        let packet = MspPacket {
            cmd: MspCommandCode::MSP_SET_FEATURE as u16,
            direction: MspPacketDirection::ToFlightController,
            data: features.pack().to_vec(),
        };

        self.core.write(packet).await;

        let timeout_res = future::timeout(Duration::from_millis(500), self.set_features_ack.1.recv()).await;
        if timeout_res.is_ok() {
            return Ok(timeout_res.unwrap().unwrap());
        }

        return Err(io::Error::new(io::ErrorKind::TimedOut, "timedout waiting for set features response"));
	  }

    pub async fn get_features(&self) -> io::Result<MspFeatures> {
        let packet = MspPacket {
            cmd: MspCommandCode::MSP_FEATURE as u16,
            direction: MspPacketDirection::ToFlightController,
            data: vec![],
        };

        self.core.write(packet).await;

        let timeout_res = future::timeout(Duration::from_millis(5000), self.features.1.recv()).await;
        if !timeout_res.is_ok() {
            return Err(io::Error::new(io::ErrorKind::TimedOut, "timedout waiting for get features response"));
        }

        let payload = timeout_res.unwrap().unwrap();

        return Ok(MspFeatures::unpack_from_slice(&payload).unwrap());
	  }

    pub async fn set_servo_mix_rules(&self, servo_mix_rules: Vec<MspServoMixRule>) -> io::Result<()> {
        let payload = servo_mix_rules.iter().flat_map(|s| s.pack().to_vec()).collect();
        let packet = MspPacket {
            cmd: MspCommandCode::MSP_SET_SERVO_MIX_RULE as u16,
            direction: MspPacketDirection::ToFlightController,
            data: payload,
        };

        self.core.write(packet).await;

        let timeout_res = future::timeout(Duration::from_millis(500), self.set_servo_mix_rules_ack.1.recv()).await;
        if timeout_res.is_ok() {
            return Ok(timeout_res.unwrap().unwrap());
        }

        return Err(io::Error::new(io::ErrorKind::TimedOut, "timedout waiting for set servo mix response"));
	  }

    pub async fn get_servo_mix_rules(&self) -> io::Result<Vec<MspServoMixRule>> {
        let packet = MspPacket {
            cmd: MspCommandCode::MSP_SERVO_MIX_RULES as u16,
            direction: MspPacketDirection::ToFlightController,
            data: vec![],
        };

        self.core.write(packet).await;

        let timeout_res = future::timeout(Duration::from_millis(5000), self.servo_mix_rules.1.recv()).await;
        if !timeout_res.is_ok() {
            return Err(io::Error::new(io::ErrorKind::TimedOut, "timedout waiting for get servo mixer rules response"));
        }

        let payload = timeout_res.unwrap().unwrap();

        let mut rules = vec![];
        let len = MspServoMixRule::packed_bytes();

        for i in (0..payload.len()).step_by(len) {
            let serial_setting = MspServoMixRule::unpack_from_slice(&payload[i..i+len]).unwrap();
            if serial_setting.index != 0 {
                rules.push(serial_setting);
            }
        }

        return Ok(rules);
	  }
}
