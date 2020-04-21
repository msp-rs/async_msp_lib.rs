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

#[derive(Debug)]
pub struct SettingInfo {
    pub info: MspSettingInfo,
    pub name: String,
    pub value: Vec<u8>,
    pub enum_names: Vec<String>,
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

    rx_map: (Sender<Vec<u8>>, Receiver<Vec<u8>>),
    set_rx_map_ack: (Sender<()>, Receiver<()>),

    pg_settings: (Sender<Vec<u8>>, Receiver<Vec<u8>>),
    setting_info: (Sender<Vec<u8>>, Receiver<Vec<u8>>),
    set_setting_ack: (Sender<()>, Receiver<()>),

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

            rx_map: channel::<Vec<u8>>(100),
            set_rx_map_ack: channel::<()>(100),

            pg_settings: channel::<Vec<u8>>(100),
            setting_info: channel::<Vec<u8>>(100),
            set_setting_ack: channel::<()>(100),

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

            self.rx_map.0.clone(),
            self.set_rx_map_ack.0.clone(),

            self.pg_settings.0.clone(),
            self.setting_info.0.clone(),
            self.set_setting_ack.0.clone(),

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

        rx_map_send: Sender<Vec<u8>>,
        set_rx_map_ack_send: Sender<()>,

        pg_settings: Sender<Vec<u8>>,
        setting_info: Sender<Vec<u8>>,
        set_setting_ack: Sender<()>,

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

                    Some(MspCommandCode::MSP_RX_MAP) => rx_map_send.send(packet.data).await,
                    Some(MspCommandCode::MSP_SET_RX_MAP) => set_rx_map_ack_send.send(()).await,

                    Some(MspCommandCode::MSP2_COMMON_PG_LIST) => pg_settings.send(packet.data).await,
                    Some(MspCommandCode::MSP2_COMMON_SETTING_INFO) => setting_info.send(packet.data).await,
                    Some(MspCommandCode::MSP2_COMMON_SET_SETTING) => set_setting_ack.send(()).await,

                    Some(MspCommandCode::MSP_DATAFLASH_SUMMARY) => summary_send.send(packet.data).await,

                    Some(MspCommandCode::MSP_DATAFLASH_READ) => chunk_send.send(packet.data).await,

                    _ => (),
                }

                // TODO: listen to msp error message, and notify the appropriate channel. use future.race
                // TODO: create debug(--verbose) flag for additional print on demand
            }
        });
    }

    // TODO: because this is a serial protocol, we cannot allow two reads of the file at the same time.
    //       so throw error, if this function is called while another file is open already
    // TODO: pass start and end as parameters, the cli should call the summary and get all the total size
    /// altought slower then read_flash_data, its a safe data, the reads are happening serialy
    pub async fn open_flash_data(&self) -> FlashDataFile {
        // await for summary
        let summary = self.flash_summary().await;
        let used_size = summary.used_size_bytes;

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
    // TODO: pass start and end as parameters, the cli should call the summary and get all the total size
    /// coccurent unordered data flash reading, this method assumes data is organazised into equal(except last one) chunks of data
    pub async fn read_flash_data(&self, chunk_size: usize, callback: fn(chunk: usize, total: usize)) -> io::Result<Vec<u8>> {
        // await for summary
        let summary = self.flash_summary().await;
        let used_size = summary.used_size_bytes as usize;

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
            // TODO: make it fold async
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

    pub async fn flash_summary(&self) -> MspDataFlashSummaryReply {
        let packet = MspPacket {
            cmd: MspCommandCode::MSP_DATAFLASH_SUMMARY as u16,
            direction: MspPacketDirection::ToFlightController,
            data: vec![],
        };

        self.core.write(packet).await;

        let payload = self.summary.1.recv().await.unwrap();
        let summary = MspDataFlashSummaryReply::unpack_from_slice(&payload).unwrap();

        return summary;
	  }


    /// aux 2 44 6 1300 1400
    /// TOOD: why the cli box id doesn't match what defined in the fc_msp_box.c
    /// because the cli command is working by the mode index in fc_msp_box.c, and not the mode id
    /// configurator ui will actually do the following conversion for box_modes
    /// start_step - ((value - 900) / 25), 
    /// end_step - ((value - 900) / 25),
    ///
    /// steps are 25 apart
    /// a value of 0 corresponds to a channel value of 900 or less
    /// a value of 48 corresponds to a channel value of 2100 or more
    /// inav.set_mode_range(inav_msp_lib::ModeRange {
    ///     index: 2,
    ///     box_id: 53,
    ///     aux_channel_index: 0,
    ///     start_step: 47,
    ///     end_step: 48,
    /// }).await;
    /// will ack with the same command
    /// inav.get_mode_ranges().await
    pub async fn set_mode_range(&self, mode: ModeRange) {
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

        self.set_mode_range_ack.1.recv().await.unwrap();
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
        let payload = self.mode_ranges.1.recv().await.unwrap();

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

    /// inav.set_motor_mixer(inav_msp_lib::MotorMixer {
    ///     index: 0,
    ///     throttle: 1000,
    ///     roll: 3000,
    ///     pitch: 1000,
    ///     yaw: 1000,
    /// }).await;
    /// println!("cli {:?}", inav.get_motor_mixers().await);
    pub async fn set_motor_mixer(&self, mmix: MotorMixer) {
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

        self.set_motor_mixer_ack.1.recv().await.unwrap();
	  }

    pub async fn get_motor_mixers(&self) -> Vec<MotorMixer> {
        let packet = MspPacket {
            cmd: MspCommandCode::MSP2_MOTOR_MIXER as u16,
            direction: MspPacketDirection::ToFlightController,
            data: vec![],
        };

        self.core.write(packet).await;

        let payload = self.motor_mixers.1.recv().await.unwrap();

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

        return mmixers;
	  }

    /// inav.set_osd_config_item(116, multiwii_serial_protocol::structs::OsdItemPosition { col: 11u8, row: 22u8 }).await;
    /// println!("osd {:?}", inav.get_osd_settings().await);
    pub async fn set_osd_config_item(&self, id: u8, item: OsdItemPosition) {
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

        self.set_osd_config_ack.1.recv().await.unwrap();
    }

    pub async fn set_osd_config(&self, config: OsdConfig) {
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

        return self.set_osd_config_ack.1.recv().await.unwrap();
	  }

    pub async fn get_osd_settings(&self) -> OsdSettings {
        let packet = MspPacket {
            cmd: MspCommandCode::MSP_OSD_CONFIG as u16,
            direction: MspPacketDirection::ToFlightController,
            data: vec![],
        };

        self.core.write(packet).await;

        let payload = self.osd_configs.1.recv().await.unwrap();

        let header_len = MspSetGetOsdConfig::packed_bytes();
        let osd_set_get_reply = MspSetGetOsdConfig::unpack_from_slice(&payload[..header_len]).unwrap();

        let mut item_positions = vec![];
        let len = OsdItemPosition::packed_bytes();
        for i in (header_len..payload.len()).step_by(len) {
            let item_pos = OsdItemPosition::unpack_from_slice(&payload[i..i+len]).unwrap();
            item_positions.push(item_pos);
        }

        return OsdSettings {
            osd_support: osd_set_get_reply.item_index,
            config: osd_set_get_reply.config,
            item_positions: item_positions,
        };
	  }

    /// let shitty_serials = vec![multiwii_serial_protocol::structs::SerialSetting {
    ///     index: 6,
    ///     function_mask: 0,
    ///     msp_baudrate_index: 0,
    ///     gps_baudrate_index: 0,
    ///     telemetry_baudrate_index: 0,
    ///     peripheral_baudrate_index: 0
    /// }];
    /// inav.set_serial_settings(shitty_serials).await;
    /// println!("serial {:?}", inav.get_serial_settings().await);
    pub async fn set_serial_settings(&self, serials: Vec<SerialSetting>) {
        let payload = serials.iter().flat_map(|s| s.pack().to_vec()).collect();
        let packet = MspPacket {
            cmd: MspCommandCode::MSP2_SET_SERIAL_CONFIG as u16,
            direction: MspPacketDirection::ToFlightController,
            data: payload,
        };

        self.core.write(packet).await;

        return self.set_serial_settings_ack.1.recv().await.unwrap();
	  }

    pub async fn get_serial_settings(&self) -> Vec<SerialSetting> {
        let packet = MspPacket {
            cmd: MspCommandCode::MSP2_SERIAL_CONFIG as u16,
            direction: MspPacketDirection::ToFlightController,
            data: vec![],
        };

        self.core.write(packet).await;

        let payload = self.serial_settings.1.recv().await.unwrap();

        let mut serials = vec![];
        let len = SerialSetting::packed_bytes();

        for i in (0..payload.len()).step_by(len) {
            let serial_setting = SerialSetting::unpack_from_slice(&payload[i..i+len]).unwrap();
            if serial_setting.index != 0 {
                serials.push(serial_setting);
            }
        }

        return serials;
	  }

    /// let shitty_features = multiwii_serial_protocol::structs::MspFeatures {
    ///     features: [
    ///         true, true, true, true, true, true, true, true,
    ///         true, true, true, true, true, true, true, true,
    ///         true, true, true, true, true, true, true, true,
    ///         true, true, true, true, true, true, true, true,
    ///     ]
    /// };
    /// inav.set_features(shitty_features).await;
    /// println!("features {:?}", inav.get_features().await);
    pub async fn set_features(&self, features: MspFeatures) {
        let packet = MspPacket {
            cmd: MspCommandCode::MSP_SET_FEATURE as u16,
            direction: MspPacketDirection::ToFlightController,
            data: features.pack().to_vec(),
        };

        self.core.write(packet).await;
        return self.set_features_ack.1.recv().await.unwrap();
	  }

    pub async fn get_features(&self) -> MspFeatures {
        let packet = MspPacket {
            cmd: MspCommandCode::MSP_FEATURE as u16,
            direction: MspPacketDirection::ToFlightController,
            data: vec![],
        };

        self.core.write(packet).await;

        let payload = self.features.1.recv().await.unwrap();

        return MspFeatures::unpack_from_slice(&payload).unwrap();
	  }

    pub async fn set_servo_mix_rules(&self, servo_mix_rules: Vec<MspServoMixRule>) {
        let payload = servo_mix_rules.iter().flat_map(|s| s.pack().to_vec()).collect();
        let packet = MspPacket {
            cmd: MspCommandCode::MSP_SET_SERVO_MIX_RULE as u16,
            direction: MspPacketDirection::ToFlightController,
            data: payload,
        };

        self.core.write(packet).await;

        return self.set_servo_mix_rules_ack.1.recv().await.unwrap();
	  }

    /// println!("servo mixers {:?}", inav.get_servo_mix_rules().await);
    pub async fn get_servo_mix_rules(&self) -> Vec<MspServoMixRule> {
        let packet = MspPacket {
            cmd: MspCommandCode::MSP_SERVO_MIX_RULES as u16,
            direction: MspPacketDirection::ToFlightController,
            data: vec![],
        };

        self.core.write(packet).await;

        let payload = self.servo_mix_rules.1.recv().await.unwrap();

        let mut rules = vec![];
        let len = MspServoMixRule::packed_bytes();

        for i in (0..payload.len()).step_by(len) {
            let serial_setting = MspServoMixRule::unpack_from_slice(&payload[i..i+len]).unwrap();
            if serial_setting.index != 0 {
                rules.push(serial_setting);
            }
        }

        return rules;
	  }

    /// inav.set_rx_map_rules(multiwii_serial_protocol::structs::MspRxMap { map: [0,0,0,0]}).await;
    /// println!("features {:?}", inav.get_rx_map_rules().await);
    pub async fn set_rx_map_rules(&self, rx_map: MspRxMap) {
        let packet = MspPacket {
            cmd: MspCommandCode::MSP_SET_RX_MAP as u16,
            direction: MspPacketDirection::ToFlightController,
            data: rx_map.pack().to_vec(),
        };

        self.core.write(packet).await;

        return self.set_rx_map_ack.1.recv().await.unwrap();
	  }

    pub async fn get_rx_map_rules(&self) -> MspRxMap {
        let packet = MspPacket {
            cmd: MspCommandCode::MSP_RX_MAP as u16,
            direction: MspPacketDirection::ToFlightController,
            data: vec![],
        };

        self.core.write(packet).await;

        let payload = self.rx_map.1.recv().await.unwrap();

        return MspRxMap::unpack_from_slice(&payload).unwrap();
	  }


    // TODO: on connection get version from msp, similar to inav configurator and allow to switch features based on this version
    pub fn str_from_u8_nul_utf8(utf8_src: &[u8]) -> Result<&str, std::str::Utf8Error> {
        let nul_range_end = utf8_src.iter()
            .position(|&c| c == b'\0')
            .unwrap_or(utf8_src.len()); // default to length if no `\0` present
        ::std::str::from_utf8(&utf8_src[0..nul_range_end])
    }

    pub async fn set_setting(&self, name: &str, value: Vec<u8>) {
        let mut payload = name.as_bytes().to_vec();
        payload.push(b'\0');
        payload.extend(value);

        // either send \0 then 2 bytes of id
        // or send null terminated string, name of the setting
        // the rest of the data is the value.
        let packet = MspPacket {
            cmd: MspCommandCode::MSP2_COMMON_SET_SETTING as u16,
            direction: MspPacketDirection::ToFlightController,
            data: payload,
        };

        self.core.write(packet).await;
        self.set_setting_ack.1.recv().await.unwrap();
	  }

    pub async fn get_setting_info_by_name(&self, name: &str) -> SettingInfo {
        let mut payload = name.as_bytes().to_vec();
        payload.push(b'\0');

        return self.get_setting_info(payload).await;
    }

    pub async fn get_setting_info_by_id(&self, id: &u16) -> SettingInfo {
        // then we can use MSP2_COMMON_SETTING to get the setting element count
        // if Payload starts with a zero '\0', then it will treat next u16 bytes as setting index
        // then we info where to find the setting in the setting table
        let payload = MspSettingInfoRequest {
            null: 0,
            id: *id
        };

        return self.get_setting_info(payload.pack().to_vec()).await;
    }

    pub async fn get_setting_info(&self, id: Vec<u8>) -> SettingInfo {
        let packet = MspPacket {
            cmd: MspCommandCode::MSP2_COMMON_SETTING_INFO as u16,
            direction: MspPacketDirection::ToFlightController,
            data: id,
        };

        self.core.write(packet).await;

        let payload = self.setting_info.1.recv().await.unwrap();

        let name = INavMsp::str_from_u8_nul_utf8(&payload).unwrap();
        let len = MspSettingInfo::packed_bytes();
        let mut index = name.len() + 1;
        let setting_info = MspSettingInfo::unpack_from_slice(&payload[index..index + len]).unwrap();

        index += len;

        let mut enum_values = vec![];
        if setting_info.setting_mode == SettingMode::ModeLookup {
            for _ in setting_info.min..setting_info.max + 1 {
                let enum_value = INavMsp::str_from_u8_nul_utf8(&payload[index..]).unwrap();
                index += enum_value.len() + 1;
                enum_values.push(enum_value);
            }
        }

        // value in the leftovers
        let value = &payload[index..];

        // TODO: can i get default setting value?

        return SettingInfo {
            name: String::from(name),
            value: value.to_vec(),
            info: setting_info,
            enum_names: enum_values.iter().map(|&s| String::from(s)).collect(),
        };
	  }

    // calling pg list will get all the settings groups list (the PG groups)
    // it will return the start of the group and the end of the group

    // 2 bytes, group id ... 0 is invalid
    // 2 bytes start of the setting index, this is not a memory
    // 2 bytes last setting index, this is not a memroy
    pub async fn get_pg_settings(&self) -> Vec<MspSettingGroup> {
        let packet = MspPacket {
            cmd: MspCommandCode::MSP2_COMMON_PG_LIST as u16,
            direction: MspPacketDirection::ToFlightController,
            data: vec![], // pass nothing to get all settings
        };

        self.core.write(packet).await;

        let payload = self.pg_settings.1.recv().await.unwrap();

        let mut setting_ids = vec![];
        let len = MspSettingGroup::packed_bytes();

        for i in (0..payload.len()).step_by(len) {
            let setting_id = MspSettingGroup::unpack_from_slice(&payload[i..i+len]).unwrap();
            setting_ids.push(setting_id);
        }

        return setting_ids;
	  }
}
