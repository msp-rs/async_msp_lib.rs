extern crate alloc;
extern crate multiwii_serial_protocol_v2;
extern crate serialport;
extern crate packed_struct;

use multiwii_serial_protocol_v2::{MspCommandCode, MspPacket, MspPacketDirection};
use multiwii_serial_protocol_v2::structs::*;
use packed_struct::prelude::*;

use async_std::sync::{channel, Sender, Receiver};
use async_std::{io, task};
use async_std::future;
use std::time::Duration;
use std::convert::TryInto;
use futures::future::try_join_all;

mod core;


pub struct MspDataFlashReplyWithData {
    pub read_address: u32,
    pub payload: Vec<u8>,
}

pub struct FlashDataFile {
    core: core::Core,
    chunk_recv: Receiver<Result<Vec<u8>, ()>>,
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

impl SettingInfo {
    pub fn setting_to_vec(&self, value: &str) -> Result<Vec<u8>, &str> {
        return match self.info.setting_type {
            SettingType::VarUint8 => {
                if self.info.setting_mode == SettingMode::ModeLookup {
                    let enum_name = String::from(value).to_uppercase();
                    let index = self.enum_names.iter().position(|r| r == &enum_name);
                    return match index {
                        Some(i) => Ok((i as u8).to_le_bytes().to_vec()),
                        None => {
                            eprintln!("Failed to find {} in {}", enum_name, self.enum_names.join(","));
                            return Err("Failed to find table value");
                        }
                    }
                }


                return match value.parse::<u8>() {
                    Ok(val) => Ok(val.to_le_bytes().to_vec()),
                    _ => Err("Failed to parse"),
                };
            },
            SettingType::VarInt8 => Ok(value.parse::<i8>().unwrap().to_le_bytes().to_vec()),
            SettingType::VarUint16 => Ok(value.parse::<u16>().unwrap().to_le_bytes().to_vec()),
            SettingType::VarInt16 => Ok(value.parse::<i16>().unwrap().to_le_bytes().to_vec()),
            SettingType::VarUint32 => Ok(value.parse::<u32>().unwrap().to_le_bytes().to_vec()),
            #[cfg(feature="suppport_int32_setting_type")]
            SettingType::VarInt32 => Ok(value.parse::<i32>().unwrap().to_le_bytes().to_vec()),
            SettingType::VarFloat => Ok(value.parse::<f32>().unwrap().to_le_bytes().to_vec()),
            SettingType::VarString => Ok(value.as_bytes().to_vec()),
        };
    }
}

impl From<&SettingInfo> for String {
    fn from(s: &SettingInfo) -> Self {
        match s.info.setting_type {
            SettingType::VarUint8 => {
                let (int_bytes, _rest) = s.value.split_at(std::mem::size_of::<u8>());
                let val = u8::from_le_bytes(int_bytes.try_into().unwrap());
                if s.info.setting_mode == SettingMode::ModeLookup {
                    return s.enum_names[val as usize].to_string();
                }
                return val.to_string();
            },
            SettingType::VarInt8 => {
                let (int_bytes, _rest) = s.value.split_at(std::mem::size_of::<i8>());
                return i8::from_le_bytes(int_bytes.try_into().unwrap()).to_string();
            }
            SettingType::VarUint16 => {
                let (int_bytes, _rest) = s.value.split_at(std::mem::size_of::<u16>());
                return u16::from_le_bytes(int_bytes.try_into().unwrap()).to_string();
            }
            SettingType::VarInt16 => {
                let (int_bytes, _rest) = s.value.split_at(std::mem::size_of::<i16>());
                return i16::from_le_bytes(int_bytes.try_into().unwrap()).to_string();
            }
            SettingType::VarUint32 => {
                let (int_bytes, _rest) = s.value.split_at(std::mem::size_of::<u32>());
                return u32::from_le_bytes(int_bytes.try_into().unwrap()).to_string();
            }
            #[cfg(feature="suppport_int32_setting_type")]
            SettingType::VarInt32 => {
                let (int_bytes, _rest) = s.value.split_at(std::mem::size_of::<i32>());
                return i32::from_le_bytes(int_bytes.try_into().unwrap()).to_string();
            }
            SettingType::VarFloat => {
                let (int_bytes, _rest) = s.value.split_at(std::mem::size_of::<f32>());
                return f32::from_le_bytes(int_bytes.try_into().unwrap()).to_string();
            }
            SettingType::VarString => INavMsp::str_from_u8_nul_utf8(&s.value).unwrap().to_owned(),
        }
    }
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

                let packet = MspPacket {
                    cmd: MspCommandCode::MSP_DATAFLASH_READ as u16,
                    direction: MspPacketDirection::ToFlightController,
                    data: packed.to_vec(),
                };

                self.core.write(packet).await;
            }

            let timeout_res = future::timeout(Duration::from_millis(50), self.chunk_recv.recv()).await;

            // resend the packet
            if timeout_res.is_ok() {
                match timeout_res.unwrap() {
                    Err(_) => return Err(io::Error::new(io::ErrorKind::ConnectionAborted, "device disconnected")),
                    Ok(payload) => {
                        let packet = INavMsp::parse_chunk(payload.unwrap());

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

    mode_ranges: (Sender<Result<Vec<u8>, ()>>, Receiver<Result<Vec<u8>, ()>>),
    set_mode_range_ack: (Sender<Result<Vec<u8>, ()>>, Receiver<Result<Vec<u8>, ()>>),

    motor_mixers: (Sender<Result<Vec<u8>, ()>>,Receiver<Result<Vec<u8>, ()>>),
    set_motor_mixer_ack: (Sender<Result<Vec<u8>, ()>>, Receiver<Result<Vec<u8>, ()>>),

    osd_configs: (Sender<Result<Vec<u8>, ()>>, Receiver<Result<Vec<u8>, ()>>),
    set_osd_config_ack: (Sender<Result<Vec<u8>, ()>>,Receiver<Result<Vec<u8>, ()>>),
    osd_layout_count: (Sender<Result<Vec<u8>, ()>>, Receiver<Result<Vec<u8>, ()>>),
    osd_layout_items: (Sender<Result<Vec<u8>, ()>>, Receiver<Result<Vec<u8>, ()>>),
    set_osd_layout_item_ack: (Sender<Result<Vec<u8>, ()>>, Receiver<Result<Vec<u8>, ()>>),

    serial_settings: (Sender<Result<Vec<u8>, ()>>, Receiver<Result<Vec<u8>, ()>>),
    set_serial_settings_ack: (Sender<Result<Vec<u8>, ()>>, Receiver<Result<Vec<u8>, ()>>),

    features: (Sender<Result<Vec<u8>, ()>>, Receiver<Result<Vec<u8>, ()>>),
    set_features_ack: (Sender<Result<Vec<u8>, ()>>, Receiver<Result<Vec<u8>, ()>>),

    servo_mix_rules: (Sender<Result<Vec<u8>, ()>>, Receiver<Result<Vec<u8>, ()>>),
    set_servo_mix_rules_ack: (Sender<Result<Vec<u8>, ()>>, Receiver<Result<Vec<u8>, ()>>),

    servo_configs: (Sender<Result<Vec<u8>, ()>>, Receiver<Result<Vec<u8>, ()>>),
    set_servo_configs_ack: (Sender<Result<Vec<u8>, ()>>, Receiver<Result<Vec<u8>, ()>>),

    servo_mixer: (Sender<Result<Vec<u8>, ()>>, Receiver<Result<Vec<u8>, ()>>),
    set_servo_mixer_ack: (Sender<Result<Vec<u8>, ()>>, Receiver<Result<Vec<u8>, ()>>),

    rx_map: (Sender<Result<Vec<u8>, ()>>, Receiver<Result<Vec<u8>, ()>>),
    set_rx_map_ack: (Sender<Result<Vec<u8>, ()>>, Receiver<Result<Vec<u8>, ()>>),

    pg_settings: (Sender<Result<Vec<u8>, ()>>, Receiver<Result<Vec<u8>, ()>>),
    setting_info: (Sender<Result<Vec<u8>, ()>>, Receiver<Result<Vec<u8>, ()>>),
    set_setting_ack: (Sender<Result<Vec<u8>, ()>>, Receiver<Result<Vec<u8>, ()>>),

    write_char_ack: (Sender<Result<Vec<u8>, ()>>, Receiver<Result<Vec<u8>, ()>>),

    set_raw_rc_ack: (Sender<Result<Vec<u8>, ()>>, Receiver<Result<Vec<u8>, ()>>),

    summary: (Sender<Result<Vec<u8>, ()>>, Receiver<Result<Vec<u8>, ()>>),
    chunk: (Sender<Result<Vec<u8>, ()>>, Receiver<Result<Vec<u8>, ()>>),

    reset_conf_ack: (Sender<Result<Vec<u8>, ()>>, Receiver<Result<Vec<u8>, ()>>),

    write_eeprom_ack: (Sender<Result<Vec<u8>, ()>>, Receiver<Result<Vec<u8>, ()>>),
}

impl INavMsp {
    // Create a new parserSerialPort
    pub fn new(buff_size: usize, write_delay: Duration, verbose: bool) -> INavMsp {
        let core = core::Core::new(buff_size, write_delay, verbose);

        return INavMsp {
            core,

            mode_ranges: channel::<Result<Vec<u8>, ()>>(100),
            set_mode_range_ack: channel::<Result<Vec<u8>, ()>>(100),

            motor_mixers: channel::<Result<Vec<u8>, ()>>(100),
            set_motor_mixer_ack: channel::<Result<Vec<u8>, ()>>(100),

            osd_configs: channel::<Result<Vec<u8>, ()>>(100),
            set_osd_config_ack: channel::<Result<Vec<u8>, ()>>(100),
            osd_layout_count: channel::<Result<Vec<u8>, ()>>(100),
            osd_layout_items: channel::<Result<Vec<u8>, ()>>(100),
            set_osd_layout_item_ack: channel::<Result<Vec<u8>, ()>>(100),

            serial_settings: channel::<Result<Vec<u8>, ()>>(100),
            set_serial_settings_ack: channel::<Result<Vec<u8>, ()>>(100),

            features: channel::<Result<Vec<u8>, ()>>(100),
            set_features_ack: channel::<Result<Vec<u8>, ()>>(100),

            servo_mix_rules: channel::<Result<Vec<u8>, ()>>(100),
            set_servo_mix_rules_ack: channel::<Result<Vec<u8>, ()>>(100),

            servo_mixer: channel::<Result<Vec<u8>, ()>>(100),
            set_servo_mixer_ack: channel::<Result<Vec<u8>, ()>>(100),

            servo_configs: channel::<Result<Vec<u8>, ()>>(100),
            set_servo_configs_ack: channel::<Result<Vec<u8>, ()>>(100),

            rx_map: channel::<Result<Vec<u8>, ()>>(100),
            set_rx_map_ack: channel::<Result<Vec<u8>, ()>>(100),

            pg_settings: channel::<Result<Vec<u8>, ()>>(100),
            setting_info: channel::<Result<Vec<u8>, ()>>(100),
            set_setting_ack: channel::<Result<Vec<u8>, ()>>(100),

            write_char_ack: channel::<Result<Vec<u8>, ()>>(100),

            set_raw_rc_ack: channel::<Result<Vec<u8>, ()>>(100),

            summary: channel::<Result<Vec<u8>, ()>>(100),
            chunk: channel::<Result<Vec<u8>, ()>>(4096),

            reset_conf_ack: channel::<Result<Vec<u8>, ()>>(1),

            write_eeprom_ack: channel::<Result<Vec<u8>, ()>>(1),
        };
    }

    // TODO: If serial-port rs supports standard read write interface we should use this instead of seril explocitly
    pub fn start(&self, stream: impl Send + std::io::Read + std::io::Write + Clone + 'static) {
        &self.core.start(stream);

        if &self.core.buff_size() == &0 {
            return;
        }

        INavMsp::process_route(
            self.core.clone(),

            self.mode_ranges.0.clone(),
            self.set_mode_range_ack.0.clone(),

            self.motor_mixers.0.clone(),
            self.set_motor_mixer_ack.0.clone(),

            self.osd_configs.0.clone(),
            self.set_osd_config_ack.0.clone(),
            self.osd_layout_count.0.clone(),
            self.osd_layout_items.0.clone(),
            self.set_osd_layout_item_ack.0.clone(),

            self.serial_settings.0.clone(),
            self.set_serial_settings_ack.0.clone(),

            self.features.0.clone(),
            self.set_features_ack.0.clone(),

            self.servo_mix_rules.0.clone(),
            self.set_servo_mix_rules_ack.0.clone(),

            self.servo_mixer.0.clone(),
            self.set_servo_mixer_ack.0.clone(),

            self.servo_configs.0.clone(),
            self.set_servo_configs_ack.0.clone(),

            self.rx_map.0.clone(),
            self.set_rx_map_ack.0.clone(),

            self.pg_settings.0.clone(),
            self.setting_info.0.clone(),
            self.set_setting_ack.0.clone(),

            self.write_char_ack.0.clone(),

            self.set_raw_rc_ack.0.clone(),

            self.summary.0.clone(),
            self.chunk.0.clone(),

            self.reset_conf_ack.0.clone(),

            self.write_eeprom_ack.0.clone(),
        );
    }

    fn process_route(
        core: core::Core,

        mode_ranges_send: Sender<Result<Vec<u8>, ()>>,
        set_mode_range_ack_send: Sender<Result<Vec<u8>, ()>>,

        motor_mixers_send: Sender<Result<Vec<u8>, ()>>,
        set_motor_mixer_ack_send: Sender<Result<Vec<u8>, ()>>,

        osd_configs_send: Sender<Result<Vec<u8>, ()>>,
        set_osd_config_ack_send: Sender<Result<Vec<u8>, ()>>,
        osd_layout_count_send: Sender<Result<Vec<u8>, ()>>,
        osd_layout_items_send: Sender<Result<Vec<u8>, ()>>,
        set_osd_layout_item_send_ack: Sender<Result<Vec<u8>, ()>>,

        serial_settings_send: Sender<Result<Vec<u8>, ()>>,
        set_serial_settings_ack_send: Sender<Result<Vec<u8>, ()>>,

        features_send: Sender<Result<Vec<u8>, ()>>,
        set_features_ack_send: Sender<Result<Vec<u8>, ()>>,

        servo_mix_rules_send: Sender<Result<Vec<u8>, ()>>,
        set_servo_mix_rules_ack_send: Sender<Result<Vec<u8>, ()>>,

        servo_mixer_send: Sender<Result<Vec<u8>, ()>>,
        set_servo_mixer_ack_send: Sender<Result<Vec<u8>, ()>>,

        servo_configs_send: Sender<Result<Vec<u8>, ()>>,
        set_servo_configs_ack_send: Sender<Result<Vec<u8>, ()>>,

        rx_map_send: Sender<Result<Vec<u8>, ()>>,
        set_rx_map_ack_send: Sender<Result<Vec<u8>, ()>>,

        pg_settings: Sender<Result<Vec<u8>, ()>>,
        setting_info: Sender<Result<Vec<u8>, ()>>,
        set_setting_ack: Sender<Result<Vec<u8>, ()>>,

        write_char_ack: Sender<Result<Vec<u8>, ()>>,

        set_raw_rc_ack: Sender<Result<Vec<u8>, ()>>,

        summary_send: Sender<Result<Vec<u8>, ()>>,
        chunk_send: Sender<Result<Vec<u8>, ()>>,

        reset_conf_ack: Sender<Result<Vec<u8>, ()>>,

        write_eeprom_ack: Sender<Result<Vec<u8>, ()>>,
    ) {
        task::spawn(async move {
            loop {
                let packet = match core.read().await {
                    None => break,
                    Some(packet) => packet,
                };

                // println!("process route");

                let cmd = MspCommandCode::from_primitive(packet.cmd);
                let packet_length = packet.data.len();

                let result = match packet.direction {
                    MspPacketDirection::FromFlightController => Ok(packet.data),
                    MspPacketDirection::Unsupported => {
                        println!("got unsupported msp direction {:?}", packet);
                        Err(())
                    },
                    _ => continue,
                };

                let channel = match cmd {
                    Some(MspCommandCode::MSP_MODE_RANGES) => &mode_ranges_send,
                    Some(MspCommandCode::MSP_SET_MODE_RANGE) => &set_mode_range_ack_send,

                    Some(MspCommandCode::MSP2_MOTOR_MIXER) => &motor_mixers_send,
                    Some(MspCommandCode::MSP2_SET_MOTOR_MIXER) => &set_motor_mixer_ack_send,

                    Some(MspCommandCode::MSP_OSD_CONFIG) => &osd_configs_send,
                    Some(MspCommandCode::MSP_SET_OSD_CONFIG) => &set_osd_config_ack_send,
                    // if payload size in osd_layout_items is bigger then 2 bytes its means we got the count
                    // else we got list of items in the layout
                    Some(MspCommandCode::MSP2_INAV_OSD_LAYOUTS) => if packet_length > 2 { &osd_layout_items_send } else { &osd_layout_count_send },
                    Some(MspCommandCode::MSP2_INAV_OSD_SET_LAYOUT_ITEM) => &set_osd_layout_item_send_ack,

                    Some(MspCommandCode::MSP2_SERIAL_CONFIG) => &serial_settings_send,
                    Some(MspCommandCode::MSP2_SET_SERIAL_CONFIG) => &set_serial_settings_ack_send,

                    Some(MspCommandCode::MSP_FEATURE) => &features_send,
                    Some(MspCommandCode::MSP_SET_FEATURE) => &set_features_ack_send,

                    Some(MspCommandCode::MSP_SERVO_MIX_RULES) => &servo_mix_rules_send,
                    Some(MspCommandCode::MSP_SET_SERVO_MIX_RULE) => &set_servo_mix_rules_ack_send,

                    Some(MspCommandCode::MSP2_INAV_SERVO_MIXER) => &servo_mixer_send,
                    Some(MspCommandCode::MSP2_INAV_SET_SERVO_MIXER) => &set_servo_mixer_ack_send,

                    Some(MspCommandCode::MSP_SERVO_CONFIGURATIONS) => &servo_configs_send,
                    Some(MspCommandCode::MSP_SET_SERVO_CONFIGURATION) => &set_servo_configs_ack_send,

                    Some(MspCommandCode::MSP_RX_MAP) => &rx_map_send,
                    Some(MspCommandCode::MSP_SET_RX_MAP) => &set_rx_map_ack_send,

                    Some(MspCommandCode::MSP2_COMMON_PG_LIST) => &pg_settings,
                    Some(MspCommandCode::MSP2_COMMON_SETTING_INFO) => &setting_info,
                    Some(MspCommandCode::MSP2_COMMON_SET_SETTING) => &set_setting_ack,

                    Some(MspCommandCode::MSP_OSD_CHAR_WRITE) => &write_char_ack,

                    Some(MspCommandCode::MSP_SET_RAW_RC) => &set_raw_rc_ack,

                    Some(MspCommandCode::MSP_DATAFLASH_SUMMARY) => &summary_send,
                    Some(MspCommandCode::MSP_DATAFLASH_READ) => &chunk_send,

                    Some(MspCommandCode::MSP_RESET_CONF) => &reset_conf_ack,

                    Some(MspCommandCode::MSP_EEPROM_WRITE) => &write_eeprom_ack,

                    _ => continue,
                };

                channel.send(result).await;
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
        let summary = self.flash_summary().await.unwrap();
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
        let summary = self.flash_summary().await.unwrap();
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

                let packet = MspPacket {
                    cmd: MspCommandCode::MSP_DATAFLASH_READ as u16,
                    direction: MspPacketDirection::ToFlightController,
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
                    Err(_) => return Err(io::Error::new(io::ErrorKind::ConnectionAborted, "device disconnected")),
                    Ok(payload) => {
                        let packet = INavMsp::parse_chunk(payload.unwrap());

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

    pub async fn flash_summary(&self) -> Result<MspDataFlashSummaryReply, &str> {
        if &self.core.buff_size() == &0 {
            return Err("can't read response when buff_size is 0")
        }

        let packet = MspPacket {
            cmd: MspCommandCode::MSP_DATAFLASH_SUMMARY as u16,
            direction: MspPacketDirection::ToFlightController,
            data: vec![],
        };

        self.core.write(packet).await;

        let payload = match self.summary.1.recv().await.unwrap() {
            Ok(r) => r,
            Err(_) => return Err("failed to get flash summary")
        };

        let summary = MspDataFlashSummaryReply::unpack_from_slice(&payload).unwrap();

        return Ok(summary);
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
    pub async fn set_mode_range(&self, index: u8, range: MspModeRange) -> Result<(), &str>{
        let payload = MspSetModeRange {
            index: index,
            mode_range: range,
        };

        let packet = MspPacket {
            cmd: MspCommandCode::MSP_SET_MODE_RANGE as u16,
            direction: MspPacketDirection::ToFlightController,
            data: payload.pack().to_vec(),
        };

        self.core.write(packet).await;

        if &self.core.buff_size() == &0 {
            return Ok(());
        }

        return match self.set_mode_range_ack.1.recv().await.unwrap() {
            Ok(_) => Ok(()),
            Err(_) => Err("failed to set mode range")
        };
    }

    pub async fn get_mode_ranges(&self) -> Result<Vec<MspModeRange>, &str> {
        if &self.core.buff_size() == &0 {
            return Err("can't read response when buff_size is 0")
        }

        let packet = MspPacket {
            cmd: MspCommandCode::MSP_MODE_RANGES as u16,
            direction: MspPacketDirection::ToFlightController,
            data: vec![],
        };

        self.core.write(packet).await;

        // TODO: we are not sure this ack is for our request, because there is no id for the request
        // TODO: what if we are reading packet that was sent long time ago
        // TODO: also currently if no one is reading the channges, we may hang
        let payload = match self.mode_ranges.1.recv().await.unwrap() {
            Ok(r) => r,
            Err(_) => return Err("failed to get mode_ranges")
        };

        let mut ranges = vec![];
        let len = MspModeRange::packed_bytes();
        for i in (0..payload.len()).step_by(len) {
            let r = MspModeRange::unpack_from_slice(&payload[i..i+len]).unwrap();
            ranges.push(r);
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
    pub async fn set_motor_mixer(&self, index: u8, mmix: MspMotorMixer) -> Result<(), &str> {
        let payload = MspSetMotorMixer {
            index: index,
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

        if &self.core.buff_size() == &0 {
            return Ok(());
        }

        return match self.set_motor_mixer_ack.1.recv().await.unwrap() {
            Ok(_) => Ok(()),
            Err(_) => Err("failed to set motor mixer")
        };
    }

    pub async fn get_motor_mixers(&self) -> Result<Vec<MspMotorMixer>, &str> {
        if &self.core.buff_size() == &0 {
            return Err("can't read response when buff_size is 0")
        }

        let packet = MspPacket {
            cmd: MspCommandCode::MSP2_MOTOR_MIXER as u16,
            direction: MspPacketDirection::ToFlightController,
            data: vec![],
        };

        self.core.write(packet).await;

        let payload = match self.motor_mixers.1.recv().await.unwrap() {
            Ok(r) => r,
            Err(_) => return Err("failed to get motor mixers")
        };

        let mut mmixers = vec![];
        let len = MspMotorMixer::packed_bytes();

        for i in (0..payload.len()).step_by(len) {
            let m = MspMotorMixer::unpack_from_slice(&payload[i..i+len]).unwrap();
            if m.throttle != 0 {
                mmixers.push(m);
            }
        }

        return Ok(mmixers);
    }

    /// inav.set_osd_config_item(116, multiwii_serial_protocol::structs::MspOsdItemPosition { col: 11u8, row: 22u8 }).await;
    /// println!("osd {:?}", inav.get_osd_settings().await);
    pub async fn set_osd_config_item(&self, id: u8, item: MspOsdItemPosition) -> Result<(), &str> {
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

        if &self.core.buff_size() == &0 {
            return Ok(());
        }

        return match self.set_osd_config_ack.1.recv().await.unwrap() {
            Ok(_) => Ok(()),
            Err(_) => Err("failed to set osd item")
        };
    }

    pub async fn set_osd_config(&self, config: MspOsdConfig) -> Result<(), &str> {
        // if -1 will set different kinds of configurations else the layout id
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

        if &self.core.buff_size() == &0 {
            return Ok(());
        }

        return match self.set_osd_config_ack.1.recv().await.unwrap() {
            Ok(_) => Ok(()),
            Err(_) => Err("failed to set osd config")
        };
    }

    pub async fn get_osd_settings(&self) -> Result<MspOsdSettings, &str> {
        if &self.core.buff_size() == &0 {
            return Err("can't read response when buff_size is 0")
        }

        let packet = MspPacket {
            cmd: MspCommandCode::MSP_OSD_CONFIG as u16,
            direction: MspPacketDirection::ToFlightController,
            data: vec![],
        };

        self.core.write(packet).await;

        let payload = match self.osd_configs.1.recv().await.unwrap() {
            Ok(r) => r,
            Err(_) => return Err("failed to get osd config")
        };

        let header_len = MspSetGetOsdConfig::packed_bytes();
        let osd_set_get_reply = MspSetGetOsdConfig::unpack_from_slice(&payload[..header_len]).unwrap();

        let mut item_positions = vec![];
        let len = MspOsdItemPosition::packed_bytes();
        for i in (header_len..payload.len()).step_by(len) {
            let item_pos = MspOsdItemPosition::unpack_from_slice(&payload[i..i+len]).unwrap();
            item_positions.push(item_pos);
        }

        return Ok(MspOsdSettings {
            osd_support: osd_set_get_reply.item_index,
            config: osd_set_get_reply.config,
            item_positions: item_positions,
        });
    }

    pub async fn set_osd_layout_item(&self, id: u8, item: MspSetOsdLayout) -> Result<(), &str> {
        let payload = MspSetOsdLayoutItem {
            layout_index: id,
            item: item,
        };

        let packet = MspPacket {
            cmd: MspCommandCode::MSP2_INAV_OSD_SET_LAYOUT_ITEM as u16,
            direction: MspPacketDirection::ToFlightController,
            data: payload.pack().to_vec(),
        };

        self.core.write(packet).await;

        if &self.core.buff_size() == &0 {
            return Ok(());
        }

        return match self.set_osd_layout_item_ack.1.recv().await.unwrap() {
            Ok(_) => Ok(()),
            Err(_) => Err("failed to set osd layout item")
        };
    }

    pub async fn get_osd_layout_count(&self) -> Result<MspOsdLayouts, &str> {
        if &self.core.buff_size() == &0 {
            return Err("can't read response when buff_size is 0")
        }

        let packet = MspPacket {
            cmd: MspCommandCode::MSP2_INAV_OSD_LAYOUTS as u16,
            direction: MspPacketDirection::ToFlightController,
            data: vec![],
        };

        self.core.write(packet).await;

        let payload = match self.osd_layout_count.1.recv().await.unwrap() {
            Ok(r) => r,
            Err(_) => return Err("failed to get layout count")
        };

        let header_len = MspOsdLayouts::packed_bytes();
        let osd_layout_config = MspOsdLayouts::unpack_from_slice(&payload[..header_len]).unwrap();

        return Ok(osd_layout_config);
    }

    pub async fn get_osd_layout_items(&self, layout_index: u8) -> Result<Vec<MspOsdItemPosition>, &str> {
        if &self.core.buff_size() == &0 {
            return Err("can't read response when buff_size is 0")
        }

        let packet = MspPacket {
            cmd: MspCommandCode::MSP2_INAV_OSD_LAYOUTS as u16,
            direction: MspPacketDirection::ToFlightController,
            data: vec![layout_index],
        };

        self.core.write(packet).await;

        let payload = match self.osd_layout_items.1.recv().await.unwrap() {
            Ok(r) => r,
            Err(_) => return Err("failed to get layout items")
        };

        let mut item_positions = vec![];
        let len = MspOsdItemPosition::packed_bytes();
        for i in (0..payload.len()).step_by(len) {
            let item_pos = MspOsdItemPosition::unpack_from_slice(&payload[i..i+len]).unwrap();
            item_positions.push(item_pos);
        }

        return Ok(item_positions);
    }

    pub async fn get_osd_layouts(&self) -> Result<Vec<Vec<MspOsdItemPosition>>, &str> {
        if &self.core.buff_size() == &0 {
            return Err("can't read response when buff_size is 0")
        }

        let mut layouts = vec![];

        let layout_count = self.get_osd_layout_count().await?;

        for layout_i in 0..layout_count.layout_count {
            let items = self.get_osd_layout_items(layout_i).await?;
            layouts.push(items);
        }

        return Ok(layouts);
    }

    /// let shitty_serials = vec![multiwii_serial_protocol::structs::MspSerialSetting {
    ///     index: 6,
    ///     function_mask: 0,
    ///     msp_baudrate_index: 0,
    ///     gps_baudrate_index: 0,
    ///     telemetry_baudrate_index: 0,
    ///     peripheral_baudrate_index: 0
    /// }];
    /// inav.set_serial_settings(shitty_serials).await;
    /// println!("serial {:?}", inav.get_serial_settings().await);
    pub async fn set_serial_settings(&self, serials: Vec<MspSerialSetting>) -> Result<(), &str> {
        let payload = serials.iter().flat_map(|s| s.pack().to_vec()).collect();
        let packet = MspPacket {
            cmd: MspCommandCode::MSP2_SET_SERIAL_CONFIG as u16,
            direction: MspPacketDirection::ToFlightController,
            data: payload,
        };

        self.core.write(packet).await;

        if &self.core.buff_size() == &0 {
            return Ok(());
        }

        return match self.set_serial_settings_ack.1.recv().await.unwrap() {
            Ok(_) => Ok(()),
            Err(_) => Err("failed to set serial settings")
        };
    }

    pub async fn get_serial_settings(&self) -> Result<Vec<MspSerialSetting>, &str> {
        if &self.core.buff_size() == &0 {
            return Err("can't read response when buff_size is 0")
        }

        let packet = MspPacket {
            cmd: MspCommandCode::MSP2_SERIAL_CONFIG as u16,
            direction: MspPacketDirection::ToFlightController,
            data: vec![],
        };

        self.core.write(packet).await;

        let payload = match self.serial_settings.1.recv().await.unwrap() {
            Ok(r) => r,
            Err(_) => return Err("failed to get serial settings")
        };

        let mut serials = vec![];
        let len = MspSerialSetting::packed_bytes();

        for i in (0..payload.len()).step_by(len) {
            let serial_setting = MspSerialSetting::unpack_from_slice(&payload[i..i+len]).unwrap();
            if serial_setting.identifier != SerialIdentifier::None {
                serials.push(serial_setting);
            }
        }

        return Ok(serials);
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
    pub async fn set_features(&self, feat: MspFeatures) -> Result<(), &str> {
        let mut clone = feat.clone();

        clone.features[0..8].reverse();
        clone.features[8..16].reverse();
        clone.features[16..24].reverse();
        clone.features[24..32].reverse();

        let packet = MspPacket {
            cmd: MspCommandCode::MSP_SET_FEATURE as u16,
            direction: MspPacketDirection::ToFlightController,
            data: clone.pack().to_vec(),
        };

        self.core.write(packet).await;
        if &self.core.buff_size() == &0 {
            return Ok(());
        }
        return match self.set_features_ack.1.recv().await.unwrap() {
            Ok(_) => Ok(()),
            Err(_) => Err("failed to set features")
        };
    }

    pub async fn get_features(&self) -> Result<MspFeatures, &str> {
        if &self.core.buff_size() == &0 {
            return Err("can't read response when buff_size is 0")
        }

        let packet = MspPacket {
            cmd: MspCommandCode::MSP_FEATURE as u16,
            direction: MspPacketDirection::ToFlightController,
            data: vec![],
        };

        self.core.write(packet).await;

        let payload = match self.features.1.recv().await.unwrap() {
            Ok(r) => r,
            Err(_) => return Err("failed to get features")
        };

        let mut feat = MspFeatures::unpack_from_slice(&payload).unwrap();

        // TODO: move this logic to parser, either use uin32 or fix the that booleans
        feat.features[0..8].reverse();
        feat.features[8..16].reverse();
        feat.features[16..24].reverse();
        feat.features[24..32].reverse();

        return Ok(feat);
    }

    pub async fn set_servo_mix_rule(&self, index: u8, servo_rule: MspServoMixRule) -> Result<(), &str> {
        let payload = MspSetServoMixRule {
            index: index,
            servo_rule: servo_rule,
        };

        let packet = MspPacket {
            cmd: MspCommandCode::MSP_SET_SERVO_MIX_RULE as u16,
            direction: MspPacketDirection::ToFlightController,
            data: payload.pack().to_vec(),
        };

        self.core.write(packet).await;

        if &self.core.buff_size() == &0 {
            return Ok(());
        }

        return match self.set_servo_mix_rules_ack.1.recv().await.unwrap() {
            Ok(_) => Ok(()),
            Err(_) => Err("failed to set servo mix rule")
        };
    }

    /// println!("servo mixers {:?}", inav.get_servo_mix_rules().await);
    pub async fn get_servo_mix_rules(&self) -> Result<Vec<MspServoMixRule>, &str> {
        if &self.core.buff_size() == &0 {
            return Err("can't read response when buff_size is 0")
        }
        let packet = MspPacket {
            cmd: MspCommandCode::MSP_SERVO_MIX_RULES as u16,
            direction: MspPacketDirection::ToFlightController,
            data: vec![],
        };

        self.core.write(packet).await;

        let payload = match self.servo_mix_rules.1.recv().await.unwrap() {
            Ok(r) => r,
            Err(_) => return Err("failed to get servo mix rule")
        };

        let mut rules = vec![];
        let len = MspServoMixRule::packed_bytes();

        for i in (0..payload.len()).step_by(len) {
            let serial_setting = MspServoMixRule::unpack_from_slice(&payload[i..i+len]).unwrap();
            if serial_setting.rate != 0 {
                rules.push(serial_setting);
            }
        }

        return Ok(rules);
    }

    pub async fn set_servo_mixer(&self, index: u8, servo_rule: MspServoMixer) -> Result<(), &str> {
        if &self.core.buff_size() == &0 {
            return Err("can't read response when buff_size is 0")
        }
        let payload = MspSetServoMixer {
            index: index,
            servo_rule: servo_rule,
        };

        let packet = MspPacket {
            cmd: MspCommandCode::MSP2_INAV_SET_SERVO_MIXER as u16,
            direction: MspPacketDirection::ToFlightController,
            data: payload.pack().to_vec(),
        };

        self.core.write(packet).await;

        return match self.set_servo_mixer_ack.1.recv().await.unwrap() {
            Ok(_) => Ok(()),
            Err(_) => Err("failed to set servo mixer")
        };
    }

    pub async fn get_servo_mixer(&self) -> Result<Vec<MspServoMixer>, &str> {
        if &self.core.buff_size() == &0 {
            return Err("can't read response when buff_size is 0")
        }
        let packet = MspPacket {
            cmd: MspCommandCode::MSP2_INAV_SERVO_MIXER as u16,
            direction: MspPacketDirection::ToFlightController,
            data: vec![],
        };

        self.core.write(packet).await;

        let payload = match self.servo_mixer.1.recv().await.unwrap() {
            Ok(r) => r,
            Err(_) => return Err("failed to get servo mixer")
        };

        let mut rules = vec![];
        let len = MspServoMixer::packed_bytes();

        for i in (0..payload.len()).step_by(len) {
            let serial_setting = MspServoMixer::unpack_from_slice(&payload[i..i+len]).unwrap();
            if serial_setting.rate != 0 {
                rules.push(serial_setting);
            }
        }

        return Ok(rules);
    }

    pub async fn set_servo_config(&self, index: u8, servo_config: MspServoConfig) -> Result<(), &str> {
        let payload = MspSetServoConfig {
            index: index,
            servo_config: servo_config,
        };

        let packet = MspPacket {
            cmd: MspCommandCode::MSP_SET_SERVO_CONFIGURATION as u16,
            direction: MspPacketDirection::ToFlightController,
            data: payload.pack().to_vec(),
        };

        self.core.write(packet).await;

        if &self.core.buff_size() == &0 {
            return Ok(());
        }

        return match self.set_servo_configs_ack.1.recv().await.unwrap() {
            Ok(_) => Ok(()),
            Err(_) => Err("failed to set servo config")
        };
    }

    /// println!("servo mixers {:?}", inav.get_servo_mix_rules().await);
    pub async fn get_servo_configs(&self) -> Result<Vec<MspServoConfig>, &str> {
        if &self.core.buff_size() == &0 {
            return Err("can't read response when buff_size is 0")
        }

        let packet = MspPacket {
            cmd: MspCommandCode::MSP_SERVO_CONFIGURATIONS as u16,
            direction: MspPacketDirection::ToFlightController,
            data: vec![],
        };

        self.core.write(packet).await;

        let payload = match self.servo_configs.1.recv().await.unwrap() {
            Ok(r) => r,
            Err(_) => return Err("failed to get servo configs")
        };

        let mut servo_configs = vec![];
        let len = MspServoConfig::packed_bytes();

        for i in (0..payload.len()).step_by(len) {
            let servo_config = MspServoConfig::unpack_from_slice(&payload[i..i+len]).unwrap();
            servo_configs.push(servo_config);
        }

        return Ok(servo_configs);
    }

    /// inav.set_rx_map_rules(multiwii_serial_protocol::structs::MspRxMap { map: [0,0,0,0]}).await;
    /// println!("features {:?}", inav.get_rx_map_rules().await);
    pub async fn set_rx_map(&self, rx_map: MspRxMap) -> Result<(), &str> {
        let packet = MspPacket {
            cmd: MspCommandCode::MSP_SET_RX_MAP as u16,
            direction: MspPacketDirection::ToFlightController,
            data: rx_map.pack().to_vec(),
        };

        self.core.write(packet).await;

        if &self.core.buff_size() == &0 {
            return Ok(());
        }

        return match self.set_rx_map_ack.1.recv().await.unwrap() {
            Ok(_) => Ok(()),
            Err(_) => Err("failed set rx map rules")
        };
    }

    pub async fn get_rx_map(&self) -> Result<MspRxMap, &str> {
        if &self.core.buff_size() == &0 {
            return Err("can't read response when buff_size is 0")
        }

        let packet = MspPacket {
            cmd: MspCommandCode::MSP_RX_MAP as u16,
            direction: MspPacketDirection::ToFlightController,
            data: vec![],
        };

        self.core.write(packet).await;

        let payload = match self.rx_map.1.recv().await.unwrap() {
            Ok(r) => r,
            Err(_) => return Err("failed to get rx map rules")
        };

        return Ok(MspRxMap::unpack_from_slice(&payload).unwrap());
    }


    // TODO: on connection get version from msp, similar to inav configurator and allow to switch features based on this version
    pub fn str_from_u8_nul_utf8(utf8_src: &[u8]) -> Result<&str, std::str::Utf8Error> {
        let nul_range_end = utf8_src.iter()
            .position(|&c| c == b'\0')
            .unwrap_or(utf8_src.len()); // default to length if no `\0` present
        ::std::str::from_utf8(&utf8_src[0..nul_range_end])
    }

    pub async fn set_setting_by_id<'a>(&self, id: &'a u16, value: &[u8]) -> Result<&'a u16, &str> {
        let payload = MspSettingInfoRequest {
            null: 0,
            id: *id
        };

        self.set_setting(&payload.pack(), value).await?;
        Ok(id)
    }

    pub async fn set_setting_by_name(&self, name: &str, value: &[u8]) -> Result<(), &str> {
        let mut payload = name.as_bytes().to_vec();
        payload.push(b'\0');

        return self.set_setting(&payload, value).await;
    }

    pub async fn set_setting(&self, id: &[u8], value: &[u8]) -> Result<(), &str> {
        let mut payload = id.to_vec();
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

        if &self.core.buff_size() == &0 {
            return Ok(());
        }

        return match self.set_setting_ack.1.recv().await.unwrap() {
            Ok(_) => Ok(()),
            Err(_) => Err("failed to set setting")
        };
    }

    pub async fn get_setting_info_by_name(&self, name: &str) -> Result<SettingInfo, &str> {
        if &self.core.buff_size() == &0 {
            return Err("can't read response when buff_size is 0")
        }
        self.request_setting_info_by_name(name).await?;
        return Ok(self.receive_setting_info().await?);
    }

    pub async fn get_setting_info_by_id(&self, id: &u16) -> Result<SettingInfo, &str> {
        if &self.core.buff_size() == &0 {
            return Err("can't read response when buff_size is 0")
        }
        self.request_setting_info_by_id(id).await?;
        return Ok(self.receive_setting_info().await?);
    }

    // TODO: return iteratable stream here
    /// request_buffering may increase the fetching speed but if flight controller can't handle it
    /// it will not return response and decrease reliabilty, request buffer of 1 is good for most cases
    pub async fn get_setting_infos(&self) -> Result<Vec<SettingInfo>, &str> {
        if &self.core.buff_size() == &0 {
            return Err("can't read response when buff_size is 0")
        }

        println!("describe pg groups"); // TODO: write in debug flag
        let pg_settings = self.get_pg_settings().await?;

        println!("found {} groups", pg_settings.len());
        let setting_ids: Vec<u16> = pg_settings
            .iter()
            .flat_map(|pg_s| (pg_s.start_id..=pg_s.end_id).map(u16::from).collect::<Vec<u16>>())
            .collect();

        let setting_info_futures = setting_ids
            .iter()
            .map(|id| self.get_setting_info_by_id(&id));

        return try_join_all(setting_info_futures).await;
    }

    // TODO: return iteratable stream here
    pub async fn get_setting_infos_by_names(&self, names: Vec<&String>) -> Result<Vec<SettingInfo>, &str> {
        if &self.core.buff_size() == &0 {
            return Err("can't read response when buff_size is 0")
        }

        let setting_info_futures = names
            .iter()
            .map(|name| self.get_setting_info_by_name(&name));

        return try_join_all(setting_info_futures).await;
    }

    async fn request_setting_info_by_id(&self, id: &u16) -> Result<(), &str> {
        let payload = MspSettingInfoRequest {
            null: 0,
            id: *id,
        };

        return self.request_setting_info(payload.pack().to_vec()).await;
    }

    async fn request_setting_info_by_name(&self, name: &str) -> Result<(), &str> {
        let mut payload = name.as_bytes().to_vec();
        payload.push(b'\0');
        return self.request_setting_info(payload).await;
    }

    async fn request_setting_info(&self, id: Vec<u8>) -> Result<(), &str> {
        let packet = MspPacket {
            cmd: MspCommandCode::MSP2_COMMON_SETTING_INFO as u16,
            direction: MspPacketDirection::ToFlightController,
            data: id,
        };

        return Ok(self.core.write(packet).await);
    }

    async fn receive_setting_info(&self) -> Result<SettingInfo, &str> {
        let payload = match self.setting_info.1.recv().await.unwrap() {
            Ok(r) => r,
            Err(_) => return Err("failed to get setting info"),
        };

        let name = INavMsp::str_from_u8_nul_utf8(&payload).unwrap();
        let len = MspSettingInfo::packed_bytes();
        let mut index = name.len() + 1;
        let setting_info = MspSettingInfo::unpack_from_slice(&payload[index..index + len]).expect("Failed to parse inavlid msp setting");

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

        return Ok(SettingInfo {
            name: String::from(name),
            value: value.to_vec(),
            info: setting_info,
            enum_names: enum_values.iter().map(|&s| String::from(s)).collect(),
        });
    }

    // calling pg list will get all the settings groups list (the PG groups)
    // it will return the start of the group and the end of the group

    // 2 bytes, group id ... 0 is invalid
    // 2 bytes start of the setting index, this is not a memory
    // 2 bytes last setting index, this is not a memroy
    pub async fn get_pg_settings(&self) -> Result<Vec<MspSettingGroup>, &str> {
        if &self.core.buff_size() == &0 {
            return Err("can't read response when buff_size is 0")
        }
        let packet = MspPacket {
            cmd: MspCommandCode::MSP2_COMMON_PG_LIST as u16,
            direction: MspPacketDirection::ToFlightController,
            data: vec![], // pass nothing to get all settings
        };

        self.core.write(packet).await;

        let payload = match self.pg_settings.1.recv().await.unwrap() {
            Ok(r) => r,
            Err(_) => return Err("failed to get pg settings")
        };

        let mut setting_ids = vec![];
        let len = MspSettingGroup::packed_bytes();

        for i in (0..payload.len()).step_by(len) {
            let setting_id = MspSettingGroup::unpack_from_slice(&payload[i..i+len]).unwrap();
            setting_ids.push(setting_id);
        }

        return Ok(setting_ids);
    }


    pub async fn write_char(&self, addr: usize, char_pixels: [u8; 54]) -> Result<usize, &str>{
        let mut payload = vec![];
        if addr > u8::MAX as usize {
            payload.extend((addr as u16).to_le_bytes().to_vec());
        } else {
            payload.push(addr as u8);
        }
        payload.extend(char_pixels.to_vec());

        let packet = MspPacket {
            cmd: MspCommandCode::MSP_OSD_CHAR_WRITE as u16,
            direction: MspPacketDirection::ToFlightController,
            data: payload,
        };

        self.core.write(packet).await;

        if &self.core.buff_size() == &0 {
            return Ok(addr);
        }

        return match self.write_char_ack.1.recv().await.unwrap() {
            Ok(_) => Ok(addr),
            Err(_) => Err("failed to write char")
        };
    }

    pub async fn save_to_eeprom(&self) -> Result<(), &str> {
        let packet = MspPacket {
            cmd: MspCommandCode::MSP_EEPROM_WRITE as u16,
            direction: MspPacketDirection::ToFlightController,
            data: vec![],
        };

        self.core.write(packet).await;

        if &self.core.buff_size() == &0 {
            return Ok(());
        }


        return match self.write_eeprom_ack.1.recv().await.unwrap() {
            Ok(_) => Ok(()),
            Err(_) => Err("failed to write to eeprom")
        };
    }

    pub async fn set_raw_rc(&self, channels: Vec<u16>) -> Result<(), &str> {
        let payload = unsafe {
            use std::slice;
            slice::from_raw_parts(channels.as_ptr() as *mut u8, channels.len() * 2)
        };

        let packet = MspPacket {
            cmd: MspCommandCode::MSP_SET_RAW_RC as u16,
            direction: MspPacketDirection::ToFlightController,
            data: payload.to_vec(),
        };

        self.core.write(packet).await;

        if &self.core.buff_size() == &0 {
            return Ok(());
        }

        return match self.set_raw_rc_ack.1.recv().await.unwrap() {
            Ok(_) => Ok(()),
            Err(_) => Err("failed to write raw rc")
        };
    }

    pub async fn reset_conf(&self) -> Result<(), &str> {
        let packet = MspPacket {
            cmd: MspCommandCode::MSP_RESET_CONF as u16,
            direction: MspPacketDirection::ToFlightController,
            data: vec![],
        };

        self.core.write(packet).await;

        if &self.core.buff_size() == &0 {
            return Ok(());
        }

        return match self.reset_conf_ack.1.recv().await.unwrap() {
            Ok(_) => Ok(()),
            Err(_) => Err("failed to reset eeprom")
        };
    }

    pub async fn reboot(&self) -> Result<(), &str> {
        let packet = MspPacket {
            cmd: MspCommandCode::MSP_SET_REBOOT as u16,
            direction: MspPacketDirection::ToFlightController,
            data: vec![],
        };

        self.core.write(packet).await;
        Ok(())
    }
}
