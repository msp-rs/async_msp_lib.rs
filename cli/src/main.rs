extern crate alloc;
extern crate async_msp_lib;
extern crate multiwii_serial_protocol;

extern crate serialport;

use serialport::{available_ports, open_with_settings};

extern crate packed_struct;
extern crate packed_struct_codegen;

use async_std::fs::OpenOptions;
use async_std::io::{BufReader, BufWriter};
use async_std::io::prelude::*;

use async_std::prelude::*;

use std::time::Duration;
use std::iter::Iterator;
use std::str::FromStr;
use futures::future::try_join_all;
use clap_v3::{App, AppSettings, Arg};
use multiwii_serial_protocol::structs::*;
use std::convert::TryInto;
use std::collections::HashMap;
use packed_struct::PrimitiveEnum;


static FEATURE_NAMES: [&str; 32] = [
    "THR_VBAT_COMP", "VBAT", "TX_PROF_SEL", "BAT_PROF_AUTOSWITCH", "MOTOR_STOP",
    "", "SOFTSERIAL", "GPS", "",
    "", "TELEMETRY", "CURRENT_METER", "3D", "RX_PARALLEL_PWM",
    "RX_MSP", "RSSI_ADC", "LED_STRIP", "DASHBOARD", "",
    "BLACKBOX", "", "TRANSPONDER", "AIRMODE",
    "SUPEREXPO", "VTX", "RX_SPI", "", "PWM_SERVO_DRIVER", "PWM_OUTPUT_ENABLE",
    "OSD", "FW_LAUNCH", "",
];

#[async_std::main]
async fn main() {

    // TODO: implement port arguments

    let matches = App::new("msp")
        .version("0.0.1")
        .author("Ilya G. <amfernusus@gmail.com>")
        .about("Interact with msp fligith controller")
        .subcommand(
            App::new("setting")
                .about("Common setting")
                .subcommand(
                    App::new("list").about("list common setting")
                )
                .subcommand(
                    App::new("set")
                        .about("Set common setting")
                        .setting(AppSettings::ArgRequiredElseHelp)
                        .arg(Arg::with_name("name").help("The setting name to set").required(true).takes_value(true))
                        .arg(Arg::with_name("value").help("The setting value to set").required(true).takes_value(true))
                )
                .subcommand(
                    App::new("set-all")
                        .about("Set all common setting")
                        .setting(AppSettings::ArgRequiredElseHelp)
                        .arg(Arg::with_name("input").help("settings file path").required(true).takes_value(true))
                )
        )
        .subcommand(
            App::new("aux")
                .about("Get all aux setting")
                .subcommand(
                    App::new("set")
                        .about("Set aux setting")
                        .setting(AppSettings::ArgRequiredElseHelp)
                        .arg(Arg::with_name("value").help("The setting value to set").required(true).takes_value(true))
                )
        )
        .subcommand(
            App::new("mmix")
                .about("Get all mmix setting")
                .subcommand(
                    App::new("set")
                        .about("Set mmix setting")
                        .setting(AppSettings::ArgRequiredElseHelp)
                        .arg(Arg::with_name("value").help("The setting value to set").required(true).takes_value(true))
                )
        )
        .subcommand(
            App::new("serial")
                .about("Get all serial setting")
                .subcommand(
                    App::new("set")
                        .about("Set serial setting")
                        .setting(AppSettings::ArgRequiredElseHelp)
                        .arg(Arg::with_name("value").help("The setting value to set").required(true).takes_value(true))
                )
        )
        .subcommand(
            App::new("inav_osd_item")
                .about("Get all osd setting")
                .subcommand(
                    App::new("set")
                        .about("Set osd setting")
                        .setting(AppSettings::ArgRequiredElseHelp)
                        .arg(Arg::with_name("value").help("The setting value to set").required(true).takes_value(true))
                )
        )
        .subcommand(
            App::new("feature")
                .about("Get all osd setting")
                .subcommand(
                    App::new("set")
                        .about("Set osd setting")
                        .setting(AppSettings::ArgRequiredElseHelp)
                        .arg(Arg::with_name("value").help("The setting value to set").required(true).takes_value(true))
                )
                .subcommand(
                    App::new("enable")
                        .about("Set osd setting")
                        .setting(AppSettings::ArgRequiredElseHelp)
                        .arg(Arg::with_name("value").help("The setting value to set").required(true).takes_value(true))
                )
                .subcommand(
                    App::new("disable")
                        .about("Set osd setting")
                        .setting(AppSettings::ArgRequiredElseHelp)
                        .arg(Arg::with_name("value").help("The setting value to set").required(true).takes_value(true))
                )
        )
        .subcommand(
            App::new("blackbox")
                .about("Onboard Blackbox")
                .subcommand(
                    App::new("download")
                        .about("Download blackbox concurrently")
                        .setting(AppSettings::ArgRequiredElseHelp)
                        .arg(Arg::with_name("input").help("download path").required(true).takes_value(true))
                )
                .subcommand(
                    App::new("downloadv2")
                        .about("Pull blackbox serially")
                        .setting(AppSettings::ArgRequiredElseHelp)
                        .arg(Arg::with_name("input").help("download path").required(true).takes_value(true))
                )
        )
        .subcommand(
            App::new("reboot")
                .about("Write settings to eeprom")
        )
        .arg(
            Arg::with_name("save")
                .short('s')
                .long("save")
                .help("settings file path")
                .required(false)
        )
        .arg(
            Arg::with_name("reboot")
                .short('r')
                .long("reboot")
                .help("reboot fc")
                .required(false)
        )

        // .args_from_usage(
        //     "-p, --port=[FILE] 'Serial port'
        //                       -v...                'Sets the level of verbosity'")
        //                   .subcommand(SubCommand::with_name("set")
        //                               .about("set setting")
        //                               .arg("-d, --debug 'Print debug information'"))
        .get_matches();


    let s = serialport::SerialPortSettings {
        baud_rate: 115200,
        data_bits: serialport::DataBits::Eight,
        flow_control: serialport::FlowControl::None,
        parity: serialport::Parity::None,
        stop_bits: serialport::StopBits::One,
        timeout: Duration::from_millis(1),
    };

    // TODO: what stop and start bits are inav using, is every one just using the canonical defalts?
    let serialport = open_with_settings(&available_ports().expect("No serial port")[0].port_name, &s)
        .expect("Failed to open serial port");

    // green-thread 1: read into input channel from serial(reading from serial is blocking)
    let inav = inav_msp_lib::INavMsp::new();
    inav.start(serialport);

    match matches.subcommand() {
        ("setting", Some(setting_matches)) => {
            match setting_matches.subcommand() {
                ("list", Some(_)) => {
                    let setting_list = list_settings(&inav).await.unwrap();

                    for s in setting_list {
                        println!("{} {}", &s.name, setting_to_str(&s));
                    }
                }
                ("get", Some(get_matches)) => {
                    if !get_matches.is_present("name") {
                        unreachable!();
                    }

                    let name = get_matches.value_of("name").unwrap();
                    let setting_info = inav.get_setting_info_by_name(&name).await.unwrap();
                    println!("{}", setting_to_str(&setting_info));
                }
                ("set", Some(set_matches)) => {
                    if !set_matches.is_present("name") || !set_matches.is_present("value") {
                        unreachable!();
                    }

                    let name = set_matches.value_of("name").unwrap();
                    let value = set_matches.value_of("value").unwrap();
                    let setting_info = inav.get_setting_info_by_name(&name).await.unwrap();
                    let payload = setting_to_vec(&setting_info, value).unwrap();
                    inav.set_setting_by_name(name, &payload).await.unwrap();
                },
                ("set-all", Some(set_all_matches)) => {
                    if !set_all_matches.is_present("input") {
                        print!("missing input");
                    }

                    let input = set_all_matches.value_of("input").unwrap();
                    let f = OpenOptions::new()
                        .read(true)
                        .open(input)
                        .await.unwrap();
                    let f = BufReader::new(f);

                    println!("listing settings");
                    let setting_list = list_settings(&inav).await.unwrap();

                    let setting_list_key_vals = setting_list
                        .iter()
                        .enumerate()
                        .fold(HashMap::new(), |mut acc, (i, s) | {
                            acc.insert(s.name.clone(), (i as u16, s));
                            acc
                        });

                    let id_buf_valus = f.lines().map(|l| {
                        let line = l.unwrap();
                        let mut split_iter = line.split_whitespace();
                        let name = split_iter.next().unwrap().to_string();
                        let val = match split_iter.next() {
                            Some(v) => v.to_string(),
                            None => "".to_owned(),
                        };
                        let (i, s) = setting_list_key_vals.get(&name).unwrap(); // TODO: write warnning if setting name not found
                        let buf_val = setting_to_vec(&s, &val).unwrap();
                        (i, buf_val)
                    }).collect::<Vec<(&u16, Vec<u8>)>>().await;

                    let set_setting_futures = id_buf_valus.iter()
                        .map(|(i, v)| inav.set_setting_by_id(i, v));
                    println!("writing settings");
                    try_join_all(set_setting_futures).await.unwrap();
                },
                ("", None) => println!("No subcommand was used"),
                _ => unreachable!(),
            }
        }
        ("aux", Some(aux_matches)) => {
            match aux_matches.subcommand() {
                ("set", Some(set_matches)) => {
                    if !set_matches.is_present("value") {
                        unreachable!();
                    }

                    let value = set_matches.value_of("value").unwrap();
                    let mut split_iter = value.split_whitespace();
                    let index = split_iter.next().unwrap();
                    let box_id = split_iter.next().unwrap();
                    let aux_channel_index = split_iter.next().unwrap();
                    let start_step = split_iter.next().unwrap();
                    let end_step = split_iter.next().unwrap();

                    let start_step_parsed = u32::from_str(start_step).unwrap();
                    let end_step_parsed = u32::from_str(end_step).unwrap();

                    let range = MspModeRange {
                        box_id: u8::from_str(box_id).unwrap(),
                        aux_channel_index: MspRcChannel::from_primitive(
                            u8::from_str(aux_channel_index).unwrap()
                        ).unwrap(),
                        start_step: ((start_step_parsed - 900) / 25) as u8,
                        end_step: ((end_step_parsed - 900) / 25) as u8,
                    };

                    inav.set_mode_range(u8::from_str(index).unwrap(), range).await.unwrap();
                },
                ("", None) => {
                    let ranges = inav.get_mode_ranges().await.unwrap();
                    for (i, r) in ranges.iter().enumerate() {
                        println!(
                            "{} {} {} {} {}",
                            i,
                            r.box_id,
                            r.aux_channel_index.to_primitive(),
                            (r.start_step as u32) * 25 + 900,
                            (r.end_step as u32) * 25 + 900
                        );
                    }
                },
                _ => unreachable!(),
            }
        }
        ("mmix", Some(mmix_matches)) => {
            match mmix_matches.subcommand() {
                ("set", Some(set_matches)) => {
                    if !set_matches.is_present("value") {
                        unreachable!();
                    }

                    let value = set_matches.value_of("value").unwrap();
                    let mut split_iter = value.split_whitespace();
                    let index = split_iter.next().unwrap();
                    let throttle = split_iter.next().unwrap();
                    let roll = split_iter.next().unwrap();
                    let pitch = split_iter.next().unwrap();
                    let yaw = split_iter.next().unwrap();

                    let mmix = MspMotorMixer {
                        throttle: f32::from_str(throttle).unwrap() as u16 * 1000,
                        roll: (f32::from_str(roll).unwrap() + 2f32) as u16 * 1000,
                        pitch: (f32::from_str(pitch).unwrap() + 2f32) as u16 * 1000,
                        yaw: (f32::from_str(yaw).unwrap() + 2f32) as u16 * 1000,
                    };

                    inav.set_motor_mixer(u8::from_str(index).unwrap(), mmix).await.unwrap();
                },
                ("", None) => {
                    let mixers = inav.get_motor_mixers().await.unwrap();
                    for (i, m) in mixers.iter().enumerate() {
                        println!(
                            "{} {:.3} {:.3} {:.3} {:.3}",
                            i,
                            m.throttle as f32 / 1000f32,
                            m.roll as f32 / 1000f32 - 2f32,
                            m.pitch as f32 / 1000f32 - 2f32,
                            m.yaw as f32 / 1000f32 - 2f32,
                        );
                    }
                },
                _ => unreachable!(),
            }
        }
        ("serial", Some(serial_matches)) => {
            match serial_matches.subcommand() {
                ("set", Some(set_matches)) => {
                    if !set_matches.is_present("value") {
                        unreachable!();
                    }

                    let value = set_matches.value_of("value").unwrap();
                    let mut split_iter = value.split_whitespace();

                    let identifier = split_iter.next().unwrap();
                    let function_mask = split_iter.next().unwrap();
                    let msp_baudrate_index = split_iter.next().unwrap();
                    let gps_baudrate_index = split_iter.next().unwrap();
                    let telemetry_baudrate_index = split_iter.next().unwrap();
                    let peripheral_baudrate_index = split_iter.next().unwrap();

                    let serial = MspSerialSetting {
                        identifier: u8_to_serial_identifier(u8::from_str(identifier).unwrap()).unwrap(),
                        function_mask: u32::from_str(function_mask).unwrap(),
                        msp_baudrate_index: string_to_baudrate(msp_baudrate_index).unwrap(),
                        gps_baudrate_index: string_to_baudrate(gps_baudrate_index).unwrap(),
                        telemetry_baudrate_index: string_to_baudrate(telemetry_baudrate_index).unwrap(),
                        peripheral_baudrate_index: string_to_baudrate(peripheral_baudrate_index).unwrap(),
                    };

                    inav.set_serial_settings(vec![serial]).await.unwrap();
                },
                ("", None) => {
                    let serials = inav.get_serial_settings().await.unwrap();

                    for s in serials.iter(){
                        println!(
                            "{} {} {} {} {} {}",
                            s.identifier as u8,
                            s.function_mask,
                            baudrate_to_string(&s.msp_baudrate_index).unwrap(),
                            baudrate_to_string(&s.gps_baudrate_index).unwrap(),
                            baudrate_to_string(&s.telemetry_baudrate_index).unwrap(),
                            baudrate_to_string(&s.peripheral_baudrate_index).unwrap(),
                        );
                    }
                },
                _ => unreachable!(),
            }
        }
        ("inav_osd_item", Some(serial_matches)) => {
            match serial_matches.subcommand() {
                ("set", Some(set_matches)) => {
                    if !set_matches.is_present("value") {
                        unreachable!();
                    }

                    let value = set_matches.value_of("value").unwrap();
                    let mut split_iter = value.split_whitespace();

                    let _layout_index = split_iter.next().unwrap();
                    let item_pos = u8::from_str(split_iter.next().unwrap()).unwrap();
                    let col = u8::from_str(split_iter.next().unwrap()).unwrap();
                    let row = u8::from_str(split_iter.next().unwrap()).unwrap();
                    let vis = split_iter.next().unwrap(); // v h
                    let is_visible: u16 = match vis {
                        "V" => 0x0800,
                        "H" => 0,
                        _ => 0
                    };

                    let field: u16 = ((col as u16) | ((row as u16) << 5)) | is_visible;
                    let bytes = field.to_le_bytes();

                    let item = MspOsdItemPosition {
                        col: bytes[0],
                        row: bytes[1],
                    };

                    inav.set_osd_config_item(item_pos, item).await.unwrap();
                },
                ("", None) => {
                    let osd = inav.get_osd_settings().await.unwrap();

                    for (i, item) in osd.item_positions.iter().enumerate() {
                        let field = u16::from_le_bytes([item.col, item.row]);

                        // TODO: we set support only single layout
                        println!(
                            "0 {} {} {} {}",
                            i,
                            field & 0x001F, // OSD_X
                            (field >> 5) & 0x001F, // OSD_Y
                            if field & 0x0800 > 0 { "V" } else { "H" },
                        );
                    }
                },
                _ => unreachable!(),
            }
        }
        ("feature", Some(serial_matches)) => {
            match serial_matches.subcommand() {
                ("set", Some(set_matches)) => {
                    if !set_matches.is_present("value") {
                        unreachable!();
                    }

                    let value = set_matches.value_of("value").unwrap();
                    let mut features = [false; 32];

                    for elem in value.split_whitespace() {
                        let index = FEATURE_NAMES.iter().position(|&n| n == elem).unwrap();
                        features[index] = true;
                    }

                    let set_feat = MspFeatures {
                        features: features
                    };

                    inav.set_features(set_feat).await.unwrap();
                },
                // TODO: because of the async nature of msp-lib, enable and disable calls must be chained
                ("enable", Some(enable_matches)) => {
                    if !enable_matches.is_present("value") {
                        unreachable!();
                    }

                    let value = enable_matches.value_of("value").unwrap();
                    let index = FEATURE_NAMES.iter().position(|&n| n == value).unwrap();
                    let mut feat = inav.get_features().await.unwrap();

                    feat.features[index] = true;

                    inav.set_features(feat).await.unwrap();
                },
                ("disable", Some(disable_matches)) => {
                    if !disable_matches.is_present("value") {
                        unreachable!();
                    }

                    let value = disable_matches.value_of("value").unwrap();
                    let index = FEATURE_NAMES.iter().position(|&n| n == value).unwrap();
                    let mut feat = inav.get_features().await.unwrap();

                    feat.features[index] = false;

                    inav.set_features(feat).await.unwrap();
                }
                ("", None) => {
                    let features = inav.get_features().await.unwrap();
                    for (i, &is_enabled) in features.features.iter().enumerate() {
                        if is_enabled {
                            println!("{}", FEATURE_NAMES[i]);
                        }
                    }
                },
                _ => unreachable!(),
            }
        }
        ("blackbox", Some(blackbox_matches)) => {
            match blackbox_matches.subcommand() {
                ("download", Some(download_matches)) => {
                    if !download_matches.is_present("input") {
                        print!("missing input");
                    }
                    let input = download_matches.value_of("input").unwrap();

                    let mut f = OpenOptions::new()
                        .write(true)
                        .create(true)
                        .truncate(true)
                        .open(input)
                        .await.unwrap();

                    // no point going more then this size because inav won't return bigger chunk
                    let blackbox_data = inav.read_flash_data(0x1000, |chunk, total| {
                        println!("received packet: {:?}/{:?}", chunk, total);
                    }).await.unwrap();

                    f.write(&blackbox_data[..]).await.unwrap();
                    f.flush().await.unwrap();
                }
                ("downloadv2", Some(downloadv2_matches)) => {
                    if !downloadv2_matches.is_present("input") {
                        print!("missing input");
                    }

                    let input = downloadv2_matches.value_of("input").unwrap();
                    let mut flash_data_file = inav.open_flash_data().await;

                    let f = OpenOptions::new()
                        .write(true)
                        .create(true)
                        .truncate(true)
                        .open(input)
                        .await.unwrap();

                    let mut buf_writer = BufWriter::new(f);
                    loop {
                        let chunk = flash_data_file.read_chunk().await.unwrap();
                        if chunk.len() == 0 {
                            break;
                        }
                        buf_writer.write(&chunk[..]).await.unwrap();
                    }

                    buf_writer.flush().await.unwrap();
                },
                ("", None) => println!("No subcommand was used"),
                _ => unreachable!(),
            }
        }
        ("reboot", Some(_)) => {
            inav.reboot().await.unwrap();
        }
        ("", None) => println!("No subcommand was used"), // If no subcommand was usd it'll match the tuple ("", None)
        _ => unreachable!(), // If all subcommands are defined above, anything else is unreachabe!()
    }

    if matches.is_present("save") {
        inav.save_to_eeprom().await.unwrap();
    }

    if matches.is_present("reboot") {
        inav.reboot().await.unwrap();
    }
}

async fn list_settings(inav: &inav_msp_lib::INavMsp) -> Result<Vec<inav_msp_lib::SettingInfo>, &str> {
    let pg_settings = inav.get_pg_settings().await.unwrap();
    let mut setting_ids: Vec<u16> = pg_settings
        .iter()
        .flat_map(|pg_s| (pg_s.start_id..=pg_s.end_id).map(u16::from).collect::<Vec<u16>>())
        .collect();

    setting_ids.sort();
    let setting_info_futures = setting_ids
        .iter()
        .map(|id| inav.get_setting_info_by_id(&id));
    return try_join_all(setting_info_futures).await;
}

fn setting_to_str(s: &inav_msp_lib::SettingInfo) -> String {
    return match s.info.setting_type {
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
        SettingType::VarInt32 => { // TODO: non standart in betaflight and inav
            let (int_bytes, _rest) = s.value.split_at(std::mem::size_of::<i32>());
            return i32::from_le_bytes(int_bytes.try_into().unwrap()).to_string();
        }
        SettingType::VarFloat => {
            let (int_bytes, _rest) = s.value.split_at(std::mem::size_of::<f32>());
            return f32::from_le_bytes(int_bytes.try_into().unwrap()).to_string();
        }
        SettingType::VarString => inav_msp_lib::INavMsp::str_from_u8_nul_utf8(&s.value).unwrap().to_owned(),
    };
}

fn setting_to_vec<'a>(s: &inav_msp_lib::SettingInfo, value: &str) -> Result<Vec<u8>, &'a str> {
    return match s.info.setting_type {
        SettingType::VarUint8 => {
            if s.info.setting_mode == SettingMode::ModeLookup {
                let enum_name = String::from(value);
                let index = s.enum_names.iter().position(|r| r == &enum_name);
                return match index {
                    Some(i) => Ok((i as u8).to_le_bytes().to_vec()),
                    None => {
                        eprintln!("Failed to find {} in {}", enum_name, s.enum_names.join(","));
                        return Err("Failed to find table value");
                    }
                }
            }

            return Ok(value.parse::<u8>().unwrap().to_le_bytes().to_vec());
        },
        SettingType::VarInt8 => Ok(value.parse::<i8>().unwrap().to_le_bytes().to_vec()),
        SettingType::VarUint16 => Ok(value.parse::<u16>().unwrap().to_le_bytes().to_vec()),
        SettingType::VarInt16 => Ok(value.parse::<i16>().unwrap().to_le_bytes().to_vec()),
        SettingType::VarUint32 => Ok(value.parse::<u32>().unwrap().to_le_bytes().to_vec()),
        SettingType::VarInt32 => Ok(value.parse::<i32>().unwrap().to_le_bytes().to_vec()),
        SettingType::VarFloat => Ok(value.parse::<f32>().unwrap().to_le_bytes().to_vec()),
        SettingType::VarString => Ok(value.as_bytes().to_vec()),
    };
}

// TODO: use trait from_string or implement strum
// TODO: and move this to the library
fn baudrate_to_string<'a>(baudrate: &Baudrate) -> Result<String, &'a str> {
    let s = match baudrate {
        Baudrate::BaudAuto => "0",
        Baudrate::Baud1200 => "1200",
        Baudrate::Baud2400 => "2400",
        Baudrate::Baud4800 => "4800",
        Baudrate::Baud9600 => "9600",
        Baudrate::Baud19200 => "19200",
        Baudrate::Baud38400 => "38400",
        Baudrate::Baud57600 => "57600",
        Baudrate::Baud115200 => "115200",
        Baudrate::Baud230400 => "230400",
        Baudrate::Baud250000 => "250000",
        Baudrate::Baud460800 => "460800",
        Baudrate::Baud921600 => "921600",
        Baudrate::Baud1000000 => "1000000",
        Baudrate::Baud1500000 => "1500000",
        Baudrate::Baud2000000 => "2000000",
        Baudrate::Baud2470000 => "2470000",
    };

    return Ok(s.to_owned());
}

fn string_to_baudrate<'a>(baudrate_str: &str) -> Result<Baudrate, &'a str> {
    let baudrate = match baudrate_str {
        "0" => Baudrate::BaudAuto,
        "1200" => Baudrate::Baud1200,
        "2400" => Baudrate::Baud2400,
        "4800" => Baudrate::Baud4800,
        "9600" => Baudrate::Baud9600,
        "19200" => Baudrate::Baud19200,
        "38400" => Baudrate::Baud38400,
        "57600" => Baudrate::Baud57600,
        "115200" => Baudrate::Baud115200,
        "230400" => Baudrate::Baud230400,
        "250000" => Baudrate::Baud250000,
        "460800" => Baudrate::Baud460800,
        "921600" => Baudrate::Baud921600,
        "1000000" => Baudrate::Baud1000000,
        "1500000" => Baudrate::Baud1500000,
        "2000000" => Baudrate::Baud2000000,
        "2470000" => Baudrate::Baud2470000,
        _ => return Err("Baudrate not found"),
    };

    return Ok(baudrate);
}

fn u8_to_serial_identifier<'a>(id: u8) -> Result<SerialIdentifier, &'a str> {
    let serial = match id {
        255 => SerialIdentifier::None,
        0 => SerialIdentifier::USART1,
        1 => SerialIdentifier::USART2,
        2 => SerialIdentifier::USART3,
        3 => SerialIdentifier::USART4,
        4 => SerialIdentifier::USART5,
        5 => SerialIdentifier::USART6,
        6 => SerialIdentifier::USART7,
        7 => SerialIdentifier::USART8,
        20 => SerialIdentifier::UsbVcp,
        30 => SerialIdentifier::SoftSerial1,
        31 => SerialIdentifier::SoftSerial2,
        _ => return Err("Serial identifier not found"),
    };

    return Ok(serial);
}
