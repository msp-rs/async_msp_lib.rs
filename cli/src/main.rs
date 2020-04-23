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
use futures::future::try_join_all;
use clap_v3::{App, AppSettings, Arg};
use multiwii_serial_protocol::structs::{SettingType, SettingMode};
use std::convert::TryInto;
use std::collections::HashMap;



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
                    App::new("get")
                        .about("get common setting")
                        .setting(AppSettings::ArgRequiredElseHelp)
                        .arg(Arg::with_name("name").help("The setting name to get").required(true).takes_value(true))
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
                        let val = split_iter.next().unwrap().to_string();
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
        ("", None) => println!("No subcommand was used"), // If no subcommand was usd it'll match the tuple ("", None)
        _ => unreachable!(), // If all subcommands are defined above, anything else is unreachabe!()
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
        SettingType::VarString => ::std::str::from_utf8(&s.value).unwrap().to_string(),
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
        SettingType::VarFloat => Ok(value.parse::<f32>().unwrap().to_le_bytes().to_vec()),
        SettingType::VarString => Ok(value.as_bytes().to_vec()),
    };
}
