extern crate alloc;
extern crate multiwii_serial_protocol;

extern crate serialport;

use serialport::{available_ports, open};
use std::io;
use std::io::Write;
use std::time::Duration;
use std::{mem, thread};

fn main() {
    let packet = multiwii_serial_protocol::MspPacket {
		    cmd: multiwii_serial_protocol::MspCommandCode::MSP_DATAFLASH_SUMMARY as u8,
		    direction: multiwii_serial_protocol::MspPacketDirection::ToFlightController,
		    data: alloc::borrow::Cow::Owned(vec![])
	  };

    let size = packet.packet_size_bytes();
    let mut output = vec![0; size];
    packet.serialize(&mut output).unwrap();


    // let mut output = vec![0; 6000]; // TODO: need to be the size of max msp messages
    // let mut parser = multiwii_serial_protocol::MspParser::new();
    // let mut packet_parsed = None;
    // for b in output {
    //     let s = parser.parse(b);
    //     if let Ok(Some(p)) = s {
    //         packet_parsed = Some(p);
    //     }
    // }



    // Open the first serialport available.
    let mut serialport = open(&available_ports().expect("No serial port")[0].port_name)
        .expect("Failed to open serial port");

    // Clone the port
    let mut clone = serialport.try_clone().expect("Failed to clone");

    // Send out 4 bytes every second
    // thread::spawn(move || loop {
    //     clone
    //         .write(&output)
    //         .expect("Failed to write to serial port");
    //     thread::sleep(Duration::from_millis(1000));
    // });

    // let mut packet_buf: Vec<u8> = vec![0; 6000];
    // Read the four bytes back from the cloned port

    let mut parser = multiwii_serial_protocol::MspParser::new();
    // let mut packet_parsed = None;

    let mut buffer: [u8; 1] = unsafe { mem::uninitialized() };
    clone
        .write(&output)
        .expect("Failed to write to serial port");
    loop {
        match serialport.read(&mut buffer) {
            Ok(bytes) => {
                if bytes == 1 {
                    let s = parser.parse(buffer[0]);
                    if let Ok(Some(p)) = s {
                        println!("Received: {:?}", p);
                        clone
                            .write(&output)
                            .expect("Failed to write to serial port");
                    }
                }
                // packet_buf.push(buffer[0])
            }
            Err(ref e) if e.kind() == io::ErrorKind::TimedOut => (),
            Err(e) => eprintln!("{:?}", e),
        }
    }

    println!("Hello, world!");
}
