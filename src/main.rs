extern crate alloc;
extern crate multiwii_serial_protocol;

extern crate serialport;

use serialport::{available_ports, open};
use std::io;
use std::io::Write;

use async_std::sync::channel;
use async_std::task;

#[async_std::main]
async fn main() {
    let (serial_send, serial_recv) = channel(1);

    let mut serialport = open(&available_ports().expect("No serial port")[0].port_name)
        .expect("Failed to open serial port");

    // Clone the port
    let mut clone = serialport.try_clone().expect("Failed to clone");

    // green-thread 1: read into input channel from serial(reading from serial is blocking)
    task::spawn(async move {
        loop {
            let mut serial_buf: Vec<u8> = vec![0; 1];
            match serialport.read(serial_buf.as_mut_slice()) {
                Ok(bytes) => { // where does the bytes appeared
                    if bytes == 1 { // what if we received more then one? will we miss it? 
                        serial_send.send(serial_buf[0]).await;
                    }
                }
                Err(ref e) if e.kind() == io::ErrorKind::TimedOut => task::yield_now().await,
                Err(e) => eprintln!("{:?}", e),
            }
        }
    });

    let (msp_send, msp_recv) = channel(1);

    // green-thread 2: parse the packets from channel
    //               : on new packet write into output channel the next request packet
    task::spawn(async move {
        let mut parser = multiwii_serial_protocol::MspParser::new();
        loop {
            // TODO: i think we should use epoll to know when to read the serial
            match serial_recv.recv().await {
                None => break,
                Some(b) => {
                    let s = parser.parse(b);
                    if let Ok(Some(p)) = s {
                        msp_send.send(p).await;
                    }
                },
            }
        }
    });


    let packet = multiwii_serial_protocol::MspPacket {
		    cmd: multiwii_serial_protocol::MspCommandCode::MSP_DATAFLASH_SUMMARY as u8,
		    direction: multiwii_serial_protocol::MspPacketDirection::ToFlightController,
		    data: alloc::borrow::Cow::Owned(vec![])
	  };
    let size = packet.packet_size_bytes();
    let mut output = vec![0; size];
    packet.serialize(&mut output).unwrap();

    clone
        .write(&output)
        .expect("Failed to write to serial port");

    // green-thread 3: read from the write channel and write to serial(writing to serial is blocking)
    let request_next_packet = task::spawn(async move {
        loop {
            match msp_recv.recv().await {
                None => break,
                Some(packet) => {
                    println!("Received: {:?}", packet);
                    clone
                        .write(&output)
                        .expect("Failed to write to serial port");
                }
            }
        }
    });

    request_next_packet.await;

    // TODO: use arg parse of some sort
}
