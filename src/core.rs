extern crate alloc;
extern crate multiwii_serial_protocol;
extern crate serialport;
extern crate packed_struct;

use multiwii_serial_protocol::{MspPacket, MspParser};
use serialport::SerialPort;

use async_std::sync::{channel, Arc, Mutex, Sender, Receiver};
use async_std::{io, task};

use std::time::Duration;
use std::sync::atomic::{AtomicBool, Ordering};


#[derive(Clone)]
pub struct Core {
    parser_locked: Arc<Mutex<MspParser>>,

    msp_reader_send: Sender<MspPacket>,
    msp_reader_recv: Receiver<MspPacket>,
    msp_writer_send: Sender<MspPacket>,
    msp_writer_recv: Receiver<MspPacket>,
}

impl Core {
    /// Create new core msp reader and parser
    pub fn new() -> Core {
        let (msp_reader_send, msp_reader_recv) = channel::<MspPacket>(4096);
        let (msp_writer_send, msp_writer_recv) = channel::<MspPacket>(4096);

        let parser = MspParser::new();
        let parser_locked = Arc::new(Mutex::new(parser));

        return Core {
            parser_locked: parser_locked,
            msp_reader_send: msp_reader_send,
            msp_reader_recv: msp_reader_recv,
            msp_writer_send: msp_writer_send,
            msp_writer_recv: msp_writer_recv,
        };
	  }

    pub fn start(&self, serial: Box<dyn SerialPort>) {
        let serial_clone = serial.try_clone().unwrap();

        Core::process_input(serial, self.parser_locked.clone(), self.msp_reader_send.clone());
        Core::process_output(serial_clone, self.msp_writer_recv.clone());
    }

    pub async fn read(&self) -> std::option::Option<MspPacket> {
        return match self.msp_reader_recv.recv().await {
            None => None,
            Some(packet) => Some(packet),
        };
    }

    pub async fn write(&self, packet: MspPacket)  {
        self.msp_writer_send.send(packet).await;
    }

    // TODO: return joinhandler, so we can stop the tasks on drop
    // TODO: rewrite using stream api with inspect, each command will inspect
    //       and passthorugh to next.
    //       if the stream contained response for command, it will return the read/write function
    fn process_input(
        mut serial: Box<dyn SerialPort>,
        parser_locked: Arc<Mutex<MspParser>>,
        msp_reader_send: Sender<MspPacket>
    ) -> Arc<AtomicBool> {
        let should_stop = Arc::new(AtomicBool::new(false));
        let should_stop_clone = should_stop.clone();

        // task 1: read into input channel from serial(reading from serial is blocking)
        task::spawn(async move {
            while should_stop.load(Ordering::Relaxed) == false {
                let mut serial_buf: Vec<u8> = vec![0; 0x1000];
                match serial.read(serial_buf.as_mut_slice()) {
                    Ok(bytes) => {
                        for n in 0..bytes {
                            match (*parser_locked.lock().await).parse(serial_buf[n]) {
                                Ok(Some(p)) => msp_reader_send.send(p).await,
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
    fn process_output(
        mut serial: Box<dyn SerialPort>,
        msp_writer_recv: Receiver<MspPacket>,
    ) {
        task::spawn(async move {
            loop {
                let packet = match msp_writer_recv.recv().await {
                    None => break,
                    Some(packet) => packet,
                };

                let size = packet.packet_size_bytes_v2();
                let mut output = vec![0; size];

                packet
                    .serialize_v2(&mut output)
                    .expect("Failed to serialize");

                // because inav doesn't support uart flow control, we simply try write untill success
                loop {
                    match serial.write(&output) {
                        Ok(_) => break,
                        Err(ref e) if e.kind() == io::ErrorKind::TimedOut => {
                            // controller is busy/serial buffer is full, sleep and attempt write again
                            task::sleep(Duration::from_millis(1)).await;
                        }
                        Err(e) => eprintln!("failed to write{:?}", e),
                    }
                }
            }
        });
	  }

    pub async fn reset_parser(&self) {
        (*self.parser_locked.lock().await).reset();
    }
}

// impl Clone for Core {
//     fn clone(&self) -> Self {
//         return Core {
//             parser_locked: self.parser_locked.clone(),
//             msp_reader_send: self.msp_reader_send.clone(),
//             msp_reader_recv: self.msp_reader_recv.clone(),
//             msp_writer_send: self.msp_writer_send.clone(),
//             msp_writer_recv: self.msp_writer_recv.clone(),
//         };
//     }
// }
