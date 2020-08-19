extern crate alloc;
extern crate multiwii_serial_protocol_v2;
extern crate serialport;
extern crate packed_struct;

use multiwii_serial_protocol_v2::{MspPacket, MspParser};

use async_std::sync::{channel, Arc, Condvar, Mutex, Sender, Receiver};
use async_std::task;

use std::time::{Duration, Instant};
use std::sync::atomic::{AtomicBool, Ordering};
use std::collections::VecDeque;


#[derive(Clone)]
pub struct Core {
    parser_locked: Arc<Mutex<MspParser>>,
    verbose: bool,
    buff_size: usize,
    msp_write_delay: Duration,

    msp_reader_send: Sender<MspPacket>,
    msp_reader_recv: Receiver<MspPacket>,
    msp_writer_send: Sender<MspPacket>,
    msp_writer_recv: Receiver<MspPacket>,
}

impl Core {
    /// Create new core msp reader and parser
    pub fn new(buff_size: usize, msp_write_delay: Duration, verbose: bool) -> Core {
        let (msp_reader_send, msp_reader_recv) = channel::<MspPacket>(4096);
        let write_buff_size = match buff_size {
            0 => 1,
            _ => buff_size,
        };
        let (msp_writer_send, msp_writer_recv) = channel::<MspPacket>(write_buff_size);

        let parser = MspParser::new();
        let parser_locked = Arc::new(Mutex::new(parser));

        return Core {
            buff_size,
            msp_write_delay,
            verbose,
            parser_locked,
            msp_reader_send,
            msp_reader_recv,
            msp_writer_send,
            msp_writer_recv,
        };
	  }

    pub fn start(&self, stream: impl Send + std::io::Read + std::io::Write + Clone + 'static) {
        let serial_write_lock = Arc::new((Mutex::new(self.buff_size.clone()), Condvar::new()));
        let serial_write_lock_clone = serial_write_lock.clone();

        let elapsed_queue_lock = Arc::new(Mutex::new(VecDeque::with_capacity(self.buff_size.clone())));
        let elapsed_queue_lock_clone = elapsed_queue_lock.clone();

        if &self.buff_size > &0 {
            let reader = stream.clone();
            Core::process_input(reader, self.parser_locked.clone(), self.msp_reader_send.clone(), serial_write_lock, elapsed_queue_lock, self.verbose.clone());
        }
        Core::process_output(stream, self.msp_writer_recv.clone(), serial_write_lock_clone, self.msp_write_delay.clone(), elapsed_queue_lock_clone, self.verbose.clone());
    }

    pub async fn read(&self) -> std::option::Option<MspPacket> {
        return match self.msp_reader_recv.recv().await {
            Err(_) => None,
            Ok(packet) => Some(packet),
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
        mut serial: impl Send + std::io::Read + 'static,
        parser_locked: Arc<Mutex<MspParser>>,
        msp_reader_send: Sender<MspPacket>,
        serial_write_lock: Arc<(Mutex<usize>, Condvar)>,
        elapsed_queue_lock: Arc<Mutex<VecDeque<Instant>>>,
        verbose: bool,
    ) -> Arc<AtomicBool> {
        // TODO: remove the should stop, once this object gets dropped, this will stop
        let should_stop = Arc::new(AtomicBool::new(false));
        let should_stop_clone = should_stop.clone();

        // task 1: read into input channel from serial(reading from serial is blocking)
        task::spawn(async move {
            let (lock, cvar) = &*serial_write_lock;
            let initial_lock = lock.lock().await;
            let initial_buffer_size = *initial_lock;
            drop(initial_lock);

            let mut serial_buf: Vec<u8> = vec![0; 0x1000];
            while should_stop.load(Ordering::Relaxed) == false {
                match serial.read(serial_buf.as_mut_slice()) {
                    Ok(bytes) => {
                        let mut parser = parser_locked.lock().await;
                        for n in 0..bytes {
                            let res = parser.parse(serial_buf[n]);
                            match res {
                                Ok(Some(p)) => {
                                    if verbose {
                                        println!("receive new msp packet {}", p.cmd);
                                        match (*elapsed_queue_lock.lock().await).pop_front() {
                                            Some(instant) => println!("elapsed time since send {}", instant.elapsed().subsec_millis()),
                                            None => (),
                                        };
                                    }

                                    msp_reader_send.send(p).await;

                                    // lock the condvar here and update to true, and decrement the sent packets count
                                    let mut received_lock = lock.lock().await;
                                    if *received_lock < initial_buffer_size {
                                        *received_lock += 1;
                                        // We notify the condvar that the value has changed.
                                        cvar.notify_one();
                                    }
                                },
                                Err(e) => eprintln!("bad crc {:?}", e),
                                Ok(None) => ()
                            }
                        }
                    }
                    Err(ref e) if e.kind() == async_std::io::ErrorKind::TimedOut => {
                        if verbose {
                            println!("read timeout");
                        }
                    }
                    Err(e) => eprintln!("{:?}", e),
                }

                task::yield_now().await;
            }
        });
        return should_stop_clone;
	  }

    // TODO: return joinhandler, so we can stop the tasks on drop
    fn process_output(
        mut serial: impl Send + std::io::Write + 'static,
        msp_writer_recv: Receiver<MspPacket>,
        serial_write_lock: Arc<(Mutex<usize>, Condvar)>,
        write_delay: Duration,
        elapsed_queue_lock: Arc<Mutex<VecDeque<Instant>>>,
        verbose: bool,
    ) {
        task::spawn(async move {
            let (lock, cvar) = &*serial_write_lock;

            let temp_lock_guard = lock.lock().await;
            let should_wait_for_lock = *temp_lock_guard > 0;
            drop(temp_lock_guard);

            loop {
                // lock here counter for sent packets
                // if counter is more then buffer size(10), lock then 10 turn the value to false and continue the loop
                // essentially waiting for value to change
                if should_wait_for_lock {
                    let guard = cvar.wait_until(lock.lock().await, |send_count| {
                        if *send_count > 0 {
                            *send_count -=1;
                            return true;
                        }

                        return false;
                    }).await;
                    drop(guard);
                }

                let packet = match msp_writer_recv.recv().await {
                    Err(_) => break,
                    Ok(packet) => packet,
                };

                let size = packet.packet_size_bytes_v2();
                let mut output = vec![0; size];

                packet
                    .serialize_v2(&mut output)
                    .expect("Failed to serialize");

                if verbose {
                    println!("writing {}", packet.cmd);
                }

                // because inav doesn't support uart flow control, we simply try write untill success
                loop {
                    match serial.write(&output) {
                        Ok(_) => {
                            if verbose && should_wait_for_lock {
                                (*elapsed_queue_lock.lock().await).push_back(Instant::now());
                            }

                            break;
                        },
                        Err(ref e) if e.kind() == async_std::io::ErrorKind::TimedOut => {
                            // controller is busy/serial buffer is full, sleep and attempt write again
                            task::yield_now().await;
                        },
                        Err(e) => {
                            eprintln!("failed to write{:?}", e);
                            *(lock.lock().await) += 1;
                        }
                    }
                }

                if write_delay > Duration::from_millis(0) {
                    task::sleep(write_delay).await;
                }

                task::yield_now().await;
            }
        });
	  }

    pub async fn reset_parser(&self) {
        (*self.parser_locked.lock().await).reset();
    }

    pub fn buff_size(&self) -> usize {
        self.buff_size
    }
}
