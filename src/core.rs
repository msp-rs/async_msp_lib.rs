extern crate alloc;
extern crate multiwii_serial_protocol_v2;
extern crate serialport;
extern crate packed_struct;

use multiwii_serial_protocol_v2::{MspPacket, MspParser};

use async_std::sync::{channel, Arc, Condvar, Mutex, Sender, Receiver};
use async_std::task;
use async_std::io::{Error, ErrorKind};

use std::time::{Duration, Instant};
use std::collections::VecDeque;


#[derive(Clone)]
pub struct Core {
    parser_locked: Arc<Mutex<MspParser>>,
    verbose: bool,
    buff_size: usize,
    msp_write_delay: Duration,

    msp_reader_recv: Receiver<Result<MspPacket, Error>>,
    msp_writer_send: Sender<MspPacket>,
    msp_error_recv: Receiver<Error>,
}

pub struct MspTaskHandle {
    input_handle: Option<async_std::task::JoinHandle<()>>,
    output_handle: async_std::task::JoinHandle<()>,
}

impl MspTaskHandle {
    pub async fn cancel(self) {
        self.output_handle.cancel().await;
        match self.input_handle {
            Some(h) => h.cancel().await,
            None => None,
        };
    }
}

impl Core {
    // i am thinking to close the first sender and let all the senders after collapse, but its problamatic when we have clones
    /// Create new core msp reader and parser
    pub fn open(stream: impl Send + std::io::Read + std::io::Write + Clone + 'static,
                buff_size: usize,
                msp_write_delay: Duration,
                verbose: bool,
    ) -> (Core, MspTaskHandle) {
        let (msp_reader_send, msp_reader_recv) = channel::<Result<MspPacket, Error>>(4096);
        let (msp_writer_send, msp_writer_recv) = channel::<MspPacket>(1024);
        let (msp_error_send, msp_error_recv) = channel::<Error>(1);

        let parser = MspParser::new();
        let parser_locked = Arc::new(Mutex::new(parser));

        let serial_write_lock = Arc::new((Mutex::new(buff_size.clone()), Condvar::new()));
        let serial_write_lock_clone = serial_write_lock.clone();

        let elapsed_queue_lock = Arc::new(Mutex::new(VecDeque::with_capacity(buff_size.clone())));
        let elapsed_queue_lock_clone = elapsed_queue_lock.clone();

        let input_handle = if buff_size > 0 {
            let reader = stream.clone();
            Some(Core::process_input(reader, parser_locked.clone(), msp_reader_send, serial_write_lock, elapsed_queue_lock, verbose.clone()))
        } else {
            None
        };

        let output_handle = Core::process_output(stream, msp_writer_recv.clone(), serial_write_lock_clone, msp_write_delay.clone(), msp_error_send, elapsed_queue_lock_clone, verbose.clone());

        return (Core {
            buff_size,
            msp_write_delay,
            verbose,
            parser_locked,
            msp_reader_recv,
            msp_writer_send,
            msp_error_recv,
        }, MspTaskHandle {
            input_handle,
            output_handle
        });
    }

    pub async fn read(&self) -> Result<MspPacket, Error> {
        return match self.msp_reader_recv.recv().await {
            Ok(packet_res) => packet_res,
            Err(_) => Err(Error::new(ErrorKind::BrokenPipe, "reader thread exited")),
        };
    }

    pub async fn write(&self, packet: MspPacket) -> Result<(), Error> {
        println!("111111111111111111111");
        self.msp_writer_send.send(packet).await;
        println!("2222222222222222222222222");
        match self.msp_error_recv.try_recv() {
            Ok(packet) => {
                eprintln!("should have checked the results");
                return Err(packet);
            },
            Err(_) => (),
        };

        println!("3333333333333333333333333");

        Ok(())
    }

    // TODO: return joinhandler, so we can stop the tasks on drop
    // TODO: rewrite using stream api with inspect, each command will inspect
    //       and passthorugh to next.
    //       if the stream contained response for command, it will return the read/write function
    // what if i break, the reader and the writer on error.
    // its like a broken pipe, something broke and we can't operate anymore
    // we should halt throw an error and let the upper level deal with it
    // or maybe in async thats not the way to deliver errors, maybe we need another channel for errors
    // and we will break when there is an error and propogate it top
    // because right now ther user won't receive an error until the next attempt to read or write
    fn process_input(
        serial: impl Send + std::io::Read + 'static,
        parser_locked: Arc<Mutex<MspParser>>,
        msp_reader_send: Sender<Result<MspPacket, Error>>,
        serial_write_lock: Arc<(Mutex<usize>, Condvar)>,
        elapsed_queue_lock: Arc<Mutex<VecDeque<Instant>>>,
        verbose: bool,
    ) -> async_std::task::JoinHandle<()> {
        // task 1: read into input channel from serial(reading from serial is blocking)
        task::spawn(async move {
            let (lock, cvar) = &*serial_write_lock;
            let initial_lock = lock.lock().await;
            let initial_buffer_size = *initial_lock;
            drop(initial_lock);

            for byte in serial.bytes() {
                match byte {
                    Ok(byte) => {
                        let mut parser = parser_locked.lock().await;
                        let res = parser.parse(byte);
                        match res {
                            Ok(Some(p)) => {
                                if verbose {
                                    println!("receive new msp packet {}", p.cmd);
                                    match (*elapsed_queue_lock.lock().await).pop_front() {
                                        Some(instant) => println!("elapsed time since send {}", instant.elapsed().subsec_millis()),
                                        None => (),
                                    };
                                }

                                msp_reader_send.send(Ok(p)).await;

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
                    Err(ref e) if e.kind() == ErrorKind::TimedOut => {
                        if verbose {
                            println!("read timeout");
                        }
                    }
                    Err(e) => {
                        eprintln!("read read read");
                        msp_reader_send.send(Err(e)).await;
                        break;
                    }
                };
                task::yield_now().await;
            }
        })
	  }

    // TODO: return joinhandler, so we can stop the tasks on drop
    fn process_output(
        mut serial: impl Send + std::io::Write + 'static,
        msp_writer_recv: Receiver<MspPacket>,
        serial_write_lock: Arc<(Mutex<usize>, Condvar)>,
        write_delay: Duration,
        msp_error_send: Sender<Error>,
        elapsed_queue_lock: Arc<Mutex<VecDeque<Instant>>>,
        verbose: bool,
    ) -> async_std::task::JoinHandle<()> {
        task::spawn(async move {
            let (lock, cvar) = &*serial_write_lock;

            let mut should_wait_for_lock = false;
            let temp_lock_guard = lock.lock().await;
            if *temp_lock_guard > 0 {
                should_wait_for_lock = true;
            }
            drop(temp_lock_guard);

            'outer: loop {
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
                        Err(ref e) if e.kind() == ErrorKind::TimedOut => {
                            // controller is busy/serial buffer is full, sleep and attempt write again
                            task::yield_now().await;
                        },
                        Err(e) => {
                            eprintln!("write write write");
                            msp_error_send.try_send(e);
                            break;
                            // break 'outer;
                            *(lock.lock().await) += 1;
                        }
                    }
                }

                if write_delay > Duration::from_millis(0) {
                    task::sleep(write_delay).await;
                }

                task::yield_now().await;
            }
        })
	  }

    pub async fn reset_parser(&self) {
        (*self.parser_locked.lock().await).reset();
    }

    pub fn buff_size(&self) -> usize {
        self.buff_size
    }
}
