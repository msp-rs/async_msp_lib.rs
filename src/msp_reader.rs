// extern crate alloc;
extern crate multiwii_serial_protocol_v2;

use std::pin::Pin;
use async_std::{
    io::{Error, ErrorKind},
    sync::{channel, Sender, Receiver},
    task,
};
use multiwii_serial_protocol_v2::{MspPacket, MspParser};

pub struct MspReader<R> {
    #[pin]
    inner: R,
    parser: MspParser,
}

// TODO: use the async read when async serial is available
impl<R: std::io::Read> MspReader {
    pub fn new(inner: R) -> MspReader {
        let parser = MspParser::new();

        return MspReader {
            inner: inner,
            parser: parser,
        };
    }

    pub fn read<'a>(self: Pin<&'a mut Self>) -> Result<MspPacket> {
        for byte in self.inner.bytes() {
            match byte {
                Ok(byte) => {
                    let res = self.parser.parse(byte);
                    match res {
                        Ok(Some(p)) => return Ok(p),
                        Err(e) => return Err(e),
                        Ok(None) => ()
                    }
                }
                Err(ref e) if e.kind() == ErrorKind::TimedOut => (),
                Err(e) => return Err(e),
            };
        }
	  }

    pub async fn reset_parser<'a>(self: Pin<&'a mut Self>) {
        self.parser.reset();
    }

    pub fn stream_packets<'a>(self: Pin<&'a mut Self>, capacity: usize) -> (Receiver<Result<MspPacket>>, task::JoinHandle<()>) {
        let (sender, receiver) = channel::<Result<MspPacket>>(capacity);
        let task_handle = task::spawn(async move {
            loop {
                let p = self.read();
                sender.send(p);
            }
        });

        (receiver, task_handle)
    }
}
