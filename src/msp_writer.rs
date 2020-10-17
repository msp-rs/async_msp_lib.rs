// extern crate alloc;
extern crate multiwii_serial_protocol_v2;

use std::pin::Pin;
use async_std::{
    io::{Error, ErrorKind},
    sync::{channel, Sender, Receiver},
    task,
};
use multiwii_serial_protocol_v2::{MspPacket, MspParser};

pub struct MspWriter<W> {
    #[pin]
    inner: W,
}

// TODO: use the async read when async serial is available
impl<W: std::io::Writer> MspWriter {
    pub fn new(inner: W) -> MspWriter {
        return MspWriter {
            inner: inner,
        };
    }

    pub fn write<'a>(self: Pin<&'a mut Self>, packet: MspPacket) -> Result<()> {
        let size = packet.packet_size_bytes_v2();
        let mut output = vec![0; size];

        packet.serialize_v2(&mut output)?;

        return self.inner.write(&output) {
            Ok(_) => Ok(()),
            Err(e) => Err(e),
        }
	  }

    pub fn stream_packets<'a>(self: Pin<&'a mut Self>, capacity: usize) -> (Sender<MspPacket>, Receiver<Error>, task::JoinHandle<()>) {
        let (sender, receiver) = channel::<MspPacket>(capacity);
        let (error_send, error_recv) = channel::<Error>(1);
        let task_handle = task::spawn(async move {
            loop {
                let packet = match receiver.recv().await {
                    Err(_) => break,
                    Ok(packet) => packet,
                };

                loop {
                    match self.write(packet) {
                        Ok(_) => break,
                        Err(ref e) if e.kind() == ErrorKind::TimedOut => task::yield_now().await,
                        Err(e) =>  error_send.send(e).await,
                    }
                }
            }
        });

        ((sender, error_recv), task_handle)
    }
}
