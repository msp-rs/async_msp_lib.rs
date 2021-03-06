use std::net::TcpStream;
use async_msp_lib::Msp;
use std::time::Duration;
use std::cell::RefCell;
use async_std::task::block_on;
use std::os::raw::{c_ushort, c_uint, c_char};
use std::slice;
use std::ffi::CStr;


struct ClonableTcpStream(TcpStream);

impl Clone for ClonableTcpStream {
    fn clone(&self) -> Self {
        let clone = (*self).0.try_clone().unwrap();
        return Self(clone);
    }
}

impl std::io::Write for ClonableTcpStream {
    fn write(&mut self, buf: &[u8]) -> std::io::Result<usize> {
        return self.0.write(buf);
    }

    fn flush(&mut self) -> std::io::Result<()> {
        return self.0.flush();
    }
}

impl std::io::Read for ClonableTcpStream {
    fn read(&mut self, buf: &mut [u8]) -> std::io::Result<usize> {
        return self.0.read(buf);
    }
}

thread_local!(static MSP: RefCell<Option<Msp>> = RefCell::new(None));

#[no_mangle]
pub extern fn open(s: *const c_char, buff: c_uint, verbose: c_char) {
    let c_str = unsafe {
        assert!(!s.is_null());

        CStr::from_ptr(s)
    };
    let stream = ClonableTcpStream(TcpStream::connect(c_str.to_str().unwrap()).unwrap());
    MSP.with(|msp_cell| {
        let is_verbose = verbose != 0;
        let (msp, _) = Msp::open(stream, buff as usize, Duration::from_millis(0), is_verbose);
        msp_cell.replace(Some(msp));
    });
}

#[no_mangle]
pub extern fn set_raw_rc(array: *const c_ushort, length: c_uint) {
    MSP.with(|msp_cell| {
        let msp = msp_cell.borrow_mut();
        assert!(!array.is_null(), "Null pointer in sum()");
        let channels: &[c_ushort] = unsafe {
            slice::from_raw_parts(array, length as usize)
        };
        block_on(async {
            msp.as_ref().unwrap().set_raw_rc(channels.to_vec()).await.unwrap();
        });
    });
}

