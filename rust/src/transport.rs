//! Private binding to the library's existing ZMQ transport. No motor I/O here.
use crate::{Result, err};
use std::{
    ffi::{CString, c_int, c_void},
    sync::Arc,
};
type Ptr = *mut c_void;
pub struct Api {
    _lib: libloading::Library,
    new: unsafe extern "C" fn() -> Ptr,
    term: unsafe extern "C" fn(Ptr) -> c_int,
    socket: unsafe extern "C" fn(Ptr, c_int) -> Ptr,
    close: unsafe extern "C" fn(Ptr) -> c_int,
    option: unsafe extern "C" fn(Ptr, c_int, *const c_void, usize) -> c_int,
    get_option: unsafe extern "C" fn(Ptr, c_int, *mut c_void, *mut usize) -> c_int,
    bind: unsafe extern "C" fn(Ptr, *const i8) -> c_int,
    connect: unsafe extern "C" fn(Ptr, *const i8) -> c_int,
    send: unsafe extern "C" fn(Ptr, *const c_void, usize, c_int) -> c_int,
    recv: unsafe extern "C" fn(Ptr, *mut c_void, usize, c_int) -> c_int,
    errno: unsafe extern "C" fn() -> c_int,
}
impl Api {
    pub fn load(path: Option<&std::path::Path>) -> Result<Arc<Self>> {
        let mut found = None;
        if let Some(path) = path {
            found = Some(unsafe { libloading::Library::new(path) }?);
        }
        for name in [
            "libzmq.so.5",
            "libzmq.so",
            "libzmq.5.dylib",
            "/opt/homebrew/lib/libzmq.5.dylib",
        ] {
            if found.is_some() {
                break;
            }
            if let Ok(l) = unsafe { libloading::Library::new(name) } {
                found = Some(l);
                break;
            }
        }
        let l = found.ok_or_else(|| err("libzmq unavailable"))?;
        unsafe {
            Ok(Arc::new(Self {
                new: *l.get(b"zmq_ctx_new\0")?,
                term: *l.get(b"zmq_ctx_term\0")?,
                socket: *l.get(b"zmq_socket\0")?,
                close: *l.get(b"zmq_close\0")?,
                option: *l.get(b"zmq_setsockopt\0")?,
                get_option: *l.get(b"zmq_getsockopt\0")?,
                bind: *l.get(b"zmq_bind\0")?,
                connect: *l.get(b"zmq_connect\0")?,
                send: *l.get(b"zmq_send\0")?,
                recv: *l.get(b"zmq_recv\0")?,
                errno: *l.get(b"zmq_errno\0")?,
                _lib: l,
            }))
        }
    }
}
pub struct Context {
    api: Arc<Api>,
    ptr: Ptr,
}
impl Context {
    pub fn new(path: Option<&std::path::Path>) -> Result<Self> {
        let api = Api::load(path)?;
        let ptr = unsafe { (api.new)() };
        if ptr.is_null() {
            return Err(err("ZMQ context failed"));
        }
        Ok(Self { api, ptr })
    }
}
impl Drop for Context {
    fn drop(&mut self) {
        unsafe {
            (self.api.term)(self.ptr);
        }
    }
}
pub struct Socket {
    api: Arc<Api>,
    ptr: Ptr,
    topic: Vec<u8>,
}
impl Socket {
    pub fn new(c: &Context, topic: &str, publisher: bool) -> Result<Self> {
        let p = unsafe { (c.api.socket)(c.ptr, if publisher { 1 } else { 2 }) };
        if p.is_null() {
            return Err(err("ZMQ socket failed"));
        }
        let s = Self {
            api: c.api.clone(),
            ptr: p,
            topic: topic.as_bytes().to_vec(),
        };
        s.option(17, 0)?;
        s.option(if publisher { 23 } else { 24 }, 2)?;
        if !publisher && unsafe { (s.api.option)(p, 6, s.topic.as_ptr().cast(), s.topic.len()) } < 0
        {
            return Err(err("ZMQ subscribe failed"));
        }
        let mut bound = false;
        for port in ports(topic) {
            let address = CString::new(format!("tcp://127.0.0.1:{port}"))?;
            let result = unsafe {
                if publisher {
                    (s.api.bind)(p, address.as_ptr())
                } else {
                    (s.api.connect)(p, address.as_ptr())
                }
            };
            if publisher && result == 0 {
                bound = true;
                break;
            }
            if !publisher && result < 0 {
                return Err(err("ZMQ connect failed"));
            }
        }
        if publisher && !bound {
            return Err(err("ZMQ bind candidates exhausted"));
        }
        Ok(s)
    }
    fn option(&self, key: i32, value: i32) -> Result<()> {
        if unsafe { (self.api.option)(self.ptr, key, (&value as *const i32).cast(), 4) } < 0 {
            return Err(err("ZMQ option failed"));
        }
        Ok(())
    }
    pub fn send(&self, b: &[u8]) -> Result<()> {
        if unsafe {
            (self.api.send)(
                self.ptr,
                self.topic.as_ptr().cast(),
                self.topic.len(),
                2 | 1,
            )
        } < 0
            || unsafe { (self.api.send)(self.ptr, b.as_ptr().cast(), b.len(), 1) } < 0
        {
            return Err(err("ZMQ publish failed"));
        }
        Ok(())
    }
    pub fn recv(&self) -> Result<Option<Vec<u8>>> {
        let mut t = [0u8; 512];
        let n = unsafe { (self.api.recv)(self.ptr, t.as_mut_ptr().cast(), t.len(), 1) };
        if n < 0 {
            let e = unsafe { (self.api.errno)() };
            return if e == libc::EAGAIN {
                Ok(None)
            } else {
                Err(err(format!("ZMQ receive {e}")))
            };
        }
        if !self.more()? {
            return Err(err("native packet missing payload frame"));
        }
        let mut p = [0u8; 1024];
        let n2 = unsafe { (self.api.recv)(self.ptr, p.as_mut_ptr().cast(), p.len(), 1) };
        if n as usize > t.len() || n2 < 0 || n2 as usize > p.len() || self.more()? {
            return Err(err("invalid native multipart packet"));
        }
        if t[..n as usize] != self.topic {
            return Ok(None);
        }
        Ok(Some(p[..n2 as usize].to_vec()))
    }
    fn more(&self) -> Result<bool> {
        let mut value = 0i32;
        let mut size = std::mem::size_of_val(&value);
        if unsafe {
            (self.api.get_option)(self.ptr, 13, (&mut value as *mut i32).cast(), &mut size)
        } < 0
        {
            return Err(err("native multipart status unavailable"));
        }
        Ok(value != 0)
    }
}
impl Drop for Socket {
    fn drop(&mut self) {
        unsafe {
            (self.api.close)(self.ptr);
        }
    }
}
pub fn ports(topic: &str) -> Vec<u16> {
    fn hash(s: &str) -> u32 {
        s.bytes()
            .fold(2166136261, |h, b| (h ^ u32::from(b)).wrapping_mul(16777619))
    }
    let first = hash(topic);
    let mut step = hash(&format!("{topic}\x1fprobe")) % 1000;
    if step == 0 {
        step = 1;
    }
    if step % 2 == 0 {
        step += 1;
    }
    while step % 5 == 0 {
        step += 2;
    }
    step %= 1000;
    if step == 0 {
        step = 1;
    }
    candidate_ports(first, step)
}
fn candidate_ports(first: u32, step: u32) -> Vec<u16> {
    (0u32..16)
        .map(|i| 8500 + (first.wrapping_add(i * step) % 1000) as u16)
        .collect()
}
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn candidate_ports_match_native_uint32_wrap() {
        // TopicZmq::generate_port_candidates_from_topic adds uint32_t before
        // reducing modulo 1000, including rare near-UINT32_MAX hashes.
        assert_eq!(candidate_ports(u32::MAX, 13)[..3], [8795, 8512, 8525]);
        assert_eq!(candidate_ports(100, 13)[..3], [8600, 8613, 8626]);
        assert_eq!(ports("vr-test").len(), 16);
    }
}
