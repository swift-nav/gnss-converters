#![feature(lang_items)]
#![no_std]

#[lang = "eh_personality"] extern fn eh_personality() {}

use core::fmt::Write;
use core::convert::TryInto;

struct Stderr { }

impl Stderr {
    pub fn new() -> Stderr {
        Stderr { }
    }
}

impl core::fmt::Write for Stderr {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        let buf = &s[..];
        unsafe {
            let buf = buf.as_ptr() as *const libc::c_void;
            libc::write(libc::STDERR_FILENO, buf, s.len().try_into().unwrap());
        }
        Ok(()) 
    }
}

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {

    let mut stderr = Stderr::new();
    writeln!(stderr, "{}", info).ok();

    unsafe { libc::abort() }
}
