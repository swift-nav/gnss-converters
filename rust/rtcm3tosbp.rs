#![no_std]
#![no_main]

#[cfg(target_os = "windows")]
#[windows_subsystem = "console"]

#[cfg(target_os = "windows")]
#[link(name = "msvcrt")]
extern {}

extern crate gnss_converters;

extern "C" {
    fn rtcm3tosbp_main(argc: i32, argv: *const *const u8) -> i32;
}

#[no_mangle]
pub extern fn main(argc: i32, argv: *const *const u8) -> i32 {
    unsafe {
        rtcm3tosbp_main(argc, argv)
    }
}
