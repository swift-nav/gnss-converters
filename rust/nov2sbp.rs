/*
 * Copyright (C) 2020 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#[cfg(target_os = "windows")]
#[windows_subsystem = "console"]
#[cfg(target_os = "windows")]
#[link(name = "msvcrt")]
extern "C" {}

use std::boxed::Box;
use std::error::Error;
use std::result::Result;

use libc::{c_char, c_void};

use gnss_converters::*;

#[link(name = "nov2sbp_main", kind = "static")]
#[link(name = "novatel-parser", kind = "static")]
#[link(name = "sbp", kind = "static")]
#[link(name = "swiftnav", kind = "static")]
extern "C" {
    fn nov2sbp_main(
        argc: i32,
        argv: *const *const i8,
        addition_opts_help: *const c_char,
        readfn: unsafe extern "C" fn(*mut u8, usize, *mut c_void) -> i32,
        writefn: unsafe extern "C" fn(*const u8, usize, *mut c_void) -> i32,
        context: *mut c_void,
    ) -> i32;

}

fn main() -> Result<(), Box<dyn Error>> {
    let exit_code = {
        let _stdout_flusher = StdoutFlusher::new();
        let (reader, writer) = fetch_io("nov2sbp");
        let mut context = Context { reader, writer };
        let cargs = CArgs::new();
        let argv = cargs.argv();
        let (argc, argv) = (cargs.len(), argv.as_ptr());
        unsafe {
            nov2sbp_main(
                argc,
                argv,
                ADDITIONAL_OPTS_HELP.as_ptr(),
                readfn,
                writefn,
                &mut context as *mut Context as *mut c_void,
            )
        }
    };
    std::process::exit(exit_code);
}
