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
extern "C" {}

use libc::{c_char, c_void};

use gnss_converters::*;

#[link(name = "ubx2sbp_main", kind = "static")]
#[link(name = "gnss_converters", kind = "static")]
#[link(name = "gnss_converters_extra", kind = "static")]
#[link(name = "swiftnav", kind = "static")]
#[link(name = "sbp", kind = "static")]
#[link(name = "rtcm", kind = "static")]
extern "C" {
    fn sbp2rtcm_main(
        argc: i32,
        argv: *const *const i8,
        addition_opts_help: *const c_char,
        readfn: unsafe extern "C" fn(*mut u8, u32, *mut c_void) -> i32,
        writefn: unsafe extern "C" fn(*const u8, u16, *mut c_void) -> i32,
        context: *mut c_void,
    ) -> i32;
}

fn main() {
    let exit_code = {
        let _stdout_flusher = StdoutFlusher::new();
        let (reader, writer) = fetch_io("sbp2rtcm");
        let cargs = CArgs::new();
        let argv = cargs.argv();
        let (argc, argv) = (cargs.len(), argv.as_ptr());
        let mut context = Context { reader, writer };
        unsafe {
            sbp2rtcm_main(
                argc,
                argv,
                ADDITIONAL_OPTS_HELP.as_ptr(),
                readfn_u32,
                writefn_u16,
                &mut context as *mut Context as *mut c_void,
            )
        }
    };
    std::process::exit(exit_code);
}
