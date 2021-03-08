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

use std::boxed::Box;
use std::env;
use std::ffi::CString;
use std::fs::File;

use std::io::{self, BufReader, BufWriter, Read, Write};
use std::path::Path;
use std::slice;

use if_chain::if_chain;
use lazy_static::lazy_static;

use libc::{c_char, c_int, c_void};

const _ADDITIONAL_OPTS_HELP: &str = " [<INPUT> [<OUTPUT>]]";

lazy_static! {
    pub static ref ADDITIONAL_OPTS_HELP: CString =
        CString::new(_ADDITIONAL_OPTS_HELP).expect("failed to create ADDITIONAL_OPTS_HELP");
    static ref STDOUT: std::io::Stdout = std::io::stdout();
    static ref STDIN: std::io::Stdin = std::io::stdin();
}

pub struct Context {
    pub reader: Box<dyn Read>,
    pub writer: Box<dyn Write>,
}

/// # Safety
pub unsafe extern "C" fn readfn_u32(bytes: *mut u8, n_bytes: u32, ctx: *mut c_void) -> i32 {
    readfn(bytes, n_bytes as usize, ctx)
}

/// # Safety
pub unsafe extern "C" fn readfn(bytes: *mut u8, n_bytes: usize, ctx: *mut c_void) -> i32 {
    let context: &mut Context = &mut *(ctx as *mut Context);
    let slice = slice::from_raw_parts_mut(bytes, n_bytes);
    if let Ok(read_size) = context.reader.read(slice) {
        read_size as i32
    } else {
        -1
    }
}

/// # Safety
pub unsafe extern "C" fn writefn_u16(bytes: *const u8, n_bytes: u16, ctx: *mut c_void) -> i32 {
    writefn(bytes, n_bytes as usize, ctx)
}

/// # Safety
pub unsafe extern "C" fn writefn_u32(bytes: *const u8, n_bytes: u32, ctx: *mut c_void) -> i32 {
    writefn(bytes, n_bytes as usize, ctx)
}

/// # Safety
pub unsafe extern "C" fn writefn(bytes: *const u8, n_bytes: usize, ctx: *mut c_void) -> i32 {
    let context: &mut Context = &mut *(ctx as *mut Context);
    let slice = slice::from_raw_parts(bytes, n_bytes);
    context.writer.write(slice).expect("failed to write data") as i32
}

pub struct StdoutFlusher(&'static io::Stdout);

impl StdoutFlusher {
    pub fn new() -> StdoutFlusher {
        StdoutFlusher(&STDOUT)
    }
}

impl Default for StdoutFlusher {
    fn default() -> Self {
        Self::new()
    }
}

impl Drop for StdoutFlusher {
    fn drop(&mut self) {
        self.0.lock().flush().expect("failed to flush stdout");
    }
}

#[derive(Default, Debug)]
pub struct CArgs(Vec<CString>);

impl CArgs {
    pub fn new() -> CArgs {
        CArgs(env::args().map(|arg| CString::new(arg).unwrap()).collect())
    }
    pub fn is_empty(&self) -> bool {
        self.0.is_empty()
    }
    pub fn len(&self) -> c_int {
        self.0.len() as c_int
    }
    pub fn argv(&self) -> Vec<*const c_char> {
        self.0.iter().map(|arg| arg.as_ptr()).collect()
    }
}

pub fn fetch_io(_prog_name: &str) -> (Box<dyn Read>, Box<dyn Write>) {
    // Reverse and grab all args until the first option argument
    let args: Vec<String> = env::args()
        .skip(1)
        .rev()
        .take(2)
        .take_while(|s| !s.starts_with('-'))
        .collect();
    // Reverse again
    let args: Vec<&String> = args.iter().rev().collect();

    fn file_in_file_out(input: &str, output: &str) -> (Box<dyn Read>, Box<dyn Write>) {
        (
            Box::new(BufReader::new(
                File::open(input).expect("failed to open input file"),
            )),
            Box::new(BufWriter::new(
                File::create(output).expect("failed to open output file"),
            )),
        )
    }

    fn file_in_stdout(input: &str) -> (Box<dyn Read>, Box<dyn Write>) {
        (
            Box::new(BufReader::new(
                File::open(input).expect("failed to open input file"),
            )),
            Box::new(BufWriter::new(Box::new(STDOUT.lock()))),
        )
    }

    fn stdin_stdout() -> (Box<dyn Read>, Box<dyn Write>) {
        (
            Box::new(BufReader::new(Box::new(STDIN.lock()))),
            Box::new(BufWriter::new(Box::new(STDOUT.lock()))),
        )
    }

    // Case for: <prog> -x <arg> <input_file> <output_file>
    if_chain! {
        if args.len() == 2;
        if Path::new(&args[0]).is_file();
        then {
            return file_in_file_out(args[0], args[1]);
        }
    }

    // Case for: <prog> -x <arg> <input_file>
    if_chain! {
        if args.len() == 2;
        if ! Path::new(&args[0]).is_file();
        if Path::new(&args[1]).is_file();
        then {
            return file_in_stdout(args[1]);
        }
    }

    // Otherwise return stdin and stdout
    stdin_stdout()
}
