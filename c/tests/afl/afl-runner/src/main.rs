use std::fs;
use std::collections::HashMap;
use std::path::Path;
use std::time::Duration;

use lazy_static::lazy_static;
use regex::Regex;
use walkdir::WalkDir;

use crossbeam_channel as channel;

use threadpool::ThreadPool;

use clap::{App, Arg};

use subprocess::{Popen, PopenConfig, Redirection};

enum CmdResult {
    PASS,
    FAIL,
    TIMEOUT,
    UNKNOWN,
}

fn work(
    rx: channel::Receiver<walkdir::DirEntry>,
    result_tx: channel::Sender<(CmdResult, String, Option<String>)>,
    cmd_args: &Vec<&str>,
    base_path: &Path,
    fixed_path: &Path,
    categorize: bool
) {
    for test_dir_path in rx {
        let test_path = test_dir_path.into_path();
        let (test_result, test_output) = run_binary_command(cmd_args, &test_path);

        let test_name = test_path.file_name().unwrap().to_string_lossy();

        let category = match test_output {
            Some(x) => analyse_test_result(&x),
            None => None,
        };

        match test_result {
            CmdResult::PASS => {
                if categorize {
                    let new_path = fixed_path.join(test_name.to_string());
                    fs::rename(&test_path, new_path).expect("Failed to move test to fixed dir");
                }
            }, CmdResult::FAIL => {
                if categorize{
                    if let Some(cat) = &category {
                        move_test_case(&test_path, base_path, &cat);
                    }
                }
            }, CmdResult::TIMEOUT | CmdResult::UNKNOWN => {

            }
        }

        result_tx
        .send((test_result, test_name.to_string(), category))
        .expect("Error sending to result chan");
    }
}

fn move_test_case(test_path: &Path, base_path: &Path, category: &str) {
    let path = base_path.join(category);
    fs::create_dir_all(path.clone()).expect("Error creating dir");

    let new_path = path.join(test_path.file_name().unwrap());

    fs::rename(test_path, new_path).expect("Failed to move test");
}

fn get_test_cases(dir: &Path) -> Vec<walkdir::DirEntry> {
    WalkDir::new(dir)
        .follow_links(false)
        .into_iter()
        .filter_map(|e| e.ok())
        .filter(|e| e.file_name().to_string_lossy().contains("id:"))
        .collect()
}

fn run_binary_command(
    cmd_args: &Vec<&str>,
    test_path: &Path,
) -> (CmdResult, Option<String>) {

    // POpen is used rather than the standard Process library to allow easy 
    // writing to stdin while reading from stdout/stderr to occur 
    // simultaneously. Without this, the cmd under test can block when its
    // stdout fills up which creates a deadlock.
    let mut p = Popen::create(cmd_args, PopenConfig {
        stdout: Redirection::Pipe, stderr: Redirection::Pipe, stdin: Redirection::Pipe, ..Default::default()
    }).expect("Couldn't create process");
    let data = fs::read(test_path).expect("Unable to read file");

    let err = match p.communicate_bytes(Some(&data)) {
        Ok((_, err)) => {
            err
        },

        Err(_why) => {
            return (CmdResult::UNKNOWN, None);
        }
    };

    let timeout = Duration::new(1, 0);
    let result = p.wait_timeout(timeout);

    match result {
        Ok(exit_status) => {
            if let Some(status) = exit_status {
                if status.success() {
                    return (CmdResult::PASS, None);
                }
            } else {
                return (CmdResult::TIMEOUT, None);
            }
        }, Err(why) => {
            panic!("Error running {}", why)
        }
    }

    if let Some(err_msg) = err {
        return (CmdResult::FAIL, Some(String::from_utf8_lossy(&err_msg).to_string()));
    }

    return (CmdResult::FAIL, Some("unknown".to_string()))
}

fn analyse_test_result(result: &str) -> Option<String> {
    lazy_static! {
        static ref ASSERT_RE: Regex =
            Regex::new(r#"(?:.*)/(.*):(.*\d):\s(\w*)(?:: Assertion )(.*)'"#).unwrap();

        static ref ASAN_RE: Regex =
            Regex::new(r#"AddressSanitizer: (.*) /(?:.*)/(.*) in .*"#).unwrap();

            // todo: add asan error regex
    }

    match ASSERT_RE.captures(result) {
        Some(x) => {
            let category = "assert_".to_owned()
                + x.get(1).unwrap().as_str()
                + "_"
                + x.get(3).unwrap().as_str()
                + "_ln"
                + x.get(2).unwrap().as_str();
            return Some(category);
        }
        None => {}
    }

    match ASAN_RE.captures(result) {
        Some(x) => {
            let category = "asan_".to_owned()
                + x.get(1).unwrap().as_str()
                + "_"
                + x.get(2).unwrap().as_str();
            return Some(category);
        }
        None => {}
    }

    return None;
}

fn main() {
    let matches = App::new("afl-runner")
        .version("1.0")
        .arg(
            Arg::new("CRASH_DIR")
                .about("Path to directory to recursively search for test cases to run")
                .long("crash_dir")
                .default_value("../c/tests/afl/ubx2sbp/")
                .takes_value(true),
        )
        .arg(
            Arg::new("FIXED_DIR")
                .about("Path to directory to move fixed test cases to")
                .long("fixed_dir")
                .default_value("../c/tests/afl/ubx2sbp/fixed/")
                .takes_value(true),
        )
        .arg(
            Arg::new("CMD_ARGS")
                .about("Args to pass to CMD (supply within quotes)")
                .long("cmd_args")
                .default_value("../build-afl/src/ubx2sbp/asan-ubx2sbp")
                .takes_value(true),
        )        
        .arg(
            Arg::new("CATEGORIZE")
                .about("Recategorizes test cases if supplied. Moves test cases
                        into directories based on their crash type and moves 
                        fixed tests to the fixed_dir.")
                .long("categorize")
                .takes_value(false),
        )
        .arg(
            Arg::new("VERBOSE")
                .about("Prints information about passing tests.")
                .long("verbose")
                .takes_value(false),
        )
        .get_matches();

    const TASKS: usize = 16;

    let crash_dir: &'static str = Box::leak(
        matches
            .value_of("CRASH_DIR")
            .unwrap()
            .to_owned()
            .into_boxed_str(),
    );
    let test_case_path = Path::new(crash_dir);

    let fixed_dir_str: &'static str = Box::leak(
        matches
            .value_of("FIXED_DIR")
            .unwrap()
            .to_owned()
            .into_boxed_str(),
    );
    let fixed_dir_path = Path::new(fixed_dir_str);

    let cmd_args_str: &'static str = Box::leak(
        matches
            .value_of("CMD_ARGS")
            .unwrap()
            .to_owned()
            .into_boxed_str(),
    );
    let cmd_args: Vec<&str> = cmd_args_str.split(" ").collect();

    let tests = get_test_cases(test_case_path);

    let (testcase_tx, testcase_rx) = channel::unbounded();
    let (result_tx, result_rx) = channel::unbounded();
    let pool = ThreadPool::new(TASKS);

    for test in tests {
        testcase_tx.send(test).expect("Couldn't send to channel");
    }

    let categorize = matches.is_present("CATEGORIZE");
    let verbose = matches.is_present("VERBOSE");

    for _ in 0..TASKS {
        let testcase_rx = testcase_rx.clone();
        let result_tx = result_tx.clone();
        let cmd_args = cmd_args.clone();
        pool.execute(move || {
            work(
                testcase_rx,
                result_tx,
                &cmd_args,
                test_case_path,
                fixed_dir_path,
                categorize
            );
        });
    }

    drop(testcase_rx);
    drop(testcase_tx);
    drop(result_tx);

    let mut passes = 0;
    let mut fails = 0;

    let mut error_categories = HashMap::new();

    for (result, test_name, category) in result_rx {
        if matches!(result, CmdResult::PASS) {
            if verbose {
                println!("{:70} PASS ✅", test_name);
            }
            passes += 1;
        } else {
            fails += 1;

            let category = match result {
             CmdResult::FAIL => {
                category.unwrap_or_else(|| "UNKNOWN".to_string())
             },
             CmdResult::TIMEOUT => {
                "timeout".to_string()
             },
             CmdResult::UNKNOWN => {
                 "unknown".to_string()
             },
             CmdResult::PASS =>  { panic!("Shouldn't be pass here") }
            };
            
            *error_categories.entry(category.clone()).or_insert(0) += 1;
            println!("{:70} FAIL ❌ ({})", test_name, category);
        }
    }

    println!();

    println!("Failing Tests By Category:");
    for (cat, num) in error_categories {
        println!("  {}: {}", cat, num);
    }

    println!();
    println!("{} failing of {}", fails, passes + fails);

    pool.join();

    if fails > 0 {
        std::process::exit(1); 
    }
}
