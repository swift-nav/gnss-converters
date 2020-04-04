use std::result::Result;
use std::env;

type Error = Box<dyn std::error::Error>;

fn compiler_name(cpp: bool, six_zero: bool) -> String {
    let cpp = if cpp { "++" } else { "" };
    let six_zero = if six_zero { "-6.0" } else { "" };
    format!("clang{}{}", cpp, six_zero)
}

/// Try to find just clang/clang++, if it's not available
/// fall back to clang-6.0 (a known good compiler).
fn find_clang_compiler(cpp: bool) -> Result<String, Error> {
    let mut build = cc::Build::new();
    let clang_compiler = build.compiler(compiler_name(cpp, /* six_zero = */ true));
    if let Ok(compiler) = clang_compiler.try_get_compiler() {
        let mut cmd = compiler.to_command();
        cmd.args(&["--version"]);
        if cmd.output().is_ok() {
            return Ok(format!("{}", compiler.path().display()));
        }
    }
    let clang_compiler = build.compiler(compiler_name(cpp, /* six_zero = */ false));
    if let Ok(compiler) = clang_compiler.try_get_compiler() {
        let mut cmd = compiler.to_command();
        cmd.args(&["--version"]);
        if cmd.output().is_ok() {
            return Ok(format!("{}", compiler.path().display()));
        }
    }
    let cpp = if cpp { "++" } else { "" };
    Err(format!("failed to find one of clang{} or clang{}-6.0", cpp, cpp).into())
}

fn main() -> Result<(), Error> {

    let c_compiler = {
        if let Ok(cc) = env::var("CC") {
            cc
        } else {
            find_clang_compiler(/* cpp = */ false)?
        }
    };

    let cpp_compiler = {
        if let Ok(cxx) = env::var("CXX") {
            cxx
        } else {
            find_clang_compiler(/* cpp = */ true)?
        }
    };

    let mut cmake = cmake::Config::new("c");

    if cfg!(target_os = "macos") {
        cmake.define("CMAKE_CXX_STANDARD", "11");
        cmake.define("CMAKE_CXX_FLAGS", "-stdlib=libc++");
        cmake.define("CMAKE_C_COMPILER", c_compiler);
        cmake.define("CMAKE_CXX_COMPILER", cpp_compiler);
    } else if cfg!(target_os = "linux") {
        cmake.define("CMAKE_INSTALL_RPATH", "$ORIGIN");
        cmake.define("CMAKE_POSITION_INDEPENDENT_CODE", "ON");
        cmake.define("CMAKE_C_COMPILER", c_compiler);
        cmake.define("CMAKE_CXX_COMPILER", cpp_compiler);
    } else if cfg!(target_os = "windows") {
        cmake.define("gnss-converters_ENABLE_TESTS", "OFF");
        cmake.define("CMAKE_C_COMPILER", c_compiler);
        cmake.define("CMAKE_CXX_COMPILER", cpp_compiler);
    } else {
        return Err("unknown target OS".into());
    }

    if let Ok(generator) = env::var("CMAKE_GENERATOR") {
        cmake.generator(generator);
    }

    env::set_var("NUM_JOBS", num_cpus::get().to_string());
    let dst = cmake.build();

    println!("cargo:rustc-link-search=native={}/lib", dst.display());
    println!("cargo:rustc-link-search=native={}/lib64", dst.display());

    if cfg!(target_os = "macos") || cfg!(target_os = "linux") || cfg!(target_os = "windows") {
        println!("cargo:rustc-link-lib=static=rtcm3tosbp_main");
        println!("cargo:rustc-link-lib=static=ubx2sbp_main");
        println!("cargo:rustc-link-lib=static=sbp2rtcm_main");
        println!("cargo:rustc-link-lib=static=gnss_converters");
        println!("cargo:rustc-link-lib=static=swiftnav");
        println!("cargo:rustc-link-lib=static=rtcm");
        println!("cargo:rustc-link-lib=static=sbp");
        println!("cargo:rustc-link-lib=static=ubx");
        println!("cargo:rustc-link-lib=dylib=m");
    } else {
        return Err("Unknown target OS...".into());
    }

    Ok(())
}
