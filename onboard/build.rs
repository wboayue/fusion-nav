//! For the board only: the memory map, the link script, and what the firmware reports about
//! its own build, since a cycle count without its compiler, opt-level and FPU is not one anyone
//! can reproduce (#41).

use std::process::Command;
use std::{env, fs, path::PathBuf};

fn main() {
    println!("cargo:rerun-if-changed=memory.x");
    println!("cargo:rerun-if-env-changed=ONBOARD_BUILD");
    println!("cargo:rerun-if-env-changed=ONBOARD_LTO");
    if env::var("CARGO_CFG_TARGET_OS").as_deref() != Ok("none") {
        return;
    }
    let out = PathBuf::from(env::var_os("OUT_DIR").expect("cargo sets OUT_DIR"));
    fs::copy("memory.x", out.join("memory.x")).expect("memory.x");
    println!("cargo:rustc-link-search={}", out.display());
    println!("cargo:rustc-link-arg-bins=-Tlink.x");

    let commit = Command::new("git")
        .args(["describe", "--always", "--dirty", "--abbrev=10"])
        .output()
        .ok()
        .filter(|o| o.status.success())
        .and_then(|o| String::from_utf8(o.stdout).ok())
        .map_or_else(|| "unknown".to_string(), |s| s.trim().to_string());
    let rustc = Command::new(env::var("RUSTC").unwrap_or_else(|_| "rustc".into()))
        .arg("--version")
        .output()
        .ok()
        .and_then(|o| String::from_utf8(o.stdout).ok())
        .map_or_else(|| "unknown".to_string(), |s| s.trim().replace(' ', "_"));
    // `-C target-cpu=cortex-m7` turns on the M7's double-precision FPU, which `cfg` does not
    // report (`fp64` is not a stable feature name), so the flag is read rather than the cfg.
    let flags = env::var("CARGO_ENCODED_RUSTFLAGS").unwrap_or_default();
    let cpu = flags
        .split('\x1f')
        .flat_map(|flag| flag.split_whitespace())
        .find_map(|flag| flag.strip_prefix("target-cpu="))
        .unwrap_or("generic")
        .to_string();
    let fpu = if cpu == "cortex-m7" {
        "double"
    } else {
        "single"
    };
    let build = env::var("ONBOARD_BUILD").unwrap_or_else(|_| "dev".into());
    let opt = env::var("OPT_LEVEL").unwrap_or_default();
    // A build script does not see the profile's LTO; `build.sh` says which it asked for.
    let lto = env::var("ONBOARD_LTO").unwrap_or_else(|_| "unknown".into());
    println!("cargo:rustc-env=ONBOARD_COMMIT={commit}");
    println!("cargo:rustc-env=ONBOARD_RUSTC={rustc}");
    println!("cargo:rustc-env=ONBOARD_CPU={cpu}");
    println!("cargo:rustc-env=ONBOARD_FPU={fpu}");
    println!("cargo:rustc-env=ONBOARD_BUILD={build}");
    println!("cargo:rustc-env=ONBOARD_OPT={opt}");
    println!("cargo:rustc-env=ONBOARD_LTO={lto}");
    println!("cargo:rerun-if-changed=../.git/HEAD");
}
