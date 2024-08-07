//! This build script copies the `memory.x` file from the crate root into
//! a directory where the linker can always find it at build time.
//! For many projects this is optional, as the linker always searches the
//! project root directory -- wherever `Cargo.toml` is. However, if you
//! are using a workspace or have a more complicated build setup, this
//! build script becomes required. Additionally, by requesting that
//! Cargo re-run the build script whenever `memory.x` is changed,
//! updating `memory.x` ensures a rebuild of the application with the
//! new memory settings.

use std::env;
use std::path::PathBuf;

fn main() {
    // Put `memory.x` in our output directory and ensure it's
    // on the linker search path.
    let out = &PathBuf::from(env::var_os("OUT_DIR").unwrap());
    println!("cargo:rustc-link-search={}", out.display());

    // By default, Cargo will re-run a build script whenever
    // any file in the project changes. By specifying `memory.x`
    // here, we ensure the build script is only re-run when
    // `memory.x` is changed.
    if cfg!(feature = "mcuboot") {
        std::fs::write(
            out.join("memory-mcuboot.x"),
            include_bytes!("memory-mcuboot.x"),
        )
        .unwrap();
        std::fs::write(
            out.join("link-ram-mcuboot.x"),
            include_bytes!("link-ram-mcuboot.x"),
        )
        .unwrap();
        println!("cargo:rerun-if-changed=memory-mcuboot.x");
        println!("cargo:rerun-if-changed=link-ram-mcuboot.x");
        println!("cargo:rustc-link-arg-bins=-Tlink-ram-mcuboot.x");
    } else {
        std::fs::write(out.join("memory.x"), include_bytes!("memory.x")).unwrap();
        std::fs::write(out.join("link-ram.x"), include_bytes!("link-ram.x")).unwrap();
        println!("cargo:rerun-if-changed=memory.x");
        println!("cargo:rerun-if-changed=link-ram.x");
        println!("cargo:rustc-link-arg-bins=-Tlink-ram.x");
    }

    println!("cargo:rustc-link-arg-bins=--nmagic");
    // println!("cargo:rustc-link-arg-bins=-Tlink.x");
    println!("cargo:rustc-link-arg-bins=-Tdefmt.x");
}
