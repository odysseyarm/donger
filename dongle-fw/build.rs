use std::path::PathBuf;
use std::str::FromStr;
use std::{env, fs};

use memory_spec::MemorySpec;

fn main() {
    println!("cargo:rerun-if-changed=memory.kdl");

    let manifest_dir = PathBuf::from(env::var("CARGO_MANIFEST_DIR").unwrap());
    let memory_kdl_path = manifest_dir.join("memory.kdl");

    let content = fs::read_to_string(&memory_kdl_path)
        .unwrap_or_else(|e| panic!("Failed to read memory.kdl: {}", e));

    let memoryspec = match MemorySpec::from_str(&content) {
        Ok(m) => m,
        Err(e) => {
            eprintln!("Failed to parse memory.kdl: {}", e);
            std::process::exit(1);
        }
    };

    let regions = memoryspec.regions();
    let symbols = memoryspec.render_symbols();

    let app_flash = &regions["flash"]["app_flash"];
    let _settings = &regions["flash"]["settings"];
    let app_ram = &regions["ram"]["app_ram"];

    let memory_x = format!(
        "\
MEMORY
{{
  FLASH : ORIGIN = {:#010x}, LENGTH = {:#08x}
  RAM   : ORIGIN = {:#010x}, LENGTH = {:#08x}
}}

{symbols}
",
        app_flash.origin(),
        app_flash.length(),
        app_ram.origin(),
        app_ram.length()
    );

    let out_dir = PathBuf::from(env::var("OUT_DIR").unwrap());
    let gen_mem = out_dir.join("memory.x");
    fs::write(&gen_mem, memory_x).expect("write memory.x");

    println!("cargo:rustc-link-search={}", out_dir.display());

    println!("cargo:rustc-link-arg-bins=--nmagic");
    println!("cargo:rustc-link-arg-bins=-Tlink.x");
    println!("cargo:rustc-link-arg-bins=-Tdefmt.x");
    println!("cargo:rustc-link-arg-bins=--print-memory-usage");
}
