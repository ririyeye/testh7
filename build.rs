use std::env;
use std::fs::File;
use std::io::Write;
use std::path::PathBuf;

fn main() {
    // 获取输出目录
    let out = &PathBuf::from(env::var_os("OUT_DIR").unwrap());

    // 复制 memory.x 到输出目录
    File::create(out.join("memory.x"))
        .unwrap()
        .write_all(include_bytes!("memory.x"))
        .unwrap();

    // 告诉 cargo 在哪里找链接脚本
    println!("cargo:rustc-link-search={}", out.display());

    // 当 memory.x 改变时重新构建
    println!("cargo:rerun-if-changed=memory.x");
}
