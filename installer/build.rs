use std::{env, fs, path::Path};

include!("pyproject.rs");

fn main() {
    println!("cargo:rerun-if-changed=../pyproject.toml");
    println!("cargo:rerun-if-changed=pyproject.rs");
    let pyproject = fs::read_to_string("../pyproject.toml").expect("read ../pyproject.toml");
    println!("cargo:rustc-env=DIMOS_VERSION={}", version(&pyproject));
    let names = extras(&pyproject);
    let literal = names
        .iter()
        .map(|n| format!("{n:?}"))
        .collect::<Vec<_>>()
        .join(", ");
    let out = Path::new(&env::var("OUT_DIR").expect("OUT_DIR")).join("extras.rs");
    fs::write(&out, format!("pub const EXTRAS: &[&str] = &[{literal}];\n"))
        .expect("write extras.rs");
}
