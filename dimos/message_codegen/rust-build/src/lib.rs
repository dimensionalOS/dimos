// Copyright 2026 Dimensional Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

//! Normal Cargo build integration. The Python plan owns closure and identity;
//! ros2msg performs Rust generation with the same pinned wire/value policy.

use heck::ToSnakeCase;
use ros2msg::generator::{FieldInfo, Generator, ItemInfo, ParseCallbacks};
use serde_json::Value;
use std::io::Write;
use std::{env, error::Error, fs, path::Path, path::PathBuf, process::Command};

struct Serde;
impl ParseCallbacks for Serde {
    fn sequence_type(&self, element: &str, bound: Option<u32>, _: &str) -> Option<String> {
        bound.map(|n| format!("heapless::Vec<{element}, {n}>"))
    }
    fn add_derives(&self, _: &ItemInfo) -> Vec<String> {
        vec![
            "serde::Serialize".into(),
            "serde::Deserialize".into(),
            "PartialEq".into(),
        ]
    }
    fn add_field_attributes(&self, field: &FieldInfo) -> Vec<String> {
        if field.array_size().is_some_and(|n| n > 32) && field.capacity().is_none() {
            vec!["#[serde(with = \"serde_big_array::BigArray\")]".into()]
        } else {
            vec![]
        }
    }
}

/// Called by an external message crate's build.rs, with its pyproject directory.
/// The caller supplies its ordinary Cargo dependencies and an installed Python
/// message-build environment. No tools or OS packages are installed here.
pub fn generate(project: impl AsRef<Path>) -> Result<(), Box<dyn Error>> {
    let project = project.as_ref().canonicalize()?;
    let output = PathBuf::from(env::var_os("OUT_DIR").ok_or("Cargo OUT_DIR missing")?);
    let cargo =
        PathBuf::from(env::var_os("CARGO_MANIFEST_DIR").ok_or("Cargo manifest directory missing")?)
            .join("Cargo.toml");
    let plan_path = output.join("dimos-messages.json");
    let python = env::var_os("DIMOS_MESSAGE_PYTHON").unwrap_or_else(|| "python3".into());
    let status = Command::new(python)
        .args(["-m", "dimos.message_codegen.native", "rust"])
        .arg(&project)
        .arg(&plan_path)
        .arg(&cargo)
        .status()?;
    if !status.success() {
        return Err("message preparation failed; activate the Python environment with dimos[message-build] and matching dependencies".into());
    }
    let plan: Value = serde_json::from_slice(&fs::read(&plan_path)?)?;
    for input in plan["inputs"].as_array().ok_or("invalid input list")? {
        println!(
            "cargo:rerun-if-changed={}",
            input.as_str().ok_or("invalid input path")?
        );
    }
    println!("cargo:rerun-if-env-changed=DIMOS_MESSAGE_PYTHON");
    let paths: Vec<PathBuf> = plan["interfaces"]
        .as_array()
        .ok_or("invalid interfaces")?
        .iter()
        .map(|path| {
            path.as_str()
                .map(PathBuf::from)
                .ok_or("invalid interface path")
        })
        .collect::<Result<_, _>>()?;
    let pending = output.join("dimos-message-bindings.pending");
    let bindings = output.join("dimos-message-bindings");
    if pending.exists() {
        fs::remove_dir_all(&pending)?;
    }
    fs::create_dir(&pending)?;
    Generator::new()
        .derive_debug(true)
        .derive_clone(true)
        .derive_default(false)
        .parse_callbacks(Box::new(Serde))
        .includes(paths.clone())
        .output_dir(&pending)
        .generate()?;
    // Export the conventional package::msg::Type facade without creating
    // another struct. Borrowed dependencies keep their original crate types.
    for path in &paths {
        let name = path
            .file_stem()
            .and_then(|v| v.to_str())
            .ok_or("invalid message filename")?;
        let package = path
            .parent()
            .and_then(Path::parent)
            .and_then(Path::file_name)
            .and_then(|v| v.to_str())
            .ok_or("invalid message package")?;
        let module = name.to_snake_case();
        let directory = pending.join(package).join("msg");
        if !directory.join(format!("{module}.rs")).is_file() {
            return Err(format!("Unsupported ros2msg module layout for {package}/{name}").into());
        }
        let mut declarations = fs::OpenOptions::new()
            .append(true)
            .open(directory.join("mod.rs"))?;
        writeln!(declarations, "pub use {module}::{name};")?;
    }
    let backup = output.join("dimos-message-bindings.previous");
    if backup.exists() {
        fs::remove_dir_all(&backup)?;
    }
    if bindings.exists() {
        fs::rename(&bindings, &backup)?;
    }
    if let Err(error) = fs::rename(&pending, &bindings) {
        if backup.exists() {
            fs::rename(&backup, &bindings)?;
        }
        return Err(error.into());
    }
    if backup.exists() {
        fs::remove_dir_all(&backup)?;
    }
    let source = fs::read_to_string(plan["library"].as_str().ok_or("missing library path")?)?;
    fs::copy(
        plan["schemas"].as_str().ok_or("missing schemas path")?,
        output.join("messages-schemas.json"),
    )?;
    fs::write(
        output.join("messages.rs"),
        source
            .replace("../schemas.json", "messages-schemas.json")
            .replace("/mod.rs", "/dimos-message-bindings/mod.rs"),
    )?;
    Ok(())
}
