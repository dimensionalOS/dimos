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

use ros2msg::generator::{FieldInfo, Generator, ItemInfo, ParseCallbacks};
use std::{env, fs, path::PathBuf};

struct Serde;
impl ParseCallbacks for Serde {
    fn sequence_type(&self, element: &str, bound: Option<u32>, _: &str) -> Option<String> {
        bound.map(|n| format!("heapless::Vec<{element}, {n}>"))
    }
    fn add_derives(&self, _: &ItemInfo) -> Vec<String> {
        vec!["serde::Serialize".into(), "serde::Deserialize".into(), "PartialEq".into()]
    }
    fn add_field_attributes(&self, field: &FieldInfo) -> Vec<String> {
        if field.array_size().is_some_and(|n| n > 32) && field.capacity().is_none() {
            vec!["#[serde(with = \"serde_big_array::BigArray\")]".into()]
        } else { vec![] }
    }
}
fn main() {
    println!("cargo:rerun-if-changed=interfaces");
    let mut paths = Vec::new();
    for package in fs::read_dir("interfaces").expect("missing message sources") {
        let directory = package.unwrap().path().join("msg");
        if directory.is_dir() {
            for entry in fs::read_dir(directory).unwrap() {
                let path = entry.unwrap().path();
                if path.extension().is_some_and(|ext| ext == "msg") { paths.push(path); }
            }
        }
    }
    paths.sort();
    // Upstream currently emits invalid floating-point default literals. Keep
    // native constructors explicit until the upstream Default acceptance passes.
    Generator::new().derive_debug(true).derive_clone(true).derive_default(false)
        .parse_callbacks(Box::new(Serde)).includes(paths)
        .output_dir(PathBuf::from(env::var_os("OUT_DIR").unwrap()))
        .generate().expect("ros2msg generation failed");
}
