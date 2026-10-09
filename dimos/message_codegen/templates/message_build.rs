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

use heck::ToSnakeCase;
use ros2msg::generator::{
    FieldInfo, Generator, ItemInfo, ParseCallbacks, sanitize_rust_identifier,
};
use std::{collections::BTreeMap, env, fs, path::PathBuf};

fn identifier(name: &str) -> String {
    if sanitize_rust_identifier(name) != name || name == "gen" {
        format!("{name}_")
    } else {
        name.into()
    }
}
struct Adapters;
impl ParseCallbacks for Adapters {
    fn field_name(&self, field: &FieldInfo) -> Option<String> {
        Some(identifier(field.field_name()))
    }
    fn add_derives(&self, _: &ItemInfo) -> Vec<String> {
        vec!["serde::Serialize".into(), "serde::Deserialize".into()]
    }
    fn add_field_attributes(&self, field: &FieldInfo) -> Vec<String> {
        if field.array_size().is_some() && field.capacity().is_none() {
            vec!["#[serde(with = \"serde_big_array::BigArray\")]".into()]
        } else {
            vec![]
        }
    }
    fn custom_impl(&self, item: &ItemInfo) -> Option<String> {
        let name = format!("{}/msg/{}", item.package(), item.name());
        let (_, source, fields) = ADAPTERS.iter().find(|(key, _, _)| *key == name)?;
        let mut source = source.to_string();
        for field in *fields {
            source = source.replace(&format!("@{field}@"), &identifier(field));
        }
        Some(source)
    }
}
fn main() {
    println!("cargo:rerun-if-changed=interfaces");
    let output = PathBuf::from(env::var_os("OUT_DIR").unwrap());
    if !ADAPTERS.is_empty() {
        Generator::new()
            .derive_debug(true)
            .derive_clone(true)
            .derive_default(false)
            .derive_partialeq(true)
            .parse_callbacks(Box::new(Adapters))
            .includes(
                ADAPTERS
                    .iter()
                    .map(|(name, _, _)| format!("interfaces/{name}.msg")),
            )
            .output_dir(&output)
            .generate()
            .expect("ros2msg generation failed");
    }
    let mut packages: BTreeMap<&str, String> = BTreeMap::new();
    for (name, _, _) in ADAPTERS {
        let parts: Vec<_> = name.split('/').collect();
        let (package, message) = (parts[0], parts[2]);
        let module = message.to_snake_case();
        let path = output
            .join(package)
            .join("msg")
            .join(format!("{module}.rs"));
        packages.entry(package).or_default().push_str(&format!(
            "pub mod {module} {{ include!({path:?}); }} pub use {module}::{message};\n"
        ));
    }
    for (name, owner) in IMPORTS {
        let parts: Vec<_> = name.split('/').collect();
        let (package, message) = (parts[0], parts[2]);
        let module = message.to_snake_case();
        packages.entry(package).or_default().push_str(&format!(
            "pub mod {module} {{ pub use {owner}::{package}::msg::{message}; }} pub use {module}::{message};\n"));
    }
    let root = packages
        .into_iter()
        .map(|(package, contents)| {
            format!("pub mod {package} {{ pub mod msg {{ {contents} }} }}\n")
        })
        .collect::<String>();
    fs::write(output.join("messages.rs"), root).unwrap();
}
