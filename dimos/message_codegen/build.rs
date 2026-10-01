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

// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0

use std::{env, path::PathBuf, process::Command};

fn main() {
    let root = PathBuf::from(env::var_os("CARGO_MANIFEST_DIR").unwrap());
    for input in [
        "build_rust.py",
        "definitions.py",
        "providers.py",
        "rust.py",
        "_vendor",
        "schemas",
        "templates/codec.rs",
    ] {
        println!("cargo:rerun-if-changed={input}");
    }
    println!("cargo:rerun-if-env-changed=DIMOS_CODEGEN_PYTHON");
    let python = env::var_os("DIMOS_CODEGEN_PYTHON").unwrap_or_else(|| "python3".into());
    let status = Command::new(python)
        .arg("-I")
        .arg("-B")
        .arg(root.join("build_rust.py"))
        .arg(env::var_os("OUT_DIR").unwrap())
        .status()
        .expect(
            "message generation requires Python 3.10+; set DIMOS_CODEGEN_PYTHON to its executable",
        );
    assert!(status.success(), "ROS-free Rust message generation failed");
}
