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

//! Compiles the V4L2 capture shim and the SVT-AV1 encoder shim.
//!
//! The encoder shim needs SVT-AV1's headers and library. `SVT_AV1_DIR` names an install (the nix build passes
//! the `svt-av1` derivation); otherwise `pkg-config` is asked, and failing both the crate still builds with no
//! encoder so the tests that do not need one, and every non-Linux build, keep working.

use std::path::Path;
use std::process::Command;

fn env_or(name: &str, default: &str) -> String {
    println!("cargo:rerun-if-env-changed={name}");
    std::env::var(name).unwrap_or_else(|_| default.to_string())
}

/// `(include dir, lib dir)` of an SVT-AV1 to build against.
fn find_svt_av1() -> Option<(String, String)> {
    let root = env_or("SVT_AV1_DIR", "");
    if !root.is_empty() && Path::new(&format!("{root}/include/svt-av1/EbSvtAv1Enc.h")).exists() {
        return Some((format!("{root}/include"), format!("{root}/lib")));
    }
    let include = Command::new("pkg-config")
        .args(["--variable=includedir", "SvtAv1Enc"])
        .output()
        .ok()
        .filter(|out| out.status.success())?;
    let libdir = Command::new("pkg-config")
        .args(["--variable=libdir", "SvtAv1Enc"])
        .output()
        .ok()
        .filter(|out| out.status.success())?;
    let include = String::from_utf8(include.stdout).ok()?.trim().to_string();
    let libdir = String::from_utf8(libdir.stdout).ok()?.trim().to_string();
    Path::new(&format!("{include}/svt-av1/EbSvtAv1Enc.h"))
        .exists()
        .then_some((include, libdir))
}

fn main() {
    println!("cargo::rustc-check-cfg=cfg(v4l2_capture)");
    println!("cargo::rustc-check-cfg=cfg(svt_av1)");
    println!("cargo:rerun-if-changed=csrc/capture.c");
    println!("cargo:rerun-if-changed=csrc/capture.h");
    println!("cargo:rerun-if-changed=csrc/av1enc.c");
    println!("cargo:rerun-if-changed=csrc/av1enc.h");
    if std::env::var("CARGO_CFG_TARGET_OS").as_deref() == Ok("linux") {
        cc::Build::new()
            .file("csrc/capture.c")
            .warnings(true)
            .compile("v4l2_capture");
        println!("cargo:rustc-cfg=v4l2_capture");
    }
    match find_svt_av1() {
        Some((include, libdir)) => {
            cc::Build::new()
                .file("csrc/av1enc.c")
                .include(include)
                .warnings(true)
                .compile("dimos_av1enc");
            println!("cargo:rustc-link-search=native={libdir}");
            println!("cargo:rustc-link-lib=dylib=SvtAv1Enc");
            println!("cargo:rustc-cfg=svt_av1");
        }
        None => println!(
            "cargo:warning=SVT-AV1 not found (set SVT_AV1_DIR or install it); \
             this build publishes nothing"
        ),
    }
}
