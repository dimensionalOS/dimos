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

//! Compiles the V4L2 capture shim; with the Jetson Multimedia API headers present it also gets the hardware
//! JPEG path (VIC colour convert + NVJPG encode).

use std::path::Path;

const JETSON_MULTIMEDIA_API: &str = "/usr/src/jetson_multimedia_api/include";
const JETSON_LIBS: &str = "/usr/lib/aarch64-linux-gnu/nvidia";

fn main() {
    println!("cargo::rustc-check-cfg=cfg(v4l2_capture)");
    println!("cargo:rerun-if-changed=csrc/capture.c");
    println!("cargo:rerun-if-changed=csrc/capture.h");
    if std::env::var("CARGO_CFG_TARGET_OS").as_deref() != Ok("linux") {
        return;
    }
    let mut build = cc::Build::new();
    build.file("csrc/capture.c").warnings(true);
    if Path::new(JETSON_MULTIMEDIA_API).exists() {
        build
            .define("DIMOS_JETSON_HW", None)
            .include(JETSON_MULTIMEDIA_API)
            .include(format!("{JETSON_MULTIMEDIA_API}/libjpeg-8b"));
        println!("cargo:rustc-link-search=native={JETSON_LIBS}");
        println!("cargo:rustc-link-arg=-Wl,-rpath,{JETSON_LIBS}");
        println!("cargo:rustc-link-lib=dylib=nvbufsurface");
        println!("cargo:rustc-link-lib=dylib=nvbufsurftransform");
        println!("cargo:rustc-link-lib=dylib=dl");
    }
    build.compile("v4l2_capture");
    println!("cargo:rustc-cfg=v4l2_capture");
}
