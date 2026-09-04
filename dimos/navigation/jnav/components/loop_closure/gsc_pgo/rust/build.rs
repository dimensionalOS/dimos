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

//! Compiles the gtsam C++ shim (shim/gtsam_shim.cpp) and links libgtsam.
//!
//! Dependency discovery, in order:
//!   1. Explicit env vars (what flake.nix's dev shell sets):
//!      GTSAM_INCLUDE_DIR / GTSAM_LIB_DIR, EIGEN_INCLUDE_DIR,
//!      BOOST_INCLUDE_DIR.
//!   2. CMAKE_PREFIX_PATH entries (standard nix/cmake environments).
//!   3. pkg-config (gtsam ships no .pc file everywhere, but eigen3 does).
//!   4. Conventional system prefixes (/usr, /usr/local).
//!
//! In the flake's dev shell everything resolves via (1); the later stages
//! exist so `nix-shell -p gtsam eigen boost` or a system install also work.

use std::env;
use std::path::PathBuf;
use std::process::Command;

fn env_dir(name: &str) -> Option<PathBuf> {
    println!("cargo:rerun-if-env-changed={}", name);
    env::var_os(name).map(PathBuf::from).filter(|p| p.is_dir())
}

/// Prefixes to scan: CMAKE_PREFIX_PATH entries, then conventional roots.
fn candidate_prefixes() -> Vec<PathBuf> {
    println!("cargo:rerun-if-env-changed=CMAKE_PREFIX_PATH");
    let mut prefixes: Vec<PathBuf> = Vec::new();
    if let Some(cmake_paths) = env::var_os("CMAKE_PREFIX_PATH") {
        for prefix in env::split_paths(&cmake_paths) {
            prefixes.push(prefix);
        }
    }
    prefixes.push(PathBuf::from("/usr/local"));
    prefixes.push(PathBuf::from("/usr"));
    if cfg!(target_os = "macos") {
        prefixes.push(PathBuf::from("/opt/homebrew"));
    }
    prefixes
}

/// Parse the -I directories out of `pkg-config --cflags-only-I <package>`.
fn pkg_config_include_dirs(package: &str) -> Vec<PathBuf> {
    let output = match Command::new("pkg-config")
        .args(["--cflags-only-I", package])
        .output()
    {
        Ok(output) if output.status.success() => output,
        _ => return Vec::new(),
    };
    String::from_utf8_lossy(&output.stdout)
        .split_whitespace()
        .filter_map(|flag| flag.strip_prefix("-I").map(PathBuf::from))
        .collect()
}

fn find_gtsam() -> (PathBuf, PathBuf) {
    if let (Some(include), Some(lib)) = (env_dir("GTSAM_INCLUDE_DIR"), env_dir("GTSAM_LIB_DIR")) {
        return (include, lib);
    }
    for prefix in candidate_prefixes() {
        let include = prefix.join("include");
        let lib = prefix.join("lib");
        if include.join("gtsam/nonlinear/ISAM2.h").is_file() && lib.is_dir() {
            return (include, lib);
        }
    }
    // Last resort: a gtsam.pc (some distro packages ship one).
    for include in pkg_config_include_dirs("gtsam") {
        if include.join("gtsam/nonlinear/ISAM2.h").is_file() {
            if let Some(lib) = include
                .parent()
                .map(|p| p.join("lib"))
                .filter(|p| p.is_dir())
            {
                return (include, lib);
            }
        }
    }
    panic!(
        "gtsam not found. Set GTSAM_INCLUDE_DIR + GTSAM_LIB_DIR, or add gtsam's \
         prefix to CMAKE_PREFIX_PATH (e.g. enter the flake dev shell: `nix develop`)."
    );
}

fn find_eigen() -> Option<PathBuf> {
    if let Some(dir) = env_dir("EIGEN_INCLUDE_DIR") {
        return Some(dir);
    }
    for prefix in candidate_prefixes() {
        let dir = prefix.join("include/eigen3");
        if dir.join("Eigen/Dense").is_dir() || dir.join("Eigen/Dense").is_file() {
            return Some(dir);
        }
    }
    pkg_config_include_dirs("eigen3")
        .into_iter()
        .find(|dir| dir.join("Eigen").is_dir())
}

fn find_boost() -> Option<PathBuf> {
    if let Some(dir) = env_dir("BOOST_INCLUDE_DIR") {
        return Some(dir);
    }
    candidate_prefixes()
        .into_iter()
        .map(|prefix| prefix.join("include"))
        .find(|include| include.join("boost/version.hpp").is_file())
}

fn find_tbb_include() -> Option<PathBuf> {
    if let Some(dir) = env_dir("TBB_INCLUDE_DIR") {
        return Some(dir);
    }
    candidate_prefixes()
        .into_iter()
        .map(|prefix| prefix.join("include"))
        .find(|include| include.join("tbb/scalable_allocator.h").is_file())
}

fn main() {
    println!("cargo:rerun-if-changed=shim/gtsam_shim.cpp");
    println!("cargo:rerun-if-changed=shim/gtsam_shim.h");

    let (gtsam_include, gtsam_lib) = find_gtsam();

    let mut build = cc::Build::new();
    build
        .cpp(true)
        .std("c++17")
        .file("shim/gtsam_shim.cpp")
        .include("shim")
        .include(&gtsam_include);
    if let Some(eigen) = find_eigen() {
        build.include(eigen);
    }
    if let Some(boost) = find_boost() {
        // gtsam headers still pull boost/serialization bits in 4.3a1.
        build.include(boost);
    }
    // gtsam built with GTSAM_USE_TBB includes tbb/scalable_allocator.h from
    // its public headers, so tbb headers (and libtbb below) are required.
    let gtsam_uses_tbb = std::fs::read_to_string(gtsam_include.join("gtsam/config.h"))
        .unwrap_or_default()
        .contains("#define GTSAM_USE_TBB");
    if gtsam_uses_tbb {
        if let Some(tbb) = find_tbb_include() {
            build.include(tbb);
        }
    }
    // Eigen + gtsam headers are warning-noisy on modern GCC; keep our build
    // output readable without weakening errors in the shim itself.
    build
        .flag_if_supported("-Wno-deprecated-copy")
        .flag_if_supported("-Wno-array-bounds")
        .flag_if_supported("-Wno-maybe-uninitialized")
        .flag_if_supported("-Wno-unused-parameter");
    build.compile("gtsam_shim");

    println!("cargo:rustc-link-search=native={}", gtsam_lib.display());
    println!("cargo:rustc-link-lib=dylib=gtsam");
    // Linux only: mirror the CMake link closure. libcephes-gtsam declares no
    // DT_NEEDED on libm even though it imports IFUNC symbols like `sin`, so the
    // executable must carry DT_NEEDED on libm (and the gtsam companions)
    // directly — otherwise glibc 2.42's loader crashes relocating cephes
    // ("Relink ... for IFUNC symbol"). --no-as-needed because we reference none
    // of their symbols ourselves and rustc links with --as-needed by default.
    // These are GNU-ld flags; Apple's ld rejects --no-as-needed and doesn't need
    // this (the companions load transitively via libgtsam's load commands + rpath).
    let target_os = env::var("CARGO_CFG_TARGET_OS").unwrap_or_default();
    if target_os == "linux" {
        println!("cargo:rustc-link-arg=-Wl,--no-as-needed");
        for companion in ["cephes-gtsam", "metis-gtsam"] {
            if gtsam_lib.join(format!("lib{}.so", companion)).exists() {
                println!("cargo:rustc-link-arg=-l{}", companion);
            }
        }
        println!("cargo:rustc-link-arg=-lm");
        println!("cargo:rustc-link-arg=-Wl,--as-needed");
    }
    // The tbb allocator calls inlined into the shim object need libtbb.
    if gtsam_uses_tbb {
        println!("cargo:rustc-link-lib=dylib=tbb");
        if let Some(tbb_lib) = env_dir("TBB_LIB_DIR") {
            println!("cargo:rustc-link-search=native={}", tbb_lib.display());
            println!("cargo:rustc-link-arg=-Wl,-rpath,{}", tbb_lib.display());
        }
    }
    // Bake an rpath so unit tests / binaries run without LD_LIBRARY_PATH.
    println!("cargo:rustc-link-arg=-Wl,-rpath,{}", gtsam_lib.display());

    // libgtsam.so's own DT_NEEDED entries (boost, tbb, metis-gtsam, ...) must
    // be resolvable at link time. Inside a nix shell the wrapped linker gets
    // them from NIX_LDFLAGS automatically; GTSAM_DEP_LIB_DIRS covers bare
    // setups where they live in other prefixes (e.g. raw /nix/store paths).
    // -rpath-link is GNU-ld-only (Apple's ld rejects it and resolves transitive
    // dylib deps via install names + the -L search paths instead).
    if target_os == "linux" {
        println!(
            "cargo:rustc-link-arg=-Wl,-rpath-link,{}",
            gtsam_lib.display()
        );
    }
    println!("cargo:rerun-if-env-changed=GTSAM_DEP_LIB_DIRS");
    if let Some(extra) = env::var_os("GTSAM_DEP_LIB_DIRS") {
        for dir in env::split_paths(&extra).filter(|p: &PathBuf| p.is_dir()) {
            println!("cargo:rustc-link-search=native={}", dir.display());
            println!("cargo:rustc-link-arg=-Wl,-rpath,{}", dir.display());
        }
    }
}
