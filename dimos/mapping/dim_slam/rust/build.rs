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
//
// cargo:rustc-link-arg applies only to the package that emits it, so cu_vslam_rs's rpath
// never reaches this binary and it would die at startup on @rpath/libcuvslam.dylib.
use std::env;

fn main() {
    let lib_dir = env::var("DEP_CUVSLAM_LIB_DIR").expect("cu_vslam_rs exports lib_dir");
    println!("cargo:rustc-link-arg=-Wl,-rpath,{lib_dir}");
}
