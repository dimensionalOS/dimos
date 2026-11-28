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

use dimos_dds_bridge::module::DdsBridge;

/// RustDDS narrates discovery at info; its warnings are what an operator needs.
const RUSTDDS_LOG_DIRECTIVE: &str = "rustdds=warn";

fn main() {
    // Set before the runtime starts any thread; an explicit rustdds directive wins.
    match std::env::var("RUST_LOG") {
        Ok(filter) if filter.contains("rustdds") => {}
        Ok(filter) if !filter.is_empty() => {
            std::env::set_var("RUST_LOG", format!("{filter},{RUSTDDS_LOG_DIRECTIVE}"))
        }
        _ => std::env::set_var("RUST_LOG", format!("info,{RUSTDDS_LOG_DIRECTIVE}")),
    }
    tokio::runtime::Builder::new_multi_thread()
        .enable_all()
        .build()
        .expect("tokio runtime")
        .block_on(dimos_module::run_with_transport::<DdsBridge>());
}
