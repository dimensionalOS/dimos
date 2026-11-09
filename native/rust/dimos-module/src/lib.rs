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

// #[derive(Module)] emits ::dimos_module paths, so tests in this crate that
// derive Module need the crate to be nameable from inside itself.
#[cfg(test)]
extern crate self as dimos_module;

pub mod host;
pub mod lcm;
pub mod log;
pub mod module;
pub mod pointcloud;
pub mod tf;
pub mod time;
pub mod transport;
pub mod workers;
pub mod zenoh;

pub use dimos_module_macros::{native_config, Module};
pub use host::{host_main, HostSpec, ModuleEntry};
pub use lcm::LcmTransport;
pub use module::{run, Builder, Input, Io, Module, ModuleConfig, NativeConfig, NoConfig, Output};
pub use tf::{Lookup, Tf, Transform};
pub use transport::Transport;
pub use workers::worker_pool;
pub use zenoh::ZenohTransport;

pub use nalgebra;

// Re-export LcmOptions so callers don't need to depend on dimos-lcm directly.
pub use dimos_lcm::LcmOptions;

/// Run module `M` over the transport named by the `DIMOS_TRANSPORT` env var,
/// configured by the launch line on stdin.
pub async fn run_with_transport<M: Module>() {
    crate::module::init_tracing();
    let launch = match crate::module::read_launch_config().await {
        Ok(launch) => launch,
        Err(e) => {
            tracing::error!("failed to read the launch config from stdin: {e}");
            std::process::exit(1);
        }
    };
    let transport = match crate::transport::open_transport(&launch).await {
        Ok(transport) => transport,
        Err(e) => {
            tracing::error!("failed to open the transport: {e}");
            std::process::exit(1);
        }
    };
    run::<M>(transport, launch).await
}
