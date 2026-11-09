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
pub use module::{Builder, Input, Io, Module, ModuleConfig, NativeConfig, NoConfig, Output};
pub use tf::{Lookup, Tf, Transform};
pub use transport::Transport;
pub use workers::worker_pool;
pub use zenoh::ZenohTransport;

pub use nalgebra;

// Re-export LcmOptions so callers don't need to depend on dimos-lcm directly.
pub use dimos_lcm::LcmOptions;

/// Run module `M` as its own process: open the transport `DIMOS_TRANSPORT`
/// names, configured by the launch line on stdin, and run until the module
/// finishes or SIGINT/SIGTERM arrives. Logs and exits 1 on failure.
pub async fn run_with_transport<M: Module>() {
    module::init_tracing();
    if let Err(e) = run_lone::<M>().await {
        tracing::error!("{e}");
        std::process::exit(1);
    }
}

async fn run_lone<M: Module>() -> std::io::Result<()> {
    let launch = module::read_launch_config()
        .await
        .map_err(|e| context("failed to read the launch config from stdin", e))?;
    let transport = transport::open_transport(&launch)
        .await
        .map_err(|e| context("failed to open the transport", e))?;
    let (topics, config) = module::parse_config_value::<M::Config>(&launch)?;
    module::validate_config(&config)?;
    module::log_wiring(&module::exe_name(), &topics, &config);

    let (tx, rx) = tokio::sync::watch::channel(false);
    tokio::spawn(async move {
        if module::shutdown_signal().await.is_ok() {
            let _ = tx.send(true);
        }
    });

    module::run_module_core::<M>(transport, topics, config, rx).await
}

fn context(what: &str, e: std::io::Error) -> std::io::Error {
    std::io::Error::new(e.kind(), format!("{what}: {e}"))
}
