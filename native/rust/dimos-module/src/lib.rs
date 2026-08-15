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

pub mod lcm;
pub mod log;
pub mod module;
pub mod tf;
pub mod transport;
pub mod zenoh;

pub use dimos_module_macros::{native_config, Module};
pub use lcm::LcmTransport;
pub use module::{
    read_launch_config, run, Builder, Input, Io, Module, ModuleConfig, NativeConfig, NoConfig,
    Output,
};
pub use tf::{Lookup, Tf, Transform};
pub use transport::Transport;
pub use zenoh::ZenohTransport;

pub use nalgebra;

// Re-export LcmOptions so callers don't need to depend on dimos-lcm directly.
pub use dimos_lcm::LcmOptions;

/// Run module `M` over the transport named by the `DIMOS_TRANSPORT` env var.
///
/// Every transport is compiled in, so one binary follows whichever transport the
/// coordinator picks at runtime. The coordinator always sets the variable, so an
/// unset or unknown value is an error. Its session settings come from the launch
/// config, which is why that is read first.
pub async fn run_with_transport<M: Module>() {
    crate::module::init_tracing();
    let launch = read_launch_config()
        .await
        .expect("failed to read the launch config from stdin");
    match std::env::var("DIMOS_TRANSPORT").as_deref() {
        Ok("lcm") => {
            let transport = LcmTransport::new()
                .await
                .expect("failed to create lcm transport");
            run::<M, _>(transport, launch).await
        }
        Ok("zenoh") => {
            let transport = ZenohTransport::from_launch(&launch)
                .await
                .expect("failed to create zenoh transport");
            run::<M, _>(transport, launch).await
        }
        other => panic!("DIMOS_TRANSPORT must be 'lcm' or 'zenoh', got {other:?}"),
    }
}
