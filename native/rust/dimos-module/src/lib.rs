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

pub mod lcm;
pub mod log;
pub mod module;
pub mod transport;

pub use dimos_module_macros::{native_config, Module};
pub use lcm::LcmTransport;
pub use module::{run, Builder, Input, Module, ModuleConfig, NativeConfig, NoConfig, Output};
pub use transport::Transport;

// Re-export LcmOptions so callers don't need to depend on dimos-lcm directly.
pub use dimos_lcm::LcmOptions;
