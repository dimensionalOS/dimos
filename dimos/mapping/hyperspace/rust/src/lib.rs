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

//! Hyperspace wrapped as a dimos native module.
//!
//! The [`hyperspace`] crate holds the mapping and querying; this crate is the
//! adapter: LCM messages in, a scored voxel cloud out, plus the offline driver
//! the Python CLI uses to turn a recording into an rrd.

pub mod config;
pub mod convert;
pub mod module;
pub mod query_request;

pub use config::Config;
pub use module::Hyperspace;
pub use query_request::QueryRequest;
