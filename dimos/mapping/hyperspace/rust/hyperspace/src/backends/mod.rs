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

//! Real model backends (feature-gated).

// Callers need the same candle types to build a Device/DType, so hand them ours
// rather than making them pin candle themselves.
#[cfg(any(feature = "siglip", feature = "depth2depth"))]
pub use candle_core as candle;

#[cfg(feature = "depth2depth")]
pub mod depth2depth;
#[cfg(feature = "siglip")]
pub mod siglip;
#[cfg(feature = "siglip")]
pub mod siglip_model;
