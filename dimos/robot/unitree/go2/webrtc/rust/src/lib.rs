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

//! The Go2 over its WebRTC link (the one the Unitree app uses): signaling, the data channel, the camera's H.264
//! and the lidar's voxels, as GO2Connection's native module.

pub mod clock;
pub mod conn;
pub mod h264;
pub mod lidar;
pub mod module;
pub mod signaling;
pub mod timing;
