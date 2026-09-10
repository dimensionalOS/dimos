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

//! One module per law, mirroring `control/laws/`.
//!
//! A law composes `geom` (and `stamps`, if it reads the wire dialect) into a
//! tick. The follower runs `hinted`; `seed` is the permanent baseline. A
//! research generation lands by replacing `hinted`, which is why the modules
//! stay small and the shared facilities stay in `geom`/`stamps`.

pub mod hinted;
pub mod seed;
