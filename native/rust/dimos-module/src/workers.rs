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

/// Cap the process-wide rayon worker pool. One pool per process: a no-op when
/// some other module in this process already sized it, which keeps the first
/// configured value.
pub fn init_worker_pool(threads: u32) {
    let _ = rayon::ThreadPoolBuilder::new()
        .num_threads(threads as usize)
        .build_global();
}
