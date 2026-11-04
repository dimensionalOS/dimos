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

use super::*;

#[test]
fn due_scales_by_speed_and_offsets_each_pass() {
    let span = (10.0, 12.0);
    assert_eq!(due(11.0, 0, span, 2.0, 0.5), Duration::from_secs_f64(0.5));
    assert_eq!(due(10.0, 1, span, 1.0, 0.5), Duration::from_secs_f64(2.5));
    assert_eq!(due(9.0, 0, span, 1.0, 0.5), Duration::ZERO);
}
