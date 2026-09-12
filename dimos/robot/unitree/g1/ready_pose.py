# Copyright 2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""G1 ready-pose constants shared without loading controller or model dependencies."""

G1_READY_JOINTS = {
    "left_arm": (-0.4, 0.2, 0.0, 1.2, 0.0, 0.0, 0.0),
    "right_arm": (-0.4, -0.2, 0.0, 1.2, 0.0, 0.0, 0.0),
}
G1_READY_SPEED_SCALE = 0.25
