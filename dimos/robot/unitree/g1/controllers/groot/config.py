# Copyright 2025-2026 Dimensional Inc.
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

"""GR00T-trained home pose and motor gains; no policy execution dependencies."""

# The 29 DDS motor names plus the per-joint kp/kd values used with the
# GR00T-trained policies. Diverging from these on real hardware risks
# instability because the ONNX models were trained against this control
# contract.
G1_GROOT_KP: list[float] = [
    150.0,
    150.0,
    150.0,
    200.0,
    40.0,
    40.0,  # left leg
    150.0,
    150.0,
    150.0,
    200.0,
    40.0,
    40.0,  # right leg
    250.0,
    250.0,
    250.0,  # waist
    100.0,
    100.0,
    40.0,
    40.0,
    20.0,
    20.0,
    20.0,  # left arm
    100.0,
    100.0,
    40.0,
    40.0,
    20.0,
    20.0,
    20.0,  # right arm
]
G1_GROOT_KD: list[float] = [
    2.0,
    2.0,
    2.0,
    4.0,
    2.0,
    2.0,  # left leg
    2.0,
    2.0,
    2.0,
    4.0,
    2.0,
    2.0,  # right leg
    5.0,
    5.0,
    5.0,  # waist
    5.0,
    5.0,
    2.0,
    2.0,
    2.0,
    2.0,
    2.0,  # left arm
    5.0,
    5.0,
    2.0,
    2.0,
    2.0,
    2.0,
    2.0,  # right arm
]

# Default joint angles for all 29 G1 joints. The policy treats these as
# its zero-offset pose.
G1_GROOT_HOME = [
    -0.1,
    0.0,
    0.0,
    0.3,
    -0.2,
    0.0,  # left leg
    -0.1,
    0.0,
    0.0,
    0.3,
    -0.2,
    0.0,  # right leg
    0.0,
    0.0,
    0.0,  # waist
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,  # left arm (not driven by policy)
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,  # right arm (not driven by policy)
]
