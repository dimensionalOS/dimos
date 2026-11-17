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


"""K1 serial motor order and the official k1_walk.pt deployment contract."""

from dimos.utils.data import LfsPath

JOINT_NAMES = (
    "aahead_yaw_joint",
    "aahead_pitch_joint",
    "aaleft_shoulder_pitch_joint",
    "left_shoulder_roll_joint",
    "left_elbow_pitch_joint",
    "left_elbow_yaw_joint",
    "aaright_shoulder_pitch_joint",
    "right_shoulder_roll_joint",
    "right_elbow_pitch_joint",
    "right_elbow_yaw_joint",
    "left_hip_pitch_joint",
    "left_hip_roll_joint",
    "left_hip_yaw_joint",
    "left_knee_pitch_joint",
    "left_ankle_pitch_joint",
    "left_ankle_roll_joint",
    "right_hip_pitch_joint",
    "right_hip_roll_joint",
    "right_hip_yaw_joint",
    "right_knee_pitch_joint",
    "right_ankle_pitch_joint",
    "right_ankle_roll_joint",
)

JOINTS = tuple("k1/" + name for name in JOINT_NAMES)

POLICY_INDICES = (2, 6, 10, 16, 3, 7, 11, 17, 4, 8, 12, 18, 5, 9, 13, 19, 14, 20, 15, 21)

HOME = (
    0,
    0,
    0.2,
    -1.25,
    0,
    -0.5,
    0.2,
    1.25,
    0,
    0.5,
    -0.15,
    0,
    0,
    0.3,
    -0.15,
    0,
    -0.15,
    0,
    0,
    0.3,
    -0.15,
    0,
)

KP = (
    4.0,
    4.0,
    20.0,
    20.0,
    20.0,
    20.0,
    20.0,
    20.0,
    20.0,
    20.0,
    100.0,
    100.0,
    100.0,
    100.0,
    65.0,
    65.0,
    100.0,
    100.0,
    100.0,
    100.0,
    65.0,
    65.0,
)

KD = (
    1.0,
    1.0,
    2.0,
    2.0,
    2.0,
    2.0,
    2.0,
    2.0,
    2.0,
    2.0,
    2.0,
    2.0,
    2.0,
    2.0,
    1.0,
    1.0,
    2.0,
    2.0,
    2.0,
    2.0,
    1.0,
    1.0,
)

EFFORT_LIMITS = (
    6.0,
    6.0,
    14.0,
    14.0,
    14.0,
    14.0,
    14.0,
    14.0,
    14.0,
    14.0,
    30.0,
    20.0,
    15.0,
    35.0,
    24.0,
    15.0,
    30.0,
    20.0,
    15.0,
    35.0,
    24.0,
    15.0,
)

POLICY_PATH = LfsPath("booster_k1/k1_walk.pt")
PERIOD_NS = 20_000_000

LOWER = (
    -1.012,
    -0.314,
    -2.932,
    -1.642,
    -1.885,
    -2.242,
    -2.932,
    -1.642,
    -1.885,
    -0.816,
    -2.958,
    -0.375,
    -1.012,
    0.0,
    -0.87,
    -0.345,
    -2.958,
    -1.536,
    -1.012,
    0.0,
    -0.87,
    -0.345,
)

UPPER = (
    1.012,
    0.794,
    1.196,
    1.629,
    1.885,
    0.816,
    1.196,
    1.629,
    1.885,
    2.242,
    2.226,
    1.536,
    1.012,
    2.321,
    0.345,
    0.345,
    2.226,
    0.375,
    1.012,
    2.321,
    0.345,
    0.345,
)
