#!/usr/bin/env python3
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

"""A single vertical axis for the R1 Pro torso.

The torso is three pitch joints and a yaw, with no prismatic lift, so "raise
the torso" is not one joint. But the three pitch joints are exactly enough to
place head height while holding fore-aft position and keeping the head level,
which leaves precisely one free parameter: height.

This table is that parameter, solved offline against the vendor URDF. Every
sample holds x to under a millimetre of its neutral -0.079 m and the head
within a tenth of a degree of level, so interpolating it gives the operator a
straight vertical axis. The shape is a parallelogram fold -- roughly
``j1 = a, j2 = -2a, j3 = -a`` -- but the solved values are kept rather than the
idealisation, because the links are not exactly equal.

Driving the torso from this table instead of from the teleoperation IK is a
safety property, not a simplification: the torso shares no joint with the arm
solver, so a late or jumpy hand target cannot swing the torso.
"""

from __future__ import annotations

from dimos.robot.galaxea.r1pro.joints import coordinator_name

# Metres below the torso's full extension (head_link z = 1.5304 m), ascending.
TORSO_FOLD_DROPS: list[float] = [0.00, 0.05, 0.10, 0.15, 0.20, 0.25, 0.30, 0.35, 0.40, 0.45]

# Joint angles at each drop above. torso_joint2 reaches -2.41 rad at the bottom
# against a -2.67 limit, so 0.45 m keeps a real margin off the hard stop.
TORSO_FOLD_JOINTS: dict[str, list[float]] = {
    coordinator_name("torso_joint1"): [
        0.0000,
        0.3805,
        0.5414,
        0.6672,
        0.7755,
        0.8729,
        0.9630,
        1.0477,
        1.1284,
        1.2063,
    ],
    coordinator_name("torso_joint2"): [
        0.0000,
        -0.7605,
        -1.0822,
        -1.3339,
        -1.5504,
        -1.7451,
        -1.9251,
        -2.0944,
        -2.2558,
        -2.4112,
    ],
    coordinator_name("torso_joint3"): [
        0.0000,
        -0.3797,
        -0.5405,
        -0.6663,
        -0.7745,
        -0.8719,
        -0.9618,
        -1.0464,
        -1.1270,
        -1.2045,
    ],
    # Yaw is not part of the height axis; hold it where the tray pose puts it.
    coordinator_name("torso_joint4"): [0.0] * 10,
}

TORSO_JOINT_NAMES: list[str] = list(TORSO_FOLD_JOINTS)
TORSO_MAX_DROP: float = TORSO_FOLD_DROPS[-1]
