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

import numpy as np

from dimos.simulation.mujoco.mujoco_process import filter_g1_self_points


def test_filter_g1_self_points_removes_body_points_but_keeps_floor_and_scene() -> None:
    points = np.array(
        [
            [0.20, 0.00, 0.80],  # robot body
            [0.50, 0.30, 0.20],  # robot limb envelope
            [0.20, 0.00, 0.00],  # floor under robot
            [1.20, 0.00, 0.80],  # scene object in front
            [0.20, 0.70, 0.80],  # scene object at side
        ],
        dtype=np.float32,
    )

    filtered = filter_g1_self_points(
        points,
        base_pos=np.array([0.0, 0.0, 0.75], dtype=np.float32),
        base_quat_wxyz=np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32),
    )

    assert filtered.tolist() == [
        [0.20000000298023224, 0.0, 0.0],
        [1.2000000476837158, 0.0, 0.800000011920929],
        [0.20000000298023224, 0.699999988079071, 0.800000011920929],
    ]


def test_filter_g1_self_points_tracks_robot_yaw() -> None:
    half_turn_yaw_quat = np.array([np.sqrt(0.5), 0.0, 0.0, np.sqrt(0.5)], dtype=np.float32)
    points = np.array(
        [
            [0.00, 0.40, 0.80],  # inside body envelope after +90 degree yaw
            [0.60, 0.00, 0.80],  # outside side envelope after +90 degree yaw
        ],
        dtype=np.float32,
    )

    filtered = filter_g1_self_points(
        points,
        base_pos=np.array([0.0, 0.0, 0.75], dtype=np.float32),
        base_quat_wxyz=half_turn_yaw_quat,
    )

    assert filtered.tolist() == [[0.6000000238418579, 0.0, 0.800000011920929]]
