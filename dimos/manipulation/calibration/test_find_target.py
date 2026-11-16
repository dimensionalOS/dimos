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

from __future__ import annotations

import numpy as np
from scipy.spatial.transform import Rotation as R

from dimos.manipulation.calibration.find_target import (
    board_in_base,
    centre_over_board,
    image_yaw,
    look_down,
    search_points,
)
from dimos.manipulation.calibration.hand_eye import BoardSpec, T_inv, rt_to_T

# A wrist camera 80 mm off the flange axis, tilted 10 degrees in, as on the xArm kit mount.
X = rt_to_T(R.from_euler("zyx", [1.5708, 0.0, -0.1745]).as_matrix(), np.array([0.08, 0.0, 0.02]))
BOARD = BoardSpec(squares_x=8, squares_y=5, square_m=0.030, marker_m=0.022)


def test_look_down_puts_the_camera_above_the_point_looking_down() -> None:
    T_optical = look_down(np.array([0.4, -0.1, 0.0]), 0.55, 0.3, X) @ X

    np.testing.assert_allclose(T_optical[:3, 3], [0.4, -0.1, 0.55], atol=1e-9)
    np.testing.assert_allclose(T_optical[:3, 2], [0.0, 0.0, -1.0], atol=1e-9)
    assert abs(image_yaw(T_optical @ T_inv(X), X) - 0.3) < 1e-9


def test_the_centred_view_has_the_board_centre_on_the_axis_at_the_standoff() -> None:
    # a board flat on the table, face up, turned 20 degrees, seen once from a search pose
    T_base_board = rt_to_T(
        R.from_euler("ZX", [0.35, np.pi]).as_matrix(), np.array([0.30, 0.20, 0.0])
    )
    seen_from = look_down(np.array([0.35, 0.10, 0.0]), 0.55, 0.0, X)
    T_optical_board = T_inv(seen_from @ X) @ T_base_board
    found = board_in_base(seen_from, X, T_optical_board)

    T_flange = centre_over_board(found, BOARD, 0.4, X, prefer_yaw=0.0)
    centre = np.array([0.12, 0.075, 0.0, 1.0])
    in_camera = T_inv(T_flange @ X) @ found @ centre

    np.testing.assert_allclose(in_camera[:3], [0.0, 0.0, 0.4], atol=1e-9)
    # the long side lies along the image, the way round nearer the yaw asked for
    assert abs(image_yaw(T_flange, X) - 0.35) < 1e-9


def test_the_search_starts_nearest_where_the_camera_looks() -> None:
    points = search_points((0.1, 0.7), (-0.4, 0.4), 0.01, 0.3, near=np.array([0.65, 0.35, 0.0]))

    assert len(points) == 9
    np.testing.assert_allclose(points[0], [0.7, 0.2, 0.01])
    assert all(p[2] == 0.01 for p in points)
