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

"""Where to look for a board, or a camera, nobody measured, and where to stand once it is seen.

A wrist camera finds a board lying flat on the table by looking straight down from above a grid
of points over the table; once a view shows it, the run moves the camera straight above the
board's centre, square to it, at the standoff the solve wants. A fixed camera is found the other
way round: the arm holds the plate in its fingers facing one aim point after another until the
camera sees it, and that one view says where the camera is. Everything here is geometry in the
arm's base frame from nominal mounts: a few centimetres off still puts the target in view, which
is all the search needs.
"""

from __future__ import annotations

import numpy as np

from dimos.manipulation.calibration.hand_eye import BoardSpec, FloatArray, T_inv, rt_to_T


def look_down(
    point: FloatArray, height_m: float, yaw: float, X_flange_optical: FloatArray
) -> FloatArray:
    """Base -> flange for a camera ``height_m`` above ``point`` looking straight down, its image
    right along ``yaw`` in the base's xy plane."""
    x = np.array([np.cos(yaw), np.sin(yaw), 0.0])
    z = np.array([0.0, 0.0, -1.0])
    T_base_optical = rt_to_T(
        np.column_stack([x, np.cross(z, x), z]), np.asarray(point) + np.array([0.0, 0.0, height_m])
    )
    return T_base_optical @ T_inv(X_flange_optical)


def search_points(
    x_m: tuple[float, float], y_m: tuple[float, float], z: float, spacing_m: float, near: FloatArray
) -> list[FloatArray]:
    """A grid over the table at height ``z``, nearest ``near`` first, so the search starts where
    the arm already looks."""
    xs = np.arange(x_m[0], x_m[1] + 1e-9, spacing_m)
    ys = np.arange(y_m[0], y_m[1] + 1e-9, spacing_m)
    pts = [np.array([x, y, z]) for x in xs for y in ys]
    return sorted(pts, key=lambda p: float(np.linalg.norm(p[:2] - np.asarray(near)[:2])))


def board_in_base(
    T_base_flange: FloatArray, X_flange_optical: FloatArray, T_optical_board: FloatArray
) -> FloatArray:
    """Base -> board from one detection, through the nominal mount."""
    return T_base_flange @ X_flange_optical @ T_optical_board


def image_yaw(T_base_flange: FloatArray, X_flange_optical: FloatArray) -> float:
    """Which way the camera's image right points in the base's xy plane."""
    x = (T_base_flange @ X_flange_optical)[:3, 0]
    return float(np.arctan2(x[1], x[0]))


def centre_over_board(
    T_base_board: FloatArray,
    board: BoardSpec,
    standoff_m: float,
    X_flange_optical: FloatArray,
    prefer_yaw: float,
) -> FloatArray:
    """Base -> flange for the camera ``standoff_m`` straight above the board's centre, its image
    right along the board's long side, the view the draw is centred on. Of the two ways round,
    the one nearer ``prefer_yaw``, so the wrist turns least."""
    w, h = board.squares_x * board.square_m, board.squares_y * board.square_m
    centre = (T_base_board @ np.array([w / 2, h / 2, 0.0, 1.0]))[:3]
    long_side = T_base_board[:3, 0]
    yaw = float(np.arctan2(long_side[1], long_side[0]))
    if abs(np.angle(np.exp(1j * (yaw - prefer_yaw)))) > np.pi / 2:
        yaw += np.pi
    return look_down(centre, standoff_m, yaw, X_flange_optical)


def aim_plate(
    centre: FloatArray, at: FloatArray, board: BoardSpec, T_flange_board: FloatArray
) -> FloatArray:
    """Base -> flange for the held plate centred on ``centre``, its printed face square to the
    point ``at`` and its long side level. ``T_flange_board`` is where the plate sits in the
    fingers, OpenCV's board frame: origin at a corner, z into the plate. Of the two ways to hold
    it level, the one with the tool pointing down, the way a gripper holds a card out in front."""
    z = np.asarray(centre, dtype=float) - np.asarray(at, dtype=float)
    z /= np.linalg.norm(z)
    x = np.cross([0.0, 0.0, 1.0], z)
    x = x / np.linalg.norm(x) if np.linalg.norm(x) > 1e-6 else np.array([1.0, 0.0, 0.0])
    w, h = board.squares_x * board.square_m, board.squares_y * board.square_m
    options = []
    for sx in (1.0, -1.0):
        y = np.cross(z, sx * x)
        origin = np.asarray(centre, dtype=float) - sx * x * w / 2 - y * h / 2
        options.append(rt_to_T(np.column_stack([sx * x, y, z]), origin) @ T_inv(T_flange_board))
    # the tool axis is the flange's z; the lower its z component, the more it points down
    return min(options, key=lambda T: float(T[2, 2]))


def camera_in_base(
    T_base_flange: FloatArray, T_flange_board: FloatArray, T_optical_board: FloatArray
) -> FloatArray:
    """Base -> fixed camera optical frame from one view of the held plate."""
    return T_base_flange @ T_flange_board @ T_inv(T_optical_board)
