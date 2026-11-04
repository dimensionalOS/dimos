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

"""A synthetic arm with a wrist camera (``rig``) or a fixed camera watching a plate in its
gripper (``fixed_rig``) for the hand-eye tests.

The arm's six "joints" are the flange's position and roll-pitch-yaw in the base. The camera
renders the ChArUco board by warping the board's own image through the true camera pose, so
detection, PnP and the refinement run on real pixels.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import cv2
import numpy as np
from numpy.typing import NDArray
import pytest
from scipy.spatial.transform import Rotation as R

from dimos.manipulation.calibration.hand_eye import (
    PROBE_M,
    BoardSpec,
    Frame,
    T_inv,
    charuco_board,
    rt_to_T,
)

_BOARD = BoardSpec(squares_x=8, squares_y=5, square_m=0.030, marker_m=0.022)
_WIDTH, _HEIGHT = 848, 480
_K = np.array([[610.0, 0.0, 424.0], [0.0, 610.0, 240.0], [0.0, 0.0, 1.0]])
_PX_PER_M = 2000  # the board image's resolution, 2 px per mm
# The true camera on the flange: 80 mm off the axis, 20 mm forward, 10 degrees in.
_X_TRUE = rt_to_T(
    R.from_euler("zyx", [1.5708, 0.0, -0.1745]).as_matrix(), np.array([0.08, 0.0, 0.02])
)
# The board, flat on the table 0.45 m in front of the base.
_T_BASE_BOARD = rt_to_T(R.from_euler("x", np.pi).as_matrix(), np.array([0.33, 0.075, 0.0]))
# A home pose that looks straight down at the board from PROBE_M.
_HOME = [0.45, 0.0, PROBE_M + 0.02, np.pi, 0.0, 0.0]
_LIMITS = [(-1.0, 1.0), (-1.0, 1.0), (0.1, 1.0), (-7.0, 7.0), (-1.5, 1.5), (-7.0, 7.0)]


@dataclass
class SyntheticArm:
    q: NDArray[np.float64]
    moves: int = 0

    def joint_limits(self) -> list[tuple[float, float]]:
        return _LIMITS

    def get_joints(self) -> list[float]:
        return [float(v) for v in self.q]

    def get_flange_pose(self) -> NDArray[np.float64]:
        return rt_to_T(R.from_euler("xyz", self.q[3:]).as_matrix(), self.q[:3])

    def move_to_joints(self, q: list[float]) -> bool:
        self.q = np.asarray(q, dtype=float)
        self.moves += 1
        return True

    def clock(self) -> float:
        return 0.0


class SyntheticCamera:
    """Renders ``board`` as seen from the camera: on the flange at ``X_true`` watching a board at
    ``T_base_board``, or fixed in the base at ``X_true`` watching a plate at ``T_flange_board``."""

    def __init__(
        self,
        arm: SyntheticArm,
        board: BoardSpec = _BOARD,
        X_true: NDArray[np.float64] = _X_TRUE,
        T_base_board: NDArray[np.float64] = _T_BASE_BOARD,
        T_flange_board: NDArray[np.float64] | None = None,
    ) -> None:
        self.arm = arm
        self._X = X_true
        self._T_base_board = T_base_board
        self._T_flange_board = T_flange_board
        board_obj = charuco_board(board)
        w, h = board.squares_x * board.square_m, board.squares_y * board.square_m
        self._board_px = board_obj.generateImage(
            (int(w * _PX_PER_M), int(h * _PX_PER_M)), marginSize=0
        )
        # OpenCV's board frame: origin at the image's top-left corner, x right, y down the image.
        self._board_to_px = np.array(
            [[_PX_PER_M, 0.0, 0.0], [0.0, _PX_PER_M, 0.0], [0.0, 0.0, 1.0]]
        )

    def render(self) -> NDArray[np.uint8]:
        flange = self.arm.get_flange_pose()
        if self._T_flange_board is None:
            T_cam_board = T_inv(flange @ self._X) @ self._T_base_board
        else:
            T_cam_board = T_inv(self._X) @ flange @ self._T_flange_board
        Rm, t = T_cam_board[:3, :3], T_cam_board[:3, 3]
        H = _K @ np.column_stack([Rm[:, 0], Rm[:, 1], t]) @ np.linalg.inv(self._board_to_px)
        gray = cv2.warpPerspective(
            self._board_px, H, (_WIDTH, _HEIGHT), flags=cv2.INTER_LINEAR, borderValue=110
        )
        return np.repeat(gray[:, :, None], 3, axis=2).astype(np.uint8)

    def blank(self) -> NDArray[np.uint8]:
        return np.full((_HEIGHT, _WIDTH, 3), 110, np.uint8)

    def capture(self) -> Frame:
        return Frame(self.render(), _K.copy(), np.zeros(5), 0.0)


@dataclass
class SyntheticRig:
    arm: SyntheticArm
    camera: SyntheticCamera
    board: BoardSpec = _BOARD
    home: list[float] = field(default_factory=lambda: list(_HOME))
    limits: list[tuple[float, float]] = field(default_factory=lambda: list(_LIMITS))
    X_true: NDArray[np.float64] = field(default_factory=lambda: _X_TRUE.copy())
    K: NDArray[np.float64] = field(default_factory=lambda: _K.copy())
    width: int = _WIDTH
    height: int = _HEIGHT


@pytest.fixture
def rig() -> SyntheticRig:
    arm = SyntheticArm(q=np.asarray(_HOME, dtype=float))
    return SyntheticRig(arm=arm, camera=SyntheticCamera(arm))


# The fixed camera: 0.6 m to the arm's right at the plate's height, looking along +y at the
# arm, image right along +x, image down along -z.
_PLATE = BoardSpec(squares_x=5, squares_y=4, square_m=0.030, marker_m=0.0225, ids_from=100)
_X_FIXED = rt_to_T(
    np.column_stack([[1.0, 0.0, 0.0], [0.0, 0.0, -1.0], [0.0, 1.0, 0.0]]),
    np.array([0.45, -0.6, 0.2]),
)
# The plate in the fingers: its face normal along the flange's -y (toward the camera at the
# home pose), its long side along the flange x, centred 0.2 m along the tool axis.
_T_FLANGE_PLATE = rt_to_T(
    np.column_stack([[1.0, 0.0, 0.0], [0.0, 0.0, 1.0], [0.0, -1.0, 0.0]]),
    np.array([-0.075, 0.0, 0.14]),
)
_HOME_FIXED = [0.45, 0.0, 0.3, np.pi, 0.0, 0.0]


@pytest.fixture
def fixed_rig() -> SyntheticRig:
    arm = SyntheticArm(q=np.asarray(_HOME_FIXED, dtype=float))
    camera = SyntheticCamera(arm, board=_PLATE, X_true=_X_FIXED, T_flange_board=_T_FLANGE_PLATE)
    return SyntheticRig(
        arm=arm,
        camera=camera,
        board=_PLATE,
        home=list(_HOME_FIXED),
        X_true=_X_FIXED.copy(),
    )
