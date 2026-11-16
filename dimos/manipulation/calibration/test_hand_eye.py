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

from typing import Any

import numpy as np

from dimos.manipulation.calibration.hand_eye import (
    PROBE_M,
    CameraNominal,
    Frame,
    solve_hand_eye,
)


def _nominal(rig: Any, offset_m: float = 0.0, tol_mm: float = 10.0) -> CameraNominal:
    T = rig.X_true.copy()
    T[0, 3] += offset_m
    return CameraNominal("flange", T, tol_mm, 5.0)


def test_solves_the_wrist_camera_within_a_millimetre(rig: Any) -> None:
    result = solve_hand_eye(
        rig.arm, rig.camera, rig.board, _nominal(rig), rig.home, max_poses=20, max_seconds=300
    )

    probe = np.array([0.0, 0.0, PROBE_M, 1.0])
    probe_err_mm = np.linalg.norm((result.T @ probe)[:3] - (rig.X_true @ probe)[:3]) * 1000
    assert result.passed, result.reason
    # the rendered board is a warped 2 px/mm image, which bounds the corners at about 0.1 px
    assert probe_err_mm < 1.0
    assert np.linalg.norm(result.T[:3, 3] - rig.X_true[:3, 3]) * 1000 < 0.5
    assert result.confidence["views_used"] >= 10
    assert rig.arm.moves == result.poses_moved == 20


def test_refuses_a_solution_far_from_the_stated_nominal(rig: Any) -> None:
    # the mount is believed to be 40 mm from where it is, with a 10 mm tolerance
    result = solve_hand_eye(
        rig.arm,
        rig.camera,
        rig.board,
        _nominal(rig, offset_m=0.04),
        rig.home,
        max_poses=20,
        max_seconds=300,
    )

    assert not result.passed
    assert result.reason == "deviation from nominal"
    assert 39.0 < result.confidence["deviation_from_nominal_mm"] < 41.0


def test_fails_when_the_board_is_never_seen(rig: Any) -> None:
    class _Blind:
        def capture(self) -> Frame:
            return Frame(rig.camera.blank(), rig.K.copy(), np.zeros(5), 0.0)

    result = solve_hand_eye(
        rig.arm, _Blind(), rig.board, _nominal(rig), rig.home, max_poses=8, max_seconds=300
    )

    assert not result.passed
    assert result.reason == "too few detections"
    assert np.array_equal(result.T, rig.X_true)
    assert result.poses_tried == 8


def test_a_refused_move_costs_a_pose_but_no_capture(rig: Any) -> None:
    refusals = {"left": 3}
    captures = {"n": 0}

    class _Refusing:
        def joint_limits(self) -> list[tuple[float, float]]:
            return rig.arm.joint_limits()

        def get_joints(self) -> list[float]:
            return rig.arm.get_joints()

        def get_flange_pose(self) -> np.ndarray:
            return rig.arm.get_flange_pose()

        def move_to_joints(self, q: list[float]) -> bool:
            if refusals["left"] > 0:
                refusals["left"] -= 1
                return False
            return rig.arm.move_to_joints(q)

        def clock(self) -> float:
            return 0.0

    class _Counting:
        def capture(self) -> Frame:
            captures["n"] += 1
            return rig.camera.capture()

    result = solve_hand_eye(
        _Refusing(), _Counting(), rig.board, _nominal(rig), rig.home, max_poses=12, max_seconds=300
    )

    assert result.poses_tried == 12
    assert result.poses_moved == captures["n"] == 9


def test_the_standoff_target_pays_back_both_directions_and_sweeps_on_target() -> None:
    from dimos.manipulation.calibration.hand_eye import (
        DEPTH_SWEEP_M,
        PROBE_M,
        STANDOFF_MAX_M,
        STANDOFF_MIN_M,
        _standoff_target,
    )

    too_close = _standoff_target(np.array([0.28, 0.28, 0.28]), 10)
    too_far = _standoff_target(np.array([0.55, 0.55, 0.55]), 10)
    on_target_odd = _standoff_target(np.array([0.40, 0.40, 0.40]), 10)
    on_target_even = _standoff_target(np.array([0.40, 0.40]), 10)

    assert too_close is not None and PROBE_M < too_close <= STANDOFF_MAX_M
    assert too_far is not None and STANDOFF_MIN_M <= too_far < PROBE_M
    assert on_target_odd == PROBE_M + DEPTH_SWEEP_M
    assert on_target_even == PROBE_M - DEPTH_SWEEP_M


def test_solves_a_fixed_camera_from_a_plate_in_the_gripper(fixed_rig: Any) -> None:
    nominal = CameraNominal(
        "base", fixed_rig.X_true + np.diag([0.0, 0.0, 0.0, 0.0]), tol_mm=150.0, tol_deg=10.0
    )
    nominal.T[:3, 3] += np.array([0.03, -0.02, 0.04])  # a tape-measure prior

    result = solve_hand_eye(
        fixed_rig.arm, fixed_rig.camera, fixed_rig.board, nominal, fixed_rig.home, 20, 300.0
    )

    assert result.passed, result.reason
    assert result.frame == "base"
    probe = np.array([0.0, 0.0, PROBE_M, 1.0])
    err = np.linalg.norm((result.T @ probe)[:3] - (fixed_rig.X_true @ probe)[:3]) * 1000
    assert err < 1.0, err
