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

"""The module over a fake planner: plans and executions move the synthetic arm, frames arrive
on a thread the way a camera module delivers them, and tf answers from the arm's pose."""

from __future__ import annotations

from collections.abc import Iterator
import json
import threading
import time
from types import SimpleNamespace
from typing import Any
from unittest.mock import MagicMock

import numpy as np
import pytest
from scipy.spatial.transform import Rotation as R

from dimos.manipulation.calibration import store
from dimos.manipulation.calibration.hand_eye_module import HandEyeCalibrationModule
from dimos.manipulation.manipulation_spec import (
    ExecutionResult,
    ExecutionStatus,
    PlanningGroupInfo,
    PlanningGroupState,
    PlanResult,
    PlanStatus,
)
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.msgs.sensor_msgs.JointState import JointState

JOINTS = tuple(f"arm/joint{i}" for i in range(1, 7))
# The camera's own camera_link -> colour optical edge, as its tf would carry it.
T_LINK_OPTICAL = Transform(
    translation=Vector3(0.0, 0.015, 0.0),
    rotation=Quaternion(0.0, 0.0, 0.0, 1.0),
    frame_id="camera_link",
    child_frame_id="camera_color_optical_frame",
)
FRAME_PERIOD_S = 0.02


class FakePlanner:
    """The ManipulationSpec calls the module makes, moving the synthetic arm."""

    def __init__(self, arm: Any) -> None:
        self.arm = arm
        self.pending: list[float] | None = None
        self.executed: list[list[float]] = []
        self.cancelled = 0

    def list_planning_groups(self) -> tuple[PlanningGroupInfo, ...]:
        return (PlanningGroupInfo("manipulator", JOINTS, "world", "tcp", False),)

    def get_state(self) -> SimpleNamespace:
        joints = JointState(name=list(JOINTS), position=self.arm.get_joints())
        return SimpleNamespace(groups={"manipulator": PlanningGroupState(joints, None, None)})

    def plan_to_joints(
        self, targets: dict[str, JointState], speed_scale: float | None = None
    ) -> PlanResult:
        self.pending = list(targets["manipulator"].position)
        return PlanResult(PlanStatus.SUCCEEDED, plan=SimpleNamespace(plan_id="plan-1"))

    def execute(
        self, blocking: bool = True, timeout: float | None = None, *, plan_id: str | None = None
    ) -> ExecutionResult:
        assert self.pending is not None
        self.arm.move_to_joints(self.pending)
        self.executed.append(self.pending)
        self.pending = None
        return ExecutionResult(ExecutionStatus.COMPLETED)

    def cancel(self) -> ExecutionResult:
        self.cancelled += 1
        return ExecutionResult(ExecutionStatus.ABORTED)


class FakeTf:
    def __init__(self, arm: Any) -> None:
        self.arm = arm

    def get(
        self,
        parent: str,
        child: str,
        time_point: float | None = None,
        time_tolerance: float | None = None,
    ) -> Transform | None:
        if (parent, child) == ("world", "link6"):
            return Transform.from_matrix(
                self.arm.get_flange_pose(), frame_id=parent, child_frame_id=child
            )
        if (parent, child) == ("camera_link", "camera_color_optical_frame"):
            return T_LINK_OPTICAL
        return None

    def dispose(self) -> None:
        pass


@pytest.fixture
def planner(rig: Any) -> FakePlanner:
    return FakePlanner(rig.arm)


@pytest.fixture
def module(
    rig: Any, planner: FakePlanner, tmp_path, monkeypatch
) -> Iterator[HandEyeCalibrationModule]:
    monkeypatch.setattr(store, "HAND_EYE_DIR", tmp_path)
    instance = HandEyeCalibrationModule(
        name="test_wrist",
        nominal_translation=tuple(rig.X_true[:3, 3]),
        nominal_rotation_xyzw=tuple(R.from_matrix(rig.X_true[:3, :3]).as_quat()),
        home_joints=rig.home,
        joint_limits_rad=rig.limits,
        max_poses=12,
        approve_first_moves=0,
    )
    instance._manipulation = planner
    instance._obstacles = MagicMock()
    instance._tf = FakeTf(rig.arm)
    yield instance
    instance.stop()


@pytest.fixture
def frames(module: HandEyeCalibrationModule, rig: Any) -> Iterator[threading.Event]:
    """Frames of the synthetic camera, stamped now, delivered the way the camera module does."""
    stop = threading.Event()
    module._on_info(
        CameraInfo(height=rig.height, width=rig.width, D=[0.0] * 5, K=list(rig.K.ravel()))
    )

    def publish() -> None:
        while not stop.is_set():
            # stamped at exposure, before the render, as a camera stamps its frames
            ts = time.time()
            module._on_image(Image(data=rig.camera.render(), format=ImageFormat.RGB, ts=ts))
            time.sleep(FRAME_PERIOD_S)

    thread = threading.Thread(target=publish, name="SyntheticFrames", daemon=True)
    thread.start()
    yield stop
    stop.set()
    thread.join(timeout=2.0)


def test_calibrate_solves_the_mount_and_writes_it(
    module: HandEyeCalibrationModule,
    planner: FakePlanner,
    frames: threading.Event,
    rig: Any,
    tmp_path,
) -> None:
    report = module.calibrate()

    assert report["passed"], report["reason"]
    assert len(planner.executed) == report["poses_moved"] == 12
    expected_mount = rig.X_true @ np.linalg.inv(T_LINK_OPTICAL.to_matrix())
    assert np.allclose(report["mount"]["translation_m"], expected_mount[:3, 3], atol=0.0005)
    assert report["mount"]["frame_id"] == "link6"
    assert report["mount"]["child_frame_id"] == "camera_link"
    written = json.loads((tmp_path / "test_wrist.json").read_text())
    assert written["passed"] is True
    assert written["mount"]["translation_m"] == report["mount"]["translation_m"]
    assert store.load_hand_eye("test_wrist") is not None


def test_a_target_beyond_the_joint_step_cap_never_reaches_the_planner(
    module: HandEyeCalibrationModule, planner: FakePlanner, frames: threading.Event
) -> None:
    module.config.max_joint_step_rad = 0.01

    report = module.calibrate()

    assert planner.executed == []
    assert report["passed"] is False
    assert report["reason"] == "too few detections"


def test_dry_run_plans_but_executes_nothing(
    module: HandEyeCalibrationModule, planner: FakePlanner, frames: threading.Event
) -> None:
    module.config.dry_run = True

    report = module.calibrate()

    assert planner.executed == []
    assert report["poses_moved"] == 0
    assert report["poses_tried"] == 12


def test_first_moves_wait_for_approval_and_abort_cancels(
    module: HandEyeCalibrationModule, planner: FakePlanner, frames: threading.Event
) -> None:
    module.config.approve_first_moves = 2
    results: list[dict] = []
    runner = threading.Thread(target=lambda: results.append(module.calibrate()), daemon=True)
    runner.start()
    deadline = time.monotonic() + 5.0
    while not module.status()["waiting_for_approval"] and time.monotonic() < deadline:
        time.sleep(0.01)
    waiting = module.status()["waiting_for_approval"]
    moved_before = len(planner.executed)

    module.approve()
    deadline = time.monotonic() + 5.0
    while len(planner.executed) < 1 and time.monotonic() < deadline:
        time.sleep(0.01)
    module.abort()
    runner.join(timeout=10.0)

    assert waiting and moved_before == 0
    assert len(planner.executed) == 1
    assert planner.cancelled == 1
    assert results and results[0]["passed"] is False
    assert not runner.is_alive()


def test_the_table_is_a_planner_obstacle_for_the_run_only(
    module: HandEyeCalibrationModule, planner: FakePlanner, frames: threading.Event
) -> None:
    module.config.table_top_z = 0.075
    module.config.table_x_m = (0.3, 0.9)
    module.config.table_y_m = (-0.5, 0.5)
    module._obstacles.add_obstacle.return_value = "table-id"

    module.calibrate()

    module._obstacles.add_obstacle.assert_called_once()
    name, pose, shape, dims = module._obstacles.add_obstacle.call_args.args
    assert (name, shape) == ("hand_eye_table", "box")
    assert np.allclose(dims, [0.6, 1.0, 0.2])
    assert np.allclose([pose.position.x, pose.position.y, pose.position.z], [0.6, 0.0, -0.025])
    module._obstacles.remove_obstacle.assert_called_once_with("table-id")
