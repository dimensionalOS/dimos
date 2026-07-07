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

from __future__ import annotations

from contextlib import contextmanager
from pathlib import Path

import numpy as np

from dimos.manipulation.planning.groups.models import PlanningGroup
from dimos.manipulation.planning.kinematics import drake_optimization_ik as drake_ik
from dimos.manipulation.planning.kinematics.drake_optimization_ik import DrakeOptimizationIK
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.manipulation.planning.spec.enums import IKStatus
from dimos.manipulation.planning.spec.models import IKResult
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.JointState import JointState


class FakeWorld:
    def __init__(self) -> None:
        self.robot_id = "robot-instance"
        self.config = RobotModelConfig(
            name="arm",
            model_path=Path("/tmp/fake.urdf"),
            joint_names=["base", "shoulder", "elbow", "wrist"],
        )
        self.current_state = JointState(
            {"name": ["base", "shoulder", "elbow", "wrist"], "position": [1.0, 2.0, 3.0, 4.0]}
        )
        self.collision_checked_state: JointState | None = None

    def get_robot_ids(self) -> list[str]:
        return [self.robot_id]

    def get_robot_config(self, robot_id: str) -> RobotModelConfig:
        assert robot_id == self.robot_id
        return self.config

    def get_joint_limits(self, robot_id: str) -> tuple[np.ndarray, np.ndarray]:
        assert robot_id == self.robot_id
        return np.array([-10.0] * 4), np.array([10.0] * 4)

    @contextmanager
    def scratch_context(self):
        yield object()

    def get_joint_state(self, ctx: object, robot_id: str) -> JointState:
        assert robot_id == self.robot_id
        return self.current_state

    def check_config_collision_free(self, robot_id: str, joint_state: JointState) -> bool:
        assert robot_id == self.robot_id
        self.collision_checked_state = joint_state
        return True


def test_solve_pose_targets_uses_group_tip_locks_seed_fallback_and_filters(monkeypatch) -> None:
    monkeypatch.setattr(drake_ik, "DRAKE_AVAILABLE", True)
    monkeypatch.setattr(DrakeOptimizationIK, "_validate_world", lambda self, world: None)

    world = FakeWorld()
    group = PlanningGroup(
        id="arm/reach",
        robot_name="arm",
        group_name="reach",
        joint_names=("arm/shoulder", "arm/wrist"),
        local_joint_names=("shoulder", "wrist"),
        base_link="base_link",
        tip_link="group_tip_link",
    )
    calls = []

    def fake_solve_single(self, **kwargs) -> IKResult:
        calls.append(kwargs)
        return IKResult(
            status=IKStatus.SUCCESS,
            joint_state=JointState(
                {
                    "name": ["base", "shoulder", "elbow", "wrist"],
                    "position": [10.0, 20.0, 30.0, 40.0],
                }
            ),
            position_error=0.0,
            orientation_error=0.0,
        )

    monkeypatch.setattr(DrakeOptimizationIK, "_solve_single", fake_solve_single)
    monkeypatch.setattr(drake_ik, "RigidTransform", lambda matrix: matrix, raising=False)

    result = DrakeOptimizationIK().solve_pose_targets(
        world=world,  # type: ignore[arg-type]
        pose_targets={group: PoseStamped()},
        seed=JointState({"name": ["shoulder"], "position": [22.0]}),
        check_collision=False,
        max_attempts=1,
    )

    assert result.is_success()
    assert result.joint_state is not None
    assert result.joint_state.name == ["arm/shoulder", "arm/wrist"]
    assert result.joint_state.position == [20.0, 40.0]
    assert calls[0]["target_frame_name"] == "group_tip_link"
    np.testing.assert_allclose(calls[0]["seed"], [1.0, 22.0, 3.0, 4.0])
    assert calls[0]["locked_joint_positions"] == {0: 1.0, 2: 3.0}


class FakeRigidTransform:
    def translation(self) -> np.ndarray:
        return np.array([0.5, 0.6, 0.7])

    def rotation(self) -> str:
        return "target-rotation"

    def GetAsMatrix4(self) -> np.ndarray:
        return np.eye(4)


class FakeBody:
    def __init__(self, frame_name: str) -> None:
        self.frame_name = frame_name

    def body_frame(self) -> str:
        return f"frame:{self.frame_name}"


class FakePlant:
    def __init__(self) -> None:
        self.requested_bodies: list[tuple[str, str]] = []

    def GetBodyByName(self, frame_name: str, model_instance: str) -> FakeBody:
        self.requested_bodies.append((frame_name, model_instance))
        return FakeBody(frame_name)

    def world_frame(self) -> str:
        return "world-frame"

    def num_positions(self) -> int:
        return 6


class FakeProgram:
    def __init__(self) -> None:
        self.locks: list[tuple[float, float, str]] = []
        self.initial_guess: tuple[list[str], np.ndarray] | None = None

    def AddBoundingBoxConstraint(self, lower: float, upper: float, variable: str) -> None:
        self.locks.append((lower, upper, variable))

    def SetInitialGuess(self, q: list[str], full_seed: np.ndarray) -> None:
        self.initial_guess = (q, full_seed.copy())


class FakeInverseKinematics:
    instances: list[FakeInverseKinematics] = []

    def __init__(self, plant: FakePlant) -> None:
        self.plant = plant
        self.program = FakeProgram()
        self.q_vars = [f"q{i}" for i in range(6)]
        self.position_constraints = []
        self.orientation_constraints = []
        self.instances.append(self)

    def AddPositionConstraint(self, **kwargs) -> None:
        self.position_constraints.append(kwargs)

    def AddOrientationConstraint(self, **kwargs) -> None:
        self.orientation_constraints.append(kwargs)

    def get_mutable_prog(self) -> FakeProgram:
        return self.program

    def q(self) -> list[str]:
        return self.q_vars


class FakeSolveResult:
    def is_success(self) -> bool:
        return True

    def GetSolution(self, q: list[str]) -> np.ndarray:
        return np.array([0.0, 11.0, 0.0, 22.0, 33.0, 0.0])


class FakeDrakeWorld:
    def __init__(self) -> None:
        self.plant = FakePlant()
        self._robots = {"robot-instance": _FakeRobotData()}
        self.link_pose_calls: list[tuple[str, str]] = []
        self.set_joint_state_calls: list[JointState] = []

    @contextmanager
    def scratch_context(self):
        yield "ctx"

    def set_joint_state(self, ctx: str, robot_id: str, joint_state: JointState) -> None:
        self.set_joint_state_calls.append(joint_state)

    def get_link_pose(self, ctx: str, robot_id: str, target_frame_name: str) -> np.ndarray:
        self.link_pose_calls.append((robot_id, target_frame_name))
        return np.eye(4)


class _FakeRobotData:
    model_instance = "model-instance"
    joint_indices = [1, 3, 4]


def test_solve_single_uses_target_frame_for_constraints_error_and_joint_locks(monkeypatch) -> None:
    FakeInverseKinematics.instances.clear()
    monkeypatch.setattr(drake_ik, "DRAKE_AVAILABLE", True)
    monkeypatch.setattr(drake_ik, "InverseKinematics", FakeInverseKinematics, raising=False)
    monkeypatch.setattr(drake_ik, "RotationMatrix", lambda: "identity-rotation", raising=False)
    monkeypatch.setattr(drake_ik, "Solve", lambda prog: FakeSolveResult(), raising=False)
    monkeypatch.setattr(drake_ik, "compute_pose_error", lambda actual, target: (0.01, 0.02))

    world = FakeDrakeWorld()
    result = DrakeOptimizationIK()._solve_single(
        world=world,  # type: ignore[arg-type]
        robot_id="robot-instance",
        target_transform=FakeRigidTransform(),
        seed=np.array([1.0, 2.0, 3.0]),
        joint_names=["j0", "j1", "j2"],
        position_tolerance=0.1,
        orientation_tolerance=0.2,
        lower_limits=np.array([-5.0, -5.0, -5.0]),
        upper_limits=np.array([5.0, 5.0, 5.0]),
        target_frame_name="selected_tip_link",
        locked_joint_positions={0: 1.5, 2: 3.5},
    )

    ik = FakeInverseKinematics.instances[0]
    assert result.is_success()
    assert world.plant.requested_bodies == [("selected_tip_link", "model-instance")]
    assert ik.position_constraints[0]["frameB"] == "frame:selected_tip_link"
    assert ik.orientation_constraints[0]["frameBbar"] == "frame:selected_tip_link"
    assert world.link_pose_calls == [("robot-instance", "selected_tip_link")]
    assert ik.program.locks == [(1.5, 1.5, "q1"), (3.5, 3.5, "q4")]
    assert ik.program.initial_guess is not None
    np.testing.assert_allclose(ik.program.initial_guess[1], [0.0, 1.0, 0.0, 2.0, 3.0, 0.0])
