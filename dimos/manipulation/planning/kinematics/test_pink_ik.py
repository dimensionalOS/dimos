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

"""Unit tests for the Pink IK planning backend."""

from __future__ import annotations

from contextlib import nullcontext
from pathlib import Path
from types import ModuleType, SimpleNamespace
from typing import Any, cast

import numpy as np
from pydantic import TypeAdapter
import pytest

from dimos.manipulation.planning.factory import create_kinematics
from dimos.manipulation.planning.groups.models import PlanningGroup
from dimos.manipulation.planning.kinematics import pink_ik as pink_ik_module
from dimos.manipulation.planning.kinematics.config import (
    ManipulationKinematicsConfig,
    PinkKinematicsConfig,
    RoboPlanKinematicsConfig,
    kinematics_config_from_name,
)
from dimos.manipulation.planning.kinematics.pink_ik import (
    PinkIK,
    PinkIKConfig,
    PinkIKDependencyError,
    _build_joint_mapping,
    _lock_uncontrolled_model_joints,
    _PinkRobotContext,
    _PinkRobotModelContext,
    _seed_for_robot_config,
    _seed_positions_for_mapping,
)
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.manipulation.planning.spec.enums import IKStatus
from dimos.manipulation.planning.spec.models import IKResult
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.JointState import JointState


def test_roboplan_kinematics_config_defaults_and_parsing() -> None:
    config = RoboPlanKinematicsConfig()

    assert config.backend == "roboplan"
    assert config.max_iterations == 100
    assert config.dt == pytest.approx(0.05)
    assert config.position_cost == pytest.approx(1.0)
    assert config.orientation_cost == pytest.approx(1.0)
    assert config.task_gain == pytest.approx(0.5)
    assert config.lm_damping == pytest.approx(1e-6)
    assert config.regularization == pytest.approx(1e-8)
    assert config.velocity_limit is None
    assert config.collision_check is True

    parsed = TypeAdapter(ManipulationKinematicsConfig).validate_python(
        {
            "backend": "roboplan",
            "max_iterations": 5,
            "velocity_limit": 0.25,
            "collision_check": False,
        }
    )

    assert isinstance(parsed, RoboPlanKinematicsConfig)
    assert parsed.max_iterations == 5
    assert parsed.velocity_limit == pytest.approx(0.25)
    assert parsed.collision_check is False
    assert isinstance(kinematics_config_from_name("roboplan"), RoboPlanKinematicsConfig)


class _FakeJoint:
    def __init__(self, idx_q: int) -> None:
        self.idx_q = idx_q
        self.nq = 1


class _FakeFrame:
    def __init__(self, name: str) -> None:
        self.name = name


class _FakePlacement:
    def __init__(self, translation: np.ndarray) -> None:
        self.rotation = np.eye(3)
        self.translation = translation


class _FakeData:
    def __init__(self) -> None:
        self.q = np.zeros(3)
        self.oMf = [_FakePlacement(np.zeros(3)), _FakePlacement(np.zeros(3))]


class _FakeModel:
    nq = 3

    def __init__(self) -> None:
        self.names = ["universe", "joint_b", "joint_a", "joint_c"]
        self.joints = [SimpleNamespace(idx_q=-1, nq=0), _FakeJoint(0), _FakeJoint(1), _FakeJoint(2)]
        self.frames = [_FakeFrame("tool"), _FakeFrame("wrist_tool")]
        self._joint_ids = {"joint_b": 1, "joint_a": 2, "joint_c": 3}
        self._frame_ids = {"tool": 0, "wrist_tool": 1}

    def createData(self) -> _FakeData:
        return _FakeData()

    def existJointName(self, name: str) -> bool:
        return name in self._joint_ids

    def getJointId(self, name: str) -> int:
        return self._joint_ids.get(name, len(self.joints))

    def existFrame(self, name: str) -> bool:
        return name in self._frame_ids

    def getFrameId(self, name: str) -> int:
        return self._frame_ids.get(name, len(self.frames))


class _FakeSE3:
    def __init__(self, rotation: np.ndarray, translation: np.ndarray) -> None:
        self.rotation = rotation
        self.translation = translation


class _FakeConfiguration:
    def __init__(self, model: _FakeModel, data: _FakeData, q: np.ndarray) -> None:
        self.model = model
        self.data = data
        self.q = q.copy()

    def integrate_inplace(self, velocity: np.ndarray, dt: float) -> None:
        self.q = self.q + velocity * dt


class _FakeFrameTask:
    def __init__(self, frame: str, **_: object) -> None:
        self.frame = frame
        self.target: _FakeSE3 | None = None

    def set_target(self, target: _FakeSE3) -> None:
        self.target = target


class _FakePostureTask:
    def __init__(self, cost: float) -> None:
        self.cost = cost

    def set_target_from_configuration(self, configuration: _FakeConfiguration) -> None:
        self.target = configuration.q.copy()


def _fake_modules(converge: bool = True) -> tuple[ModuleType, ModuleType]:
    pinocchio = ModuleType("pinocchio")
    pinocchio.SE3 = _FakeSE3  # type: ignore[attr-defined]
    pinocchio.neutral = lambda model: np.zeros(model.nq)  # type: ignore[attr-defined]

    def forward_kinematics(model: _FakeModel, data: _FakeData, q: np.ndarray) -> None:
        data.q = q.copy()

    def update_frame_placements(model: _FakeModel, data: _FakeData) -> None:
        data.oMf[0] = _FakePlacement(data.q.copy())
        data.oMf[1] = _FakePlacement(data.q.copy())

    pinocchio.forwardKinematics = forward_kinematics  # type: ignore[attr-defined]
    pinocchio.updateFramePlacements = update_frame_placements  # type: ignore[attr-defined]

    pink = ModuleType("pink")
    pink.Configuration = _FakeConfiguration  # type: ignore[attr-defined]
    pink.tasks = SimpleNamespace(FrameTask=_FakeFrameTask, PostureTask=_FakePostureTask)

    def solve_ik(
        configuration: _FakeConfiguration,
        tasks: list[object],
        dt: float,
        **_: object,
    ) -> np.ndarray:
        if not converge:
            return np.zeros_like(configuration.q)
        frame_task = tasks[0]
        target = frame_task.target.translation  # type: ignore[attr-defined,union-attr]
        return (target - configuration.q) / dt

    pink.solve_ik = solve_ik  # type: ignore[attr-defined]

    return pink, pinocchio


def _install_fake_modules(converge: bool = True) -> None:
    pink, pinocchio = _fake_modules(converge=converge)
    pink_ik_module.pink = pink
    pink_ik_module.pinocchio = pinocchio
    pink_ik_module.qpsolvers = SimpleNamespace(available_solvers=["proxqp"])
    pink_ik_module._PINK_IMPORT_ERROR = None


def _robot_config() -> RobotModelConfig:
    return RobotModelConfig(
        name="arm",
        model_path=Path("/tmp/fake.urdf"),
        base_pose=PoseStamped(position=Vector3(), orientation=Quaternion(0.0, 0.0, 0.0, 1.0)),
        joint_names=["joint_a", "joint_b", "joint_c"],
        end_effector_link="tool",
        base_link="base",
    )


def _pose_stamped(x: float, y: float, z: float, yaw: float = 0.0) -> PoseStamped:
    half_yaw = yaw / 2.0
    return PoseStamped(
        frame_id="world",
        position=Vector3(x, y, z),
        orientation=Quaternion(0.0, 0.0, float(np.sin(half_yaw)), float(np.cos(half_yaw))),
    )


class _TestPinkIK(PinkIK):
    def __init__(self, converge: bool = True) -> None:
        self.config = PinkIKConfig(max_iterations=3)
        _install_fake_modules(converge=converge)
        self._robot_model_contexts = {}


def _pink_ik(converge: bool = True) -> PinkIK:
    return _TestPinkIK(converge=converge)


def _context() -> _PinkRobotContext:
    model = _FakeModel()
    mapping = _build_joint_mapping(model, _robot_config())
    return _PinkRobotContext(
        model=model,
        data=model.createData(),
        frame_id=0,
        frame_name="tool",
        mapping=mapping,
    )


def _patch_robot_contexts(
    monkeypatch: pytest.MonkeyPatch,
    ik: PinkIK,
    contexts_by_frame: dict[str, _PinkRobotContext],
) -> None:
    def fake_get_robot_context(
        world: object,
        robot_id: str,
        frame_name: str | None = None,
        config: RobotModelConfig | None = None,
    ) -> _PinkRobotContext:
        del world, robot_id, config
        target_frame = frame_name or "tool"
        return contexts_by_frame[target_frame]

    monkeypatch.setattr(ik, "_get_robot_context", fake_get_robot_context)


class _FakeWorld:
    is_finalized = True

    def __init__(self, collision_free: bool = True) -> None:
        self.config = _robot_config()
        self.collision_free = collision_free
        self.groups = {
            "arm/wrist": PlanningGroup(
                id="arm/wrist",
                robot_name="arm",
                group_name="wrist",
                joint_names=("arm/joint_a", "arm/joint_b"),
                local_joint_names=("joint_a", "joint_b"),
                base_link="base",
                tip_link="wrist_tool",
            ),
            "arm/gripper": PlanningGroup(
                id="arm/gripper",
                robot_name="arm",
                group_name="gripper",
                joint_names=("arm/joint_c",),
                local_joint_names=("joint_c",),
                base_link="base",
                tip_link=None,
            ),
        }

    def get_robot_ids(self) -> list[str]:
        return ["robot"]

    def get_robot_config(self, robot_id: str) -> RobotModelConfig:
        return self.config

    def scratch_context(self) -> nullcontext[None]:
        return nullcontext(None)

    def get_joint_state(self, ctx: object, robot_id: str) -> JointState:
        return JointState(
            name=["joint_b", "joint_c", "joint_a"],
            position=[0.0, 0.0, 0.0],
        )

    def get_joint_limits(self, robot_id: str) -> tuple[np.ndarray, np.ndarray]:
        return np.array([-1.0, -1.0, -1.0]), np.array([1.0, 1.0, 1.0])

    def check_config_collision_free(self, robot_id: str, joint_state: JointState) -> bool:
        return self.collision_free


class _CountingWorld(_FakeWorld):
    def __init__(self, collision_free: bool = True) -> None:
        super().__init__(collision_free=collision_free)
        self.scratch_calls = 0
        self.current_state_calls = 0
        self.joint_limit_calls = 0

    def scratch_context(self) -> nullcontext[None]:
        self.scratch_calls += 1
        return nullcontext(None)

    def get_joint_state(self, ctx: object, robot_id: str) -> JointState:
        self.current_state_calls += 1
        return super().get_joint_state(ctx, robot_id)

    def get_joint_limits(self, robot_id: str) -> tuple[np.ndarray, np.ndarray]:
        self.joint_limit_calls += 1
        return super().get_joint_limits(robot_id)


class _FakeMultiRobotWorld:
    is_finalized = True

    def __init__(self) -> None:
        self.configs = {
            "left_robot": RobotModelConfig(
                name="left",
                model_path=Path("/tmp/left.urdf"),
                base_pose=PoseStamped(
                    position=Vector3(), orientation=Quaternion(0.0, 0.0, 0.0, 1.0)
                ),
                joint_names=["joint_a", "joint_b"],
                end_effector_link="tool",
                base_link="base",
            ),
            "right_robot": RobotModelConfig(
                name="right",
                model_path=Path("/tmp/right.urdf"),
                base_pose=PoseStamped(
                    position=Vector3(), orientation=Quaternion(0.0, 0.0, 0.0, 1.0)
                ),
                joint_names=["joint_c"],
                end_effector_link="tool",
                base_link="base",
            ),
        }

    def get_robot_ids(self) -> list[str]:
        return list(self.configs)

    def get_robot_config(self, robot_id: str) -> RobotModelConfig:
        return self.configs[robot_id]

    def scratch_context(self) -> nullcontext[None]:
        return nullcontext(None)

    def get_joint_state(self, ctx: object, robot_id: str) -> JointState:
        config = self.get_robot_config(robot_id)
        return JointState(name=list(config.joint_names), position=[0.0] * len(config.joint_names))

    def get_joint_limits(self, robot_id: str) -> tuple[np.ndarray, np.ndarray]:
        config = self.get_robot_config(robot_id)
        count = len(config.joint_names)
        return np.full(count, -1.0), np.full(count, 1.0)


class _CountingMultiRobotWorld(_FakeMultiRobotWorld):
    def __init__(self) -> None:
        super().__init__()
        self.joint_limit_calls: list[str] = []

    def get_joint_limits(self, robot_id: str) -> tuple[np.ndarray, np.ndarray]:
        self.joint_limit_calls.append(robot_id)
        return super().get_joint_limits(robot_id)


def test_create_kinematics_pink_missing_dependency_is_actionable(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    def missing_dependencies(_solver: str) -> object:
        raise PinkIKDependencyError(
            "Pink IK backend requires Pink. Install manipulation dependencies with: "
            "uv sync --extra manipulation. PyPI package: pin-pink; import name: pink."
        )

    monkeypatch.setattr(pink_ik_module, "_check_optional_dependencies", missing_dependencies)

    with pytest.raises(PinkIKDependencyError) as exc_info:
        create_kinematics("pink")
    assert "pin-pink" in str(exc_info.value)
    assert "--extra manipulation" in str(exc_info.value)


def test_create_kinematics_pink_unavailable_solver_mentions_manipulation_extra(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    def unavailable_solver(_solver: str) -> object:
        raise PinkIKDependencyError(
            "Pink IK solver 'proxqp' is not available from qpsolvers. "
            "Install manipulation dependencies with uv sync --extra manipulation."
        )

    monkeypatch.setattr(pink_ik_module, "_check_optional_dependencies", unavailable_solver)

    with pytest.raises(PinkIKDependencyError, match="--extra manipulation"):
        create_kinematics("pink")


def test_create_kinematics_pink_returns_backend(monkeypatch: pytest.MonkeyPatch) -> None:
    _install_fake_modules()

    assert isinstance(create_kinematics("pink"), PinkIK)


def test_create_kinematics_pink_config_passes_tuning(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    _install_fake_modules()

    ik = create_kinematics(config=PinkKinematicsConfig(max_iterations=7, dt=0.02, posture_cost=0.0))

    assert isinstance(ik, PinkIK)
    assert ik.config.max_iterations == 7
    assert ik.config.dt == 0.02
    assert ik.config.posture_cost == 0.0


def test_pink_ik_config_overrides_are_applied(monkeypatch: pytest.MonkeyPatch) -> None:
    _install_fake_modules()

    ik = PinkIK(PinkIKConfig(solver="proxqp", dt=0.1), max_iterations=7, posture_cost=0.0)

    assert ik.config == PinkIKConfig(
        solver="proxqp",
        dt=0.1,
        max_iterations=7,
        posture_cost=0.0,
    )


def test_joint_order_mapping_uses_names_not_positions() -> None:
    mapping = _build_joint_mapping(_FakeModel(), _robot_config())
    seed = JointState(name=["joint_b", "joint_c", "joint_a"], position=[20.0, 30.0, 10.0])

    assert mapping.idx_q == [1, 0, 2]
    assert _seed_positions_for_mapping(seed, mapping).tolist() == [10.0, 20.0, 30.0]


def test_seed_for_robot_config_uses_complete_global_seed_without_world() -> None:
    seed = JointState(name=["arm/joint_a", "arm/joint_b", "arm/joint_c"], position=[1.0, 2.0, 3.0])

    result = _seed_for_robot_config(_robot_config(), seed)

    assert result.name == ["joint_a", "joint_b", "joint_c"]
    assert result.position == [1.0, 2.0, 3.0]


def test_solve_pose_targets_complete_seed_does_not_read_world_state(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    ik = _pink_ik(converge=True)
    context = _context()
    context.frame_name = "wrist_tool"
    context.frame_id = 1
    _patch_robot_contexts(monkeypatch, ik, {"wrist_tool": context})
    world = _CountingWorld(collision_free=True)

    def fake_solve_single(**_: object) -> IKResult:
        return IKResult(
            status=IKStatus.SUCCESS,
            joint_state=JointState(
                name=["joint_a", "joint_b", "joint_c"], position=[0.1, 0.2, 0.3]
            ),
        )

    monkeypatch.setattr(ik, "_solve_single", fake_solve_single)

    result = ik.solve_pose_targets(
        world=cast("Any", world),
        pose_targets={world.groups["arm/wrist"]: _pose_stamped(0.1, 0.0, 0.0)},
        seed=JointState(
            name=["arm/joint_a", "arm/joint_b", "arm/joint_c"], position=[1.0, 2.0, 3.0]
        ),
        max_attempts=1,
    )

    assert result.status == IKStatus.SUCCESS
    assert world.scratch_calls == 0
    assert world.current_state_calls == 0


def test_solve_pose_targets_incomplete_seed_reads_world_state_once(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    ik = _pink_ik(converge=True)
    context = _context()
    context.frame_name = "wrist_tool"
    context.frame_id = 1
    _patch_robot_contexts(monkeypatch, ik, {"wrist_tool": context})
    world = _CountingWorld(collision_free=True)

    def fake_solve_single(**_: object) -> IKResult:
        return IKResult(
            status=IKStatus.SUCCESS,
            joint_state=JointState(
                name=["joint_a", "joint_b", "joint_c"], position=[0.1, 0.2, 0.3]
            ),
        )

    monkeypatch.setattr(ik, "_solve_single", fake_solve_single)

    result = ik.solve_pose_targets(
        world=cast("Any", world),
        pose_targets={world.groups["arm/wrist"]: _pose_stamped(0.1, 0.0, 0.0)},
        seed=JointState(name=["arm/joint_a"], position=[1.0]),
        max_attempts=1,
    )

    assert result.status == IKStatus.SUCCESS
    assert world.scratch_calls == 1
    assert world.current_state_calls == 1


def test_mapping_failure_for_missing_joint() -> None:
    config = _robot_config()
    config.joint_names = ["joint_a", "missing", "joint_c"]

    with pytest.raises(ValueError, match="missing"):
        _build_joint_mapping(_FakeModel(), config)


def test_uncontrolled_urdf_joints_are_locked_out_of_pink_model() -> None:
    pinocchio = ModuleType("pinocchio")
    model = _FakeModel()
    model.names.append("gripper_joint")
    model.joints.append(_FakeJoint(3))
    reduced_model = _FakeModel()
    seen_locked_joint_ids: list[list[int]] = []

    def build_reduced_model(
        input_model: _FakeModel, locked_joint_ids: list[int], reference: np.ndarray
    ) -> _FakeModel:
        assert input_model is model
        np.testing.assert_allclose(reference, np.zeros(model.nq))
        seen_locked_joint_ids.append(list(locked_joint_ids))
        return reduced_model

    pinocchio.neutral = lambda input_model: np.zeros(input_model.nq)  # type: ignore[attr-defined]
    pinocchio.buildReducedModel = build_reduced_model  # type: ignore[attr-defined]

    result = _lock_uncontrolled_model_joints(pinocchio, model, _robot_config())

    assert result is reduced_model
    assert seen_locked_joint_ids == [[4]]


def test_solve_single_returns_successful_ik_result() -> None:
    ik = _pink_ik(converge=True)
    target = np.eye(4)
    target[:3, 3] = [0.1, 0.2, 0.3]

    result = ik._solve_single(
        robot_context=_context(),
        target_model=target,
        seed_q=np.zeros(3),
        lower_limits=np.array([-1.0, -1.0, -1.0]),
        upper_limits=np.array([1.0, 1.0, 1.0]),
        position_tolerance=0.001,
        orientation_tolerance=0.01,
    )

    assert result.status == IKStatus.SUCCESS
    assert result.joint_state is not None
    assert result.joint_state.name == ["joint_a", "joint_b", "joint_c"]
    assert result.joint_state.position == pytest.approx([0.2, 0.1, 0.3])


def test_solve_single_reports_non_convergence() -> None:
    ik = _pink_ik(converge=False)
    target = np.eye(4)
    target[:3, 3] = [0.1, 0.0, 0.0]

    result = ik._solve_single(
        robot_context=_context(),
        target_model=target,
        seed_q=np.zeros(3),
        lower_limits=np.array([-1.0, -1.0, -1.0]),
        upper_limits=np.array([1.0, 1.0, 1.0]),
        position_tolerance=0.001,
        orientation_tolerance=0.01,
    )

    assert result.status == IKStatus.NO_SOLUTION
    assert "did not converge" in result.message


def test_solve_does_not_filter_collision_candidates(monkeypatch: pytest.MonkeyPatch) -> None:
    ik = _pink_ik(converge=True)
    context = _context()
    _patch_robot_contexts(monkeypatch, ik, {"tool": context})

    result = ik.solve(
        world=cast("Any", _FakeWorld(collision_free=False)),
        robot_id="robot",
        target_pose=PoseStamped(
            position=Vector3(0.1, 0.0, 0.0),
            orientation=Quaternion(0.0, 0.0, 0.0, 1.0),
        ),
        max_attempts=1,
    )

    assert result.status == IKStatus.SUCCESS
    assert result.joint_state is not None


def test_solve_pose_targets_returns_selected_resolved_joints_and_group_tip(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    ik = _pink_ik(converge=True)
    context = _context()
    context.frame_name = "wrist_tool"
    context.frame_id = 1
    _patch_robot_contexts(monkeypatch, ik, {"wrist_tool": context})
    seen_frame_names: list[str] = []

    def fake_solve_single(**kwargs: object) -> IKResult:
        robot_context = cast("_PinkRobotContext", kwargs["robot_context"])
        seen_frame_names.append(robot_context.frame_name)
        return IKResult(
            status=IKStatus.SUCCESS,
            joint_state=JointState(
                {"name": ["joint_a", "joint_b", "joint_c"], "position": [0.1, 0.2, 0.3]}
            ),
            position_error=0.0,
            orientation_error=0.0,
            iterations=1,
        )

    monkeypatch.setattr(ik, "_solve_single", fake_solve_single)

    world = _FakeWorld(collision_free=True)
    result = ik.solve_pose_targets(
        world=cast("Any", world),
        pose_targets={
            world.groups["arm/wrist"]: PoseStamped(
                position=Vector3(0.1, 0.0, 0.0),
                orientation=Quaternion(0.0, 0.0, 0.0, 1.0),
            )
        },
        auxiliary_groups=[world.groups["arm/gripper"]],
        seed=JointState(
            {"name": ["arm/joint_a", "arm/joint_b", "arm/joint_c"], "position": [0.0, 0.0, 0.0]}
        ),
        max_attempts=1,
    )

    assert seen_frame_names == ["wrist_tool"]
    assert result.status == IKStatus.SUCCESS
    assert result.joint_state is not None
    assert result.joint_state.name == ["arm/joint_a", "arm/joint_b", "arm/joint_c"]
    assert result.joint_state.position == [0.1, 0.2, 0.3]


def test_solve_pose_targets_same_robot_uses_one_multi_frame_solve(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    ik = _pink_ik(converge=True)
    wrist_context = _context()
    wrist_context.frame_name = "wrist_tool"
    wrist_context.frame_id = 1
    tool_context = _context()
    _patch_robot_contexts(monkeypatch, ik, {"wrist_tool": wrist_context, "tool": tool_context})
    world = _FakeWorld(collision_free=True)
    tool_group = PlanningGroup(
        id="arm/tool",
        robot_name="arm",
        group_name="tool",
        joint_names=("arm/joint_c",),
        local_joint_names=("joint_c",),
        base_link="base",
        tip_link="tool",
    )
    seen_frames: list[list[str]] = []

    def fake_solve_multi_frame(**kwargs: object) -> IKResult:
        contexts = cast("list[_PinkRobotContext]", kwargs["robot_contexts"])
        seen_frames.append([context.frame_name for context in contexts])
        return IKResult(
            status=IKStatus.SUCCESS,
            joint_state=JointState(
                {"name": ["joint_a", "joint_b", "joint_c"], "position": [0.1, 0.2, 0.3]}
            ),
            position_error=0.0,
            orientation_error=0.0,
            iterations=2,
        )

    monkeypatch.setattr(ik, "_solve_multi_frame", fake_solve_multi_frame)

    result = ik.solve_pose_targets(
        world=cast("Any", world),
        pose_targets={
            world.groups["arm/wrist"]: PoseStamped(
                position=Vector3(0.1, 0.0, 0.0),
                orientation=Quaternion(0.0, 0.0, 0.0, 1.0),
            ),
            tool_group: PoseStamped(
                position=Vector3(0.2, 0.0, 0.0),
                orientation=Quaternion(0.0, 0.0, 0.0, 1.0),
            ),
        },
        seed=JointState(
            {"name": ["arm/joint_a", "arm/joint_b", "arm/joint_c"], "position": [0.0, 0.0, 0.0]}
        ),
        max_attempts=1,
    )

    assert seen_frames == [["wrist_tool", "tool"]]
    assert result.status == IKStatus.SUCCESS
    assert result.joint_state is not None
    assert result.joint_state.name == ["arm/joint_a", "arm/joint_b", "arm/joint_c"]
    assert result.joint_state.position == [0.1, 0.2, 0.3]


def test_solve_pose_targets_same_robot_builds_one_model_context(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    ik = _pink_ik(converge=True)
    world = _FakeWorld(collision_free=True)
    tool_group = PlanningGroup(
        id="arm/tool",
        robot_name="arm",
        group_name="tool",
        joint_names=("arm/joint_c",),
        local_joint_names=("joint_c",),
        base_link="base",
        tip_link="tool",
    )
    model = _FakeModel()
    build_calls = 0

    def fake_build_model_context(config: RobotModelConfig) -> _PinkRobotModelContext:
        nonlocal build_calls
        build_calls += 1
        mapping = _build_joint_mapping(model, config)
        return _PinkRobotModelContext(
            model=model,
            mapping=mapping,
            neutral_q=np.zeros(model.nq),
            frame_ids={},
        )

    def fake_solve_multi_frame(**_: object) -> IKResult:
        return IKResult(
            status=IKStatus.SUCCESS,
            joint_state=JointState(
                name=["joint_a", "joint_b", "joint_c"], position=[0.1, 0.2, 0.3]
            ),
        )

    monkeypatch.setattr(ik, "_build_robot_model_context", fake_build_model_context)
    monkeypatch.setattr(ik, "_solve_multi_frame", fake_solve_multi_frame)

    result = ik.solve_pose_targets(
        world=cast("Any", world),
        pose_targets={
            world.groups["arm/wrist"]: _pose_stamped(0.1, 0.0, 0.0),
            tool_group: _pose_stamped(0.2, 0.0, 0.0),
        },
        seed=JointState(
            name=["arm/joint_a", "arm/joint_b", "arm/joint_c"], position=[0.0, 0.0, 0.0]
        ),
        max_attempts=1,
    )

    assert result.status == IKStatus.SUCCESS
    assert build_calls == 1


def test_solve_multi_frame_updates_fk_once_per_iteration(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    ik = _pink_ik(converge=True)
    ik.config = PinkIKConfig(max_iterations=1)
    pinocchio = cast("Any", pink_ik_module.pinocchio)
    original_forward_kinematics = pinocchio.forwardKinematics
    forward_calls = 0

    def counting_forward_kinematics(model: _FakeModel, data: _FakeData, q: np.ndarray) -> None:
        nonlocal forward_calls
        forward_calls += 1
        original_forward_kinematics(model, data, q)

    monkeypatch.setattr(pinocchio, "forwardKinematics", counting_forward_kinematics)
    wrist_context = _context()
    wrist_context.frame_name = "wrist_tool"
    wrist_context.frame_id = 1
    tool_context = _context()
    target = np.eye(4)
    target[:3, 3] = [0.1, 0.0, 0.0]

    result = ik._solve_multi_frame(
        robot_contexts=[wrist_context, tool_context],
        target_models=[target, target],
        seed_q=np.zeros(3),
        lower_limits=np.array([-1.0, -1.0, -1.0]),
        upper_limits=np.array([1.0, 1.0, 1.0]),
        position_tolerance=0.001,
        orientation_tolerance=0.01,
    )

    assert result.status == IKStatus.NO_SOLUTION
    assert forward_calls == 1


def test_target_in_model_frame_converts_world_pose_through_robot_base() -> None:
    ik = _pink_ik(converge=True)
    config = _robot_config()
    config.base_pose = _pose_stamped(1.0, 2.0, 0.0, yaw=np.pi / 2.0)
    target_world = _pose_stamped(0.8, 2.1, 0.3, yaw=np.pi / 2.0)

    target_model = ik._target_in_model_frame(config, target_world)

    np.testing.assert_allclose(target_model[:3, 3], [0.1, 0.2, 0.3], atol=1e-12)
    np.testing.assert_allclose(target_model[:3, :3], np.eye(3), atol=1e-12)


def test_solve_pose_targets_passes_world_target_to_solver_in_model_frame(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    ik = _pink_ik(converge=True)
    context = _context()
    context.frame_name = "wrist_tool"
    context.frame_id = 1
    _patch_robot_contexts(monkeypatch, ik, {"wrist_tool": context})
    world = _FakeWorld(collision_free=True)
    world.config.base_pose = _pose_stamped(1.0, 2.0, 0.0, yaw=np.pi / 2.0)
    seen_target_models: list[np.ndarray] = []

    def fake_solve_single(**kwargs: object) -> IKResult:
        target_model = cast("np.ndarray", kwargs["target_model"])
        seen_target_models.append(target_model)
        return IKResult(
            status=IKStatus.SUCCESS,
            joint_state=JointState(
                name=["joint_a", "joint_b", "joint_c"], position=[0.1, 0.2, 0.3]
            ),
            position_error=0.0,
            orientation_error=0.0,
            iterations=1,
        )

    monkeypatch.setattr(ik, "_solve_single", fake_solve_single)

    result = ik.solve_pose_targets(
        world=cast("Any", world),
        pose_targets={world.groups["arm/wrist"]: _pose_stamped(0.8, 2.1, 0.3, yaw=np.pi / 2.0)},
        seed=JointState(
            {"name": ["arm/joint_a", "arm/joint_b", "arm/joint_c"], "position": [0.0, 0.0, 0.0]}
        ),
        max_attempts=1,
    )

    assert result.status == IKStatus.SUCCESS
    assert len(seen_target_models) == 1
    np.testing.assert_allclose(seen_target_models[0][:3, 3], [0.1, 0.2, 0.3], atol=1e-12)
    np.testing.assert_allclose(seen_target_models[0][:3, :3], np.eye(3), atol=1e-12)


def test_solve_pose_targets_cross_robot_combines_global_joint_names(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    ik = _pink_ik(converge=True)
    world = _FakeMultiRobotWorld()
    left_group = PlanningGroup(
        id="left/arm",
        robot_name="left",
        group_name="arm",
        joint_names=("left/joint_a",),
        local_joint_names=("joint_a",),
        base_link="base",
        tip_link="tool",
    )
    right_group = PlanningGroup(
        id="right/arm",
        robot_name="right",
        group_name="arm",
        joint_names=("right/joint_c",),
        local_joint_names=("joint_c",),
        base_link="base",
        tip_link="tool",
    )
    seen_robot_ids: list[str] = []

    def fake_solve_pose_targets_for_robot(**kwargs: object) -> IKResult:
        robot_id = str(kwargs["robot_id"])
        seen_robot_ids.append(robot_id)
        if robot_id == "left_robot":
            return IKResult(
                status=IKStatus.SUCCESS,
                joint_state=JointState(name=["joint_a", "joint_b"], position=[1.0, 9.0]),
            )
        return IKResult(
            status=IKStatus.SUCCESS,
            joint_state=JointState(name=["joint_c"], position=[2.0]),
        )

    monkeypatch.setattr(ik, "_solve_pose_targets_for_robot", fake_solve_pose_targets_for_robot)

    result = ik.solve_pose_targets(
        world=cast("Any", world),
        pose_targets={
            left_group: PoseStamped(
                position=Vector3(0.1, 0.0, 0.0),
                orientation=Quaternion(0.0, 0.0, 0.0, 1.0),
            ),
            right_group: PoseStamped(
                position=Vector3(0.2, 0.0, 0.0),
                orientation=Quaternion(0.0, 0.0, 0.0, 1.0),
            ),
        },
        seed=JointState(
            name=["left/joint_a", "left/joint_b", "right/joint_c"],
            position=[0.0, 0.0, 0.0],
        ),
        max_attempts=1,
    )

    assert seen_robot_ids == ["left_robot", "right_robot"]
    assert result.status == IKStatus.SUCCESS
    assert result.joint_state is not None
    assert result.joint_state.name == ["left/joint_a", "right/joint_c"]
    assert result.joint_state.position == [1.0, 2.0]


def test_solve_pose_targets_returns_first_robot_failure_before_touching_later_limits(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    ik = _pink_ik(converge=True)
    world = _CountingMultiRobotWorld()
    left_group = PlanningGroup(
        id="left/arm",
        robot_name="left",
        group_name="arm",
        joint_names=("left/joint_a",),
        local_joint_names=("joint_a",),
        base_link="base",
        tip_link="tool",
    )
    right_group = PlanningGroup(
        id="right/arm",
        robot_name="right",
        group_name="arm",
        joint_names=("right/joint_c",),
        local_joint_names=("joint_c",),
        base_link="base",
        tip_link="tool",
    )

    def fail_first_robot(**_: object) -> IKResult:
        return IKResult(status=IKStatus.NO_SOLUTION, joint_state=None, message="left failed")

    monkeypatch.setattr(ik, "_solve_pose_targets_for_robot", fail_first_robot)

    result = ik.solve_pose_targets(
        world=cast("Any", world),
        pose_targets={
            left_group: _pose_stamped(0.1, 0.0, 0.0),
            right_group: _pose_stamped(0.2, 0.0, 0.0),
        },
        seed=JointState(
            name=["left/joint_a", "left/joint_b", "right/joint_c"],
            position=[0.0, 0.0, 0.0],
        ),
        max_attempts=1,
    )

    assert result.status == IKStatus.NO_SOLUTION
    assert result.message == "left failed"
    assert world.joint_limit_calls == ["left_robot"]


def test_solve_pose_targets_auxiliary_robot_does_not_read_joint_limits(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    ik = _pink_ik(converge=True)
    world = _CountingMultiRobotWorld()
    left_group = PlanningGroup(
        id="left/arm",
        robot_name="left",
        group_name="arm",
        joint_names=("left/joint_a",),
        local_joint_names=("joint_a",),
        base_link="base",
        tip_link="tool",
    )
    right_auxiliary = PlanningGroup(
        id="right/gripper",
        robot_name="right",
        group_name="gripper",
        joint_names=("right/joint_c",),
        local_joint_names=("joint_c",),
        base_link="base",
        tip_link=None,
    )

    def solve_left_robot(**_: object) -> IKResult:
        return IKResult(
            status=IKStatus.SUCCESS,
            joint_state=JointState(name=["joint_a", "joint_b"], position=[1.0, 9.0]),
        )

    monkeypatch.setattr(ik, "_solve_pose_targets_for_robot", solve_left_robot)

    result = ik.solve_pose_targets(
        world=cast("Any", world),
        pose_targets={left_group: _pose_stamped(0.1, 0.0, 0.0)},
        auxiliary_groups=[right_auxiliary],
        seed=JointState(
            name=["left/joint_a", "left/joint_b", "right/joint_c"],
            position=[0.0, 0.0, 2.0],
        ),
        max_attempts=1,
    )

    assert result.status == IKStatus.SUCCESS
    assert result.joint_state is not None
    assert result.joint_state.name == ["left/joint_a", "right/joint_c"]
    assert result.joint_state.position == [1.0, 2.0]
    assert world.joint_limit_calls == ["left_robot"]


def test_solve_retries_after_joint_limit_failure(monkeypatch: pytest.MonkeyPatch) -> None:
    ik = _pink_ik(converge=True)
    context = _context()
    _patch_robot_contexts(monkeypatch, ik, {"tool": context})
    calls = 0

    def fake_solve_single(**_: object) -> IKResult:
        nonlocal calls
        calls += 1
        if calls == 1:
            return IKResult(
                status=IKStatus.JOINT_LIMITS,
                joint_state=None,
                message="first attempt hit limits",
            )
        return IKResult(
            status=IKStatus.SUCCESS,
            joint_state=JointState(
                name=["joint_a", "joint_b", "joint_c"],
                position=[0.1, 0.2, 0.3],
            ),
            position_error=0.0,
            orientation_error=0.0,
            iterations=1,
        )

    monkeypatch.setattr(ik, "_solve_single", fake_solve_single)

    result = ik.solve(
        world=cast("Any", _FakeWorld(collision_free=True)),
        robot_id="robot",
        target_pose=PoseStamped(
            position=Vector3(0.1, 0.0, 0.0),
            orientation=Quaternion(0.0, 0.0, 0.0, 1.0),
        ),
        max_attempts=2,
    )

    assert calls == 2
    assert result.status == IKStatus.SUCCESS
