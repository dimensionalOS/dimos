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
from pink.exceptions import NoSolutionFound
import pytest
from pytest_mock import MockerFixture

from dimos.manipulation.planning.factory import create_kinematics
from dimos.manipulation.planning.groups.models import PlanningGroup, PlanningGroupDefinition
from dimos.manipulation.planning.kinematics.config import PinkKinematicsConfig
import dimos.manipulation.planning.kinematics.pink_ik as pink_ik
from dimos.manipulation.planning.kinematics.pink_ik import (
    PinkIK,
    PinkIKConfig,
    PinkIKDependencyError,
    _build_joint_mapping,
    _PinkModules,
    _PinkRobotContext,
    _seed_positions_for_mapping,
)
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.manipulation.planning.spec.enums import IKStatus
from dimos.manipulation.planning.spec.models import IKResult
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.assets.model import LoadedRobotModel, RobotModel


class _FakeJoint:
    def __init__(self, idx_q: int) -> None:
        self.idx_q = idx_q
        self.nq = 1


class _FakeFrame:
    def __init__(self, name: str, parent_joint: int = 0) -> None:
        self.name = name
        self.parentJoint = parent_joint


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
        self.frames = [_FakeFrame("base", 0), _FakeFrame("tool", 3)]
        self._joint_ids = {"joint_b": 1, "joint_a": 2, "joint_c": 3}
        self._frame_ids = {"base": 0, "tool": 1}

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


def _fake_modules(converge: bool = True) -> _PinkModules:
    pinocchio = ModuleType("pinocchio")
    pinocchio.SE3 = _FakeSE3  # type: ignore[attr-defined]
    pinocchio.neutral = lambda model: np.zeros(model.nq)  # type: ignore[attr-defined]

    def forward_kinematics(model: _FakeModel, data: _FakeData, q: np.ndarray) -> None:
        data.q = q.copy()

    def update_frame_placements(model: _FakeModel, data: _FakeData) -> None:
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

    return _PinkModules(pink=pink, pinocchio=pinocchio)


def _robot_config() -> RobotModelConfig:
    return RobotModelConfig(
        model=RobotModel.from_file(Path("/tmp/fake.urdf")),
        base_pose=PoseStamped(position=Vector3(), orientation=Quaternion(0.0, 0.0, 0.0, 1.0)),
        joint_names=["joint_a", "joint_b", "joint_c"],
        base_link="base",
        planning_groups=[
            PlanningGroupDefinition(
                name="manipulator",
                joint_names=("joint_a", "joint_b", "joint_c"),
                base_link="base",
                tip_link="tool",
            )
        ],
    )


def _pink_ik(mocker: MockerFixture, converge: bool = True) -> PinkIK:
    mocker.patch.object(
        pink_ik, "_load_optional_dependencies", return_value=_fake_modules(converge=converge)
    )
    return PinkIK(PinkIKConfig(max_iterations=3))


def _context() -> _PinkRobotContext:
    model = _FakeModel()
    mapping = _build_joint_mapping(model, _robot_config())
    return _PinkRobotContext(
        model=model,
        data=model.createData(),
        frame_id=1,
        frame_name="tool",
        mapping=mapping,
    )


class _FakeWorld:
    is_finalized = True

    def __init__(self, collision_free: bool = True) -> None:
        self.config = _robot_config()
        self.collision_free = collision_free
        self.joint_state_calls = 0
        self.groups = {
            "manipulator": PlanningGroup(
                id="manipulator",
                joint_names=("joint_a", "joint_b"),
                base_link="base",
                tip_link="tool",
            ),
            "no_tip": PlanningGroup(
                id="no_tip",
                joint_names=("joint_c",),
                base_link="base",
                tip_link=None,
            ),
            "wrist": PlanningGroup(
                id="wrist",
                joint_names=("joint_c",),
                base_link="base",
                tip_link="base",
            ),
        }

    def get_model_config(self) -> RobotModelConfig:
        return self.config

    def scratch_context(self) -> nullcontext[None]:
        return nullcontext(None)

    def get_joint_state(self, ctx: object) -> JointState:
        self.joint_state_calls += 1
        return JointState({"name": ["joint_b", "joint_c", "joint_a"], "position": [0.0, 0.0, 0.0]})

    def get_joint_limits(self) -> tuple[np.ndarray, np.ndarray]:
        return np.array([-1.0, -1.0, -1.0]), np.array([1.0, 1.0, 1.0])

    def check_config_collision_free(self, joint_state: JointState) -> bool:
        return self.collision_free

    def set_joint_state(self, ctx: object, joint_state: JointState) -> None:
        self.joint_state = joint_state

    def is_collision_free(self, ctx: object) -> bool:
        return self.collision_free


def test_create_kinematics_pink_missing_dependency_is_actionable(
    mocker: MockerFixture,
) -> None:
    def fake_import_module(name: str) -> ModuleType:
        if name == "pink":
            raise ImportError("missing pink")
        return ModuleType(name)

    mocker.patch.object(pink_ik.importlib, "import_module", side_effect=fake_import_module)

    with pytest.raises(PinkIKDependencyError) as exc_info:
        create_kinematics("pink")
    assert "pin-pink" in str(exc_info.value)
    assert "--extra manipulation" in str(exc_info.value)


def test_create_kinematics_pink_unavailable_solver_mentions_manipulation_extra(
    mocker: MockerFixture,
) -> None:
    def fake_import_module(name: str) -> ModuleType:
        module = ModuleType(name)
        if name == "qpsolvers":
            module.available_solvers = []  # type: ignore[attr-defined]
        return module

    mocker.patch.object(pink_ik.importlib, "import_module", side_effect=fake_import_module)

    with pytest.raises(PinkIKDependencyError, match="--extra manipulation"):
        create_kinematics("pink")


def test_create_kinematics_pink_returns_backend(mocker: MockerFixture) -> None:
    mocker.patch.object(pink_ik, "_load_optional_dependencies", return_value=_fake_modules())

    assert isinstance(create_kinematics("pink"), PinkIK)


def test_create_kinematics_pink_config_passes_tuning(
    mocker: MockerFixture,
) -> None:
    mocker.patch.object(pink_ik, "_load_optional_dependencies", return_value=_fake_modules())

    ik = create_kinematics(config=PinkKinematicsConfig(max_iterations=7, dt=0.02, posture_cost=0.0))

    assert isinstance(ik, PinkIK)
    assert ik.config.max_iterations == 7
    assert ik.config.dt == 0.02
    assert ik.config.posture_cost == 0.0


def test_pink_ik_config_overrides_are_applied(mocker: MockerFixture) -> None:
    mocker.patch.object(pink_ik, "_load_optional_dependencies", return_value=_fake_modules())

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


def test_mapping_failure_for_missing_joint() -> None:
    config = _robot_config()
    config.joint_names = ["joint_a", "missing", "joint_c"]

    with pytest.raises(ValueError, match="missing"):
        _build_joint_mapping(_FakeModel(), config)


def test_solve_single_returns_successful_ik_result(mocker: MockerFixture) -> None:
    ik = _pink_ik(mocker, converge=True)
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


def test_solve_single_reports_non_convergence(mocker: MockerFixture) -> None:
    ik = _pink_ik(mocker, converge=False)
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


def test_solve_rejects_collision_candidate(mocker: MockerFixture) -> None:
    ik = _pink_ik(mocker, converge=True)
    context = _context()
    ik._model_context = context

    result = ik.solve(
        world=cast("Any", _FakeWorld(collision_free=False)),
        target_pose=PoseStamped(
            position=Vector3(0.1, 0.0, 0.0),
            orientation=Quaternion(0.0, 0.0, 0.0, 1.0),
        ),
        check_collision=True,
        max_attempts=1,
    )

    assert result.status == IKStatus.COLLISION
    assert result.joint_state is None


def test_solve_retries_after_joint_limit_failure(mocker: MockerFixture) -> None:
    ik = _pink_ik(mocker, converge=True)
    context = _context()
    ik._model_context = context
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

    solve_single = mocker.patch.object(ik, "_solve_single", side_effect=fake_solve_single)

    result = ik.solve(
        world=cast("Any", _FakeWorld(collision_free=True)),
        target_pose=PoseStamped(
            position=Vector3(0.1, 0.0, 0.0),
            orientation=Quaternion(0.0, 0.0, 0.0, 1.0),
        ),
        check_collision=True,
        max_attempts=2,
    )

    assert solve_single.call_count == 2
    assert result.status == IKStatus.SUCCESS


def test_robot_context_is_built_once_and_reused_for_every_tip_frame(
    mocker: MockerFixture, tmp_path: Path
) -> None:
    modules = _fake_modules()
    modules.pinocchio.buildModelFromUrdf = lambda path: _FakeModel()  # type: ignore[attr-defined]
    mocker.patch.object(pink_ik, "_load_optional_dependencies", return_value=modules)
    mocker.patch.object(
        pink_ik,
        "prepare_urdf_for_drake",
        return_value=LoadedRobotModel("<robot/>", tmp_path / "prepared.urdf", {}),
    )
    model_path = tmp_path / "fake.urdf"
    model_path.write_text("<robot/>")
    world = _FakeWorld()
    world.config.model = RobotModel.from_file(model_path)
    ik = PinkIK(PinkIKConfig(max_iterations=1))
    build_context = mocker.spy(ik, "_build_robot_context")

    first = ik._get_model_context(cast("Any", world), "tool")
    second = ik._get_model_context(cast("Any", world), "base")

    assert first is not second
    assert first.model is second.model
    assert first.data is second.data
    build_context.assert_called_once_with(world.config, "tool")


def test_build_robot_context_rejects_base_link_not_model_root(
    mocker: MockerFixture, tmp_path: Path
) -> None:
    model = _FakeModel()
    model.frames[0] = _FakeFrame("base", parent_joint=1)
    modules = _fake_modules()
    modules.pinocchio.buildModelFromUrdf = lambda path: model  # type: ignore[attr-defined]
    mocker.patch.object(pink_ik, "_load_optional_dependencies", return_value=modules)
    mocker.patch.object(
        pink_ik,
        "prepare_urdf_for_drake",
        return_value=LoadedRobotModel("<robot/>", tmp_path / "prepared.urdf", {}),
    )
    model_path = tmp_path / "fake.urdf"
    model_path.write_text("<robot/>")
    config = _robot_config()
    config.model = RobotModel.from_file(model_path)

    with pytest.raises(ValueError, match="base_link 'base'.*model root"):
        PinkIK(PinkIKConfig(max_iterations=1))._build_robot_context(config, "tool")


def test_solve_pose_targets_uses_group_tip_and_filters_group_joints(
    mocker: MockerFixture,
) -> None:
    ik = _pink_ik(mocker, converge=True)
    context = _context()
    get_context = mocker.patch.object(ik, "_get_model_context", return_value=context)
    mocker.patch.object(
        ik,
        "_solve_single",
        return_value=IKResult(
            status=IKStatus.SUCCESS,
            joint_state=JointState(
                {"name": ["joint_a", "joint_b", "joint_c"], "position": [0.1, 0.2, 0.3]}
            ),
        ),
    )
    world = _FakeWorld()

    result = ik.solve_pose_targets(
        world=cast("Any", world),
        pose_targets={
            world.groups["manipulator"]: PoseStamped(
                position=Vector3(), orientation=Quaternion(0.0, 0.0, 0.0, 1.0)
            )
        },
        seed=JointState({"name": ["joint_a", "joint_b", "joint_c"], "position": [0.0, 0.0, 0.0]}),
        max_attempts=1,
    )

    get_context.assert_called_once_with(cast("Any", world), "tool")
    assert result.status == IKStatus.SUCCESS
    assert result.joint_state is not None
    assert result.joint_state.name == ["joint_a", "joint_b"]
    assert result.joint_state.position == [0.1, 0.2]
    assert world.joint_state_calls == 1


def test_solve_pose_targets_rejects_group_without_tip(mocker: MockerFixture) -> None:
    ik = _pink_ik(mocker)
    world = _FakeWorld()

    result = ik.solve_pose_targets(
        world=cast("Any", world),
        pose_targets={
            world.groups["no_tip"]: PoseStamped(
                position=Vector3(), orientation=Quaternion(0.0, 0.0, 0.0, 1.0)
            )
        },
    )

    assert result.status == IKStatus.UNSUPPORTED
    assert "no pose target frame" in result.message


def test_solve_pose_targets_partial_seed_reads_world_state(mocker: MockerFixture) -> None:
    ik = _pink_ik(mocker)
    mocker.patch.object(ik, "_get_model_context", return_value=_context())
    mocker.patch.object(
        ik,
        "_solve_single",
        return_value=IKResult(
            status=IKStatus.SUCCESS,
            joint_state=JointState(
                {"name": ["joint_a", "joint_b", "joint_c"], "position": [0.1, 0.2, 0.3]}
            ),
        ),
    )
    world = _FakeWorld()

    result = ik.solve_pose_targets(
        world=cast("Any", world),
        pose_targets={
            world.groups["manipulator"]: PoseStamped(
                position=Vector3(), orientation=Quaternion(0.0, 0.0, 0.0, 1.0)
            )
        },
        seed=JointState({"name": ["joint_a"], "position": [0.0]}),
        max_attempts=1,
    )

    assert result.status == IKStatus.SUCCESS
    assert world.joint_state_calls == 1


def test_solve_pose_targets_multi_target_uses_multi_frame_solve(mocker: MockerFixture) -> None:
    ik = _pink_ik(mocker)
    world = _FakeWorld()
    mocker.patch.object(ik, "_get_model_context", return_value=_context())
    solve_multi = mocker.patch.object(
        ik,
        "_solve_multi",
        return_value=IKResult(
            status=IKStatus.SUCCESS,
            joint_state=JointState(
                {"name": ["joint_a", "joint_b", "joint_c"], "position": [0.1, 0.2, 0.3]}
            ),
            position_error=0.0,
            orientation_error=0.0,
        ),
    )

    result = ik.solve_pose_targets(
        world=cast("Any", world),
        pose_targets={
            world.groups["manipulator"]: PoseStamped(
                position=Vector3(), orientation=Quaternion(0.0, 0.0, 0.0, 1.0)
            ),
            world.groups["wrist"]: PoseStamped(
                position=Vector3(), orientation=Quaternion(0.0, 0.0, 0.0, 1.0)
            ),
        },
        seed=JointState({"name": ["joint_a", "joint_b", "joint_c"], "position": [0.0, 0.0, 0.0]}),
        max_attempts=1,
    )

    solve_multi.assert_called_once()
    assert len(solve_multi.call_args.kwargs["targets"]) == 2
    assert result.joint_state is not None
    assert result.joint_state.name == ["joint_a", "joint_b", "joint_c"]
    assert result.joint_state.position == [0.1, 0.2, 0.3]


def test_solve_pose_targets_auxiliary_only_retains_seed_selection_order(
    mocker: MockerFixture,
) -> None:
    ik = _pink_ik(mocker)
    world = _FakeWorld()

    result = ik.solve_pose_targets(
        world=cast("Any", world),
        pose_targets={},
        auxiliary_groups=[world.groups["no_tip"], world.groups["manipulator"]],
        seed=JointState({"name": ["joint_a", "joint_b", "joint_c"], "position": [0.1, 0.2, 0.3]}),
    )

    assert result.status == IKStatus.SUCCESS
    assert result.joint_state is not None
    assert result.joint_state.name == ["joint_c", "joint_a", "joint_b"]
    assert result.joint_state.position == [0.3, 0.1, 0.2]
def _solved_joint_state() -> JointState:
    return JointState({"name": ["joint_a", "joint_b", "joint_c"], "position": [0.1, 0.2, 0.3]})


def _qp_infeasible() -> NoSolutionFound:
    return NoSolutionFound(
        problem=cast("Any", SimpleNamespace()),
        results=cast("Any", SimpleNamespace()),
    )


def test_solve_retries_after_no_solution_found(mocker: MockerFixture) -> None:
    ik = _pink_ik(mocker, converge=True)
    ik._robot_contexts = {("robot", "tool"): _context()}
    outcomes: list[object] = [
        _qp_infeasible(),
        IKResult(
            status=IKStatus.SUCCESS,
            joint_state=_solved_joint_state(),
            position_error=0.0006,
            orientation_error=0.0,
            iterations=1,
        ),
    ]

    def fake_solve_targets(**_: object) -> IKResult:
        outcome = outcomes.pop(0)
        if isinstance(outcome, Exception):
            raise outcome
        return cast("IKResult", outcome)

    solve_targets = mocker.patch.object(ik, "_solve_targets", side_effect=fake_solve_targets)

    result = ik.solve(
        world=cast("Any", _FakeWorld(collision_free=True)),
        robot_id="robot",
        target_pose=PoseStamped(
            position=Vector3(0.1, 0.0, 0.0),
            orientation=Quaternion(0.0, 0.0, 0.0, 1.0),
        ),
        check_collision=True,
        max_attempts=2,
    )

    assert solve_targets.call_count == 2
    assert result.status == IKStatus.SUCCESS
    assert result.joint_state is not None


def test_solve_reports_first_no_solution_found_after_all_attempts(
    mocker: MockerFixture,
) -> None:
    ik = _pink_ik(mocker, converge=True)
    ik._robot_contexts = {("robot", "tool"): _context()}

    def fake_solve_targets(**_: object) -> IKResult:
        raise _qp_infeasible()

    solve_targets = mocker.patch.object(ik, "_solve_targets", side_effect=fake_solve_targets)

    result = ik.solve(
        world=cast("Any", _FakeWorld(collision_free=True)),
        robot_id="robot",
        target_pose=PoseStamped(
            position=Vector3(0.1, 0.0, 0.0),
            orientation=Quaternion(0.0, 0.0, 0.0, 1.0),
        ),
        check_collision=True,
        max_attempts=3,
    )

    assert solve_targets.call_count == 3
    assert result.status == IKStatus.NO_SOLUTION
    assert "QP solver did not find a solution" in result.message


def test_solve_does_not_retry_unexpected_exception(mocker: MockerFixture) -> None:
    ik = _pink_ik(mocker, converge=True)
    ik._robot_contexts = {("robot", "tool"): _context()}
    solve_targets = mocker.patch.object(
        ik, "_solve_targets", side_effect=RuntimeError("unexpected solver failure")
    )

    result = ik.solve(
        world=cast("Any", _FakeWorld(collision_free=True)),
        robot_id="robot",
        target_pose=PoseStamped(
            position=Vector3(0.1, 0.0, 0.0),
            orientation=Quaternion(0.0, 0.0, 0.0, 1.0),
        ),
        check_collision=True,
        max_attempts=3,
    )

    assert solve_targets.call_count == 1
    assert result.status == IKStatus.NO_SOLUTION
    assert result.message == "Pink IK solver failed: unexpected solver failure"


def test_solve_mapping_value_error_fails_without_retrying(mocker: MockerFixture) -> None:
    ik = _pink_ik(mocker, converge=True)
    ik._robot_contexts = {("robot", "tool"): _context()}
    solve_targets = mocker.patch.object(
        ik, "_solve_targets", side_effect=ValueError("joint 'joint_z' is not in the model")
    )

    result = ik.solve(
        world=cast("Any", _FakeWorld(collision_free=True)),
        robot_id="robot",
        target_pose=PoseStamped(
            position=Vector3(0.1, 0.0, 0.0),
            orientation=Quaternion(0.0, 0.0, 0.0, 1.0),
        ),
        check_collision=True,
        max_attempts=5,
    )

    assert solve_targets.call_count == 1
    assert result.status == IKStatus.NO_SOLUTION
    assert "Pink IK mapping failed" in result.message


def _pose_targets_seed() -> JointState:
    return JointState(
        {"name": ["arm/joint_a", "arm/joint_b", "arm/joint_c"], "position": [0.0, 0.0, 0.0]}
    )


def test_solve_pose_targets_retries_after_no_solution_found(mocker: MockerFixture) -> None:
    ik = _pink_ik(mocker, converge=True)
    mocker.patch.object(ik, "_get_robot_context", return_value=_context())
    world = _FakeWorld()
    outcomes: list[object] = [
        _qp_infeasible(),
        IKResult(
            status=IKStatus.SUCCESS,
            joint_state=_solved_joint_state(),
            position_error=0.0006,
            orientation_error=0.0,
        ),
    ]

    def fake_solve_targets(**_: object) -> IKResult:
        outcome = outcomes.pop(0)
        if isinstance(outcome, Exception):
            raise outcome
        return cast("IKResult", outcome)

    solve_targets = mocker.patch.object(ik, "_solve_targets", side_effect=fake_solve_targets)

    result = ik.solve_pose_targets(
        world=cast("Any", world),
        pose_targets={
            world.groups["arm/manipulator"]: PoseStamped(
                position=Vector3(), orientation=Quaternion(0.0, 0.0, 0.0, 1.0)
            )
        },
        seed=_pose_targets_seed(),
        max_attempts=2,
    )

    assert solve_targets.call_count == 2
    assert result.status == IKStatus.SUCCESS
    assert result.joint_state is not None


def test_solve_pose_targets_reports_first_no_solution_found_after_all_attempts(
    mocker: MockerFixture,
) -> None:
    ik = _pink_ik(mocker, converge=True)
    mocker.patch.object(ik, "_get_robot_context", return_value=_context())
    world = _FakeWorld()

    def fake_solve_targets(**_: object) -> IKResult:
        raise _qp_infeasible()

    solve_targets = mocker.patch.object(ik, "_solve_targets", side_effect=fake_solve_targets)

    result = ik.solve_pose_targets(
        world=cast("Any", world),
        pose_targets={
            world.groups["arm/manipulator"]: PoseStamped(
                position=Vector3(), orientation=Quaternion(0.0, 0.0, 0.0, 1.0)
            )
        },
        seed=_pose_targets_seed(),
        max_attempts=3,
    )

    assert solve_targets.call_count == 3
    assert result.status == IKStatus.NO_SOLUTION
    assert "QP solver did not find a solution" in result.message


def test_solve_pose_targets_does_not_retry_unexpected_exception(
    mocker: MockerFixture,
) -> None:
    ik = _pink_ik(mocker, converge=True)
    mocker.patch.object(ik, "_get_robot_context", return_value=_context())
    world = _FakeWorld()
    solve_targets = mocker.patch.object(
        ik, "_solve_targets", side_effect=RuntimeError("unexpected solver failure")
    )

    result = ik.solve_pose_targets(
        world=cast("Any", world),
        pose_targets={
            world.groups["arm/manipulator"]: PoseStamped(
                position=Vector3(), orientation=Quaternion(0.0, 0.0, 0.0, 1.0)
            )
        },
        seed=_pose_targets_seed(),
        max_attempts=3,
    )

    assert solve_targets.call_count == 1
    assert result.status == IKStatus.NO_SOLUTION
    assert result.message == "Pink IK solver failed: unexpected solver failure"


def test_solve_pose_targets_mapping_value_error_fails_without_retrying(
    mocker: MockerFixture,
) -> None:
    ik = _pink_ik(mocker, converge=True)
    mocker.patch.object(ik, "_get_robot_context", return_value=_context())
    world = _FakeWorld()
    solve_targets = mocker.patch.object(
        ik, "_solve_targets", side_effect=ValueError("joint 'joint_z' is not in the model")
    )

    result = ik.solve_pose_targets(
        world=cast("Any", world),
        pose_targets={
            world.groups["arm/manipulator"]: PoseStamped(
                position=Vector3(), orientation=Quaternion(0.0, 0.0, 0.0, 1.0)
            )
        },
        seed=_pose_targets_seed(),
        max_attempts=5,
    )

    assert solve_targets.call_count == 1
    assert result.status == IKStatus.NO_SOLUTION
    assert "Pink IK mapping failed" in result.message
    assert world.joint_state_calls == 1
