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

"""Seeed B601-DM over its direct connection: selectors, mock, tasks and limits."""

from __future__ import annotations

from types import SimpleNamespace
from typing import NamedTuple
from unittest.mock import ANY

import can_motor_control
from can_motor_control import damiao
import numpy as np
import pinocchio
import pytest

from dimos.control.connection.mock_joint_connection import MockJointConnection
from dimos.control.coordinator import ControlCoordinator, TaskConfig
from dimos.control.task import CoordinatorState, JointStateSnapshot
from dimos.control.tasks.eef_twist_task.eef_twist_task import EEFTwistTaskParams, create_task
from dimos.control.tasks.trajectory_task.trajectory_task import TrajectoryExecutionStatus
from dimos.core.coordination.blueprint_config.errors import BlueprintConfigError
from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.core.global_config import global_config
from dimos.manipulation.manipulation_module import ManipulationModule, ManipulationModuleConfig
from dimos.manipulation.planning.groups.models import PlanningGroup
from dimos.manipulation.planning.groups.registry import PlanningGroupRegistry
from dimos.manipulation.planning.kinematics.config import PinkKinematicsConfig
from dimos.manipulation.planning.kinematics.pink_ik import PinkIK
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.manipulation.planning.spec.validation import prepare_robot_model
from dimos.manipulation.planning.trajectory_generator.joint_trajectory_generator import (
    JointTrajectoryGenerator,
)
from dimos.manipulation.planning.world.roboplan_world import RoboPlanWorld
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.TwistStamped import TwistStamped
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.trajectory_msgs.JointTrajectory import JointTrajectory
from dimos.msgs.trajectory_msgs.TrajectoryPoint import TrajectoryPoint
from dimos.robot.assets.model import RobotModel
from dimos.robot.manipulators._modeling import named_for_robot
from dimos.robot.manipulators.common.topics import EEF_TWIST_TASK_NAME
from dimos.robot.manipulators.damiao._damiao_test_helpers import mock_connection_rig
from dimos.robot.manipulators.damiao.arm import DamiaoArm
from dimos.robot.manipulators.damiao.connection import DamiaoConnection
from dimos.robot.manipulators.seeed_b601_dm.blueprints.basic import (
    B601_DM_PLANNING_MODEL,
    seeed_b601_dm_control,
    seeed_b601_dm_planner_coordinator,
)
from dimos.robot.manipulators.seeed_b601_dm.blueprints.teleop import keyboard_teleop_seeed_b601_dm
from dimos.robot.manipulators.seeed_b601_dm.config import (
    B601_DM,
    B601_DM_ACCELERATION_LIMIT,
    B601_DM_FINGER_JOINTS,
    B601_DM_JOINT2_LIMITS,
    B601_DM_MODEL,
    B601_DM_SOURCE,
    B601_DM_VELOCITY_LIMITS,
    GRIPPER_CALIBRATION_TRAVEL_RAD,
    GRIPPER_CURRENT,
    b601_dm_gripper_task,
    b601_dm_jog_task,
    b601_dm_model_config,
)
from dimos.robot.manipulators.seeed_b601_dm.connection import B601DMConnection
from dimos.robot.manipulators.seeed_b601_dm.coordinator import SeeedB601DMCoordinator
from dimos.robot.manipulators.seeed_b601_dm.joints import B601_DM_ARM_JOINTS
from dimos.teleop.keyboard.keyboard_teleop_module import KeyboardTeleopModule
from dimos.utils.transform_utils import matrix_to_pose, pose_to_matrix


def test_connection_requires_an_explicit_selector() -> None:
    with pytest.raises(ValueError, match="Field required"):
        B601DMConnection()
    with pytest.raises(ValueError, match="CAN selector must not be empty"):
        B601DMConnection(can_port=" ")


@pytest.mark.parametrize("can_port", ["can0", "/dev/ttyACM0"])
def test_connection_uses_its_explicit_selector(can_port: str) -> None:
    connection = B601DMConnection(can_port=can_port)
    try:
        assert connection.can_ports() == [can_port]
        assert connection.config.source == "b601_dm"
    finally:
        connection.stop()


def test_connection_declares_seeed_gains_and_topology() -> None:
    assert B601_DM.arm_joints == {"arm": list(B601_DM_ARM_JOINTS)}
    assert B601_DM.gripper_joints == {"gripper": "finger_left"}
    assert B601_DM.buses[0].kp == [120.0, 120.0, 120.0, 18.0, 18.0, 18.0]
    assert B601_DM.buses[0].kd == [5.0, 5.0, 5.0, 2.0, 2.0, 2.0]
    assert B601_DM.gravity is True
    assert B601_DM.teach_kd is None
    assert B601DMConnection.robot is B601_DM


def test_connection_builds_seeed_motor_table(mocker) -> None:
    class Builder:
        def __init__(self) -> None:
            self.calls: list[tuple[object, ...]] = []

        def add_bus(self, *args):
            self.calls.append(("bus", *args))
            return self

        def add_arm(self, name, **kwargs):
            self.calls.append(("arm", name, kwargs))
            return self

        def add_gripper(self, name, **kwargs):
            self.calls.append(("gripper", name, kwargs))
            return self

        def build(self):
            return self.calls

    builder = Builder()
    mocker.patch.object(can_motor_control, "MotorSpec", side_effect=lambda *args: args)
    # The bus object is transport-specific (SocketCAN, gs_usb, serial); the
    # motor table is what this asserts, so resolve the selector without one.
    mocker.patch.object(DamiaoArm, "_make_can_bus", lambda self, port: port)
    mocker.patch.object(
        can_motor_control, "Robot", SimpleNamespace(builder=lambda: builder), create=True
    )
    arm = DamiaoArm(B601_DM, ["can0"])
    calls = arm._build()

    bus, arm_call, gripper = calls
    assert bus == ("bus", "arm", "can0", ANY)
    assert arm_call[1] == "arm"
    assert [spec[:4] for spec in arm_call[2]["motors"]] == [
        ("joint1", damiao.MotorType.DM4340, 0x01, 0x11),
        ("joint2", damiao.MotorType.DM4340, 0x02, 0x12),
        ("joint3", damiao.MotorType.DM4340, 0x03, 0x13),
        ("joint4", damiao.MotorType.DM4310, 0x04, 0x14),
        ("joint5", damiao.MotorType.DM4310, 0x05, 0x15),
        ("joint6", damiao.MotorType.DM4310, 0x06, 0x16),
    ]
    assert gripper[1] == "gripper"
    options = gripper[2]
    assert options["motor"] == ("gripper_motor", damiao.MotorType.DM4310, 0x07, 0x17)
    assert options["opening_direction"] == "decreasing_position"
    assert options["default_current"] == GRIPPER_CURRENT
    assert options["calibration_travel"] == GRIPPER_CALIBRATION_TRAVEL_RAD


def test_simulation_composes_a_mock_connection(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(global_config, "simulation", "mujoco")
    blueprint = seeed_b601_dm_control().solidify()

    assert not any(issubclass(atom.module, DamiaoConnection) for atom in blueprint.blueprints)
    [mock] = [atom for atom in blueprint.blueprints if atom.module is MockJointConnection]
    assert mock.kwargs["source"] == "arm"
    assert list(mock.kwargs["initial_positions"]) == [*B601_DM_ARM_JOINTS, "finger_left"]
    assert set(mock.kwargs["initial_positions"].values()) == {0.0}
    assert blueprint.remapping_map[(mock.name, "position_command")] == "arm_position_command"


def test_planner_blueprint_requires_a_can_port() -> None:
    with pytest.raises(BlueprintConfigError, match="b601dmconnection.can_port: Field required"):
        BlueprintConfigParser(seeed_b601_dm_planner_coordinator.solidify()).parse([], environ={})


def test_planner_coordinator_drives_arm_and_gripper() -> None:
    blueprint = seeed_b601_dm_planner_coordinator.solidify()
    parsed = BlueprintConfigParser(blueprint).parse(["--can-port", "/dev/ttyACM0"], environ={})
    coordinator_atom = next(
        a for a in blueprint.active_blueprints if issubclass(a.module, SeeedB601DMCoordinator)
    )
    planner_atom = next(
        a for a in blueprint.active_blueprints if issubclass(a.module, ManipulationModule)
    )
    connection_atom = next(
        a for a in blueprint.active_blueprints if issubclass(a.module, DamiaoConnection)
    )
    tasks = {task.type: task for task in coordinator_atom.kwargs["tasks"]}

    assert coordinator_atom.kwargs["instance_name"] == "ControlCoordinator"
    assert coordinator_atom.kwargs.get("hardware", []) == []
    assert coordinator_atom.kwargs["zero_stiffness_joints"] == []
    assert connection_atom.module is B601DMConnection
    assert connection_atom.kwargs["source"] == "arm"
    assert parsed.module_kwargs("b601dmconnection")["can_port"] == "/dev/ttyACM0"
    assert tasks["trajectory"].joint_names == [f"arm/{name}" for name in B601_DM_ARM_JOINTS]
    assert tasks["gripper"].name == "b601_dm_gripper"
    assert tasks["gripper"].joint_names == ["arm/finger_left"]

    planner_config = ManipulationModuleConfig.model_validate(
        parsed.module_kwargs(planner_atom.name)
    )
    assert planner_config.gripper_tasks == {"manipulator": "b601_dm_gripper"}
    assert planner_config.model.joint_names == B601_DM_PLANNING_MODEL.joint_names
    assert planner_config.model.joint_names == [f"arm/{name}" for name in B601_DM_ARM_JOINTS]


def test_gripper_task_uses_the_deployed_joint_name() -> None:
    task = b601_dm_gripper_task()

    assert (task.name, task.type, task.joint_names, task.priority, task.params) == (
        "b601_dm_gripper",
        "gripper",
        ["arm/finger_left"],
        20,
        {"open_at": "upper"},
    )


def test_every_arm_joint_is_limited_to_2_rad_s() -> None:
    assert B601_DM_VELOCITY_LIMITS == dict.fromkeys(B601_DM_ARM_JOINTS, 2.0)
    assert B601_DM_MODEL == (
        RobotModel.from_file(B601_DM_SOURCE / "Rebot_Arm_description/DM/urdf/ReBot_Arm_DM.urdf")
        .with_joint_position_limits(
            "joint2", lower=B601_DM_JOINT2_LIMITS[0], upper=B601_DM_JOINT2_LIMITS[1]
        )
        .with_joint_velocity_limits(B601_DM_VELOCITY_LIMITS)
        .with_default_joint_acceleration_limit(B601_DM_ACCELERATION_LIMIT)
    )


def test_jog_task_moves_the_tool_link_with_the_planning_model() -> None:
    task = b601_dm_jog_task()
    params = EEFTwistTaskParams.model_validate(task.params)

    assert (task.name, task.type, task.priority) == (EEF_TWIST_TASK_NAME, "eef_twist", 20)
    assert task.joint_names == [f"arm/{name}" for name in B601_DM_ARM_JOINTS]
    assert params.robot_model == B601_DM_PLANNING_MODEL
    assert params.target_frame == "end_link"


def test_keyboard_teleop_jogs_plans_and_drives_the_gripper() -> None:
    blueprint = keyboard_teleop_seeed_b601_dm.solidify()
    parsed = BlueprintConfigParser(blueprint).parse(["--can-port", "/dev/ttyACM0"], environ={})
    modules = {atom.module for atom in blueprint.active_blueprints}
    coordinator_atom = next(
        a for a in blueprint.active_blueprints if issubclass(a.module, ControlCoordinator)
    )
    planner_atom = next(
        a for a in blueprint.active_blueprints if issubclass(a.module, ManipulationModule)
    )
    tasks = {task.type: task for task in coordinator_atom.kwargs["tasks"]}

    assert KeyboardTeleopModule in modules
    assert B601DMConnection in modules
    assert coordinator_atom.kwargs["instance_name"] == "ControlCoordinator"
    assert tasks["eef_twist"] == b601_dm_jog_task()
    assert (tasks["eef_twist"].priority, tasks["trajectory"].priority) == (20, 10)
    assert tasks["gripper"] == b601_dm_gripper_task()
    planner_config = ManipulationModuleConfig.model_validate(
        parsed.module_kwargs(planner_atom.name)
    )
    assert planner_config.model == B601_DM_PLANNING_MODEL
    assert planner_config.gripper_tasks == {"manipulator": "b601_dm_gripper"}


def _model_with_velocity_limits(limits: dict[str, float]) -> RobotModelConfig:
    """The B601 planning config named under "arm", with other velocity limits."""
    model = (
        RobotModel.from_file(B601_DM_SOURCE / "Rebot_Arm_description/DM/urdf/ReBot_Arm_DM.urdf")
        .with_joint_position_limits(
            "joint2", lower=B601_DM_JOINT2_LIMITS[0], upper=B601_DM_JOINT2_LIMITS[1]
        )
        .with_joint_velocity_limits(limits)
        .with_default_joint_acceleration_limit(B601_DM_ACCELERATION_LIMIT)
    )
    stock = b601_dm_model_config()
    return named_for_robot(
        stock.model_copy(update={"model": model}), "arm", fingers=B601_DM_FINGER_JOINTS
    )


# The limits this arm had before: 0.2 rad/s on joints 1 to 3, 0.8 on 4 to 6.
_SLOW_LIMITS = dict(zip(B601_DM_ARM_JOINTS, (0.2, 0.2, 0.2, 0.8, 0.8, 0.8), strict=True))


class _Jog(NamedTuple):
    speed: float
    """Hand speed along the twist, m/s."""
    peak_hand_speed: float
    """Fastest the hand moved in any direction over one tick, m/s."""
    peak_joint_speed: float
    """Fastest joint command step, rad/s."""


def _jog(task_config: TaskConfig, start: list[float], direction: list[float]) -> _Jog:
    """Hold a 0.05 m/s twist for 1 s at 100 Hz on a connection that follows every command."""
    task = create_task(task_config)
    names = list(task_config.joint_names)
    frame = str(task_config.params["target_frame"])
    joints = np.array(start)

    def tip() -> np.ndarray:
        state = JointState(name=names, position=joints.tolist())
        return np.asarray(pose_to_matrix(task._solver.frame_poses(state, [frame])[frame]))[:3, 3]

    twist = TwistStamped(linear=[0.05 * value for value in direction], angular=[0.0, 0.0, 0.0])
    begin = previous = tip()
    peak_hand = peak_joint = 0.0
    t = 100.0
    for _ in range(100):
        task.on_ee_twist_command(twist, t)
        state = CoordinatorState(
            joints=JointStateSnapshot(
                joint_positions=dict(zip(names, joints.tolist(), strict=True))
            ),
            t_now=t,
            dt=0.01,
        )
        output = task.compute(state)
        assert output is not None
        commanded = np.array(output.positions)
        peak_joint = max(peak_joint, float(np.max(np.abs(commanded - joints))) / 0.01)
        joints = commanded
        now = tip()
        peak_hand = max(peak_hand, float(np.linalg.norm(now - previous)) / 0.01)
        previous = now
        t += 0.01
    return _Jog(float((tip() - begin) @ np.array(direction)), peak_hand, peak_joint)


@pytest.mark.self_hosted
@pytest.mark.parametrize("direction", [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])
def test_jog_moves_the_hand_at_the_twist_speed(direction: list[float]) -> None:
    # The hand 20 cm over the table, 29 cm in front of the base.
    ready = [0.0, -0.94, -0.86, 0.71, 0.0, 0.0]
    slow = TaskConfig(
        name="slow_limits",
        type="eef_twist",
        joint_names=b601_dm_jog_task().joint_names,
        priority=20,
        params={
            "robot_model": _model_with_velocity_limits(_SLOW_LIMITS),
            "target_frame": "end_link",
        },
    )

    jog = _jog(b601_dm_jog_task(), ready, direction)
    assert jog.speed > 0.04
    assert jog.peak_hand_speed < 0.1
    # With the earlier 0.2 / 0.8 rad/s the same jog crawled under 2 cm/s.
    assert _jog(slow, ready, direction).speed < 0.02


@pytest.mark.self_hosted
def test_jog_stays_bounded_near_a_wrist_singularity() -> None:
    # Here a 0.05 m/s pull asks a wrist joint for more than 2 rad/s.
    wrist = [0.05, -0.46, -1.13, 0.68, -1.28, 0.26]

    jog = _jog(b601_dm_jog_task(), wrist, [-1.0, 0.0, 0.0])

    assert jog.speed > 0.04
    assert jog.peak_hand_speed < 0.1
    assert jog.peak_joint_speed <= 2.0


def _planning_world(config: RobotModelConfig) -> tuple[RoboPlanWorld, PlanningGroup]:
    world = RoboPlanWorld()
    world.load_model(prepare_robot_model(config))
    world.finalize()
    return world, PlanningGroupRegistry(config.planning_groups).get("manipulator")


@pytest.mark.self_hosted
@pytest.mark.parametrize(
    "joints",
    [
        [0.055, -0.464, -1.132, 0.681, -1.282, 0.258],
        [0.044, -0.387, -2.005, 0.188, -1.383, -0.705],
        [0.588, -1.124, -1.016, -1.351, -0.187, -1.635],
    ],
)
def test_planner_reaches_reachable_poses_from_the_folded_rest(joints: list[float]) -> None:
    # The pose the arm is in at these joints, asked from the folded rest (all zeros).
    world, group = _planning_world(B601_DM_PLANNING_MODEL)
    names = list(group.joint_names)
    with world.scratch_context() as ctx:
        world.set_joint_state(ctx, JointState(name=names, position=joints))
        pose = matrix_to_pose(np.asarray(world.get_link_pose(ctx, "end_link")))
    target = PoseStamped(frame_id="world", position=pose.position, orientation=pose.orientation)
    folded = JointState(name=names, position=[0.0] * len(names))

    def solved(world: RoboPlanWorld, group: PlanningGroup) -> bool:
        world.sync_from_joint_state(folded)
        result = PinkIK(PinkKinematicsConfig()).solve_pose_targets(
            world, {group: target}, seed=folded, max_attempts=1
        )
        return result.is_success()

    assert solved(world, group)
    # The earlier 0.2 / 0.8 rad/s held each IK iteration to a step too short to get there.
    assert not solved(*_planning_world(_model_with_velocity_limits(_SLOW_LIMITS)))


@pytest.mark.self_hosted
def test_long_planned_moves_are_acceleration_limited() -> None:
    joint_space = prepare_robot_model(B601_DM_PLANNING_MODEL).joint_space
    coordinates = [joint_space.coordinate(name) for name in B601_DM_PLANNING_MODEL.joint_names]
    generator = JointTrajectoryGenerator(
        num_joints=len(coordinates),
        max_velocity=[c.max_velocity for c in coordinates],
        max_acceleration=[c.max_acceleration for c in coordinates],
    )
    # Folded rest to the hand over the table in front of the base: 1.57 rad on joint 1 ...
    move = generator.generate([[0.0] * 6, [1.57, -0.94, -0.86, 0.71, 0.0, 0.0]])

    # ... is a 0.5 rad/s2 triangle, about 3.5 s, peaking under 1 rad/s: below the 2 rad/s limit.
    assert move.points[-1].time_from_start == pytest.approx(2 * (1.57 / 0.5) ** 0.5, rel=0.02)


def test_model_plans_arm_joints_to_the_tool_link() -> None:
    model = b601_dm_model_config()

    assert model.joint_names == list(B601_DM_ARM_JOINTS)
    group = model.planning_groups[0]
    assert (group.base_link, group.tip_link) == ("base_link", "end_link")


@pytest.mark.self_hosted
def test_model_moves_fingers_and_widens_joint2_for_the_folded_rest() -> None:
    loaded = B601_DM_MODEL.load()

    assert loaded.get_joint("finger_left").type == "prismatic"
    assert loaded.get_joint("finger_right").mimic == ("finger_left", -1.0, 0.0)
    model = pinocchio.buildModelFromXML(loaded.xml)
    assert tuple(str(name) for name in model.names[1:7]) == B601_DM_ARM_JOINTS
    joint2 = model.getJointId("joint2") - 1
    assert (model.lowerPositionLimit[joint2], model.upperPositionLimit[joint2]) == pytest.approx(
        B601_DM_JOINT2_LIMITS
    )


def test_mock_coordinator_executes_trajectory_and_gripper_command(tmp_path, mocker) -> None:
    clock = mocker.patch("dimos.control.tick_loop.time.perf_counter", return_value=100.0)
    atom = next(
        a
        for a in seeed_b601_dm_planner_coordinator.solidify().blueprints
        if issubclass(a.module, ControlCoordinator)
    )
    coordinator = atom.module(publish_joint_state=False, **atom.kwargs)
    with mock_connection_rig(B601DMConnection, coordinator, tmp_path=tmp_path) as rig:
        target = [0.01, -0.01, -0.01, 0.01, 0.01, 0.01]
        trajectory = JointTrajectory(
            joint_names=[f"arm/{name}" for name in B601_DM_ARM_JOINTS],
            points=[
                TrajectoryPoint(positions=[0.0] * 6, velocities=[0.0] * 6, time_from_start=0.0),
                TrajectoryPoint(positions=target, velocities=[0.0] * 6, time_from_start=0.3),
            ],
        )
        assert coordinator.execute_trajectory(trajectory).status is (
            TrajectoryExecutionStatus.ACCEPTED
        )
        # The first tick after submission anchors the start point at now.
        rig.tick()
        clock.return_value = 101.0
        rig.tick()
        assert [rig.positions[f"arm/joint{index}"] for index in range(1, 7)] == pytest.approx(
            target
        )

        assert coordinator.task_invoke("b601_dm_gripper", "set_normalized", {"values": [1.0]})
        rig.tick()
        # finger_left is fully open at 0.05 m.
        assert rig.positions["arm/finger_left"] == pytest.approx(0.05)
