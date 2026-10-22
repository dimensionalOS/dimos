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

"""Connection contracts and native xArm/Piper trajectories through sim2."""

from contextlib import ExitStack
from dataclasses import replace
import importlib
from pathlib import Path

import numpy as np
import pytest

from dimos.control.coordinator import ControlCoordinator
from dimos.control.tasks.trajectory_task.trajectory_task import TrajectoryExecutionStatus
from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.core.global_config import global_config
from dimos.hardware.manipulators.registry import adapter_registry
from dimos.hardware.manipulators.spec import ControlMode
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.trajectory_msgs.JointTrajectory import JointTrajectory
from dimos.msgs.trajectory_msgs.TrajectoryPoint import TrajectoryPoint
from dimos.robot.manipulators.piper.connection import PiperConnection
from dimos.robot.manipulators.xarm.connection import XArmConnection
from dimos.sim2.client import SimRobotClient
from dimos.sim2.messages import FrameMetadata, StateFrame, StreamDescriptor
from dimos.sim2.module import SimulationModule
from dimos.utils.testing.waiting import wait_until


@pytest.fixture(params=[(XArmConnection, 6), (XArmConnection, 7), (PiperConnection, 6)])
def connection(request):
    cls, dof = request.param
    with ExitStack() as stack:
        device = cls(dof=dof)
        stack.callback(device.stop)
        yield device


def test_arm_contract_uses_native_gripper_units(connection):
    description = connection.describe_control().description
    assert list(description.joint_names()) == [
        *(f"arm/joint{i}" for i in range(1, connection.config.dof + 1)),
        "arm/gripper",
    ]
    assert description.limits["arm/gripper/position"].hi == (
        850 if isinstance(connection, XArmConnection) else 0.08
    )


def test_explicit_mount_is_used_without_loading_a_different_scene(connection):
    config = connection.config.model_copy(
        update={
            "base_pose": PoseStamped(frame_id="world", position=Vector3(1, 2, 0.7)),
        }
    )
    instance = type(connection).describe_simulated_robot(config, Path("/unused/world.xml"))
    assert instance.xyz == (1, 2, 0.7)


@pytest.fixture
def sdk_connection(connection, mocker):
    sdk = mocker.MagicMock()
    n = connection.config.dof
    upper = connection.describe_control().description.limits["arm/gripper/position"].hi
    sdk.read_joint_positions.return_value = [0.0] * n + [upper]
    sdk.read_joint_velocities.return_value = [0.0] * (n + 1)
    sdk.read_joint_efforts.return_value = [0.0] * (n + 1)
    mocker.patch.object(adapter_registry, "create", return_value=sdk)
    connection.connect()
    yield connection, sdk


def test_sdk_readings_match_the_declared_contract(sdk_connection):
    connection, _ = sdk_connection
    assert connection.read_state().keys() == set(connection.description.state_keys())
    assert "arm/gripper/velocity" not in connection.read_state()


def test_partial_joint_command_preserves_other_targets(sdk_connection):
    connection, sdk = sdk_connection
    connection.write({"arm/joint1/position": 0.15})
    targets = sdk.write_joint_positions.call_args.args[0]
    assert targets == [0.15] + [0.0] * (connection.config.dof - 1) + [
        connection.description.limits["arm/gripper/position"].hi
    ]


def test_halt_discards_held_sdk_targets(sdk_connection):
    connection, sdk = sdk_connection
    connection.write({"arm/joint1/position": 0.15})
    connection.halt()
    sdk.write_stop.assert_called_once()
    sdk.read_joint_positions.return_value[0] = 0.05
    connection.write({"arm/joint2/position": 0.1})
    assert sdk.write_joint_positions.call_args.args[0][:2] == [0.05, 0.1]


@pytest.fixture
def simulated_connection(connection, mocker):
    client = mocker.create_autospec(SimRobotClient, instance=True)
    size = len(connection._names)
    frame = StateFrame(
        FrameMetadata(1, 0, 1, 1, 0.005),
        {
            "position": np.zeros(size),
            "velocity": np.zeros(size),
            "effort": np.zeros(size),
            "wall_time": np.array([123.0]),
        },
    )
    client.read.return_value = frame
    mocker.patch.object(connection, "_sim", client)
    yield connection, client, frame


def test_sim_feedback_keeps_acquisition_time(simulated_connection):
    connection, _, _ = simulated_connection
    assert connection.read_state().source_ts == 123.0


def test_reset_discards_partial_targets_before_next_state_poll(simulated_connection):
    connection, client, frame = simulated_connection
    connection.write({"arm/joint1/position": 0.5}, source_ts=123.1)
    reset = replace(frame, metadata=replace(frame.metadata, sequence=2, episode_id=1))
    client.read.return_value = reset

    connection.write({"arm/joint2/position": 0.2}, source_ts=124.0)

    client.write_positions.assert_called_with(
        [0.0, 0.2] + [0.0] * (len(connection._names) - 2),
        observed=reset,
        source_ts=124.0,
    )


def test_xarm_velocity_and_gripper_do_not_change_each_others_mode(mocker):
    with ExitStack() as stack:
        connection = XArmConnection(dof=6)
        stack.callback(connection.stop)
        sdk = mocker.MagicMock()
        sdk.read_joint_positions.return_value = [0.0] * 6 + [850.0]
        sdk.read_joint_velocities.return_value = [0.0] * 7
        sdk.read_joint_efforts.return_value = [0.0] * 7
        mocker.patch.object(adapter_registry, "create", return_value=sdk)
        connection.connect()
        connection.write({"arm/joint1/velocity": 0.1})
        connection.write({"arm/gripper/position": 400.0})
        sdk.set_control_mode.assert_called_once_with(ControlMode.VELOCITY)
        sdk.write_gripper_position.assert_called_once_with(400.0)
        sdk.write_joint_positions.assert_not_called()
        connection.write({"arm/joint1/position": 0.1})
        assert sdk.set_control_mode.call_args.args == (ControlMode.SERVO_POSITION,)
        assert sdk.write_joint_positions.call_args.args[0][-1] == 400.0


@pytest.fixture
def running_arm(request, monkeypatch):
    family, dof = request.param
    monkeypatch.setattr(global_config, "simulation", "mujoco")
    monkeypatch.setattr(global_config, "scene_package", None)
    path = f"dimos.robot.manipulators.{family}.blueprints.basic"
    module = importlib.reload(importlib.import_module(path))
    name = f"coordinator_xarm{dof}" if family == "xarm" else "coordinator_piper"
    blueprint = getattr(module, name)
    config = BlueprintConfigParser(blueprint).parse(
        [],
        environ={},
        global_overrides={
            "simulation": "mujoco",
            "transport": "zenoh",
            "viewer": "none",
            "n_workers": 2,
        },
    )
    with ExitStack() as stack:
        app = ModuleCoordinator.build(blueprint, config)
        stack.callback(app.stop)
        world = app.get_instance(SimulationModule)
        client = stack.enter_context(
            SimRobotClient(StreamDescriptor.from_dict(world.describe()["robots"]["arm_connection"]))
        )
        yield app, client, dof


@pytest.mark.self_hosted
@pytest.mark.parametrize("running_arm", [("xarm", 6), ("xarm", 7), ("piper", 6)], indirect=True)
def test_native_arm_executes_trajectory(running_arm):
    app, client, dof = running_arm
    controller = app.get_instance(ControlCoordinator)
    wait_until(lambda: len(controller.get_joint_positions()) >= dof, timeout=5)
    start = client.read().values["position"][:dof].copy()
    target = start.copy()
    target[0] += 0.15
    trajectory = JointTrajectory(
        joint_names=[f"arm/joint{i}" for i in range(1, dof + 1)],
        points=[
            TrajectoryPoint(positions=start.tolist(), time_from_start=0),
            TrajectoryPoint(positions=target.tolist(), time_from_start=1),
        ],
    )
    # Only arm axes participate; the gripper has its independent task/hold.
    assert np.isfinite(start).all()
    result = controller.execute_trajectory(trajectory)
    assert result.status is TrajectoryExecutionStatus.ACCEPTED
    wait_until(lambda: abs(client.read().values["position"][0] - target[0]) < 0.025, timeout=5)
    assert client.read().values["position"][:dof] == pytest.approx(target, abs=0.025)
    assert app.get_instance(SimulationModule).status()["error"] is None


@pytest.mark.self_hosted
@pytest.mark.parametrize("running_arm", [("xarm", 6)], indirect=True)
def test_native_gripper_uses_sdk_aperture_units(running_arm):
    app, client, _ = running_arm
    controller = app.get_instance(ControlCoordinator)
    wait_until(lambda: "arm/gripper" in controller.get_joint_positions(), timeout=5)
    assert controller.task_invoke("arm_gripper", "set_normalized", {"values": [0.5]})
    wait_until(lambda: abs(client.read().values["position"][-1] - 425.0) < 25, timeout=5)
    assert client.read().values["position"][-1] == pytest.approx(425, abs=25)


@pytest.mark.self_hosted
def test_two_arms_share_one_world_without_overlapping(monkeypatch):
    monkeypatch.setattr(global_config, "simulation", "mujoco")
    monkeypatch.setattr(global_config, "scene_package", None)
    module = importlib.reload(
        importlib.import_module("dimos.robot.manipulators.xarm.blueprints.basic")
    )
    blueprint = module.coordinator_dual_xarm
    parsed = BlueprintConfigParser(blueprint).parse(
        [],
        environ={},
        global_overrides={
            "simulation": "mujoco",
            "transport": "zenoh",
            "viewer": "none",
            "n_workers": 2,
        },
    )
    with ExitStack() as stack:
        app = ModuleCoordinator.build(blueprint, parsed)
        stack.callback(app.stop)
        world = app.get_instance(SimulationModule)
        status = world.status()
        assert sorted(status["robots"]) == ["left_arm_connection", "right_arm_connection"]
        assert status["robots"]["left_arm_connection"][1] == pytest.approx(0.4)
        assert status["robots"]["right_arm_connection"][1] == pytest.approx(-0.4)
        coordinator = app.get_instance(ControlCoordinator)
        wait_until(lambda: len(coordinator.get_joint_positions()) == 13, timeout=5)
        assert len(coordinator.get_joint_positions()) == 13
        assert world.status()["error"] is None
