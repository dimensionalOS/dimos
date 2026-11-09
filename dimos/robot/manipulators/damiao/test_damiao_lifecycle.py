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

"""Every Damiao arm through one fake-device lifecycle, without opening CAN.

The real ``PhysicalModel`` is used, so each arm's own URDF limits and gravity
are exercised; only the native robot and its transports are fakes.

OpenYAM's descriptions are large LFS archives, so those cases (and the teach
case, which reads the same gravity model) only run where big LFS pulls are
allowed; the regular CI job caps them at 1 MiB on purpose.
"""

from __future__ import annotations

import numpy as np
import pytest

from dimos.message_utils import time as message_time
from dimos.messages.sensor_msgs.msg import JointState
from dimos.robot.manipulators.damiao._damiao_native_test_helpers import LocalTransport
from dimos.robot.manipulators.damiao._damiao_test_helpers import _local_test_model
from dimos.robot.manipulators.damiao.arm import PhysicalModel
from dimos.robot.manipulators.damiao.connection import DamiaoConnection
from dimos.robot.manipulators.dual_makermods_metal.connection import DualMetalConnection
from dimos.robot.manipulators.dual_openyam.connection import DualOpenYamConnection
from dimos.robot.manipulators.makermods_metal.connection import MetalConnection
from dimos.robot.manipulators.openarm.connection import OpenArmConnection
from dimos.robot.manipulators.openyam.config import OPENYAM
from dimos.robot.manipulators.openyam.connection import OpenYamConnection
from dimos.robot.manipulators.seeed_b601_dm.connection import B601DMConnection
from dimos.utils.testing.waiting import wait_until

_LFS = pytest.mark.self_hosted

ARMS = [
    pytest.param(OpenYamConnection, {"can_port": "can0"}, id="openyam", marks=_LFS),
    pytest.param(
        DualOpenYamConnection,
        {"left_can_port": "can0", "right_can_port": "can1"},
        id="dual-openyam",
        marks=_LFS,
    ),
    pytest.param(
        OpenArmConnection,
        {"left_can_port": "can0", "right_can_port": "can1"},
        id="openarm",
    ),
    pytest.param(MetalConnection, {"can_port": "can0"}, id="metal"),
    pytest.param(
        DualMetalConnection,
        {"left_can_port": "can0", "right_can_port": "can1"},
        id="dual-metal",
    ),
    pytest.param(B601DMConnection, {"can_port": "can0"}, id="b601"),
]


def _physical(connection_type: type[DamiaoConnection]) -> PhysicalModel:
    robot = connection_type.robot
    return PhysicalModel(robot.gravity_model, robot.arm_joint_names, gravity_comp=robot.gravity)


def _middle(physical: PhysicalModel) -> np.ndarray:
    model = physical._model
    return (np.asarray(model.lowerPositionLimit) + np.asarray(model.upperPositionLimit)) / 2.0


def _set_pose(native, connection_type: type[DamiaoConnection], pose: np.ndarray) -> None:
    offset = 0
    for bus in connection_type.robot.buses:
        group = native.groups[bus.name]
        group.q = pose[offset : offset + len(bus.arm)].copy()
        offset += len(bus.arm)


def _rig(connection_type, ports, boundary, mocker, tmp_path, **kwargs):
    mocker.patch("dimos.protocol.rpc.zenohrpc.ZenohRPC.start")
    mocker.patch("dimos.protocol.rpc.zenohrpc.ZenohRPC.serve_module_rpc")
    # The fake arms start at the zero pose, which every deployed arm allows.
    boundary[0].default_position = 0.0
    module = connection_type(
        **ports,
        source="arm",
        model=_local_test_model(connection_type, tmp_path / "control.urdf"),
        deadman_timeout_s=10,
        **kwargs,
    )
    commands, readings = LocalTransport(), LocalTransport()
    module.set_transport("position_command", commands)
    module.set_transport("joint_state", readings)
    return module, commands, readings


@pytest.mark.parametrize("connection_type,ports", ARMS)
def test_fake_device_lifecycle(connection_type, ports, boundary, mocker, tmp_path):
    physical = _physical(connection_type)
    robot = connection_type.robot
    native = boundary[0]
    module, commands, readings = _rig(connection_type, ports, boundary, mocker, tmp_path)
    try:
        # Connect opens the buses but enables nothing and accepts nothing.
        module.connect()
        assert "enable" not in native.events
        assert not any(group.modes for group in native.groups.values())

        module.start()
        assert "enable" in native.events
        wait_until(lambda: bool(readings.sent), timeout=2)
        assert readings.sent[0].name == [f"arm/{name}" for name in robot.joint_names]

        pose = _middle(physical)
        _set_pose(native, connection_type, pose)
        target = robot.arm_joint_names[0]
        jaw = robot.grippers[0]
        halfway = sum(robot.gripper_travel()[jaw.joint]) / 2.0
        commands.publish(
            JointState(
                header=message_time.header_now(),
                name=[f"arm/{target}", f"arm/{jaw.joint}"],
                position=np.asarray([float(pose[0]), halfway], dtype=np.float64),
                velocity=np.asarray([], dtype=np.float64),
                effort=np.asarray([], dtype=np.float64),
            )
        )
        first_bus = robot.buses[0]
        rows = native.groups[first_bus.name].commands[-1]
        np.testing.assert_equal(rows[0, :2], [first_bus.kp[0], first_bus.kd[0]])
        assert rows[0, 2] == pytest.approx(pose[0])
        if robot.gravity:
            assert rows[0, 4] != 0.0
        else:
            assert rows[0, 4] == 0.0
        assert native.groups[jaw.name].commands[-1] == pytest.approx(0.5)

        # The reader keeps publishing the fake feedback in declared order.
        wait_until(lambda: len(readings.sent) > 1, timeout=2)
        assert readings.sent[-1].name == [f"arm/{name}" for name in robot.joint_names]

        module.halt()
        np.testing.assert_allclose(
            native.groups[first_bus.name].commands[-1][:, 2], pose[: len(first_bus.arm)]
        )

        module.stop()
        assert native.events.count("disable") == 1
        assert module._arm is None
    finally:
        module.stop()


@pytest.mark.parametrize("connection_type,ports", ARMS)
def test_fake_feedback_fault_disables_each_arm(connection_type, ports, boundary, mocker, tmp_path):
    native = boundary[0]
    module, commands, _ = _rig(connection_type, ports, boundary, mocker, tmp_path)
    try:
        module.start()
        native.refresh_error = RuntimeError("adapter unplugged")
        commands.publish(
            JointState(
                header=message_time.header_now(),
                name=[f"arm/{connection_type.robot.arm_joint_names[0]}"],
                position=np.asarray([0.0], dtype=np.float64),
                velocity=np.asarray([], dtype=np.float64),
                effort=np.asarray([], dtype=np.float64),
            )
        )
        wait_until(lambda: module._fault is not None, timeout=2)
        assert "feedback failed" in module._fault
        assert "disable" in native.events
        sent = sum(len(getattr(group, "commands", [])) for group in native.groups.values())
        commands.publish(
            JointState(
                header=message_time.header_now(),
                name=[f"arm/{connection_type.robot.arm_joint_names[0]}"],
                position=np.asarray([0.1], dtype=np.float64),
                velocity=np.asarray([], dtype=np.float64),
                effort=np.asarray([], dtype=np.float64),
            )
        )
        assert sum(len(getattr(group, "commands", [])) for group in native.groups.values()) == sent
    finally:
        native.refresh_error = None
        module.stop()


@pytest.mark.self_hosted
def test_teach_mode_zero_stiffness_passive_jaw_and_live_gravity(boundary, mocker, tmp_path):
    native = boundary[0]
    module, commands, readings = _rig(
        OpenYamConnection, {"can_port": "can0"}, boundary, mocker, tmp_path, teach=True
    )
    physical = _physical(OpenYamConnection)
    try:
        module.start()
        pose = _middle(physical)
        _set_pose(native, OpenYamConnection, pose)
        gripper = native.groups["gripper"]
        assert gripper.modes == ["mit"]
        assert set(gripper.mit_commands) == {(0.0, 0.0, 0.4, 0.0, 0.0)}

        # Nothing is sent by the coordinator; the reader still re-commands the
        # arm so its gravity torque follows the moved pose.
        sent_before = len(native.groups["arm"].commands)
        wait_until(lambda: len(native.groups["arm"].commands) > sent_before, timeout=2)
        rows = native.groups["arm"].commands[-1]
        np.testing.assert_equal(rows[:, 0], [0.0] * len(OPENYAM.arm_joint_names))
        np.testing.assert_equal(rows[:, 1], OPENYAM.teach_kd)
        assert np.any(rows[:, 4] != 0.0)
        # The held targets do not move without a coordinator command; only the
        # gravity torque follows the measured pose.
        np.testing.assert_allclose(rows[:, 2], 0.0)
        assert gripper.commands == []
        closed, opened = OPENYAM.gripper_travel()["joint7"]
        assert readings.sent[-1].position[-1] == pytest.approx(closed + 0.4 * (opened - closed))
        assert commands.sent == []
    finally:
        module.stop()
