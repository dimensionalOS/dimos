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

"""Opt-in fixtures for native Damiao connection tests."""

from types import SimpleNamespace

import can_motor_control
from can_motor_control import damiao
from pydantic import model_validator
import pytest
from typing_extensions import Self

from dimos.control._control_test_helpers import fake_arm_model
from dimos.manipulation.planning.groups.models import PlanningGroupDefinition
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.robot.manipulators.common import damiao_connection as common
from dimos.robot.manipulators.common._damiao_native_test_helpers import (
    Builder,
    FakeArm,
    FakeGripper,
    FakeRobot,
    LocalTransport,
)
from dimos.robot.manipulators.common.damiao_connection import (
    DamiaoConnection,
    DamiaoConnectionConfig,
)
from dimos.robot.manipulators.common.damiao_model import PhysicalModel
from dimos.utils.testing.waiting import wait_until


@pytest.fixture
def connection_class():
    class Config(DamiaoConnectionConfig):
        source: str = "arm"
        native: bool = True

        @model_validator(mode="after")
        def _model(self) -> Self:
            if self.model is None:
                self.model = Connection.control_model()
            return self

    class Connection(DamiaoConnection):
        config: Config
        bus_names = ["right", "left"]
        arm_joints = {"left_arm": ["joint1", "joint2"], "right_arm": ["joint3"]}
        gripper_joints = {"left_gripper": "left_gripper", "right_gripper": "right_gripper"}
        kinematic_joint_names = ["joint1", "joint2", "joint3"]
        kinematic_model = fake_arm_model(("joint1", "joint2", "joint3"))
        planning = RobotModelConfig(
            model=kinematic_model,
            joint_names=["joint1", "joint2", "joint3"],
            planning_groups=[
                PlanningGroupDefinition("both", ("joint1", "joint2", "joint3"), "link0")
            ],
            base_link="link0",
        )
        kp = {"left_arm": [80.0, 10.0], "right_arm": [20.0]}
        kd = {"left_arm": [5.0, 1.5], "right_arm": [0.6]}
        mock_positions = [0.2, 0.2, 0.2, 0.4, 0.4]
        gravity_comp = False

        def _can_devices(self):
            return {"left": "can1", "right": "can0"} if self.config.native else None

        def _build_robot(self):
            return (
                can_motor_control.Robot.builder()
                .add_bus("right", self._make_can_bus("right"), damiao.DamiaoCodec())
                .add_bus("left", self._make_can_bus("left"), damiao.DamiaoCodec())
                .add_arm(
                    "left_arm",
                    bus="left",
                    motors=[
                        can_motor_control.MotorSpec("joint1", damiao.MotorType.DM4340, 1, 17),
                        can_motor_control.MotorSpec("joint2", damiao.MotorType.DM4310, 2, 18),
                    ],
                )
                .add_arm(
                    "right_arm",
                    bus="right",
                    motors=[
                        can_motor_control.MotorSpec("joint3", damiao.MotorType.DM4310, 1, 17),
                    ],
                )
                .add_gripper(
                    "left_gripper",
                    bus="left",
                    motor=can_motor_control.MotorSpec(
                        "left_gripper", damiao.MotorType.DM4310, 8, 24
                    ),
                    opening_direction="decreasing_position",
                    default_current=0.15,
                )
                .add_gripper(
                    "right_gripper",
                    bus="right",
                    motor=can_motor_control.MotorSpec(
                        "right_gripper", damiao.MotorType.DM4310, 8, 24
                    ),
                    opening_direction="increasing_position",
                    default_current=0.1,
                )
                .build()
            )

    Connection.__annotations__["config"] = Config
    return Connection


@pytest.fixture
def make_physical(connection_class):
    def make(**kwargs):
        return PhysicalModel(
            connection_class.kinematic_model,
            connection_class.kinematic_joint_names,
            gravity_comp=kwargs.pop("gravity_comp", False),
            **kwargs,
        )

    return make


@pytest.fixture(params=["linux", "darwin"])
def boundary(mocker, request):
    mocker.patch.object(common, "sys", SimpleNamespace(platform=request.param))
    native = FakeRobot()
    mocker.patch.object(
        can_motor_control, "Robot", SimpleNamespace(builder=lambda: Builder(native))
    )
    mocker.patch.object(can_motor_control, "Arm", FakeArm)
    mocker.patch.object(can_motor_control, "Gripper", FakeGripper)
    mocker.patch.object(can_motor_control, "MotorSpec", side_effect=lambda *args: args)
    if common.sys.platform == "darwin":
        bus = mocker.patch.object(
            can_motor_control,
            "GsUsbBus",
            side_effect=lambda **kwargs: ("socket", kwargs["serial_number"]),
            create=True,
        )
    else:
        bus = mocker.patch.object(
            can_motor_control,
            "SocketCanBus",
            side_effect=lambda name: ("socket", name),
            create=True,
        )
    return native, bus


@pytest.fixture
def make_rig(connection_class, boundary, mocker):
    mocker.patch("dimos.protocol.rpc.zenohrpc.ZenohRPC.start")
    mocker.patch("dimos.protocol.rpc.zenohrpc.ZenohRPC.serve_module_rpc")
    made = []

    def make(**kwargs):
        module = connection_class(deadman_timeout_s=10, **kwargs)
        commands, readings = LocalTransport(), LocalTransport()
        module.set_transport("position_command", commands)
        module.set_transport("joint_state", readings)
        made.append(module)
        module.start()
        wait_until(lambda: bool(readings.sent), timeout=2)
        return module, commands, readings

    yield make
    for module in made:
        module.stop()
