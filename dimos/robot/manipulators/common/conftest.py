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

"""Opt-in fixtures for direct-native Damiao connection tests."""

from __future__ import annotations

from types import SimpleNamespace
from typing import ClassVar

import can_motor_control
from can_motor_control import damiao
from pydantic import model_validator
import pytest
from typing_extensions import Self

from dimos.control._control_test_helpers import fake_arm_model
from dimos.manipulation.planning.groups.models import PlanningGroupDefinition
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.robot.manipulators.common import damiao_connection as common, damiao_model
from dimos.robot.manipulators.common._damiao_native_test_helpers import (
    Builder,
    FakeArm,
    FakeGripper,
    FakeRobot,
    LocalTransport,
)
from dimos.robot.manipulators.common.damiao_connection import (
    ArmSpec,
    DamiaoConnection,
    DamiaoConnectionConfig,
    DamiaoRobot,
    GripperSpec,
    JointDrive,
)
from dimos.robot.manipulators.common.damiao_model import local_control_model
from dimos.utils.testing.waiting import wait_until


@pytest.fixture
def robot_spec(tmp_path, monkeypatch):
    monkeypatch.setattr(damiao_model, "DEFAULT_ROBOT_ASSET_CACHE_ROOT", tmp_path)
    arm_model = fake_arm_model(("joint1", "joint2", "joint3"))
    return DamiaoRobot(
        buses=("right", "left"),
        arms=(
            ArmSpec(
                "left_arm",
                "left",
                (
                    JointDrive("joint1", "DM4340", 1, 17, 80, 5),
                    JointDrive("joint2", "DM4310", 2, 18, 10, 1.5),
                ),
            ),
            ArmSpec("right_arm", "right", (JointDrive("joint3", "DM4310", 1, 17, 20, 0.6),)),
        ),
        grippers=(
            GripperSpec("left_gripper", "left", "DM4310", 8, 24, "decreasing_position", 0.15),
            GripperSpec("right_gripper", "right", "DM4310", 8, 24, "increasing_position", 0.1),
        ),
        physical_model=arm_model,
        planning=RobotModelConfig(
            model=arm_model,
            joint_names=["joint1", "joint2", "joint3"],
            planning_groups=[
                PlanningGroupDefinition("both", ("joint1", "joint2", "joint3"), "link0")
            ],
        ),
        mock_positions=(0.2, 0.2, 0.2, 0.4, 0.4),
        gravity_comp=False,
    )


@pytest.fixture
def connection_class(robot_spec):
    class Config(DamiaoConnectionConfig):
        source: str = "arm"
        native: bool = True

        @model_validator(mode="after")
        def _model(self) -> Self:
            if self.model is None:
                self.model = local_control_model(robot_spec)
            return self

    class Connection(DamiaoConnection):
        config: Config
        robot: ClassVar[DamiaoRobot] = robot_spec

        def _can_devices(self):
            return {"left": "can1", "right": "can0"} if self.config.native else None

    Connection.__annotations__["config"] = Config
    return Connection


@pytest.fixture(params=["linux", "darwin"])
def boundary(mocker, request):
    mocker.patch.object(common, "sys", SimpleNamespace(platform=request.param))
    native = FakeRobot()
    mocker.patch.object(common, "_native", return_value=can_motor_control)
    mocker.patch.object(common, "_codec", return_value=damiao)
    mocker.patch.object(
        can_motor_control, "Robot", SimpleNamespace(builder=lambda: Builder(native))
    )
    mocker.patch.object(can_motor_control, "Arm", FakeArm)
    mocker.patch.object(can_motor_control, "Gripper", FakeGripper)
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
