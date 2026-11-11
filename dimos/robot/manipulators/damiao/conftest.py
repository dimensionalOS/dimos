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
from pydantic import model_validator
import pytest
from typing_extensions import Self

from dimos.robot.manipulators.damiao import arm as arm_module
from dimos.robot.manipulators.damiao._damiao_native_test_helpers import (
    TEST_ROBOT,
    wire_hosted_connection,
)
from dimos.robot.manipulators.damiao.arm import PhysicalModel
from dimos.robot.manipulators.damiao.connection import (
    DamiaoConnection,
    DamiaoConnectionConfig,
)
from dimos.robot.manipulators.damiao.testing.fake_native import (
    Builder,
    FakeArm,
    FakeGripper,
    FakeRobot,
)
from dimos.utils.testing.waiting import wait_until


@pytest.fixture
def connection_class():
    class Config(DamiaoConnectionConfig):
        source: str = "arm"

        @model_validator(mode="after")
        def _model(self) -> Self:
            if self.model is None:
                self.model = Connection.robot.model
            return self

    class Connection(DamiaoConnection):
        config: Config
        robot = TEST_ROBOT

        def can_ports(self):
            return ["can1", "can0"]

    Connection.__annotations__["config"] = Config
    return Connection


@pytest.fixture
def make_physical():
    def make(**kwargs):
        robot = kwargs.pop("robot", TEST_ROBOT)
        return PhysicalModel(
            robot.gravity_model,
            robot.arm_joint_names,
            gravity_comp=kwargs.pop("gravity_comp", False),
            **kwargs,
        )

    return make


@pytest.fixture(params=["linux", "darwin"])
def boundary(mocker, request):
    mocker.patch.object(arm_module, "sys", SimpleNamespace(platform=request.param))
    native = FakeRobot()
    mocker.patch.object(
        can_motor_control, "Robot", SimpleNamespace(builder=lambda: Builder(native))
    )
    mocker.patch.object(can_motor_control, "Arm", FakeArm)
    mocker.patch.object(can_motor_control, "Gripper", FakeGripper)
    mocker.patch.object(can_motor_control, "MotorSpec", side_effect=lambda *args: args)
    if arm_module.sys.platform == "darwin":
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
        commands, readings = wire_hosted_connection(module)
        made.append(module)
        module.start()
        # The hosted cycle is the only reader; publish one measured sample so
        # tests that inspect telemetry have the same view as a running tick.
        module.publish_measured(module.read_measured())
        wait_until(lambda: bool(readings.sent), timeout=2)
        return module, commands, readings

    yield make
    for module in made:
        module.stop()
