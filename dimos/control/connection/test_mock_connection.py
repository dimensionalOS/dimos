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

"""Tests for ``MockConnection``: a real driver module, end to end.

These go through the module's own calls and ports, so they also check that the
lifecycle calls added by ``ConnectionRpcMixin`` are found by the module's call
machinery, and that the module fits the published ``ConnectionControlSpec``.
The ports are given an in-process transport; nothing touches the network.
"""

from __future__ import annotations

import pytest

from dimos.control.connection.conftest import LocalTransport
from dimos.control.connection.mock_connection import ARM_JOINTS, MockConnection
from dimos.control.connection.status import LifecycleState
from dimos.msgs.control_msgs.ControlValues import ControlValues
from dimos.spec.control import ConnectionControlSpec
from dimos.spec.utils import spec_annotation_compliance

LIFECYCLE_CALLS = {
    "describe_control",
    "status",
    "prepare_arm",
    "commit_arm",
    "abort_arm",
    "safe_stop",
    "clear_safe_stop",
    "estop",
    "clear_estop",
}


@pytest.fixture
def mock_for():
    made = []

    def build(shape, source="mock"):
        module = MockConnection(shape=shape, source=source, background=False)
        state, command = LocalTransport(), LocalTransport()
        module.set_transport("control_state", state)
        module.set_transport("control_command", command)
        module.start()
        made.append(module)
        return module, state, command

    yield build
    for module in made:
        module.stop()


def command(epoch, sequence, values):
    return ControlValues(
        source="coordinator",
        epoch=epoch,
        sequence=sequence,
        interface_names=list(values),
        values=list(values.values()),
    )


def arm_it(module, epoch=1):
    module.hw.poll_state()
    [prepared] = module.prepare_arm(1)
    module.hw.poll_state()
    [committed] = module.commit_arm(2, epoch=epoch)
    assert prepared.ok and committed.ok, (prepared, committed)


def test_the_lifecycle_calls_on_the_mixin_are_found():
    assert LIFECYCLE_CALLS <= set(MockConnection.rpcs)


def test_a_driver_module_fits_the_published_spec():
    assert spec_annotation_compliance(MockConnection, ConnectionControlSpec)


def test_the_mixin_brings_the_ports(mock_for):
    module, state, _ = mock_for("arm")
    module.hw.poll_state()
    assert state.sent[-1].source == "mock"


def test_an_arm_goes_where_it_is_told_and_stops_on_request(mock_for):
    module, state, bus = mock_for("arm", source="arm")
    [desc] = module.describe_control()
    assert desc.source == "arm"
    arm_it(module)

    targets = {f"arm/{j}/position": 0.1 * i for i, j in enumerate(ARM_JOINTS)}
    targets["arm/gripper/position"] = 0.05
    bus.publish(command(1, 1, targets))
    module.hw.poll_state()
    reading = state.sent[-1].as_dict()
    assert reading["arm/joint4/position"] == pytest.approx(0.3)
    assert reading["arm/gripper/position"] == 0.05

    [ack] = module.safe_stop(3)
    assert ack.ok and ack.state is LifecycleState.SAFE_STOPPED
    [status] = module.status()
    assert status.state is LifecycleState.SAFE_STOPPED and status.fault == "requested"
    # The command, then the hold that stopped it.
    assert module.commands_received() == 2


def test_an_arm_told_a_speed_keeps_moving_at_it(mock_for):
    module, state, bus = mock_for("arm", source="arm")
    arm_it(module)
    speeds = {f"arm/{j}/velocity": 0.0 for j in ARM_JOINTS} | {"arm/joint1/velocity": 1.0}
    speeds["arm/gripper/position"] = 0.0
    bus.publish(command(1, 1, speeds))
    for _ in range(10):
        module.hw.poll_state()
    # Ten steps at 100 readings a second.
    assert state.sent[-1].as_dict()["arm/joint1/position"] == pytest.approx(0.1)


def test_a_base_moves_as_its_speeds_say(mock_for):
    module, state, bus = mock_for("base", source="base")
    arm_it(module)
    bus.publish(command(1, 1, {"base/base/vx": 1.0, "base/base/vy": 0.0, "base/base/wz": 0.0}))
    for _ in range(25):
        module.hw.poll_state()
    reading = state.sent[-1].as_dict()
    # Half a second at 50 readings a second.
    assert reading["base/base/x"] == pytest.approx(0.5)
    assert reading["base/base/vx"] == 1.0


def test_a_humanoid_accepts_a_full_command_and_damps_when_stopped(mock_for):
    module, _, bus = mock_for("pd", source="g1")
    [desc] = module.describe_control()
    arm_it(module)
    bus.publish(command(1, 1, dict.fromkeys(desc.command_keys(), 0.0)))
    [ack] = module.safe_stop(3)
    assert ack.ok
    [status] = module.status()
    assert status.rejections == ()
    assert module.commands_received() == 2


def test_a_damped_joint_stays_where_it_is_pushed(mock_for):
    module, state, bus = mock_for("pd", source="g1")
    [desc] = module.describe_control()
    arm_it(module)
    values = dict.fromkeys(desc.command_keys(), 0.0)
    values |= {key: 60.0 for key in values if key.endswith("/kp")}
    bus.publish(command(1, 1, values))
    module.safe_stop(3)
    module.set_measured({"g1/joint1/position": 0.5})
    module.hw.poll_state()
    # No stiffness while damping, so nothing pulls it back to its old target.
    assert state.sent[-1].as_dict()["g1/joint1/position"] == 0.5


def test_a_reported_fault_stops_it(mock_for):
    module, _, _ = mock_for("arm")
    arm_it(module)
    module.inject_fault("motor too hot")
    module.hw.supervise()
    [status] = module.status()
    assert status.state is LifecycleState.SAFE_STOPPED and status.fault == "motor too hot"


def test_set_measured_moves_the_readings(mock_for):
    module, state, _ = mock_for("arm", source="arm")
    module.set_measured({"arm/joint2/position": 1.25})
    module.hw.poll_state()
    assert state.sent[-1].as_dict()["arm/joint2/position"] == 1.25


def test_a_source_it_does_not_run_gets_no_answers(mock_for):
    module, _, _ = mock_for("arm", source="arm")
    assert module.estop(1, source="chassis") == []
    [ack] = module.estop(2, source="arm")
    assert ack.state is LifecycleState.ESTOPPED


def test_stopping_the_module_stops_an_armed_robot(mock_for):
    module, _, _ = mock_for("arm")
    arm_it(module)
    hw = module.hw
    module.stop()
    assert hw.state is LifecycleState.SAFE_STOPPED
