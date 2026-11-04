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

from queue import Queue

import cbor2
from dimos_lcm.control_msgs import CborCommand
from dimos_lcm.sim_msgs import CborExtension, WorldState
import pytest

from dimos.sim2.geometry.poses import poses_by_id
from dimos.sim2.module import ControllerConfig
from dimos.sim2.physics.mujoco import MujocoBackend
from dimos.utils.testing.waiting import wait_until

pytestmark = pytest.mark.native_e2e


@pytest.fixture
def controlled(make_sim):
    return make_sim(
        controllers=(ControllerConfig("test", 20_000_000, ("left/slide_x",)),),
        joint_ids=("left/slide_x",),
    )


def test_missing_controller_pauses_without_stepping_or_resetting(make_sim):
    sim = make_sim(
        controllers=(ControllerConfig("test", 20_000_000, ("left/slide_x",)),),
        startup_timeout_s=0.5,
        control_timeout_s=0.1,
    )
    wait_until(lambda: sim.status()["status"] == "fault", timeout=3)
    status = sim.status()
    assert status["time_ns"] == 0
    assert status["epoch"] == 0
    assert "deadline" in status["error"]


def test_control_uses_exact_boundary_then_waits_for_next(controlled, command):
    controlled.submit(command(controlled.inspect().header))
    wait_until(lambda: controlled.status()["time_ns"] == 20_000_000, timeout=3)
    assert controlled.status()["epoch"] == 0
    assert controlled.inspect().header.time_ns == 20_000_000


def test_nondivisible_export_rate_has_no_rounding_drift(make_sim, command):
    stamps = Queue()

    def receive(sample):
        if sample.header.epoch == 1:
            stamps.put(sample.header.time_ns)

    sim = make_sim(
        controllers=(ControllerConfig("test", 20_000_000, ("left/slide_x",)),),
        on_observations=receive,
        observation_hz=200,
    )
    baseline = sim.reset()
    # Wait for the reset sample to arrive before releasing the controller boundary.
    assert stamps.get(timeout=3) == 0
    sim.submit(command(baseline.header))
    assert [stamps.get(timeout=3) for _ in range(4)] == [
        6_000_000,
        10_000_000,
        16_000_000,
        20_000_000,
    ]


def test_wrong_clock_and_duplicate_commands_are_rejected(controlled, command):
    header = controlled.inspect().header
    wrong = command(header)
    wrong.header.clock_id = "other-simulation"
    good = command(header)
    controlled.submit(wrong)
    controlled.submit(good)
    controlled.submit(good)
    wait_until(lambda: controlled.status()["rejected_commands"] == 2, timeout=3)
    wait_until(lambda: controlled.status()["time_ns"] == 20_000_000, timeout=3)
    assert controlled.status()["error"] == ""


def test_command_expiry_holds_motor_and_reset_rejects_previous_epoch(make_sim, command):
    sim = make_sim(
        controllers=(ControllerConfig("test", 1_220_000_000, ("right/slide_x",)),),
    )
    header = sim.inspect().header
    short = command(header, positions=(0.6,), ttl=0.02)
    short.header.source_id = "uncontrolled"
    sim.submit(short)
    sim.submit(command(header, names=("right/slide_x",), positions=(0.0,), ttl=1.22))
    wait_until(lambda: sim.status()["time_ns"] == 1_220_000_000, timeout=5)
    poses = sim.inspect()
    joints = poses.joint_truth
    assert joints.position[joints.name.index("left/slide_x")] < 0.2
    stale = command(poses.header, names=("right/slide_x",), positions=(0.0,), ttl=1.22)
    reset = sim.reset()
    sim.submit(stale)
    wait_until(lambda: sim.status()["rejected_commands"] == 1, timeout=3)
    assert reset.header.epoch == header.epoch + 1
    assert reset.header.time_ns == 0
    joints = sim.inspect().joint_truth
    assert joints.position[joints.name.index("right/slide_x")] == pytest.approx(0)


def test_place_is_atomic_and_invalidates_old_samples(controlled):
    before = controlled.inspect()
    with pytest.raises(ValueError, match="free root"):
        controlled.place("left/tool", (0.2, 0.2, 0.4, 0, 0, 0, 1))
    assert controlled.inspect().header.epoch == before.header.epoch
    placed = controlled.place("box", (0.2, 0.2, 0.4, 0, 0, 0, 1))
    position = poses_by_id(placed)["box"].position
    assert (position.x, position.y, position.z) == pytest.approx((0.2, 0.2, 0.4))
    assert placed.header.epoch == before.header.epoch + 1
    assert placed.header.time_ns == 0


class ExtendedMujocoBackend(MujocoBackend):
    """Test a backend-owned optional property through the real module/transport."""

    def read_extensions(self):
        if self.data.time > 0:
            return []
        value = cbor2.dumps({"temperature_c": 21.5})
        return [
            CborExtension(
                entity_id="box",
                key="thermal",
                schema="example.thermal.v1",
                value_length=len(value),
                value=value,
            )
        ]


def test_complete_state_replaces_extensions_and_keeps_one_acquisition(make_sim, command):
    samples = Queue()
    sim = make_sim(
        backend_type=ExtendedMujocoBackend,
        controllers=(ControllerConfig("test", 20_000_000, ("left/slide_x",)),),
        observation_hz=50,
        motion_frames=("left/imu",),
        on_observations=samples.put,
    )
    first = samples.get(timeout=5)
    assert isinstance(first, WorldState)
    assert first.header.time_ns == 0
    assert cbor2.loads(bytes(first.extensions[0].value)) == {"temperature_c": 21.5}
    sim.submit(command(first.header))
    while (second := samples.get(timeout=5)).header.time_ns == 0:
        pass
    assert second.header.time_ns == 20_000_000
    assert second.extensions_length == 0 and second.extensions == []
    assert second.model_revision == first.model_revision
    assert {p.body_id for p in second.body_poses} == {p.body_id for p in first.body_poses}
    assert second.body_motion[0].frame_id == "left/imu"
    joints = second.joint_truth
    q = joints.position[joints.name.index("left/slide_x")]
    assert poses_by_id(second)["left/carriage"].position.x == pytest.approx(-0.4 + q)


class CommandMujocoBackend(MujocoBackend):
    """A real physics backend with one test-only CBOR operation and observed property."""

    def reset(self):
        super().reset()
        self.count = 0

    def apply_action(self, joint_targets, cbor_commands):
        increments = []
        for command in cbor_commands:
            if (command.entity_id, command.key, command.schema) != (
                "world",
                "increment",
                "example.increment.v1",
            ):
                raise ValueError("unsupported test operation")
            value = cbor2.loads(bytes(command.value))
            if type(value) is not int:
                raise ValueError("increment requires an integer")
            increments.append(value)
        # Both CBOR semantics and joint targets must validate before any effect.
        super().apply_action(joint_targets, [])
        self.count += sum(increments)

    def read_extensions(self):
        value = cbor2.dumps(self.count)
        return [
            CborExtension(
                entity_id="world",
                key="count",
                schema="example.count.v1",
                value_length=len(value),
                value=value,
            )
        ]


@pytest.fixture
def cbor_action(command):
    def make(header, *, with_joints=False):
        action = command(header)
        if not with_joints:
            action.joint_targets = []
            action.joint_targets_length = 0
        value = cbor2.dumps(1)
        action.cbor_commands = [
            CborCommand(
                entity_id="world",
                key="increment",
                schema="example.increment.v1",
                value_length=len(value),
                value=value,
            )
        ]
        action.cbor_commands_length = 1
        return action

    return make


def test_cbor_stream_executes_once_and_cannot_release_motor_boundary(
    make_sim, command, cbor_action
):
    sim, actions = make_sim(
        backend_type=CommandMujocoBackend,
        with_actions=True,
        controllers=(ControllerConfig("test", 20_000_000, ("left/slide_x",)),),
    )
    first = sim.inspect()
    action = cbor_action(first.header)
    actions.publish(action)
    wait_until(lambda: cbor2.loads(bytes(sim.inspect().extensions[0].value)) == 1, timeout=3)
    actions.publish(action)
    wait_until(lambda: sim.status()["rejected_commands"] == 1, timeout=3)
    held = sim.inspect()
    assert held.header.time_ns == 0
    assert cbor2.loads(bytes(held.extensions[0].value)) == 1
    motors = command(held.header)
    motors.header.sequence += 1
    actions.publish(motors)
    wait_until(lambda: sim.status()["time_ns"] == 20_000_000, timeout=3)
    assert sim.status()["error"] == ""
    assert cbor2.loads(bytes(sim.reset().extensions[0].value)) == 0


@pytest.mark.parametrize("invalid", ["schema", "joint", "bytes", "empty", "expired"])
def test_invalid_action_has_no_partial_effect(make_sim, cbor_action, invalid):
    sim = make_sim(
        backend_type=CommandMujocoBackend,
        controllers=(ControllerConfig("test", 20_000_000, ("left/slide_x",)),),
    )
    action = cbor_action(sim.inspect().header, with_joints=True)
    if invalid == "schema":
        action.cbor_commands[0].schema = "unknown.v1"
    elif invalid == "joint":
        action.joint_targets[0].position = 999
    elif invalid == "bytes":
        action.cbor_commands[0].value = b"\x01\x02"
        action.cbor_commands[0].value_length = 2
    elif invalid == "empty":
        action.joint_targets = action.cbor_commands = []
        action.joint_targets_length = action.cbor_commands_length = 0
    else:
        action.valid_until_ns = 0
    sim.submit(action)
    wait_until(lambda: sim.status()["rejected_commands"] == 1, timeout=3)
    rejected = sim.inspect()
    assert rejected.header.time_ns == 0
    assert cbor2.loads(bytes(rejected.extensions[0].value)) == 0
    # Rejection consumes no sequence and changes no motor settings: a valid
    # replacement at the same boundary is accepted and advances physics.
    accepted = cbor_action(rejected.header, with_joints=True)
    accepted.joint_targets[0].position = 0
    sim.submit(accepted)
    wait_until(lambda: sim.status()["time_ns"] == 20_000_000, timeout=3)
    after = sim.inspect()
    assert cbor2.loads(bytes(after.extensions[0].value)) == 1
    joints = after.joint_truth
    assert joints.position[joints.name.index("left/slide_x")] == pytest.approx(0)
