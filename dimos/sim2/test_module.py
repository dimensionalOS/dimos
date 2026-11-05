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
from dimos_lcm.sim_msgs import CborObservation, WorldState
import pytest

from dimos.sim2.geometry.poses import poses_by_id
from dimos.sim2.module import ControllerConfig, SimModule
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
        if sample.header.time_ns > 0:
            stamps.put(sample.header.time_ns)

    sim = make_sim(
        controllers=(ControllerConfig("test", 20_000_000, ("left/slide_x",)),),
        on_observations=receive,
        observation_hz=200,
    )
    baseline = sim.inspect()
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


def test_command_expiry_holds_motor_and_new_run_rejects_previous_commands(make_sim, command):
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
    sim.stop()
    restarted = make_sim(
        controllers=(ControllerConfig("test", 1_220_000_000, ("right/slide_x",)),),
    )
    baseline = restarted.inspect()
    assert baseline.header.clock_id != header.clock_id
    restarted.submit(stale)
    wait_until(lambda: restarted.status()["rejected_commands"] == 1, timeout=3)
    assert baseline.header.epoch == baseline.header.time_ns == 0
    assert baseline.joint_truth.header.stamp.sec == 0
    joints = restarted.inspect().joint_truth
    assert joints.position[joints.name.index("right/slide_x")] == pytest.approx(0)


def test_place_is_atomic_and_does_not_restart_time(controlled, command):
    controlled.submit(command(controlled.inspect().header))
    wait_until(lambda: controlled.status()["time_ns"] == 20_000_000, timeout=3)
    before = controlled.inspect()
    with pytest.raises(ValueError, match="free root"):
        controlled.place("left/tool", (0.2, 0.2, 0.4, 0, 0, 0, 1))
    assert controlled.inspect().header.epoch == before.header.epoch
    placed = controlled.place("box", (0.2, 0.2, 0.4, 0, 0, 0, 1))
    position = poses_by_id(placed)["box"].position
    assert (position.x, position.y, position.z) == pytest.approx((0.2, 0.2, 0.4))
    assert placed.header.epoch == before.header.epoch
    assert placed.header.time_ns == before.header.time_ns


class ExtendedMujocoBackend(MujocoBackend):
    """Test a backend-owned optional property through the real module/transport."""

    def read_cbor_observations(self):
        if self.data.time > 0:
            return []
        value = cbor2.dumps({"temperature_c": 21.5})
        return [
            CborObservation(
                entity_id="box",
                key="thermal",
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
        on_observations=samples.put,
    )
    first = samples.get(timeout=5)
    assert isinstance(first, WorldState)
    assert first.header.time_ns == 0
    assert cbor2.loads(bytes(first.cbor_observations[0].value)) == {"temperature_c": 21.5}
    sim.submit(command(first.header))
    while (second := samples.get(timeout=5)).header.time_ns == 0:
        pass
    assert second.header.time_ns == 20_000_000
    assert second.cbor_observations_length == 0 and second.cbor_observations == []
    assert second.model_revision == first.model_revision
    assert first.joint_truth.header.stamp.sec == 0
    assert first.joint_truth.header.stamp.nsec == first.header.time_ns
    assert second.joint_truth.header.stamp.nsec == second.header.time_ns
    assert {p.body_id for p in second.bodies} == {p.body_id for p in first.bodies}
    assert "left/imu" in {b.body_id for b in second.bodies}
    joints = second.joint_truth
    q = joints.position[joints.name.index("left/slide_x")]
    assert poses_by_id(second)["left/carriage"].position.x == pytest.approx(-0.4 + q)


class CommandMujocoBackend(MujocoBackend):
    """A real physics backend with one test-only CBOR operation and observed property."""

    def reset(self):
        super().reset()
        self.count = 0

    def apply_action(self, joint_commands, cbor_commands):
        increments = []
        for command in cbor_commands:
            if (command.entity_id, command.key) != (
                "world",
                "increment",
            ):
                raise ValueError("unsupported test operation")
            value = cbor2.loads(bytes(command.value))
            if type(value) is not int:
                raise ValueError("increment requires an integer")
            increments.append(value)
        # Both CBOR semantics and joint targets must validate before any effect.
        super().apply_action(joint_commands, [])
        self.count += sum(increments)

    def read_cbor_observations(self):
        value = cbor2.dumps(self.count)
        return [
            CborObservation(
                entity_id="world",
                key="count",
                value_length=len(value),
                value=value,
            )
        ]


@pytest.fixture
def cbor_action(command):
    def make(header, *, with_joints=False):
        action = command(header)
        if not with_joints:
            action.joint_commands = []
            action.joint_commands_length = 0
        value = cbor2.dumps(1)
        action.cbor_commands = [
            CborCommand(
                entity_id="world",
                key="increment",
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
    wait_until(lambda: cbor2.loads(bytes(sim.inspect().cbor_observations[0].value)) == 1, timeout=3)
    actions.publish(action)
    wait_until(lambda: sim.status()["rejected_commands"] == 1, timeout=3)
    held = sim.inspect()
    assert held.header.time_ns == 0
    assert cbor2.loads(bytes(held.cbor_observations[0].value)) == 1
    motors = command(held.header)
    motors.header.sequence += 1
    actions.publish(motors)
    wait_until(lambda: sim.status()["time_ns"] == 20_000_000, timeout=3)
    assert sim.status()["error"] == ""


@pytest.mark.parametrize("invalid", ["key", "payload", "joint", "bytes", "empty", "expired"])
def test_invalid_action_has_no_partial_effect(make_sim, cbor_action, invalid):
    sim = make_sim(
        backend_type=CommandMujocoBackend,
        controllers=(ControllerConfig("test", 20_000_000, ("left/slide_x",)),),
    )
    action = cbor_action(sim.inspect().header, with_joints=True)
    if invalid == "key":
        action.cbor_commands[0].key = "unknown"
    elif invalid == "payload":
        action.cbor_commands[0].value = cbor2.dumps("wrong type")
        action.cbor_commands[0].value_length = len(action.cbor_commands[0].value)
    elif invalid == "joint":
        action.joint_commands[0].position = 999
    elif invalid == "bytes":
        action.cbor_commands[0].value = b"\x01\x02"
        action.cbor_commands[0].value_length = 2
    elif invalid == "empty":
        action.joint_commands = action.cbor_commands = []
        action.joint_commands_length = action.cbor_commands_length = 0
    else:
        action.valid_until_ns = 0
    sim.submit(action)
    wait_until(lambda: sim.status()["rejected_commands"] == 1, timeout=3)
    rejected = sim.inspect()
    assert rejected.header.time_ns == 0
    assert cbor2.loads(bytes(rejected.cbor_observations[0].value)) == 0
    # Rejection consumes no sequence and changes no motor settings: a valid
    # replacement at the same boundary is accepted and advances physics.
    accepted = cbor_action(rejected.header, with_joints=True)
    accepted.joint_commands[0].position = 0
    sim.submit(accepted)
    wait_until(lambda: sim.status()["time_ns"] == 20_000_000, timeout=3)
    after = sim.inspect()
    assert cbor2.loads(bytes(after.cbor_observations[0].value)) == 1
    joints = after.joint_truth
    assert joints.position[joints.name.index("left/slide_x")] == pytest.approx(0)


@pytest.mark.parametrize("reply_delay", [0.008, 0.080])
def test_controller_reply_time_is_inside_the_wall_clock_budget(world, command, mocker, reply_delay):
    # An exact virtual wall clock makes the pacing assertion independent of host load.
    clock = mocker.patch("dimos.sim2.module.time")
    clock.monotonic.return_value = 0.0
    sim = SimModule(
        scene=world,
        backend=MujocoBackend,
        controllers=(ControllerConfig("test", 20_000_000, ("left/slide_x",)),),
    )
    reply_at = {}
    answered = set()

    def wait(seconds):
        now = clock.monotonic.return_value + seconds
        clock.monotonic.return_value = now
        sample = sim._last_state
        if sample is not None and sample.header.time_ns % 20_000_000 == 0:
            stamp = sample.header.time_ns
            due = reply_at.setdefault(stamp, now + reply_delay)
            if stamp not in answered and now >= due:
                sim.submit(command(sample.header))
                answered.add(stamp)
        if sim.status()["time_ns"] >= 100_000_000:
            sim._done.set()
        return sim._done.is_set()

    mocker.patch.object(sim._done, "wait", side_effect=wait)
    try:
        sim._run()
        assert sim.status()["error"] == ""
        assert sim.status()["time_ns"] == 100_000_000
        # Short replies consume existing slack. Long replies slow the run; no
        # missing command is skipped to pretend the clock kept up.
        expected = 0.1 if reply_delay < 0.02 else 5 * (reply_delay + 0.02)
        assert clock.monotonic.return_value == pytest.approx(expected, abs=0.015)
        assert len(answered) == 5
    finally:
        sim.dispose()
