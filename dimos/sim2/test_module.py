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

from itertools import pairwise
from queue import Queue

import cbor2
from dimos_lcm.control_msgs import CborCommand
from dimos_lcm.sim_msgs import CborObservation, WorldState
import pytest

from dimos.sim2.geometry.poses import poses_by_id
from dimos.sim2.physics.mujoco import MujocoBackend
from dimos.utils.testing.waiting import wait_until

pytestmark = pytest.mark.self_hosted


def test_physics_advances_without_any_controller(make_sim):
    sim = make_sim()
    start = sim.inspect().header.time_ns
    wait_until(lambda: sim.status()["time_ns"] >= start + 100_000_000, timeout=3)
    assert sim.status()["status"] == "running"
    assert sim.status()["accepted_commands"] == 0
    assert sim.status()["epoch"] == 0


def test_delayed_valid_action_applies_without_rewinding_or_waiting(make_sim, command):
    sim = make_sim(joint_ids=("left/slide_x",))
    sample = sim.inspect()
    wait_until(lambda: sim.status()["time_ns"] >= sample.header.time_ns + 100_000_000, timeout=3)
    sim.submit(command(sample.header, positions=(0.3,)))
    wait_until(lambda: sim.inspect().joint_truth.position[0] > 0.1, timeout=3)
    assert sim.status()["accepted_commands"] == 1
    assert sim.status()["late_commands"] == 1
    assert sim.status()["rejected_commands"] == 0


def test_nondivisible_export_rate_has_no_rounding_drift(make_sim):
    stamps = Queue()
    make_sim(on_observations=lambda sample: stamps.put(sample.header.time_ns), observation_hz=200)
    values = [stamps.get(timeout=3) for _ in range(8)]
    assert all(t % 2_000_000 == 0 for t in values)
    assert [b - a for a, b in pairwise(values)] in (
        [6_000_000, 4_000_000] * 3 + [6_000_000],
        [4_000_000, 6_000_000] * 3 + [4_000_000],
    )


def test_wrong_clock_and_duplicate_commands_are_rejected(make_sim, command):
    sim = make_sim()
    header = sim.inspect().header
    wrong = command(header)
    wrong.header.clock_id = "other-simulation"
    good = command(header)
    sim.submit(wrong)
    sim.submit(good)
    sim.submit(good)
    wait_until(lambda: sim.status()["rejected_commands"] == 2, timeout=3)
    assert sim.status()["accepted_commands"] == 1
    assert sim.status()["error"] == ""


def test_future_action_waits_for_its_time_without_stalling_physics(make_sim, command):
    sim = make_sim(joint_ids=("left/slide_x",))
    initial = sim.inspect()
    action = command(initial.header, positions=(0.3,))
    action.apply_at_ns += 500_000_000
    sim.submit(action)
    wait_until(lambda: sim.status()["time_ns"] >= initial.header.time_ns + 100_000_000, timeout=3)
    before = sim.inspect()
    assert before.header.time_ns < action.apply_at_ns
    assert before.joint_truth.position[0] == pytest.approx(0)
    wait_until(lambda: sim.status()["accepted_commands"] == 1, timeout=3)
    assert sim.inspect().header.time_ns >= action.apply_at_ns
    wait_until(lambda: sim.inspect().joint_truth.position[0] > 0.1, timeout=3)


def test_command_expiry_holds_motor_while_physics_continues(make_sim, command):
    sim = make_sim(joint_ids=("left/slide_x",))
    action = command(sim.inspect().header, positions=(0.6,), ttl=0.1)
    # Give RPC delivery time without changing the 100 ms active command window.
    action.apply_at_ns += 500_000_000
    action.valid_until_ns += 500_000_000
    sim.submit(action)
    wait_until(lambda: sim.status()["expired_targets"] == 1, timeout=3)
    # Holding changes the PD target, not the joint's instantaneous velocity.
    # Observe a settled hold over a simulation-time interval, independent of RPC latency.
    wait_until(lambda: sim.status()["time_ns"] >= action.valid_until_ns + 1_000_000_000, timeout=3)
    state = sim.inspect()
    assert 0 < state.joint_truth.position[0] < 0.55
    assert abs(state.joint_truth.velocity[0]) < 0.05
    wait_until(lambda: sim.status()["time_ns"] >= state.header.time_ns + 200_000_000, timeout=3)
    held = sim.inspect()
    assert held.joint_truth.position == pytest.approx(state.joint_truth.position, abs=0.005)
    assert abs(held.joint_truth.velocity[0]) < 0.05
    assert sim.status()["status"] == "running"


def test_new_run_rejects_previous_commands(make_sim, command):
    first = make_sim()
    old = command(first.inspect().header)
    first.stop()
    second = make_sim()
    assert second.inspect().header.clock_id != old.header.clock_id
    second.submit(old)
    wait_until(lambda: second.status()["rejected_commands"] == 1, timeout=3)
    assert second.status()["accepted_commands"] == 0


def test_place_is_atomic_and_does_not_restart_time(make_sim):
    sim = make_sim()
    before = sim.inspect()
    with pytest.raises(ValueError, match="free root"):
        sim.place("left/tool", (0.2, 0.2, 0.4, 0, 0, 0, 1))
    placed = sim.place("box", (0.2, 0.2, 0.4, 0, 0, 0, 1))
    position = poses_by_id(placed)["box"].position
    assert (position.x, position.y, position.z) == pytest.approx((0.2, 0.2, 0.4))
    assert placed.header.epoch == before.header.epoch
    assert placed.header.time_ns >= before.header.time_ns


class ExtendedMujocoBackend(MujocoBackend):
    """Optional truth disappears when the pusher leaves its starting position."""

    def read_cbor_observations(self):
        if self.data.qpos[self.model.joint("left/slide_x").qposadr[0]] > 0.01:
            return []
        value = cbor2.dumps({"temperature_c": 21.5})
        return [
            CborObservation(entity_id="box", key="thermal", value_length=len(value), value=value)
        ]


def test_complete_state_replaces_extensions_and_keeps_one_acquisition(make_sim, command):
    sim = make_sim(backend_type=ExtendedMujocoBackend, observation_hz=50)
    first = sim.inspect()
    assert isinstance(first, WorldState)
    assert cbor2.loads(bytes(first.cbor_observations[0].value)) == {"temperature_c": 21.5}
    sim.submit(command(first.header))
    wait_until(lambda: sim.inspect().cbor_observations_length == 0, timeout=3)
    second = sim.inspect()
    assert second.cbor_observations == []
    assert second.model_revision == first.model_revision
    stamp = second.joint_truth.header.stamp
    assert stamp.sec * 10**9 + stamp.nsec == second.header.time_ns
    assert {p.body_id for p in second.bodies} == {p.body_id for p in first.bodies}
    joints = second.joint_truth
    q = joints.position[joints.name.index("left/slide_x")]
    assert poses_by_id(second)["left/carriage"].position.x == pytest.approx(-0.4 + q)


class CommandMujocoBackend(MujocoBackend):
    """A real physics backend with one test-only CBOR operation."""

    def reset(self):
        super().reset()
        self.count = 0

    def apply_action(self, joint_commands, cbor_commands):
        increments = []
        for command in cbor_commands:
            if (command.entity_id, command.key) != ("world", "increment"):
                raise ValueError("unsupported test operation")
            value = cbor2.loads(bytes(command.value))
            if type(value) is not int:
                raise ValueError("increment requires an integer")
            increments.append(value)
        super().apply_action(joint_commands, [])
        self.count += sum(increments)

    def read_cbor_observations(self):
        value = cbor2.dumps(self.count)
        return [
            CborObservation(entity_id="world", key="count", value_length=len(value), value=value)
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
            CborCommand(entity_id="world", key="increment", value_length=len(value), value=value)
        ]
        action.cbor_commands_length = 1
        return action

    return make


def test_cbor_stream_executes_once_while_physics_keeps_advancing(make_sim, cbor_action):
    sim, actions = make_sim(backend_type=CommandMujocoBackend, with_actions=True)
    first = sim.inspect()
    action = cbor_action(first.header)
    actions.publish(action)
    wait_until(lambda: sim.status()["accepted_commands"] == 1, timeout=3)
    actions.publish(action)
    wait_until(lambda: sim.status()["rejected_commands"] == 1, timeout=3)
    state = sim.inspect()
    assert state.header.time_ns > first.header.time_ns
    assert cbor2.loads(bytes(state.cbor_observations[0].value)) == 1


@pytest.mark.parametrize("invalid", ["key", "payload", "joint", "bytes", "empty", "expired"])
def test_invalid_action_has_no_partial_effect(make_sim, cbor_action, invalid):
    sim = make_sim(backend_type=CommandMujocoBackend)
    action = cbor_action(sim.inspect().header, with_joints=True)
    if invalid == "key":
        action.cbor_commands[0].key = "unknown"
    elif invalid == "payload":
        action.cbor_commands[0].value = cbor2.dumps("wrong type")
        action.cbor_commands[0].value_length = len(action.cbor_commands[0].value)
    elif invalid == "joint":
        action.joint_commands[0].values[0] = 999
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
    assert cbor2.loads(bytes(rejected.cbor_observations[0].value)) == 0
    accepted = cbor_action(rejected.header, with_joints=True)
    accepted.joint_commands[0].values[0] = 0
    sim.submit(accepted)
    wait_until(lambda: sim.status()["accepted_commands"] == 1, timeout=3)
    after = sim.inspect()
    assert cbor2.loads(bytes(after.cbor_observations[0].value)) == 1
    joints = after.joint_truth
    assert joints.position[joints.name.index("left/slide_x")] == pytest.approx(0)
