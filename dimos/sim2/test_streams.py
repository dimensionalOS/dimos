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

from dataclasses import replace
import time

import numpy as np
import pytest

from dimos.msgs.sensor_msgs.MotorCommandArray import MotorCommandArray
from dimos.sim2.messages import MotorFrame


def test_out_of_order_command_does_not_replace_newer_target(worlds, clients):
    world = worlds()
    client = clients(world, writable=True)
    channel = world.robots["left"].channel
    client.write_positions([0.2, 850], observed=client.read())
    first = channel.read_action()
    sequence = client.write_positions([0.4, 850], observed=client.read())
    channel._accept_command(first)
    world.step()
    assert world.robots["left"].applied_action_sequence == sequence
    assert channel.read_action().values["position"][0] == 0.4


def test_old_owner_cannot_command_after_replacement(worlds, clients):
    world = worlds()
    first = clients(world, writable=True)
    first.write_positions([0.2, 850], observed=first.read())
    channel = world.robots["left"].channel
    delayed = channel.read_action()
    first.close()
    second = clients(world, writable=True)
    sequence = second.write_positions([0.4, 850], observed=second.read())
    channel._accept_command(replace(delayed, metadata=replace(delayed.metadata, sequence=999)))
    world.step()
    assert world.robots["left"].applied_action_sequence == sequence
    assert channel.read_action().values["position"][0] == 0.4


@pytest.mark.parametrize(
    "field,value", [("kp", [-1, 0]), ("position", [float("nan"), 0]), ("velocity", [0])]
)
def test_physics_rejects_malformed_targets_at_its_boundary(worlds, clients, field, value):
    world = worlds()
    client = clients(world, writable=True)
    client.write_positions([0.2, 850], observed=client.read())
    channel = world.robots["left"].channel
    valid = channel.read_action()
    bad = MotorFrame(
        replace(valid.metadata, sequence=99),
        {**valid.values, field: np.array(value)},
        valid.writer,
        valid.source_ts,
    )
    channel._accept_command(bad)
    assert channel.read_action().metadata.sequence == valid.metadata.sequence


def test_snapshot_history_is_bounded_and_cleared_at_reset(worlds):
    world = worlds()
    for _ in range(50):
        world.step()
    frames = world.snapshots.read_observations()
    assert len(frames) <= world.snapshot_descriptor.observation_slots
    world.reset()
    assert {frame.metadata.episode_id for frame in world.snapshots.read_observations()} == {
        world.generation
    }


def test_dead_writer_lease_expires_without_os_cleanup(worlds, clients, mocker):
    world = worlds()
    first = clients(world, writable=True)
    first.write_motors(MotorCommandArray(q=[0.1, 850]), observed=first.read())
    channel = world.robots["left"].channel
    old = channel.read_action()
    mocker.patch("dimos.sim2.streams.time.monotonic", return_value=time.monotonic() + 1)
    world.step()
    second = clients(world, writable=True)
    second.write_positions([0.4, 850], observed=second.read())
    assert channel.read_action().writer != old.writer
