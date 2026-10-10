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

"""Direct clients face exactly the same admission rules as the robot module."""

import json
import time

import pytest

from dimos.evals.isolated_mujoco.runtime import ActionBoundary
from dimos.protocol.mujoco_eval import Command, session_config


@pytest.fixture
def boundary(tmp_path):
    value = ActionBoundary(
        "run", "episode", [-1.0, 0.0], [1.0, 0.8], [2.0, 0.0], tmp_path / "events.jsonl"
    )
    yield value
    value.close()


def command(sequence=1, kind="enable", values=None, **overrides):
    return (
        Command.model_validate(
            dict(
                run="run",
                episode="episode",
                sequence=sequence,
                sent_at=time.time(),
                kind=kind,
                values=values or [],
                **overrides,
            )
        )
        .model_dump_json()
        .encode()
    )


def test_normal_actuator_commands_are_admitted_and_counted(boundary):
    assert boundary.receive(command())
    assert boundary.receive(command(2, "position", [0.5, 0.4]))
    assert boundary.pending.values == [0.5, 0.4]
    assert boundary.receive(command(3, "stop"))
    assert boundary.attempts == 3
    assert boundary.sequence == 3


@pytest.mark.parametrize(
    "payload",
    [
        b"not json",
        b"x" * 4097,
        b'{"kind":"reset"}',
        b'{"kind":"rpc","method":"eval"}',
        b'{"kind":"position","values":[NaN]}',
    ],
)
def test_malformed_or_admin_payload_cannot_reach_actuators(boundary, payload):
    assert not boundary.receive(payload)
    assert boundary.pending is None
    assert boundary.sequence == -1
    assert boundary.attempts == 1


@pytest.mark.parametrize(
    "mutation",
    [
        {"episode": "other"},
        {"run": "other"},
        {"sequence": 1},
        {"sent_at": 0.0},
        {"values": [2.0, 0.4]},
        {"values": [0.0]},
        {"kind": "velocity", "values": [0.0, 0.1]},
        {"kind": "disable", "values": [0.1]},
    ],
)
def test_replay_episode_expiry_and_bounds_leave_previous_command_intact(boundary, mutation):
    assert boundary.receive(command())
    before = boundary.pending
    data = dict(
        run="run",
        episode="episode",
        sequence=2,
        sent_at=time.time(),
        kind="position",
        values=[0.0, 0.4],
    )
    data.update(mutation)
    assert not boundary.receive(json.dumps(data).encode())
    assert boundary.pending == before
    assert boundary.sequence == 1


def test_disabled_actuators_and_sensor_forgery_are_rejected(boundary):
    assert not boundary.receive(command(1, "position", [0.0, 0.4]))
    assert not boundary.receive(b'{"tf":"forged","score":1}')
    assert boundary.pending is None
    assert boundary.attempts == 2


def test_zenoh_config_has_no_discovery_shm_or_admin_and_denies_other_keys():
    config = json.loads(
        str(session_config("tcp/127.0.0.1:7449", trusted=True, run="run", episode="episode"))
    )
    assert config["access_control"]["default_permission"] == "deny"
    assert config["scouting"]["multicast"]["enabled"] is False
    assert config["scouting"]["gossip"]["enabled"] is False
    assert config["adminspace"]["enabled"] is False
    assert config["transport"]["shared_memory"]["enabled"] is False
    actions = config["access_control"]["rules"][0]
    assert actions["messages"] == ["put"]
    assert actions["key_exprs"] == ["mujoco-eval/run/episode/command"]


def test_closed_episode_cannot_accept_more_commands(boundary):
    boundary.close()
    assert not boundary.receive(command())
    assert boundary.pending is None
