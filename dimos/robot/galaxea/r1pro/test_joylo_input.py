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

"""Recovery through the actual OpenArm reader configuration and JoyLo guard."""

import math

import pytest

from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.robot.galaxea.r1pro.blueprints.teleop.r1pro_joylo import r1pro_openarm_joylo_sim2
from dimos.robot.galaxea.r1pro.joylo import ARMS, JAWS, UPPER, JoyloConfig, JoyloController
from dimos.robot.galaxea.r1pro.leader import R1ProLeaderConnection
from dimos.teleop.joycon.mapping import JoyConReading


@pytest.fixture(scope="module")
def parsed(tmp_path_factory):
    # One attached leader and one left Joy-Con are launch options, not a blueprint.
    return BlueprintConfigParser(r1pro_openarm_joylo_sim2).parse(
        [
            '--r1proleaderconnection.enabled-sides=["left"]',
            '--r1projoylo.enabled-sides=["left"]',
            "--r1projoylo.arm-only=true",
            f"--recorder.recording={tmp_path_factory.mktemp('session') / 'take'}",
            "--episodes.task=check the left leader",
        ],
        environ={},
    )


@pytest.fixture
def pipeline(mocker, parsed):
    kwargs = parsed.module_kwargs("r1proleaderconnection")
    reader = R1ProLeaderConnection(**{**kwargs, "mock_positions": {}})
    try:
        reader.connect()
        read = mocker.Mock(return_value=dict.fromkeys((f"joint_{i}" for i in range(1, 8)), 0.2))
        mocker.patch.object(reader._arm, "read", side_effect=lambda: {"left": read()})
        mocker.patch.object(reader.joint_state, "publish")
        profile = JoyloController(
            dict.fromkeys(UPPER, (-2.0, 2.0)),
            JoyloConfig(**parsed.module_kwargs("r1projoylo")),
        )
        yield reader, profile, read
    finally:
        reader.stop()


def tick(reader, profile, now, pressed=()):
    sample = None
    try:
        readings = reader._arm.read()
    except OSError:
        pass  # The connection skips failed serial reads; JoyLo expires the last sample.
    else:
        reader.publish_reading(readings)
        sample = reader.joint_state.publish.call_args.args[0]
        profile.set_leader(dict(zip(sample.name, sample.position, strict=True)), now)
    profile.set_feedback({**dict.fromkeys(UPPER, 0.2), **dict.fromkeys(JAWS, 0.05)}, now)
    profile.set_joycon(JoyConReading(frozenset(pressed)), None, now)
    return sample, profile.step(now, 0.02)


def engage(reader, profile):
    tick(reader, profile, 0)
    tick(reader, profile, 0.02, ("minus",))
    tick(reader, profile, 0.04)
    assert profile.active


def test_read_loss_then_large_operator_movement_resumes_same_absolute_mapping_with_minus(pipeline):
    reader, profile, read = pipeline
    engage(reader, profile)
    read.side_effect = OSError("serial read timed out")
    sample, stopped = tick(reader, profile, 0.4, ("minus",))
    assert sample is None and not profile.active
    assert stopped.positions == dict.fromkeys(UPPER, 0.2)

    read.side_effect = None
    read.return_value = {**read.return_value, "joint_1": 1.2}
    sample, held = tick(reader, profile, 0.42, ("minus",))
    assert sample is not None  # A moved leader must not remain permanently rejected.
    assert profile.leader[ARMS[0]] == pytest.approx(1.2)
    assert not profile.active and held.positions == stopped.positions
    tick(reader, profile, 0.44, ("minus",))
    assert not profile.active
    tick(reader, profile, 0.46)
    _, resumed = tick(reader, profile, 0.48, ("minus",))
    assert profile.active
    assert resumed.positions[ARMS[0]] == pytest.approx(1.2)

    read.return_value = {**read.return_value, "joint_1": 1.21}
    _, moved = tick(reader, profile, 0.5)
    assert moved.positions[ARMS[0]] == pytest.approx(1.21)
    assert {n: q for n, q in moved.positions.items() if n != ARMS[0]} == dict.fromkeys(
        (n for n in UPPER if n != ARMS[0]), 0.2
    )
    assert moved.velocity == (0, 0, 0)


def test_unsmoothed_jump_pauses_before_it_can_move_the_follower(pipeline):
    reader, profile, read = pipeline
    engage(reader, profile)
    read.return_value = {**read.return_value, "joint_1": 1.2}
    sample, stopped = tick(reader, profile, 0.06)
    assert sample is not None
    assert not profile.active and "discontinuity" in profile.reason
    assert stopped.positions == dict.fromkeys(UPPER, 0.2)
    _, resumed = tick(reader, profile, 0.08, ("minus",))
    assert profile.active
    assert resumed.positions[ARMS[0]] == pytest.approx(1.2)


def test_late_sample_cannot_hide_a_timeout_between_guard_ticks(pipeline):
    reader, profile, _ = pipeline
    engage(reader, profile)
    _, stopped = tick(reader, profile, 0.4)
    assert not profile.active
    assert stopped.positions == dict.fromkeys(UPPER, 0.2)
    tick(reader, profile, 0.42)
    _, resumed = tick(reader, profile, 0.44, ("minus",))
    assert profile.active and resumed.positions == pytest.approx(stopped.positions)


@pytest.mark.parametrize("direction", [-1, 1])
def test_feetech_encoder_wrap_keeps_following_a_small_real_motion(pipeline, direction):
    reader, profile, read = pipeline
    # Captured joint-3 wrap: -3.133922749650365 -> pi in 10.8 ms.
    profile.limits[ARMS[2]] = (-4.0, 4.0)
    before = direction * -3.133922749650365
    after = direction * math.pi
    read.return_value = {**read.return_value, "joint_3": before}
    engage(reader, profile)
    for i in range(3, 130):
        tick(reader, profile, i * 0.02)
    read.return_value = {**read.return_value, "joint_3": after}
    _, moved = tick(reader, profile, 2.6)
    assert profile.active
    assert profile.pause_count == 0
    assert moved.positions[ARMS[2]] == pytest.approx(before - direction * 0.007669903939428)
    # Moving back across the boundary must restore the original target.
    read.return_value = {**read.return_value, "joint_3": before}
    _, returned = tick(reader, profile, 2.62)
    assert profile.active
    assert returned.positions[ARMS[2]] == pytest.approx(before)


def test_encoder_wrap_after_missing_input_preserves_mapping_but_requires_enable(pipeline):
    reader, profile, read = pipeline
    read.return_value = {**read.return_value, "joint_3": -3.13}
    engage(reader, profile)
    read.return_value = {**read.return_value, "joint_3": 3.14}
    _, held = tick(reader, profile, 0.4)
    assert not profile.active
    assert profile.pause_count == 1
    assert "timeout" in profile.last_pause_reason
    assert held.positions == dict.fromkeys(UPPER, 0.2)
    tick(reader, profile, 0.42)
    _, resumed = tick(reader, profile, 0.44, ("minus",))
    assert profile.active
    assert resumed.positions[ARMS[2]] == pytest.approx(-2.0)
    for i in range(23, 150):
        _, moved = tick(reader, profile, i * 0.02)
    assert profile.active
    assert moved.positions[ARMS[2]] == pytest.approx(-2.0)  # Absolute pose, limited by model.
    assert "timeout" in profile.last_pause_reason  # Retain the cause after recovery.
