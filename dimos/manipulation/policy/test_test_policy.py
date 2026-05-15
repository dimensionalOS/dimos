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

"""Unit tests for the deterministic `TestPolicy` backend."""

from __future__ import annotations

import math

import pytest

from dimos.manipulation.policy import TestPolicy, create_backend
from dimos.manipulation.policy.command import JointPositionCommand, NoOpCommand
from dimos.manipulation.policy.observation import PolicyObservation
from dimos.msgs.sensor_msgs.JointState import JointState


def test_sample_at_zero_is_center_for_zero_phase():
    policy = TestPolicy(joint_names=["j1", "j2"], amplitude=0.5, frequency=1.0, phase=0.0)
    sample = policy.sample_at(0.0)
    assert sample == {"j1": 0.0, "j2": 0.0}


def test_sample_at_quarter_period_hits_amplitude():
    policy = TestPolicy(joint_names=["j"], amplitude=0.7, frequency=1.0, phase=0.0)
    sample = policy.sample_at(0.25)
    assert sample["j"] == pytest.approx(0.7, abs=1e-9)


def test_per_joint_amplitude_and_phase_broadcast():
    policy = TestPolicy(
        joint_names=["a", "b", "c"],
        amplitude=[0.1, 0.2, 0.3],
        frequency=1.0,
        phase=[0.0, math.pi / 2.0, math.pi],
    )
    s = policy.sample_at(0.0)
    # phase=π/2 → sin = 1, phase=π → sin = 0
    assert s["a"] == pytest.approx(0.0, abs=1e-9)
    assert s["b"] == pytest.approx(0.2, abs=1e-9)
    assert s["c"] == pytest.approx(0.0, abs=1e-9)


def test_center_offsets_each_joint():
    policy = TestPolicy(
        joint_names=["a", "b"],
        amplitude=0.0,
        frequency=1.0,
        center=[0.4, -0.2],
    )
    s = policy.sample_at(0.123)
    assert s == {"a": 0.4, "b": -0.2}


def test_select_action_returns_joint_position_command():
    policy = TestPolicy(joint_names=["j1", "j2"], amplitude=0.1, frequency=1.0)
    policy.initialize()
    cmd = policy.select_action(_obs(["j1", "j2"], [0.0, 0.0]))
    assert isinstance(cmd, JointPositionCommand)
    assert cmd.joint_names == ("j1", "j2")
    assert len(cmd.positions) == 2


def test_reset_restarts_phase_clock_without_error():
    policy = TestPolicy(joint_names=["j"], amplitude=1.0, frequency=2.0)
    policy.initialize()
    # Multiple resets must succeed and remain idempotent.
    policy.reset()
    policy.reset()
    cmd = policy.select_action(_obs(["j"], [0.0]))
    assert isinstance(cmd, JointPositionCommand)


def test_test_policy_uses_registry_path():
    backend = create_backend("test", joint_names=["a", "b"], amplitude=0.3, frequency=0.5)
    backend.initialize()
    cmd = backend.select_action(_obs(["a", "b"], [0.0, 0.0]))
    assert isinstance(cmd, JointPositionCommand)
    assert cmd.joint_names == ("a", "b")


def test_amplitude_length_mismatch_raises():
    with pytest.raises(ValueError, match="amplitude"):
        TestPolicy(joint_names=["j1", "j2"], amplitude=[0.1, 0.2, 0.3])


def test_zero_joint_names_rejected():
    with pytest.raises(ValueError, match="at least one joint"):
        TestPolicy(joint_names=[])


# ── center auto-capture from observation (no explicit `center` arg) ───────


def _obs(name, position) -> PolicyObservation:
    return PolicyObservation(joint_state=JointState(name=list(name), position=list(position)))


def test_center_auto_captured_from_first_observation():
    policy = TestPolicy(joint_names=["j1", "j2"], amplitude=0.0, frequency=1.0)
    cmd = policy.select_action(_obs(["j1", "j2"], [0.7, -0.3]))
    # amplitude=0 → output equals captured center
    assert cmd.positions == pytest.approx((0.7, -0.3), abs=1e-9)


def test_center_recaptured_on_reset():
    policy = TestPolicy(joint_names=["j1"], amplitude=0.0, frequency=1.0)
    policy.select_action(_obs(["j1"], [0.5]))
    policy.reset()
    cmd = policy.select_action(_obs(["j1"], [-0.2]))
    assert cmd.positions == pytest.approx((-0.2,), abs=1e-9)


def test_center_not_recaptured_when_explicit_center_provided():
    """Static-center mode: callers that pass `center=` get the legacy
    open-loop behavior — center stays fixed across reset()."""
    policy = TestPolicy(joint_names=["j1"], amplitude=0.0, frequency=1.0, center=[0.1])
    cmd = policy.select_action(_obs(["j1"], [9.9]))
    assert cmd.positions == pytest.approx((0.1,), abs=1e-9)
    policy.reset()
    cmd = policy.select_action(_obs(["j1"], [-9.9]))
    assert cmd.positions == pytest.approx((0.1,), abs=1e-9)


def test_capture_failure_returns_noop_until_observation_arrives():
    """No `joint_state` in the observation → emit `NoOpCommand` rather
    than command the arm to a stale-trajectory pose. Once a real
    observation arrives, the trajectory anchors and a real command
    flows."""
    policy = TestPolicy(joint_names=["j1"], amplitude=0.0, frequency=1.0)
    cmd = policy.select_action(PolicyObservation())  # no joint_state
    assert isinstance(cmd, NoOpCommand)
    # Once a real observation arrives, the center is captured.
    cmd2 = policy.select_action(_obs(["j1"], [0.42]))
    assert isinstance(cmd2, JointPositionCommand)
    assert cmd2.positions == pytest.approx((0.42,), abs=1e-9)


def test_first_sample_after_capture_equals_observation_pose_under_non_zero_phase():
    """Regression: with non-zero phase the open-loop formula
    ``center + amplitude*sin(2π·f·t + phase)`` would emit ``center +
    amplitude*sin(phase)`` at t=0 — a jump of up to ``amplitude`` from the
    captured pose. The center adjustment ensures position(0) ≡ captured pose.
    """
    policy = TestPolicy(
        joint_names=["j1", "j2"],
        amplitude=0.5,
        frequency=1.0,
        phase=[math.pi / 2, math.pi],  # would jump by amplitude*sin(phase)
    )
    cmd = policy.select_action(_obs(["j1", "j2"], [0.3, -0.7]))
    # Tolerance covers the few µs between t0 reset and sample_at; with
    # amplitude=0.5 and freq=1, that's bounded by 2π·1·1e-5·0.5 ≈ 3e-5 rad.
    assert cmd.positions[0] == pytest.approx(0.3, abs=1e-3)
    assert cmd.positions[1] == pytest.approx(-0.7, abs=1e-3)


def test_first_sample_after_reset_equals_post_reset_observation_pose():
    """End-to-end of the engage→disengage scenario: capture pose A, then
    reset (simulating teleop engage), then call select_action with pose B
    (simulating post-teleop). First sample equals B, not A."""
    policy = TestPolicy(
        joint_names=["j1"],
        amplitude=0.3,
        frequency=0.5,
        phase=math.pi / 3,
    )
    policy.select_action(_obs(["j1"], [0.1]))  # capture pose A=0.1
    policy.reset()  # simulate teleop engage
    cmd = policy.select_action(_obs(["j1"], [0.85]))  # post-teleop pose B
    assert cmd.positions[0] == pytest.approx(0.85, abs=1e-3)


def test_trajectory_anchors_to_select_action_pose_not_reset_pose():
    """The trajectory must be generated at the next `select_action()`,
    not at `reset()`, since the arm can move between the two — e.g.,
    between a teleop disengage and the next policy inference tick.
    Reset captures the pose at the moment the policy resumes, not the
    pose at the moment reset was called.
    """
    policy = TestPolicy(joint_names=["j"], amplitude=0.1, frequency=1.0, phase=math.pi / 4)
    # Initial capture at pose=0.0.
    policy.select_action(_obs(["j"], [0.0]))
    # Reset (teleop engage). At this moment the arm is at 0.0.
    policy.reset()
    # Simulate the arm moving during the engaged window (operator drags
    # it to 1.2). The next select_action must anchor to 1.2, not 0.0.
    cmd = policy.select_action(_obs(["j"], [1.2]))
    assert isinstance(cmd, JointPositionCommand)
    assert cmd.positions[0] == pytest.approx(1.2, abs=1e-3)


def test_wave_oscillates_symmetrically_around_captured_pose():
    """Regression for the "extreme motion" failure: without the captured
    `phase_offset`, the wave would start at the captured pose (good) but
    then sweep a full `2·amplitude` to one side as it walked through the
    configured phase (bad — joint with phase=π/2 oscillates entirely
    below the captured pose, not symmetrically around it).
    """
    # `phase=π/2` is the worst case: sin(phase)=1, so the un-shifted
    # formula would emit `center + amp` at t=0 and `center - amp` at the
    # opposite extreme — a peak-to-trough swing of `2·amp` ENTIRELY below
    # `pose`. With the phase-offset capture, the swing is centered on pose.
    policy = TestPolicy(joint_names=["j"], amplitude=0.5, frequency=1.0, phase=math.pi / 2)
    policy.select_action(_obs(["j"], [1.0]))  # capture pose = 1.0

    # Sample across a full period at fine resolution and confirm the
    # extremes are at pose±amp, not pose and pose-2·amp.
    samples = [policy.sample_at(t)["j"] for t in [i * 0.01 for i in range(101)]]
    assert max(samples) == pytest.approx(1.0 + 0.5, abs=1e-3)
    assert min(samples) == pytest.approx(1.0 - 0.5, abs=1e-3)


def test_partial_observation_returns_noop_until_all_joints_present():
    """Observation present but missing one of the configured joints →
    emit `NoOpCommand` rather than partially anchoring the trajectory.
    Once all joints are present, capture succeeds and a real command
    flows."""
    policy = TestPolicy(joint_names=["j1", "j2"], amplitude=0.0, frequency=1.0)
    cmd = policy.select_action(_obs(["j1"], [0.3]))  # j2 missing
    assert isinstance(cmd, NoOpCommand)
    # Once both joints are present, both centers are captured.
    cmd2 = policy.select_action(_obs(["j1", "j2"], [0.3, -0.1]))
    assert isinstance(cmd2, JointPositionCommand)
    assert cmd2.positions == pytest.approx((0.3, -0.1), abs=1e-9)
