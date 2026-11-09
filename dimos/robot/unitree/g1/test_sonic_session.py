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
import json
import struct

import numpy as np
import pytest

from dimos.robot.unitree.g1.sonic_feedback import SonicFeedback, SonicReferenceGuard
from dimos.robot.unitree.g1.sonic_session import (
    JsonlSessionJournal,
    SessionPhase,
    SonicModelProfile,
    SonicTaskSession,
)


def feedback(now, index=100, *, planner=True, epoch=7, accepted=41, applied=None):
    return SonicFeedback(
        index=index,
        source_timestamp=None,
        received_at=1000.0 + now,
        received_monotonic=now,
        body_q=(0.0,) * 29,
        body_dq=(0.0,) * 29,
        left_hand_q=(0.1,) * 7,
        left_hand_dq=(0.0,) * 7,
        right_hand_q=(0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7),
        right_hand_dq=(0.0,) * 7,
        token_state=(0.125,) * 64,
        reference_guard=SonicReferenceGuard(
            armed=not planner,
            expired=False,
            planner_selected=planner,
            planner_confirmed=planner,
            rearm_requires_planner=False,
            accepted_index=accepted,
            applied_index=accepted if applied is None else applied,
            rejected=0,
            epoch=epoch,
            age_ms=0.0,
        ),
    )


def pose_values(packet):
    payload = packet[4 + 1280 :]
    return np.concatenate(
        [np.frombuffer(payload[:256], dtype="<f4"), np.frombuffer(payload[264:], dtype="<f4")]
    )


@pytest.fixture
def rig():
    packets, events = [], []
    profile = SonicModelProfile(
        name="offline-test",
        source_fps=50,
        right_hand_order="native",
        handback_policy="native_planner",
        seed_seconds=0.04,
        blend_seconds=0.04,
        first_chunk_timeout=0.12,
        max_observation_age=0.4,
    )
    return (
        SonicTaskSession("task-1", profile, send=packets.append, journal=events.append),
        packets,
        events,
    )


def activate(session):
    session.start(feedback(10.0), now=10.0)
    session.tick(now=10.0)
    for tick in range(1, 5):
        now = 10.0 + tick * 0.02
        session.observe(
            feedback(
                now,
                100 + tick,
                planner=tick <= 2,
                epoch=7 if tick <= 2 else 8,
                accepted=session.status.last_frame_index,
            ),
            now=now,
        )
        session.tick(now=now)
    assert session.status.phase == SessionPhase.WAITING


def test_caller_failure_requests_handback_and_preserves_failure_reason(rig):
    session, packets, _ = rig
    activate(session)
    session.fail("inference worker failed", now=10.09)
    assert session.status.outcome == "failed"
    assert session.status.reason == "inference worker failed"
    assert session.status.phase == SessionPhase.HANDBACK
    assert packets[-1][-3:] == b"\0\0\1"


def submit(session, *, now=10.08, observed_at=10.08, sequence=0, task_id="task-1", actions=None):
    session.submit_chunk(
        task_id=task_id,
        sequence=sequence,
        observed_at=observed_at,
        actions=np.full((20, 78), 0.25) if actions is None else actions,
        now=now,
    )


def advance(session, now, index=110):
    session.observe(
        feedback(now, index, planner=False, epoch=8, accepted=session.status.last_frame_index),
        now=now,
    )
    session.tick(now=now)


def test_start_preserves_measured_hands_and_never_powers_controller(rig):
    session, packets, _ = rig
    activate(session)
    first = next(packet for packet in packets if packet.startswith(b"pose"))
    np.testing.assert_allclose(
        pose_values(first), [0.125] * 64 + [0.1] * 7 + [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7]
    )
    assert struct.unpack_from("<q", first, 4 + 1280 + 256)[0] == 42
    assert [tuple(packet[-3:]) for packet in packets if packet.startswith(b"command")] == [
        (0, 0, 1),
        (0, 0, 0),
    ]


@pytest.mark.parametrize(
    "change",
    [
        {"received_monotonic": 9.0},
        {"received_monotonic": 11.0},
        {"reference_guard": None},
        {"token_state": None},
        {"token_state": (2.0,) * 64},
    ],
)
def test_invalid_start_never_sends(rig, change):
    session, packets, _ = rig
    with pytest.raises(ValueError):
        session.start(replace(feedback(10.0), **change), now=10.0)
    assert packets == []


def test_session_is_one_shot_even_after_cancel(rig):
    session, _, _ = rig
    activate(session)
    session.cancel(now=10.08)
    with pytest.raises(RuntimeError, match="one-shot"):
        session.start(feedback(10.1, 111), now=10.1)


def test_previous_epoch_indices_cannot_activate(rig):
    session, _, _ = rig
    session.start(feedback(10.0), now=10.0)
    session.tick(now=10.0)
    session.observe(feedback(10.04, 101, accepted=42), now=10.04)
    session.tick(now=10.04)
    session.observe(feedback(10.06, 102, planner=False, epoch=8, accepted=42), now=10.06)
    session.tick(now=10.06)
    assert session.status.phase == SessionPhase.ARMING
    session.observe(feedback(10.08, 103, planner=False, epoch=8, accepted=42), now=10.08)
    session.tick(now=10.08)
    assert session.status.phase == SessionPhase.ARMING


def test_policy_chunk_blends_then_reaches_target_without_aliasing_input(rig):
    session, packets, _ = rig
    activate(session)
    actions = np.full((20, 78), 0.25)
    submit(session, actions=actions)
    actions[:] = 1.0
    advance(session, 10.10)
    assert pose_values(packets[-1])[0] == pytest.approx(0.175)  # .05 token step limit
    advance(session, 10.12, 111)
    advance(session, 10.14, 112)
    assert pose_values(packets[-1])[0] == pytest.approx(0.25)


def test_no_catchup_or_duplicate_tick_packets(rig):
    session, packets, _ = rig
    activate(session)
    count = len(packets)
    session.tick(now=10.08)
    session.tick(now=10.09)
    assert len(packets) == count
    advance(session, 10.13)
    assert len(packets) == count + 1


@pytest.mark.parametrize(
    "kwargs",
    [
        {"task_id": "old-task"},
        {"observed_at": 9.0},
        {"observed_at": 10.09},
        {"actions": np.zeros((20, 77))},
        {"actions": np.full((20, 78), np.nan)},
        {"actions": np.full((20, 78), 2.0)},
        {"sequence": True},
        {"actions": np.zeros((0, 78))},
        {"actions": np.full((20, 78), True)},
    ],
)
def test_invalid_chunk_does_not_start_execution(rig, kwargs):
    session, packets, _ = rig
    activate(session)
    count = len(packets)
    with pytest.raises(ValueError):
        submit(session, **kwargs)
    assert len(packets) == count
    assert session.status.phase == SessionPhase.WAITING


def test_old_chunk_does_not_refresh_age_or_horizon(rig):
    session, _, _ = rig
    activate(session)
    submit(session, actions=np.zeros((3, 78)))
    with pytest.raises(ValueError):
        submit(session, sequence=0)
    advance(session, 10.14)
    assert session.status.phase == SessionPhase.HANDBACK
    assert session.status.reason == "chunk horizon exhausted"


@pytest.mark.parametrize("new_observation", [10.07, 10.08])
def test_new_sequence_with_old_observation_is_rejected(rig, new_observation):
    session, _, _ = rig
    activate(session)
    submit(session)
    with pytest.raises(ValueError, match="observation"):
        submit(session, sequence=1, observed_at=new_observation)
    assert session.status.last_chunk_sequence == 0


def test_first_chunk_wait_has_deadline(rig):
    session, _, _ = rig
    activate(session)
    advance(session, 10.16)
    advance(session, 10.201, 111)
    assert session.status.reason == "first chunk timeout"
    assert session.status.phase == SessionPhase.HANDBACK


def test_lost_feedback_stops_pose_output_and_duplicate_does_not_refresh(rig):
    session, packets, _ = rig
    activate(session)
    assert not session.observe(feedback(10.16, 104), now=10.16)
    session.tick(now=10.16)
    count = sum(p.startswith(b"pose") for p in packets)
    session.tick(now=10.30)
    assert session.status.phase == SessionPhase.HANDBACK
    assert sum(p.startswith(b"pose") for p in packets) == count
    assert packets[-1][-3:] == bytes([0, 0, 1])


@pytest.mark.parametrize("operation,outcome", [("cancel", "cancelled"), ("finish", "finished")])
def test_handback_requires_new_feedback_and_never_sends_stop(rig, operation, outcome):
    session, packets, events = rig
    activate(session)
    getattr(session, operation)(now=10.08)
    count = sum(p.startswith(b"pose") for p in packets)
    assert session.status.phase == SessionPhase.HANDBACK
    assert not session.status.handback_confirmed
    assert packets[-1][-3:] == bytes([0, 0, 1])
    assert not session.observe(feedback(10.08, 104, epoch=8), now=10.08)
    session.tick(now=10.09)
    assert not session.status.handback_confirmed
    session.observe(feedback(10.10, 105, epoch=8), now=10.10)
    session.tick(now=10.10)
    assert session.status.phase == SessionPhase.DONE
    assert session.status.outcome == outcome
    assert session.status.handback_confirmed
    session.tick(now=10.2)
    assert sum(p.startswith(b"pose") for p in packets) == count
    assert events[-1]["event"] == "terminal"
    assert events[-1]["status"]["handback_confirmed"] is True


def test_handback_timeout_reports_unconfirmed_failure(rig):
    session, _, _ = rig
    activate(session)
    session.cancel(now=10.08)
    session.tick(now=12.09)
    assert session.status.phase == SessionPhase.DONE
    assert session.status.outcome == "failed"
    assert not session.status.handback_confirmed
    assert "handback timeout" in session.status.reason


@pytest.mark.parametrize("epoch,accepted", [(9, 45), (8, 9999)])
def test_other_stream_ownership_evidence_aborts(rig, epoch, accepted):
    session, _, _ = rig
    activate(session)
    session.observe(feedback(10.10, 105, planner=False, epoch=epoch, accepted=accepted), now=10.10)
    session.tick(now=10.10)
    assert session.status.phase == SessionPhase.HANDBACK
    assert session.status.outcome == "failed"


def test_packet_failure_attempts_handback_and_keeps_failure_outcome(rig, mocker):
    session, _, _ = rig
    activate(session)
    send = mocker.patch.object(session, "_send", side_effect=[OSError("disconnected"), None])
    advance(session, 10.10)
    assert session.status.phase == SessionPhase.HANDBACK
    assert "packet send failed" in session.status.reason
    assert send.call_args.args[0][-3:] == bytes([0, 0, 1])


def test_journal_failure_prevents_further_policy_output(rig, mocker):
    session, packets, _ = rig
    activate(session)
    mocker.patch.object(session, "_journal", side_effect=OSError("disk full"))
    with pytest.raises(OSError, match="disk full"):
        submit(session)
    assert session.status.phase == SessionPhase.DONE
    assert session.status.outcome == "failed"
    assert not session.status.handback_confirmed
    assert packets[-1][-3:] == bytes([0, 0, 1])


@pytest.mark.parametrize(
    "field,value",
    [
        ("source_fps", 0),
        ("source_fps", float("nan")),
        ("source_fps", 60),
        ("right_hand_order", "guess"),
        ("handback_policy", "zero_hands"),
        ("max_observation_age", float("inf")),
        ("seed_seconds", -1.0),
        ("max_tick_gap", 0.3),
        ("token_step_limit", True),
    ],
)
def test_profile_rejects_unsupported_or_invalid_contract(rig, field, value):
    session, _, _ = rig
    with pytest.raises(ValueError):
        replace(session.profile, **{field: value})


def test_jsonl_journal_is_incremental_and_does_not_overwrite(tmp_path):
    path = tmp_path / "session.jsonl"
    journal = JsonlSessionJournal(path)
    journal({"event": "started", "task_id": "one"})
    assert json.loads(path.read_text()) == {"event": "started", "task_id": "one"}
    journal({"event": "terminal", "outcome": "cancelled"})
    assert [json.loads(line)["event"] for line in path.read_text().splitlines()] == [
        "started",
        "terminal",
    ]
    with pytest.raises(FileExistsError):
        JsonlSessionJournal(path)


def test_late_first_chunk_cannot_revive_expired_wait(rig):
    session, _, _ = rig
    activate(session)
    advance(session, 10.16)
    session.observe(
        feedback(10.201, 111, planner=False, epoch=8, accepted=session.status.last_frame_index),
        now=10.201,
    )
    with pytest.raises(ValueError, match="first chunk timeout"):
        submit(session, now=10.201, observed_at=10.201)
    assert session.status.phase == SessionPhase.HANDBACK


def test_late_replacement_cannot_revive_exhausted_chunk(rig):
    session, _, _ = rig
    activate(session)
    submit(session, actions=np.zeros((3, 78)))
    with pytest.raises(ValueError, match="horizon"):
        submit(session, sequence=1, now=10.15, observed_at=10.15)
    assert session.status.phase == SessionPhase.HANDBACK


def test_confirmation_after_handback_deadline_does_not_report_success(rig):
    session, _, _ = rig
    activate(session)
    session.cancel(now=10.08)
    session.observe(feedback(12.10, 111, epoch=8), now=12.10)
    session.tick(now=12.10)
    assert session.status.outcome == "failed"
    assert not session.status.handback_confirmed


def test_expired_guard_during_arming_stops_references(rig):
    session, _, _ = rig
    session.start(feedback(10.0), now=10.0)
    session.tick(now=10.0)
    session.observe(feedback(10.04, 101, accepted=42), now=10.04)
    session.tick(now=10.04)
    state = feedback(10.06, 102, epoch=8, accepted=session.status.last_frame_index)
    state = replace(
        state,
        reference_guard=replace(state.reference_guard, expired=True, rearm_requires_planner=True),
    )
    session.observe(state, now=10.06)
    session.tick(now=10.06)
    assert session.status.phase == SessionPhase.HANDBACK


@pytest.mark.parametrize("fps", [30, 50])
def test_linear_sampling_uses_observation_time_at_both_source_rates(rig, fps):
    _, packets, events = rig
    profile = replace(rig[0].profile, source_fps=fps, blend_seconds=0.001, token_step_limit=1.0)
    session = SonicTaskSession("task-1", profile, send=packets.append, journal=events.append)
    activate(session)
    actions = np.repeat((np.arange(20) * 0.01)[:, None], 78, axis=1)
    submit(session, actions=actions, observed_at=10.06)
    advance(session, 10.10)
    assert pose_values(packets[-1])[0] == pytest.approx(0.04 * fps * 0.01)


def test_sim_hand_mapping_preserves_startup_then_permutates_policy_only_at_wire(rig):
    _, packets, events = rig
    profile = replace(rig[0].profile, right_hand_order="mujoco_swap", blend_seconds=0.001)
    session = SonicTaskSession("task-1", profile, send=packets.append, journal=events.append)
    activate(session)
    np.testing.assert_allclose(pose_values(packets[-1])[71:], [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7])
    actions = np.full((20, 78), 0.125)
    actions[:, 71:] = np.arange(7)
    submit(session, actions=actions)
    advance(session, 10.10)
    np.testing.assert_allclose(pose_values(packets[-1])[71:], [0, 1, 2, 5, 6, 3, 4])


def test_pre_stream_epoch_cannot_confirm_handback_after_stream_activation(rig):
    session, _, _ = rig
    activate(session)
    session.cancel(now=10.08)
    session.observe(feedback(10.10, 111, epoch=7), now=10.10)
    session.tick(now=10.10)
    assert session.status.phase == SessionPhase.HANDBACK
    assert not session.status.handback_confirmed


def test_exhausted_wire_index_is_rejected_before_start(rig):
    session, packets, _ = rig
    with pytest.raises(ValueError, match="index"):
        session.start(feedback(10.0, accepted=2**63 - 1), now=10.0)
    assert packets == []


def test_wire_index_exhaustion_during_session_requests_handback(rig):
    session, packets, _ = rig
    session.start(feedback(10.0, accepted=2**63 - 2), now=10.0)
    session.tick(now=10.0)
    session.tick(now=10.02)
    assert session.status.phase == SessionPhase.HANDBACK
    assert session.status.reason == "wire index exhausted"
    assert packets[-1][-3:] == bytes([0, 0, 1])


def test_stream_acknowledgement_after_arm_deadline_is_rejected(rig):
    original, packets, events = rig
    session = SonicTaskSession(
        "task-1",
        replace(original.profile, arm_timeout=0.02),
        send=packets.append,
        journal=events.append,
    )
    session.start(feedback(10.0), now=10.0)
    session.tick(now=10.0)
    session.observe(feedback(10.04, 101, accepted=42), now=10.04)
    session.tick(now=10.04)
    session.observe(
        feedback(10.08, 102, planner=False, epoch=8, accepted=session.status.last_frame_index),
        now=10.08,
    )
    session.tick(now=10.08)
    assert session.status.phase == SessionPhase.HANDBACK
    assert session.status.reason == "stream arm timeout"


@pytest.mark.parametrize("epoch,accepted", [(9, 46), (8, 9999)])
def test_observed_owner_violation_cannot_be_overwritten_before_tick(rig, epoch, accepted):
    session, packets, events = rig
    activate(session)
    session.observe(feedback(10.10, 105, planner=False, epoch=epoch, accepted=accepted), now=10.10)
    session.observe(
        feedback(10.11, 106, planner=False, epoch=8, accepted=session.status.last_frame_index),
        now=10.11,
    )
    count = sum(p.startswith(b"pose") for p in packets)
    session.tick(now=10.11)
    assert session.status.phase == SessionPhase.HANDBACK
    assert sum(p.startswith(b"pose") for p in packets) == count
    assert any(event["event"] == "feedback_fault" for event in events)
