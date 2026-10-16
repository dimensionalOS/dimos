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

from __future__ import annotations

from dataclasses import asdict, replace
import threading

import msgpack
import pytest
import zmq

from dimos.robot.unitree.g1.sonic_feedback import FeedbackBuffer, decode_feedback
from dimos.robot.unitree.g1.sonic_feedback_module import SonicFeedbackModule
from dimos.utils.testing.waiting import wait_until


@pytest.fixture
def payload():
    # Wire values are already in motor order, with default offsets restored.
    return {
        "index": 12,
        "ros_timestamp": 1234.5,
        "body_q": list(range(29)),
        "body_dq": [-0.25] * 29,
        "left_hand_q": [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7],
        "left_hand_dq": [0.0] * 7,
        "right_hand_q": [-0.1, -0.2, -0.3, -0.4, -0.5, -0.6, -0.7],
        "right_hand_dq": [0.0] * 7,
        "body_q_target": [99.0] * 29,
        "control_loop_type": "cpp",
    }


def decode(payload):
    return decode_feedback(
        b"g1_debug" + msgpack.packb(payload), received_at=9000.0, received_monotonic=100.0
    )


def test_body_mapping_uses_measurements_without_reordering_or_offsets(payload):
    state = decode(payload)
    body, left, right = state.joint_states()

    assert dict(zip(body.name, body.position, strict=True))["g1/left_elbow"] == 18.0
    assert dict(zip(body.name, body.position, strict=True))["g1/right_wrist_yaw"] == 28.0
    assert body.position == list(range(29))
    assert body.velocity == [-0.25] * 29
    assert body.effort == []
    assert left.name == [f"g1/left_hand_motor_{i}" for i in range(7)]
    assert right.name == [f"g1/right_hand_motor_{i}" for i in range(7)]
    assert right.position == [-0.1, -0.2, -0.3, -0.4, -0.5, -0.6, -0.7]
    assert left.position == [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7]
    assert body.ts == 9000.0
    assert state.source_timestamp == 1234.5


def test_zero_ros_timestamp_is_explicitly_unavailable(payload):
    payload["ros_timestamp"] = 0.0
    assert decode(payload).source_timestamp is None


def test_planner_token_is_optional_and_coherent_with_feedback(payload):
    assert decode(payload).token_state is None
    payload["token_state"] = [0.125] * 64
    assert decode(payload).token_state == (0.125,) * 64


@pytest.mark.parametrize("token", [[0.0] * 63, [float("nan")] * 64, [True] * 64, None])
def test_present_planner_token_must_be_valid(payload, token):
    payload["token_state"] = token
    with pytest.raises(ValueError, match="token_state"):
        decode(payload)


@pytest.mark.parametrize(
    ("field", "value"),
    [
        ("body_q", [0.0] * 28),
        ("body_dq", [0.0] * 30),
        ("left_hand_q", [0.0] * 6),
        ("right_hand_dq", [0.0] * 8),
        ("body_q", [float("nan")] * 29),
        ("left_hand_q", [float("inf")] * 7),
        ("right_hand_q", ["0.0"] * 7),
        ("left_hand_dq", [False] * 7),
        ("index", True),
        ("index", -1),
        ("index", 1.5),
        ("ros_timestamp", float("nan")),
        ("ros_timestamp", -1.0),
    ],
)
def test_invalid_state_is_rejected(payload, field, value):
    payload[field] = value
    with pytest.raises(ValueError):
        decode(payload)


def test_missing_hand_feedback_is_not_filled_with_zeroes(payload):
    del payload["right_hand_q"]
    with pytest.raises(ValueError, match="right_hand_q"):
        decode(payload)


@pytest.mark.parametrize("raw", [b"", b"robot_config\x80", b"g1_debug\xc1", b"g1_debug\x90"])
def test_invalid_envelope_is_rejected(raw):
    with pytest.raises(ValueError):
        decode_feedback(raw, received_at=9000.0, received_monotonic=100.0)


def test_freshness_uses_local_monotonic_arrival_and_expires_at_threshold(payload):
    buffer = FeedbackBuffer(stale_after=0.25)
    assert buffer.latest(now=100.0) is None
    assert buffer.accept(decode(payload))
    assert buffer.latest(now=100.249).index == 12
    assert buffer.latest(now=100.25) is None
    assert buffer.age(now=100.25) == 0.25


@pytest.mark.parametrize("index", [12, 11, 0])
def test_duplicate_or_restarted_index_cannot_refresh_state(payload, index):
    buffer = FeedbackBuffer(stale_after=0.25)
    original = decode(payload)
    buffer.accept(original)

    assert not buffer.accept(replace(original, index=index, received_monotonic=100.2))
    assert buffer.latest(now=100.26) is None
    assert buffer.rejected_indices == 1


def test_new_measurement_recovers_from_stale_state(payload):
    buffer = FeedbackBuffer(stale_after=0.25)
    original = decode(payload)
    buffer.accept(original)
    assert buffer.latest(now=101.0) is None

    assert buffer.accept(replace(original, index=13, received_monotonic=101.0))
    assert buffer.latest(now=101.1).index == 13


@pytest.mark.parametrize("threshold", [0, -1, float("nan"), float("inf")])
def test_invalid_freshness_threshold_is_rejected(threshold):
    with pytest.raises(ValueError):
        FeedbackBuffer(stale_after=threshold)


@pytest.fixture
def telemetry():
    # XPUB lets the fixture wait for subscription readiness without sleeps.
    with zmq.Context() as context, context.socket(zmq.XPUB) as publisher:
        publisher.setsockopt(zmq.LINGER, 0)
        port = publisher.bind_to_random_port("tcp://127.0.0.1")
        module = SonicFeedbackModule(endpoint=f"tcp://127.0.0.1:{port}", stale_after=0.2)
        try:
            module.start()
            assert publisher.poll(3000)
            assert publisher.recv() == b"\x01g1_debug"
            yield module, publisher
        finally:
            module.stop()


def test_socket_feedback_reaches_typed_streams_and_rpc(telemetry, payload):
    module, publisher = telemetry
    received = []
    bodies = []
    done = threading.Event()
    unsubscribe_body = module.body_joint_state.subscribe(bodies.append)
    unsubscribe = module.feedback.subscribe(lambda state: (received.append(state), done.set()))
    try:
        publisher.send(b"g1_debug" + msgpack.packb(payload))
        assert done.wait(3.0)
        assert received[0].body_q[18] == 18.0
        assert bodies[0].name[18] == "g1/left_elbow"
        assert bodies[0].position[18] == 18.0
        assert module.get_feedback().index == 12
        assert module.get_status()["state"] == "fresh"
        assert module.inputs == {}
    finally:
        unsubscribe()
        unsubscribe_body()


def test_bad_frame_does_not_kill_receiver_or_refresh_feedback(telemetry, payload):
    module, publisher = telemetry
    publisher.send(b"g1_debug\xc1")
    wait_until(lambda: module.get_status()["invalid_packets"] == 1, timeout=3.0)
    assert module.get_status()["state"] == "waiting"
    assert module.get_feedback() is None

    publisher.send(b"g1_debug" + msgpack.packb(payload))
    wait_until(lambda: module.get_status()["accepted_packets"] == 1, timeout=3.0)
    assert module.get_feedback().index == 12


def test_absent_publisher_and_stopped_module_never_report_fresh(telemetry):
    module, _ = telemetry
    assert module.get_status()["state"] == "waiting"
    assert module.get_feedback() is None
    module.stop()
    module.stop()
    assert module.get_status()["state"] == "stopped"
    assert module.get_feedback() is None


def test_lost_stream_expires_without_republishing_old_measurements(telemetry, payload):
    module, publisher = telemetry
    received = []
    unsubscribe = module.feedback.subscribe(received.append)
    try:
        publisher.send(b"g1_debug" + msgpack.packb(payload))
        wait_until(lambda: module.get_status()["accepted_packets"] == 1, timeout=3.0)
        wait_until(lambda: module.get_status()["state"] == "stale", timeout=3.0)
        assert module.get_feedback() is None
        assert len(received) == 1
    finally:
        unsubscribe()


def test_stop_invalidates_a_previously_fresh_snapshot(telemetry, payload):
    module, publisher = telemetry
    publisher.send(b"g1_debug" + msgpack.packb(payload))
    wait_until(lambda: module.get_status()["accepted_packets"] == 1, timeout=3.0)
    assert module.get_feedback() is not None

    module.stop()

    assert module.get_feedback() is None
    assert module.get_status()["state"] == "stopped"


def test_output_failure_marks_fault_and_hides_cached_snapshot(telemetry, payload, mocker):
    module, publisher = telemetry
    mocker.patch.object(
        module.body_joint_state, "publish", side_effect=RuntimeError("transport lost")
    )
    publisher.send(b"g1_debug" + msgpack.packb(payload))

    wait_until(lambda: module.get_status()["state"] == "fault", timeout=3.0)

    assert module.get_feedback() is None
    assert module.get_status()["accepted_packets"] == 1
    assert module.get_status()["error"] == "RuntimeError: transport lost"


def test_failed_socket_setup_reports_fault_until_cleanup():
    module = SonicFeedbackModule(endpoint="unsupported-sonic-test://local")
    try:
        with pytest.raises(zmq.ZMQError):
            module.start()
        assert module.get_status()["state"] == "fault"
        assert "Protocol not supported" in module.get_status()["error"]
        assert module.get_feedback() is None
    finally:
        module.stop()
    assert module.get_status()["state"] == "stopped"


@pytest.fixture
def guard_payload():
    return {
        "armed": False,
        "expired": True,
        "planner_selected": True,
        "planner_confirmed": True,
        "rearm_requires_planner": True,
        "accepted_index": 144,
        "applied_index": 143,
        "rejected": 6,
        "epoch": 2,
        "age_ms": 420.5,
    }


def test_guard_decoding_preserves_expiry_and_handover_as_separate_facts(payload, guard_payload):
    payload["reference_guard"] = guard_payload

    state = decode(payload)

    assert asdict(state.reference_guard) == guard_payload
    assert state.reference_guard.expired and state.reference_guard.planner_confirmed
    # A decoded sample must not share mutable wire payload state.
    guard_payload["planner_confirmed"] = False
    assert state.reference_guard.planner_confirmed


def test_legacy_feedback_has_no_inferred_guard_status(payload):
    payload["planner_reference_active"] = [1.0]
    assert decode(payload).reference_guard is None


@pytest.mark.parametrize("value", [None, [], True, "planner", {}])
def test_present_but_malformed_guard_is_rejected(payload, value):
    payload["reference_guard"] = value
    with pytest.raises(ValueError, match="reference_guard"):
        decode(payload)


@pytest.mark.parametrize(
    ("field", "value"),
    [
        ("armed", 0),
        ("expired", "true"),
        ("planner_selected", None),
        ("planner_confirmed", 1),
        ("rearm_requires_planner", []),
        ("accepted_index", True),
        ("accepted_index", -2),
        ("applied_index", 143.0),
        ("rejected", -1),
        ("epoch", 2.5),
        ("age_ms", float("nan")),
        ("age_ms", float("inf")),
        ("age_ms", -0.5),
        ("age_ms", "420.5"),
    ],
)
def test_guard_fields_require_strict_types_and_valid_ranges(payload, guard_payload, field, value):
    guard_payload[field] = value
    payload["reference_guard"] = guard_payload
    with pytest.raises(ValueError, match="reference_guard"):
        decode(payload)


@pytest.mark.parametrize(
    "changes",
    [
        {"armed": True, "planner_confirmed": False},
        {"planner_selected": False},
        {"armed": True, "expired": False},
        {"applied_index": 145},
    ],
)
def test_guard_rejects_contradictory_confirmation_or_sequence(payload, guard_payload, changes):
    payload["reference_guard"] = guard_payload | changes
    with pytest.raises(ValueError, match="reference_guard"):
        decode(payload)


def test_guard_uninitialized_age_and_indices_remain_unavailable(payload, guard_payload):
    payload["reference_guard"] = guard_payload | {
        "expired": False,
        "rearm_requires_planner": False,
        "accepted_index": -1,
        "applied_index": -1,
        "epoch": 0,
        "age_ms": -1,
    }
    status = decode(payload).reference_guard
    assert status.accepted_index == status.applied_index == -1
    assert status.age_ms == -1


def test_guard_reaches_typed_stream_and_status_rpc(telemetry, payload, guard_payload):
    module, publisher = telemetry
    payload["reference_guard"] = guard_payload
    received = []
    unsubscribe = module.feedback.subscribe(received.append)
    try:
        publisher.send(b"g1_debug" + msgpack.packb(payload))
        wait_until(lambda: len(received) == 1, timeout=3.0)
        assert received[0].reference_guard.planner_confirmed
        assert module.get_status()["reference_guard"] == guard_payload
        assert module.get_feedback().reference_guard.expired
    finally:
        unsubscribe()


def test_stale_guard_never_reports_current_handover_confirmation(telemetry, payload, guard_payload):
    module, publisher = telemetry
    payload["reference_guard"] = guard_payload
    publisher.send(b"g1_debug" + msgpack.packb(payload))
    wait_until(lambda: module.get_status()["accepted_packets"] == 1, timeout=3.0)
    wait_until(lambda: module.get_status()["state"] == "stale", timeout=3.0)
    assert module.get_feedback() is None
    assert module.get_status()["reference_guard"] is None


def test_missing_guard_replaces_previous_confirmation(telemetry, payload, guard_payload):
    module, publisher = telemetry
    publisher.send(b"g1_debug" + msgpack.packb(payload | {"reference_guard": guard_payload}))
    wait_until(lambda: module.get_status()["accepted_packets"] == 1, timeout=3.0)
    publisher.send(b"g1_debug" + msgpack.packb(payload | {"index": 13}))
    wait_until(lambda: module.get_status()["accepted_packets"] == 2, timeout=3.0)
    assert module.get_status()["state"] == "fresh"
    assert module.get_status()["reference_guard"] is None
    assert module.get_feedback().reference_guard is None


def test_malformed_guard_does_not_refresh_last_confirmed_sample(telemetry, payload, guard_payload):
    module, publisher = telemetry
    publisher.send(b"g1_debug" + msgpack.packb(payload | {"reference_guard": guard_payload}))
    wait_until(lambda: module.get_status()["accepted_packets"] == 1, timeout=3.0)
    publisher.send(b"g1_debug" + msgpack.packb(payload | {"index": 13, "reference_guard": {}}))
    wait_until(lambda: module.get_status()["invalid_packets"] == 1, timeout=3.0)
    wait_until(lambda: module.get_status()["state"] == "stale", timeout=3.0)
    assert module.get_status()["reference_guard"] is None
    assert module.get_status()["accepted_packets"] == 1


@pytest.mark.parametrize("ending", ["stopped", "fault"])
def test_guard_confirmation_is_unavailable_when_receiver_ends(
    telemetry, payload, guard_payload, mocker, ending
):
    module, publisher = telemetry
    publisher.send(b"g1_debug" + msgpack.packb(payload | {"reference_guard": guard_payload}))
    wait_until(lambda: module.get_status()["accepted_packets"] == 1, timeout=3.0)
    if ending == "stopped":
        module.stop()
    else:
        mocker.patch.object(
            module.body_joint_state, "publish", side_effect=RuntimeError("transport lost")
        )
        publisher.send(
            b"g1_debug" + msgpack.packb(payload | {"index": 13, "reference_guard": guard_payload})
        )
        wait_until(lambda: module.get_status()["state"] == "fault", timeout=3.0)
    assert module.get_status()["state"] == ending
    assert module.get_status()["reference_guard"] is None
    assert module.get_feedback() is None
