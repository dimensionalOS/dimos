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

import numpy as np
import pytest

from dimos.robot.unitree.g1.sonic_delivery_observation import (
    SonicCameraDelivery,
    SonicDeliveryPairer,
    SonicJointDelivery,
)
from dimos.robot.unitree.g1.sonic_observation import (
    POLICY_JOINT_NAMES,
    SonicDeliveryStamp,
)


@pytest.fixture
def pairer():
    return SonicDeliveryPairer(
        left_hand_slot_names=POLICY_JOINT_NAMES[14:21],
        right_hand_slot_names=POLICY_JOINT_NAMES[21:28],
        max_age=0.2,
        max_span=0.1,
    )


def joints(group, marker=1, received_at=10.0, source_id=None):
    values = tuple(float(i) for i in range(29 if group == "body" else 7))
    return SonicJointDelivery(
        group, values, SonicDeliveryStamp(source_id or group, marker, received_at)
    )


def camera(marker=1, received_at=10.01, source_id="camera"):
    return SonicCameraDelivery(
        np.full((480, 640, 3), [10, 20, 30], np.uint8),
        SonicDeliveryStamp(source_id, marker, received_at),
    )


def feed(pairer, marker=1, start=10.0, missing=None):
    for group in ("body", "left_hand", "right_hand"):
        if group != missing:
            pairer.push_joints(joints(group, marker, start), now=start + 0.01)
    if missing != "camera":
        return pairer.push_camera(camera(marker, start + 0.01), now=start + 0.02)
    return None


def test_delivery_snapshot_uses_oldest_receipt_and_preserves_named_state(pairer):
    obs = feed(pairer)
    assert obs is not None and obs.time_basis == "delivery"
    assert obs.observed_at == 10.0
    np.testing.assert_array_equal(obs.state, list(range(15, 29)) + list(range(7)) * 2)
    np.testing.assert_array_equal(obs.rgb[0, 0], [10, 20, 30])
    assert tuple(s.source_id for s in obs.delivery_stamps) == (
        "camera",
        "body",
        "left_hand",
        "right_hand",
    )
    status = pairer.get_status(now=10.03)
    assert status["acquisition_age_s"] is None and status["acquisition_skew_s"] is None
    assert status["delivery_ages_s"]["left_hand"] == pytest.approx(0.03)


@pytest.mark.parametrize("missing", ["camera", "body", "left_hand", "right_hand"])
def test_each_stopped_input_prevents_new_pairs_and_expires(pairer, missing):
    assert feed(pairer) is not None
    assert feed(pairer, marker=2, start=10.05, missing=missing) is None
    assert feed(pairer, marker=3, start=10.22, missing=missing) is None
    assert pairer.get_status(now=10.25)["state"] == "stale"
    assert pairer.get_status(now=10.25)["delivery_ages_s"][missing] >= 0.2


def test_repeated_marker_cannot_refresh_receipt_or_emit(pairer):
    assert feed(pairer) is not None
    with pytest.raises(ValueError, match="duplicate"):
        pairer.push_joints(joints("left_hand", received_at=10.1), now=10.1)
    assert pairer.get_status(now=10.11)["delivery_ages_s"]["left_hand"] == pytest.approx(0.11)


@pytest.mark.parametrize("group", ["camera", "body", "left_hand", "right_hand"])
@pytest.mark.parametrize("change", ["restart", "rollback"])
def test_source_change_or_marker_rollback_latches_session_fault(pairer, group, change):
    assert feed(pairer, marker=5) is not None
    marker = 1 if change == "rollback" else 6
    source = "replacement" if change == "restart" else group
    with pytest.raises(ValueError, match="replace"):
        if group == "camera":
            pairer.push_camera(camera(marker, 10.05, source), now=10.06)
        else:
            pairer.push_joints(joints(group, marker, 10.05, source), now=10.06)
    assert pairer.get_status(now=10.07)["state"] == "fault"
    with pytest.raises(ValueError, match="replace"):
        feed(pairer, marker=10, start=10.1)


@pytest.mark.parametrize("received_at", [9.7, 10.2])
def test_stale_or_future_receipt_does_not_consume_marker(pairer, received_at):
    with pytest.raises(ValueError, match="receipt"):
        pairer.push_joints(joints("body", received_at=received_at), now=10.0)
    assert pairer.push_joints(joints("body"), now=10.01) is None


def test_delivery_span_budget_is_separate_from_sensor_skew(pairer):
    for group in ("body", "left_hand", "right_hand"):
        pairer.push_joints(joints(group), now=10.01)
    assert pairer.push_camera(camera(received_at=10.15), now=10.16) is None
    status = pairer.get_status(now=10.17)
    assert status["delivery_span_s"] == pytest.approx(0.15)
    assert status["acquisition_skew_s"] is None


def test_receiver_record_owns_image_before_transport_or_pairing(pairer):
    rgb = np.ones((480, 640, 3), np.uint8)
    record = SonicCameraDelivery(rgb, SonicDeliveryStamp("camera", 1, 10.0))
    rgb[:] = 99
    pairer.push_camera(record, now=10.01)
    for group in ("body", "left_hand", "right_hand"):
        obs = pairer.push_joints(joints(group), now=10.02)
    assert obs is not None and np.all(obs.rgb == 1)
    assert not record.rgb.flags.writeable


def test_wrong_hand_mapping_cannot_create_pairer():
    with pytest.raises(ValueError, match="mapping"):
        SonicDeliveryPairer(
            left_hand_slot_names=POLICY_JOINT_NAMES[21:28],
            right_hand_slot_names=POLICY_JOINT_NAMES[14:21],
            max_age=0.2,
            max_span=0.1,
        )


@pytest.mark.parametrize(
    "change",
    [
        {"source_id": ""},
        {"source_marker": float("nan")},
        {"source_marker": True},
        {"received_at": float("inf")},
    ],
)
def test_invalid_receipt_provenance_rejected(change):
    with pytest.raises(ValueError):
        SonicDeliveryStamp(**(dict(source_id="body", source_marker=1, received_at=10.0) | change))


@pytest.mark.parametrize(
    "change",
    [
        {"time_basis": "unknown"},
        {"delivery_stamps": None},
        {"delivery_stamps": ()},
        {"observed_at": 10.1},
    ],
)
def test_delivery_observation_cannot_omit_or_relabel_oldest_receipt(pairer, change):
    obs = feed(pairer)
    with pytest.raises(ValueError):
        replace(obs, **change)


def test_delivery_metadata_cannot_be_presented_as_acquisition(pairer):
    obs = feed(pairer)
    with pytest.raises(ValueError, match="acquisition"):
        replace(obs, time_basis="acquisition")


def test_freshness_checks_all_receipts_after_deserialization(pairer):
    obs = feed(pairer)
    # A later receipt cannot be hidden behind the oldest timestamp.
    later = replace(obs.delivery_stamps[0], received_at=10.15)
    altered = replace(obs, delivery_stamps=(later, *obs.delivery_stamps[1:]))
    with pytest.raises(ValueError, match="receipt"):
        altered.require_fresh(now=10.1)
