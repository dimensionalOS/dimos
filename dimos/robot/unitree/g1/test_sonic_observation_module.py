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

from contextlib import ExitStack
from dataclasses import replace
from uuid import uuid4

import numpy as np
import pytest

from dimos.core.transport_factory import make_transport
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.robot.unitree.g1.sonic_feedback import SonicFeedback
from dimos.robot.unitree.g1.sonic_observation import POLICY_JOINT_NAMES
from dimos.robot.unitree.g1.sonic_observation_module import SonicObservationModule
from dimos.robot.unitree.g1.sonic_observation_pairing import SonicMeasuredState
from dimos.utils.testing.waiting import wait_until


@pytest.fixture
def config():
    return dict(
        camera_frame_id="d435_color_optical_frame",
        max_skew_seconds=0.03,
        camera_timestamp_error_seconds=0.0,
        left_hand_slot_names=POLICY_JOINT_NAMES[14:21],
        right_hand_slot_names=POLICY_JOINT_NAMES[21:28],
    )


@pytest.fixture
def clock(mocker):
    clocks = [1000.0, 10.0]
    mocker.patch(
        "dimos.robot.unitree.g1.sonic_observation_module.wall_time", side_effect=lambda: clocks[0]
    )
    mocker.patch(
        "dimos.robot.unitree.g1.sonic_observation_module.monotonic", side_effect=lambda: clocks[1]
    )
    return clocks


@pytest.fixture
def adapters(config, clock):
    with ExitStack() as cleanup:

        def create(**overrides):
            module = SonicObservationModule(**(config | overrides))
            cleanup.callback(module.stop)
            outputs = []
            for name in (*module.inputs, *module.outputs):
                port = getattr(module, name)
                module.set_transport(
                    name, make_transport(f"/observation-test/{uuid4().hex}/{name}", port.type)
                )
            cleanup.callback(module.observation.subscribe(outputs.append))
            module.start()
            return module, outputs

        yield create


@pytest.fixture
def adapter(adapters):
    return adapters()


@pytest.fixture
def image():
    return Image(
        data=np.full((480, 640, 3), [10, 20, 30], np.uint8),
        format=ImageFormat.RGB,
        frame_id="d435_color_optical_frame",
        ts=999.98,
    )


@pytest.fixture
def state():
    return SonicMeasuredState(0, dict.fromkeys(POLICY_JOINT_NAMES, 0.25), 9.98, 9.97, 9.99)


@pytest.fixture
def feedback():
    return SonicFeedback(
        index=1,
        source_timestamp=999.99,
        received_at=1000,
        received_monotonic=10,
        body_q=tuple(range(29)),
        body_dq=(0.0,) * 29,
        left_hand_q=tuple(range(7)),
        left_hand_dq=(0.0,) * 7,
        right_hand_q=tuple(range(10, 17)),
        right_hand_dq=(0.0,) * 7,
    )


def test_feedback_receipt_and_ros_time_never_unlock_service_observations(adapter, image, feedback):
    module, outputs = adapter
    module.handle_feedback(feedback)
    module.handle_color_image(image)
    status = module.get_observation_status()
    assert status["state"] == "waiting" and status["observations_published"] == 0
    assert status["readiness_reason"] == "measurement acquisition timestamps unavailable"
    assert status["diagnostic_state"] == list(range(15, 29)) + list(range(7)) + list(range(10, 17))
    assert status["camera_source_to_feedback_receipt_s"] == pytest.approx(0.02)
    assert outputs == []


def test_typed_inputs_publish_oldest_source_time_and_expire(adapter, image, state, clock):
    module, outputs = adapter
    module.measured_state.transport.publish(state)
    module.color_image.transport.publish(image)
    wait_until(lambda: len(outputs) == 1, timeout=2)
    assert outputs[0].observed_at == pytest.approx(9.97)
    np.testing.assert_array_equal(outputs[0].rgb[0, 0], [10, 20, 30])
    assert module.get_observation_status()["state"] == "fresh"
    clock[:] = [1000.21, 10.21]
    assert module.get_observation_status()["state"] == "stale"
    module.handle_feedback(
        SonicFeedback(
            2,
            None,
            1000.21,
            10.21,
            (0.0,) * 29,
            (0.0,) * 29,
            (0.0,) * 7,
            (0.0,) * 7,
            (0.0,) * 7,
            (0.0,) * 7,
        )
    )
    assert len(outputs) == 1


@pytest.mark.parametrize(
    "change",
    [dict(frame_id="wrong"), dict(format=ImageFormat.BGR), dict(ts=1000.1), dict(ts=999.0)],
)
def test_wrong_or_stale_camera_rejected(adapter, image, state, change):
    module, outputs = adapter
    module.handle_measured_state(state)
    module.handle_color_image(replace(image, **change))
    assert module.get_observation_status()["rejected_images"] == 1
    assert outputs == []


def test_clock_step_latches_fault_until_module_replaced(adapter, adapters, image, state, clock):
    module, outputs = adapter
    module.handle_measured_state(state)
    clock[:] = [1001.0, 10.01]
    module.handle_color_image(replace(image, ts=1000.99))
    assert module.get_observation_status()["state"] == "fault"
    clock[:] = [1000.02, 10.02]
    module.handle_color_image(image)
    assert module.get_observation_status()["state"] == "fault"
    assert outputs == []
    module.stop()
    replacement, _ = adapters()
    assert replacement.get_observation_status()["state"] == "waiting"


def test_stopped_module_rejects_rpcs_and_replacement_has_no_old_buffers(
    adapter, adapters, image, state
):
    module, outputs = adapter
    module.handle_measured_state(state)
    module.stop()
    with pytest.raises(RuntimeError, match="running module loop"):
        module.handle_color_image(image)
    assert module.get_observation_status()["state"] == "stopped"
    replacement, replacement_outputs = adapters()
    replacement.handle_color_image(image)
    assert replacement.get_observation_status()["observations_published"] == 0
    assert outputs == [] and replacement_outputs == []


def test_hand_slot_mapping_must_name_each_correct_finger_once(config):
    config["right_hand_slot_names"] = POLICY_JOINT_NAMES[14:21]
    with pytest.raises(ValueError):
        SonicObservationModule(**config)


def test_explicit_right_hand_mapping_is_used_for_diagnostics(adapters, feedback):
    module, _ = adapters(right_hand_slot_names=tuple(reversed(POLICY_JOINT_NAMES[21:28])))
    module.handle_feedback(feedback)
    assert module.get_observation_status()["diagnostic_state"][21:] == list(range(16, 9, -1))


def test_small_clock_drift_cannot_make_old_camera_frame_fresh(adapter, clock, image, state):
    module, outputs = adapter
    clock[:] = [1000.209, 10.2]  # +9 ms offset drift remains under the fault threshold.
    module.handle_measured_state(
        replace(
            state, body_observed_at=10.01, left_hand_observed_at=10.01, right_hand_observed_at=10.01
        )
    )
    module.handle_color_image(replace(image, ts=1000.008))
    assert module.get_observation_status()["rejected_images"] == 1
    assert outputs == []


def test_small_negative_drift_cannot_hide_excess_acquisition_skew(adapter, clock, image, state):
    module, outputs = adapter
    clock[:] = [999.991, 10.0]
    module.handle_measured_state(
        replace(
            state, body_observed_at=9.941, left_hand_observed_at=9.941, right_hand_observed_at=9.941
        )
    )
    module.handle_color_image(replace(image, ts=999.968))
    assert module.get_observation_status()["accepted_images"] == 1
    assert module.get_observation_status()["observations_published"] == 0
    assert outputs == []


def test_duplicate_start_preserves_pending_state(adapter, image, state):
    module, outputs = adapter
    module.handle_measured_state(state)
    module.start()
    module.handle_color_image(image)
    wait_until(lambda: len(outputs) == 1, timeout=2)
    assert outputs[0].observed_at == pytest.approx(9.97)


def test_clock_sampling_uncertainty_is_preserved_in_output(adapters, image, state, clock, mocker):
    samples = iter([10.0, 10.008])
    mocker.patch(
        "dimos.robot.unitree.g1.sonic_observation_module.monotonic",
        side_effect=lambda: next(samples, clock[1]),
    )
    clock[:] = [1000.004, 10.004]
    module, outputs = adapters()
    clock[:] = [1000.02, 10.02]
    module.handle_measured_state(
        replace(
            state, body_observed_at=9.99, left_hand_observed_at=9.99, right_hand_observed_at=9.99
        )
    )
    module.handle_color_image(replace(image, ts=999.99))
    wait_until(lambda: len(outputs) == 1, timeout=2)
    assert outputs[0].observed_at == pytest.approx(9.976)
    assert module.get_observation_status()["clock_mapping_uncertainty_s"] == pytest.approx(0.004)


def test_later_clock_sampling_uncertainty_counts_against_drift_limit(adapter, image, clock, mocker):
    module, outputs = adapter
    samples = iter([10.2, 10.208])
    clock[:] = [1000.213, 10.208]
    mocker.patch(
        "dimos.robot.unitree.g1.sonic_observation_module.monotonic",
        side_effect=lambda: next(samples, clock[1]),
    )
    module.handle_color_image(replace(image, ts=1000.15))
    assert module.get_observation_status()["state"] == "fault"
    assert outputs == []


def test_typed_state_uncertainty_is_preserved_in_age_and_service_snapshot(adapter, image, state):
    module, outputs = adapter
    module.measured_state.transport.publish(
        replace(state, acquisition_uncertainty_seconds=(0.001, 0.004, 0.001))
    )
    module.color_image.transport.publish(image)
    wait_until(lambda: len(outputs) == 1, timeout=2)
    assert outputs[0].observed_at == pytest.approx(9.966)
    assert module.get_observation_status()["measured_state_age_s"] == pytest.approx(0.034)


def test_invalid_state_uncertainty_rejected_without_unlocking_service(adapter, image, state):
    module, outputs = adapter
    module.handle_measured_state(replace(state, acquisition_uncertainty_seconds=(0, -1, 0)))
    module.handle_color_image(image)
    status = module.get_observation_status()
    assert status["accepted_measured_states"] == 0 and status["rejected_states"] == 1
    assert status["state"] == "waiting" and outputs == []
