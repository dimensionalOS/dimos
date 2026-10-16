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
from uuid import uuid4

import pytest

from dimos.core.transport_factory import make_transport
from dimos.robot.unitree.g1.sonic_delivery_observation_module import SonicDeliveryObservationModule
from dimos.robot.unitree.g1.sonic_observation import POLICY_JOINT_NAMES
from dimos.robot.unitree.g1.test_sonic_delivery_observation import camera, joints
from dimos.utils.testing.waiting import wait_until


@pytest.fixture
def clock(mocker):
    now = [10.03]
    mocker.patch(
        "dimos.robot.unitree.g1.sonic_delivery_observation_module.monotonic",
        side_effect=lambda: now[0],
    )
    return now


@pytest.fixture
def adapter(clock):
    module = SonicDeliveryObservationModule(
        left_hand_slot_names=POLICY_JOINT_NAMES[14:21],
        right_hand_slot_names=POLICY_JOINT_NAMES[21:28],
        max_delivery_span_seconds=0.1,
    )
    outputs = []
    with ExitStack() as cleanup:
        cleanup.callback(module.stop)
        for name in (*module.inputs, *module.outputs):
            port = getattr(module, name)
            module.set_transport(
                name, make_transport(f"/delivery-test/{uuid4().hex}/{name}", port.type)
            )
        cleanup.callback(module.observation.subscribe(outputs.append))
        module.start()
        yield module, outputs


def publish_all(module):
    for group in ("body", "left_hand", "right_hand"):
        getattr(module, f"{group}_delivery").transport.publish(joints(group))
    module.camera_delivery.transport.publish(camera())


def test_typed_delivery_streams_keep_original_receiver_time(adapter, clock):
    module, outputs = adapter
    publish_all(module)
    wait_until(lambda: len(outputs) == 1, timeout=2)
    assert outputs[0].observed_at == 10.0
    assert outputs[0].time_basis == "delivery"
    assert module.get_observation_status()["acquisition_age_s"] is None
    clock[0] = 10.25
    assert module.get_observation_status()["state"] == "stale"


def test_fault_survives_duplicate_start_and_cannot_be_reset_by_restart(adapter):
    module, outputs = adapter
    publish_all(module)
    wait_until(lambda: len(outputs) == 1, timeout=2)
    module.handle_left_hand_delivery(
        joints("left_hand", marker=2, received_at=10.02, source_id="replacement")
    )
    module.start()
    assert module.get_observation_status()["state"] == "fault"
    assert module.get_observation_status()["rejected_deliveries"] == 1
    assert len(outputs) == 1
    module.stop()
    with pytest.raises(RuntimeError, match="replace"):
        module.start()


def test_stopped_adapter_does_not_accept_or_publish(adapter):
    module, outputs = adapter
    module.stop()
    with pytest.raises(RuntimeError, match="running module"):
        module.handle_camera_delivery(camera())
    assert module.get_observation_status()["state"] == "stopped"
    assert outputs == []
