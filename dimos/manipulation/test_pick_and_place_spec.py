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

"""Public object RPC contracts and agent formatting."""

import json
import pickle

import pytest

from dimos.manipulation.pick_and_place_module import PickAndPlaceModule
from dimos.manipulation.pick_and_place_spec import (
    DetectedObject,
    PickAndPlaceSpec,
    PickPlaceStatus,
    PickResult,
    PlaceResult,
    ScanResult,
)
from dimos.spec.utils import get_protocol_method_signatures, spec_annotation_compliance


def test_pick_and_place_spec_matches_advertised_rpcs() -> None:
    assert spec_annotation_compliance(PickAndPlaceModule, PickAndPlaceSpec)
    for name in get_protocol_method_signatures(PickAndPlaceSpec):
        assert getattr(PickAndPlaceModule, name).__rpc__


@pytest.mark.parametrize(
    "result",
    [
        ScanResult(
            PickPlaceStatus.SUCCEEDED,
            objects=(DetectedObject("cup-1", "cup"),),
            prompts=("cup",),
        ),
        PickResult(
            PickPlaceStatus.EXECUTION_FAILED,
            "retract failed",
            object_id="cup-1",
            holding_object=True,
            rank=0,
            score=0.8,
            candidates=2,
        ),
        PlaceResult(PickPlaceStatus.GRIPPER_FAILED, "release failed", holding_object=True),
    ],
)
def test_rpc_serialization_preserves_domain_results_and_agent_formatting(result) -> None:
    received = pickle.loads(pickle.dumps(result))

    assert received == result
    encoded = json.loads(received.agent_encode()[0]["text"])
    assert encoded["status"] == result.status.name
    assert encoded["succeeded"] == result.succeeded
    assert encoded["message"] == result.message


def test_scan_agent_output_contains_selectable_object_ids() -> None:
    result = ScanResult(PickPlaceStatus.SUCCEEDED, objects=(DetectedObject("cup-1", "cup"),))

    assert json.loads(result.agent_encode()[0]["text"])["objects"] == [
        {"object_id": "cup-1", "name": "cup"}
    ]
