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

import json

from pydantic import ValidationError
import pytest

from dimos.teleop.webxr.body_tracking import BodyTrackingSnapshot


def _payload(*, joints) -> str:
    return json.dumps(
        {
            "type": "body_tracking_snapshot",
            "capture_time_s": 1234.5,
            "frame_id": "bounded-floor",
            "joints": joints,
        }
    )


def test_body_tracking_snapshot_validates_named_poses() -> None:
    snapshot = BodyTrackingSnapshot.model_validate_json(
        _payload(
            joints={
                "hips": {
                    "position": [1.0, 2.0, 3.0],
                    "orientation": [0.1, 0.2, 0.3, 0.9],
                },
                "left-foot-ankle": {
                    "position": [-0.2, 0.1, 0.4],
                    "orientation": [0.0, 0.0, 0.0, 1.0],
                },
            }
        )
    )

    assert snapshot.capture_time_s == 1234.5
    assert snapshot.frame_id == "bounded-floor"
    assert snapshot.joints is not None
    assert list(snapshot.joints) == ["hips", "left-foot-ankle"]
    assert snapshot.joints["hips"].position == (1.0, 2.0, 3.0)
    assert snapshot.joints["hips"].orientation == (0.1, 0.2, 0.3, 0.9)


@pytest.mark.parametrize("joints", [None, {}])
def test_body_tracking_snapshot_preserves_absence_state(joints) -> None:
    snapshot = BodyTrackingSnapshot.model_validate_json(_payload(joints=joints))

    assert snapshot.joints == joints


@pytest.mark.parametrize(
    "payload",
    [
        "not json",
        '{"type":"unknown"}',
        _payload(joints={"hips": {"position": [1.0, 2.0], "orientation": [0, 0, 0, 1]}}),
        _payload(joints={"hips": {"position": [1.0, 2.0, 3.0], "orientation": [0, 0, 1]}}),
        _payload(joints={"": {"position": [1.0, 2.0, 3.0], "orientation": [0, 0, 0, 1]}}),
        _payload(joints={"hips": {"position": [True, 2.0, 3.0], "orientation": [0, 0, 0, 1]}}),
        '{"type":"body_tracking_snapshot","capture_time_s":NaN,"frame_id":"local-floor","joints":{}}',
        json.dumps(
            {
                "type": "body_tracking_snapshot",
                "capture_time_s": 1.0,
                "frame_id": "local-floor",
                "joints": {},
                "unexpected": True,
            }
        ),
    ],
)
def test_body_tracking_snapshot_rejects_malformed_payloads(payload: str) -> None:
    with pytest.raises(ValidationError):
        BodyTrackingSnapshot.model_validate_json(payload)
