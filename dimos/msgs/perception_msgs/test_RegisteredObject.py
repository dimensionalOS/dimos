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

from typing import cast

import pytest
import rerun as rr

from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.perception_msgs.RegisteredObject import RegisteredObject


def test_registered_object_lcm_round_trip_preserves_bounds() -> None:
    msg = RegisteredObject(
        object_id="obj-7",
        name="mug",
        center=Vector3(1.0, 2.0, 3.0),
        size=Vector3(0.1, 0.2, 0.3),
        frame_id="map",
        ts=123.5,
    )

    decoded = RegisteredObject.lcm_decode(msg.lcm_encode())

    assert decoded.object_id == "obj-7"
    assert decoded.name == "mug"
    assert decoded.frame_id == "map"
    assert decoded.ts == pytest.approx(123.5)
    assert (decoded.center.x, decoded.center.y, decoded.center.z) == pytest.approx((1.0, 2.0, 3.0))
    assert (decoded.size.x, decoded.size.y, decoded.size.z) == pytest.approx((0.1, 0.2, 0.3))


def test_registered_object_to_rerun_exposes_box_archetype() -> None:
    msg = RegisteredObject(
        object_id="obj-7",
        name="mug",
        center=Vector3(1.0, 2.0, 3.0),
        size=Vector3(0.1, 0.2, 0.3),
        ts=123.5,
    )

    boxes = msg.to_rerun()

    assert isinstance(boxes, rr.Boxes3D)
    boxes = cast("rr.Boxes3D", boxes)
    assert boxes.centers.as_arrow_array().to_pylist() == [[1.0, 2.0, 3.0]]
    assert boxes.half_sizes.as_arrow_array().to_pylist()[0] == pytest.approx([0.05, 0.1, 0.15])
    assert boxes.labels.as_arrow_array().to_pylist() == ["mug:obj-7"]
