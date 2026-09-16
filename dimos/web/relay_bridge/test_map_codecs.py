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

"""The map panel's nav codecs: path.json.v1 out, point.json.v1 and bool.json.v1 in."""

import math

from dimos_lcm.std_msgs import Bool
import pytest

from dimos.msgs.geometry_msgs.PointStamped import PointStamped
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.nav_msgs.Path import Path
from dimos.web.codecs import resolve_decoder, resolve_encoder
from dimos.web.relay_bridge.builtin_codecs import decode_bool, decode_point, encode_path


def _pose(x: float, y: float) -> PoseStamped:
    return PoseStamped(ts=1.0, position=[x, y, 0.0], orientation=[0.0, 0.0, 0.0, 1.0])


def test_path_encodes_a_rounded_xy_polyline() -> None:
    path = Path(ts=1.0, frame_id="world", poses=[_pose(1.23456, -2.0), _pose(0.0004, 3.5)])
    assert encode_path(path) == b"[[1.235,-2.0],[0.0,3.5]]"
    assert resolve_encoder("path.json.v1", Path).encode is encode_path


def test_empty_path_encodes_as_a_clear() -> None:
    # The planner publishes Path() on cancel and arrival: the overlay must go.
    assert encode_path(Path()) == b"[]"


def test_point_decodes_a_click() -> None:
    point = decode_point({"x": 1.5, "y": -2})
    assert isinstance(point, PointStamped)
    assert (point.x, point.y, point.z, point.frame_id) == (1.5, -2.0, 0.0, "world")
    assert resolve_decoder("point.json.v1", PointStamped).decode is decode_point


@pytest.mark.parametrize(
    "value",
    [
        [1.5, 2.0],
        {"x": 1.5},
        {"x": "1.5", "y": 2.0},
        {"x": True, "y": 2.0},
        {"x": math.inf, "y": 0.0},
    ],
    ids=["list", "missing_y", "string", "bool", "inf"],
)
def test_point_rejects_a_malformed_click(value: object) -> None:
    with pytest.raises(ValueError, match="point.json.v1"):
        decode_point(value)


def test_bool_decodes_to_std_msgs_bool() -> None:
    assert isinstance(decode_bool(True), Bool)
    assert decode_bool(True).data is True
    assert decode_bool(False).data is False
    assert resolve_decoder("bool.json.v1", Bool).decode is decode_bool
    with pytest.raises(ValueError, match="bool.json.v1"):
        decode_bool(1)
