# Copyright 2025-2026 Dimensional Inc.
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

from dimos_lcm.actionlib_msgs import GoalID, GoalStatus
from dimos_lcm.sensor_msgs import Image
from dimos_lcm.std_msgs import Time
import pytest

from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.protocol import DimosMsg
from dimos.orchestration.expressions import UNKNOWN, Plain, Shape
from dimos.orchestration.plain import from_plain, shape_of, to_plain

STATUS: dict[str, Plain] = {
    "goal_id": {"stamp": {"sec": 1000, "nsec": 25}, "id": "goal-1"},
    "status": 3,
    "text": "reached",
}
POSE: dict[str, Plain] = {
    "header": {"seq": 0, "stamp": {"sec": 1000, "nsec": 250000000}, "frame_id": "map"},
    "pose": {
        "position": {"x": 1.0, "y": 2.0, "z": 3.0},
        "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
    },
}
MESSAGES: list[tuple[DimosMsg, dict[str, Plain]]] = [
    (
        GoalStatus(goal_id=GoalID(stamp=Time(1000, 25), id="goal-1"), status=3, text="reached"),
        STATUS,
    ),
    (PoseStamped(position=(1.0, 2.0, 3.0), frame_id="map", ts=1000.25), POSE),
]
SHAPES: list[tuple[object, Shape]] = [
    (GoalStatus, {"goal_id": {"stamp": {"sec": 0, "nsec": 0}, "id": ""}, "status": 0, "text": ""}),
    (bool, False),
    (int, 0),
    (float, 0.0),
    (str, ""),
    (None, None),
    (Image, UNKNOWN),
    (list[str], UNKNOWN),
]


def _typed(value: Shape) -> object:
    """A value with the type of every leaf, since 0, 0.0 and False compare equal."""
    if isinstance(value, dict):
        return {key: _typed(item) for key, item in value.items()}
    return (type(value), value)


@pytest.mark.parametrize(("message", "plain"), MESSAGES)
def test_a_message_round_trips_through_plain_data(
    message: DimosMsg, plain: dict[str, Plain]
) -> None:
    assert _typed(to_plain(message)) == _typed(plain)
    back = from_plain(plain, type(message))
    assert type(back) is type(message)
    assert to_plain(back) == plain


@pytest.mark.parametrize(("annotation", "shape"), SHAPES)
def test_shape_of(annotation: object, shape: Shape) -> None:
    assert _typed(shape_of(annotation)) == _typed(shape)


def test_a_field_left_out_is_the_default_of_its_type() -> None:
    # This encode writes its header into the default that every generated PoseStamped shares.
    PoseStamped(frame_id="map", ts=1000.25).lcm_encode()
    built = to_plain(from_plain({"pose": {"position": {"x": 1.0}}}, PoseStamped))
    assert isinstance(built, dict)
    assert built["header"] == {"seq": 0, "stamp": {"sec": 0, "nsec": 0}, "frame_id": ""}


def test_a_field_that_is_not_plain_data_is_an_error() -> None:
    with pytest.raises(TypeError, match="`camera.data` is bytes"):
        to_plain(Image(), "camera")
