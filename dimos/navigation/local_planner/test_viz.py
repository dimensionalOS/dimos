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

"""The plan-body overlay."""

from dataclasses import replace
import math

from dimos_cdrgen.registry import decode, encode
import numpy as np
from numpy.typing import NDArray
import rerun as rr

from dimos.message_utils.geometry import quaternion_from_euler
from dimos.message_utils.time import time_from_nanoseconds, time_from_seconds, to_nanoseconds
from dimos.messages.geometry_msgs.msg import Point, Pose, PoseStamped
from dimos.messages.nav_msgs.msg import Path
from dimos.messages.std_msgs.msg import Header
from dimos.navigation.embodiment.go2 import GO2
from dimos.navigation.local_planner.viz import (
    motion_visual_override,
    plan_clearance,
    render_body,
    render_plan,
)


def _pose(x: float, y: float, yaw: float = 0.0) -> PoseStamped:
    return PoseStamped(
        header=Header(
            frame_id="odom",
            stamp=time_from_nanoseconds(1_700_000_000_000_000_000 + round(x * 10000)),
        ),
        pose=Pose(position=Point(x=x, y=y, z=0.0), orientation=quaternion_from_euler(0, 0, yaw)),
    )


def _plan(n: int = 40, yaw: float = 0.0) -> Path:
    return Path(
        header=Header(frame_id="odom", stamp=time_from_nanoseconds(0)),
        poses=[_pose(k * 0.1, 0.0, yaw) for k in range(n)],
    )


def _stamp(plan: Path, speeds: NDArray[np.float64]) -> Path:
    """Per-segment speeds as stamp deltas over the 0.1 m spacing, the planner's dialect."""
    ts = np.concatenate([[0.0], np.cumsum(0.1 / speeds)])
    for pose, t in zip(plan.poses, ts, strict=True):
        pose.header.stamp = time_from_seconds(float(t))
    return plan


def _rgba(boxes) -> NDArray[np.float64]:
    """rerun packs colors as u32 0xRRGGBBAA, not as rows."""
    packed = np.asarray(boxes.colors.pa_array.to_pylist(), dtype=np.uint32)
    return np.column_stack(
        [(packed >> 24) & 0xFF, (packed >> 16) & 0xFF, (packed >> 8) & 0xFF, packed & 0xFF]
    )


def test_cdr_plan_draws_without_mutating_exact_source_values() -> None:
    plan = _plan(n=2, yaw=math.pi / 2)
    plan.header.stamp = time_from_nanoseconds(1_700_000_000_123_456_789)
    payload = encode(plan)
    received = decode(payload, Path)
    drawn = render_plan(received, GO2)
    assert drawn is not None
    assert drawn[0] == ("world/path", rr.Transform3D(parent_frame="tf#/odom"))
    assert len(drawn[1][1].centers.pa_array) == 2
    assert to_nanoseconds(received.header.stamp) == 1_700_000_000_123_456_789
    assert encode(received) == payload


def test_boxes_are_subsampled_by_arc_not_by_index() -> None:
    """Every waypoint would be an opaque wall hiding the geometry it shows."""
    plan = _plan()  # 3.9 m of plan discretised at 0.1 m
    boxes = render_body(plan, GO2, stride_m=0.35)
    n = len(boxes.centers.pa_array)
    assert 8 <= n <= 14, f"{n} boxes for 3.9 m at 0.35 m stride"
    # ...and the stride is what sets it, not the waypoint count
    assert len(render_body(plan, GO2, stride_m=1.0).centers.pa_array) < n


def test_colour_tracks_the_precision_the_planner_stamped() -> None:
    """The colour channel is decoded from the path, not passed alongside it."""
    fast, slow = GO2.max_speed, GO2.min_speed
    plan = _stamp(
        _plan(), np.concatenate([np.full(14, fast), np.full(11, slow), np.full(14, fast)])
    )

    room = plan_clearance(plan, GO2)
    assert room is not None
    assert room.min() <= 0.05 and room.max() >= 0.34

    colors = _rgba(render_body(plan, GO2))
    reds = [c for c in colors if c[0] > 200 and c[1] < 100]
    greens = [c for c in colors if c[1] > 200 and c[0] < 100]
    assert len(reds), "the pinched middle drew no at-floor box"
    assert len(greens), "the open ends drew no roomy box"


def test_an_unstamped_plan_draws_one_flat_colour() -> None:
    """No pinch is claimed where no precision was stamped.

    The fixture carries deterministic stamps microseconds apart, so the
    decoder clips the absurd speed to cruise rather than returning None. It
    must not manufacture a varying profile out of noise: every box comes out
    the same colour.
    """
    plan = _plan()  # never stamped
    room = plan_clearance(plan, GO2)
    assert room is not None and len(np.unique(np.round(room, 6))) == 1
    colors = _rgba(render_body(plan, GO2))
    assert len({tuple(c[:3]) for c in colors}) == 1


def test_the_veto_stub_is_drawn_rather_than_blanked() -> None:
    """An empty viewport reads as a dead module; a refusal must look refused."""
    veto = render_body(
        Path(
            header=Header(frame_id="odom", stamp=time_from_nanoseconds(0)),
            poses=[_pose(1.0, 2.0, 0.5)],
        ),
        GO2,
    )
    assert veto is not None
    assert len(veto.centers.pa_array) == 1
    # an EMPTY path is different: hold the last good picture
    assert (
        render_body(
            Path(header=Header(frame_id="odom", stamp=time_from_nanoseconds(0)), poses=[]), GO2
        )
        is None
    )


def test_the_box_sits_on_the_body_not_on_the_pose_point() -> None:
    """center_off is along the pose's own heading, so yaw has to rotate it."""
    facing_y = render_body(_plan(yaw=math.pi / 2), replace(GO2, center_off=-0.10, envelope=()))
    cx, cy, _ = facing_y.centers.pa_array.to_pylist()[0]
    # the yaw round-trips through a float32 quaternion, so this is not exact
    assert abs(cx - 0.0) < 1e-6, "offset leaked into x while facing +y"
    assert abs(cy - (-0.10)) < 1e-6


def test_the_plan_pins_its_frame_and_draws_its_body_off_the_one_path_topic() -> None:
    drawn = render_plan(_plan(), GO2)
    assert drawn is not None
    by_entity = {e: type(a) for e, a in drawn}
    assert by_entity == {"world/path": rr.Transform3D, "world/path/body": rr.Boxes3D}
    assert drawn[0] == ("world/path", rr.Transform3D(parent_frame="tf#/odom"))
    # a refusal stub is drawn too, so it is not mistaken for a dead module
    veto = render_plan(
        Path(
            header=Header(frame_id="odom", stamp=time_from_nanoseconds(0)), poses=[_pose(1.0, 2.0)]
        ),
        GO2,
    )
    assert veto is not None and veto[-1][0] == "world/path/body"
    # an empty path is "no route": hold the last picture rather than blank it
    assert (
        render_plan(
            Path(header=Header(frame_id="odom", stamp=time_from_nanoseconds(0)), poses=[]), GO2
        )
        is None
    )
    assert set(motion_visual_override(GO2, body_dilate_m=-0.03)) == {
        "world/path",
        "world/ground_support",
    }
