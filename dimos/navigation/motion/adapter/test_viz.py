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

"""The plan-body overlay and the stall reporter."""

from dataclasses import replace
import math
from types import SimpleNamespace

import numpy as np

from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.Path import Path
from dimos.navigation.motion.adapter.diagnostics import StallReporter
from dimos.navigation.motion.adapter.viz import (
    motion_visual_override,
    plan_clearance,
    render_plan_body,
)
from dimos.navigation.motion.control.profile import encode_precision
from dimos.navigation.motion.embodiment.go2 import GO2


def _pose(x: float, y: float, yaw: float = 0.0) -> PoseStamped:
    return PoseStamped(
        frame_id="odom",
        position=Vector3(x, y, 0.0),
        orientation=Quaternion.from_euler(Vector3(0, 0, yaw)),
    )


def _plan(n: int = 40, yaw: float = 0.0) -> Path:
    return Path(frame_id="odom", poses=[_pose(k * 0.1, 0.0, yaw) for k in range(n)])


def _rgba(boxes) -> np.ndarray:  # type: ignore[no-untyped-def]
    """rerun packs colors as u32 0xRRGGBBAA, not as rows."""
    packed = np.asarray(boxes.colors.pa_array.to_pylist(), dtype=np.uint32)
    return np.column_stack(
        [(packed >> 24) & 0xFF, (packed >> 16) & 0xFF, (packed >> 8) & 0xFF, packed & 0xFF]
    )


def test_boxes_are_subsampled_by_arc_not_by_index() -> None:
    """Every waypoint would be an opaque wall hiding the geometry it shows."""
    plan = _plan()  # 3.9 m of plan discretised at 0.1 m
    boxes = render_plan_body(plan, stride_m=0.35)
    n = len(boxes.centers.pa_array)
    assert 8 <= n <= 14, f"{n} boxes for 3.9 m at 0.35 m stride"
    # ...and the stride is what sets it, not the waypoint count
    assert len(render_plan_body(plan, stride_m=1.0).centers.pa_array) < n


def test_colour_tracks_the_precision_the_planner_stamped() -> None:
    """The colour channel is decoded from the path, not passed alongside it."""
    plan = _plan()
    tight = np.concatenate([np.full(15, 0.6), np.full(10, 0.02), np.full(15, 0.6)])
    encode_precision(plan, tight, GO2)

    room = plan_clearance(plan, GO2)
    assert room is not None
    assert room.min() <= 0.05 and room.max() >= 0.34

    colors = _rgba(render_plan_body(plan))
    reds = [c for c in colors if c[0] > 200 and c[1] < 100]
    greens = [c for c in colors if c[1] > 200 and c[0] < 100]
    assert len(reds), "the pinched middle drew no at-floor box"
    assert len(greens), "the open ends drew no roomy box"


def test_an_unstamped_plan_draws_one_flat_colour() -> None:
    """No pinch is claimed where no precision was stamped.

    Freshly built poses carry wall-clock stamps microseconds apart, so the
    decoder sees an absurd speed and clips it to cruise rather than returning
    None -- which is the safe direction (a stamp can only ask for MORE care).
    What matters here is that it cannot manufacture a varying profile out of
    noise: every box comes out the same colour.
    """
    plan = _plan()  # never passed through encode_precision
    room = plan_clearance(plan, GO2)
    assert room is None or len(np.unique(np.round(room, 6))) == 1
    colors = _rgba(render_plan_body(plan))
    assert len({tuple(c[:3]) for c in colors}) == 1


def test_the_veto_stub_is_drawn_rather_than_blanked() -> None:
    """An empty viewport reads as a dead module; a refusal must look refused."""
    veto = render_plan_body(Path(frame_id="odom", poses=[_pose(1.0, 2.0, 0.5)]))
    assert veto is not None
    assert len(veto.centers.pa_array) == 1
    # an EMPTY path is different: hold the last good picture
    assert render_plan_body(Path(frame_id="odom", poses=[])) is None


def test_the_box_sits_on_the_body_not_on_the_pose_point() -> None:
    """center_off is along the pose's own heading, so yaw has to rotate it."""
    facing_y = render_plan_body(
        _plan(yaw=math.pi / 2), replace(GO2, tag="t", center_off=-0.10, envelope=())
    )
    cx, cy, _ = facing_y.centers.pa_array.to_pylist()[0]
    # the yaw round-trips through a float32 quaternion, so this is not exact
    assert abs(cx - 0.0) < 1e-6, "offset leaked into x while facing +y"
    assert abs(cy - (-0.10)) < 1e-6


def test_the_override_is_off_when_the_planner_is_not_publishing() -> None:
    assert motion_visual_override(0.0)["world/plan_body"] is None
    assert motion_visual_override(2.0)["world/plan_body"] is not None


def _said(monkeypatch) -> list[str]:  # type: ignore[no-untyped-def]
    """Capture the reporter's own lines; the project logger bypasses caplog."""
    from dimos.navigation.motion.adapter import diagnostics

    lines: list[str] = []
    sink = SimpleNamespace(
        info=lambda msg, **kw: lines.append(msg),
        warning=lambda msg, **kw: lines.append(msg),
    )
    monkeypatch.setattr(diagnostics, "logger", sink)
    return lines


def test_stall_reporter_names_the_first_missing_input(monkeypatch) -> None:  # type: ignore[no-untyped-def]
    """Ordered dependencies: the first absent input is the cause, not a symptom."""
    lines = _said(monkeypatch)
    r = StallReporter("Stage", heartbeat_s=100.0)
    assert not r.check({"odometry": False, "path": False})
    assert len(lines) == 1 and "odometry" in lines[0]
    assert "path" not in lines[0], "reported a symptom alongside its cause"


def test_stall_reporter_is_edge_triggered(monkeypatch) -> None:  # type: ignore[no-untyped-def]
    """The same complaint every tick at 10 Hz is not a diagnostic."""
    lines = _said(monkeypatch)
    r = StallReporter("Stage", heartbeat_s=100.0)
    for _ in range(20):
        r.check({"odometry": False})
    assert sum("waiting on odometry" in ln for ln in lines) == 1
    # and recovery is announced, so the log says when it started working
    lines.clear()
    r.check({"odometry": True})
    assert any("resuming" in ln for ln in lines)


def test_stall_reporter_distinguishes_stages(monkeypatch) -> None:  # type: ignore[no-untyped-def]
    """The whole point is answering "planner or follower?" without bisecting."""
    lines = _said(monkeypatch)
    StallReporter("MotionPlanner", 100.0).check({"local_map": False})
    StallReporter("TrajectoryFollower", 100.0).check({"path (local plan)": False})
    assert any(ln.startswith("MotionPlanner:") for ln in lines)
    assert any(ln.startswith("TrajectoryFollower:") for ln in lines)
