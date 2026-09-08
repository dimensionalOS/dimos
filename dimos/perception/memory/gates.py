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

"""Per-frame gates and lookups over a memory recording: poses, stillness, frames.

Two per-frame gates live here and both are required:

* The **camera-motion gate** differences tf. It rejects frames captured while
  the wrist sweeps, because a stale or interpolated transform smears the
  projection.
* The **scene-motion gate** differences images, conditioned on camera
  stillness at both compared instants. It rejects frames captured while the
  scene itself changes - a parked camera watching hands rearrange objects is
  exactly the case tf cannot see. The reference frame is anchored at the
  start of the surrounding camera-still interval, so a frame is trusted only
  while the scene still matches the state it had when the camera parked.

Every function here takes the store and/or tf plus primitives; the stillness
intervals and the grayscale memo are allocated by the caller and passed in,
one per query.
"""

from __future__ import annotations

from bisect import bisect_right
from typing import TYPE_CHECKING, Any

import numpy as np

if TYPE_CHECKING:
    from dimos.memory.type.observation import Observation
    from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
    from dimos.msgs.sensor_msgs.Image import Image
    from dimos.protocol.tf.tf import TFLookup

OPTICAL_FRAME = "camera_color_optical_frame"
WORLD_FRAME = "world"

# One world-pose period plus margin.
TF_TOLERANCE = 0.12

SPEED_MAX = 0.02  # m/s - camera counts as still below this
STILL_ENVELOPE = 0.15  # s - stillness must hold over the whole capture envelope

# Scene-motion gate: downscaled grayscale absolute difference.
DIFF_PIXEL_THRESHOLD = 28  # gray levels - per-pixel change floor
MOTION_THRESHOLD = 0.02  # fraction of changed pixels that flags scene motion
SHORT_DIFF_DT = 0.45  # s - bilateral diff span for active motion
DIFF_WIDTH = 212  # px - diff resolution (1/4 of 848)


def camera_pose(
    tf: TFLookup,
    ts: float,
    optical_frame: str = OPTICAL_FRAME,
    world_frame: str = WORLD_FRAME,
    tolerance: float = TF_TOLERANCE,
) -> PoseStamped | None:
    """World pose of the camera optical frame at ts - it rides the wrist."""
    transform = tf.get(optical_frame, world_frame, ts, tolerance)
    return (-transform).to_pose() if transform is not None else None


def camera_speed(
    tf: TFLookup,
    ts: float,
    optical_frame: str = OPTICAL_FRAME,
    world_frame: str = WORLD_FRAME,
    tolerance: float = TF_TOLERANCE,
    dt: float = 0.06,
) -> float | None:
    """Linear speed of the camera (m/s) around ts, from tf differencing."""
    a = camera_pose(tf, ts - dt, optical_frame, world_frame, tolerance)
    b = camera_pose(tf, ts + dt, optical_frame, world_frame, tolerance)
    if a is None or b is None:
        return None
    return float((b.position - a.position).magnitude() / (2 * dt))


def camera_still(
    tf: TFLookup,
    ts: float,
    optical_frame: str = OPTICAL_FRAME,
    world_frame: str = WORLD_FRAME,
    tolerance: float = TF_TOLERANCE,
    speed_max: float = SPEED_MAX,
    envelope: float = STILL_ENVELOPE,
) -> bool:
    """Camera is still over the whole capture envelope, not just at ts."""
    for offset in (-envelope, 0.0, envelope):
        speed = camera_speed(tf, ts + offset, optical_frame, world_frame, tolerance)
        if speed is None or speed > speed_max:
            return False
    return True


def still_intervals(
    tf: TFLookup,
    t0: float,
    t1: float,
    optical_frame: str = OPTICAL_FRAME,
    world_frame: str = WORLD_FRAME,
    tolerance: float = TF_TOLERANCE,
    speed_max: float = SPEED_MAX,
) -> list[tuple[float, float]]:
    """Maximal camera-still intervals inside [t0, t1], sampled at 0.25 s.

    Computed once per query by the caller; every scene-motion query resolves
    its surrounding interval from this list.
    """
    step = 0.25
    times = np.arange(t0, t1 + step, step)
    intervals: list[tuple[float, float]] = []
    run_start: float | None = None
    for t in times:
        speed = camera_speed(tf, float(t), optical_frame, world_frame, tolerance)
        still = speed is not None and speed <= speed_max
        if still and run_start is None:
            run_start = float(t)
        elif not still and run_start is not None:
            intervals.append((run_start, float(t) - step))
            run_start = None
    if run_start is not None:
        intervals.append((run_start, float(times[-1])))
    return [(a, b) for a, b in intervals if b >= a]


def _interval_containing(
    ts: float, intervals: list[tuple[float, float]]
) -> tuple[float, float] | None:
    idx = bisect_right([a for a, _ in intervals], ts) - 1
    if idx < 0:
        return None
    a, b = intervals[idx]
    return (a, b) if a - 0.25 <= ts <= b + 0.25 else None


def _gray_small(store: Any, ts: float, gray: dict[float, np.ndarray | None]) -> np.ndarray | None:
    """Downscaled grayscale of the color frame nearest ts, memoized in *gray*."""
    key = round(ts, 2)
    if key in gray:
        return gray[key]

    import cv2

    small_gray: np.ndarray | None = None
    try:
        frame = store.streams.color_image.at(ts, 0.1).first().data
    except LookupError:
        frame = None
    if frame is not None:
        img = frame.to_opencv()
        h = int(img.shape[0] * DIFF_WIDTH / img.shape[1])
        small = cv2.resize(img, (DIFF_WIDTH, h), interpolation=cv2.INTER_AREA)
        small_gray = cv2.cvtColor(small, cv2.COLOR_BGR2GRAY)

    gray[key] = small_gray
    return small_gray


def _diff_fraction(
    store: Any, ts_a: float, ts_b: float, gray: dict[float, np.ndarray | None]
) -> float | None:
    """Fraction of pixels changed between the frames nearest the two instants."""
    a, b = _gray_small(store, ts_a, gray), _gray_small(store, ts_b, gray)
    if a is None or b is None or a.shape != b.shape:
        return None
    delta = np.abs(a.astype(np.int16) - b.astype(np.int16))
    return float((delta > DIFF_PIXEL_THRESHOLD).mean())


def scene_still(
    store: Any,
    ts: float,
    intervals: list[tuple[float, float]],
    gray: dict[float, np.ndarray | None],
    motion_threshold: float = MOTION_THRESHOLD,
) -> bool:
    """True when the scene around ts is static and unchanged since the camera parked.

    Requires camera stillness at every compared instant - unconditioned,
    a moving camera changes every pixel and scans become indistinguishable
    from manipulation. Three image terms, all below ``motion_threshold``:

    * anchored: against the start of the surrounding camera-still
      interval, so anything the scene changed since the camera parked
      (an object placed, moved, or removed) rejects every later frame of
      that interval;
    * bilateral: against frames a fixed short span before and after ts,
      which catches hands actively moving through the view.
    """
    interval = _interval_containing(ts, intervals)
    if interval is None:
        return False
    a, b = interval

    anchor = min(a + 0.3, ts)
    fraction = _diff_fraction(store, anchor, ts, gray)
    if fraction is None or fraction > motion_threshold:
        return False

    for other in (max(a, ts - SHORT_DIFF_DT), min(b, ts + SHORT_DIFF_DT)):
        if abs(other - ts) < 0.05:
            continue
        fraction = _diff_fraction(store, other, ts, gray)
        if fraction is None or fraction > motion_threshold:
            return False
    return True


def depth_at(store: Any, ts: float, tolerance: float = 0.06) -> Image | None:
    """Temporal join: aligned depth frame for a color timestamp."""
    try:
        depth: Image = store.streams.depth_image.at(ts, tolerance).first().data
    except LookupError:
        return None
    return depth


def keyframes(
    store: Any,
    tf: TFLookup,
    t0: float,
    t1: float,
    stride: float,
    intervals: list[tuple[float, float]],
    gray: dict[float, np.ndarray | None],
    motion_threshold: float = MOTION_THRESHOLD,
    optical_frame: str = OPTICAL_FRAME,
    world_frame: str = WORLD_FRAME,
    tolerance: float = TF_TOLERANCE,
) -> list[Observation[Image]]:
    """Camera-still, scene-still color frames on a coarse grid over [t0, t1].

    For each grid point the nearest passing frame within half a stride is
    selected, so a grid point landing mid-sweep snaps to the neighboring
    pause instead of being lost.
    """
    selected: list[Observation[Image]] = []
    seen: set[float] = set()
    offsets = [0.0]
    probe = 0.35
    while probe <= stride / 2:
        offsets.extend([probe, -probe])
        probe += 0.35

    t = t0 + 0.5
    while t < t1:
        for offset in offsets:
            ts = t + offset
            if ts < t0 or ts > t1:
                continue
            if not camera_still(tf, ts, optical_frame, world_frame, tolerance):
                continue
            if not scene_still(store, ts, intervals, gray, motion_threshold):
                continue
            try:
                obs = store.streams.color_image.at(ts, 0.1).first()
            except LookupError:
                continue
            if obs.ts in seen:
                break
            if tf.get(optical_frame, world_frame, obs.ts, tolerance) is None:
                continue
            seen.add(obs.ts)
            selected.append(obs)
            break
        t += stride
    return selected
