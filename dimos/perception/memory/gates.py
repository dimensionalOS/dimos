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

"""The scene-motion gate: image differencing conditioned on camera stillness.

Pose-derived gates (camera pose, speed, stillness, keyframes) live on
:class:`~dimos.perception.memory.rig.Rig` - they depend on where the rig's
poses come from. What stays here is purely image-based: the scene-motion
gate differences frames of the color stream, conditioned on camera
stillness at both compared instants. It rejects frames captured while the
scene itself changes - a parked camera watching hands rearrange objects is
exactly the case poses cannot see. The reference frame is anchored at the
start of the surrounding camera-still interval, so a frame is trusted only
while the scene still matches the state it had when the camera parked.

The stillness intervals and the grayscale memo are allocated by the caller
and passed in, one per query.
"""

from __future__ import annotations

from bisect import bisect_right
from typing import Any

import numpy as np

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


def _interval_containing(
    ts: float, intervals: list[tuple[float, float]]
) -> tuple[float, float] | None:
    idx = bisect_right([a for a, _ in intervals], ts) - 1
    if idx < 0:
        return None
    a, b = intervals[idx]
    return (a, b) if a - 0.25 <= ts <= b + 0.25 else None


def _gray_small(color: Any, ts: float, gray: dict[float, np.ndarray | None]) -> np.ndarray | None:
    """Downscaled grayscale of the color frame nearest ts, memoized in *gray*."""
    key = round(ts, 2)
    if key in gray:
        return gray[key]

    import cv2

    small_gray: np.ndarray | None = None
    try:
        frame = color.at(ts, 0.1).first().data
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
    color: Any, ts_a: float, ts_b: float, gray: dict[float, np.ndarray | None]
) -> float | None:
    """Fraction of pixels changed between the frames nearest the two instants."""
    a, b = _gray_small(color, ts_a, gray), _gray_small(color, ts_b, gray)
    if a is None or b is None or a.shape != b.shape:
        return None
    delta = np.abs(a.astype(np.int16) - b.astype(np.int16))
    return float((delta > DIFF_PIXEL_THRESHOLD).mean())


def scene_still(
    color: Any,
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
    fraction = _diff_fraction(color, anchor, ts, gray)
    if fraction is None or fraction > motion_threshold:
        return False

    for other in (max(a, ts - SHORT_DIFF_DT), min(b, ts + SHORT_DIFF_DT)):
        if abs(other - ts) < 0.05:
            continue
        fraction = _diff_fraction(color, other, ts, gray)
        if fraction is None or fraction > motion_threshold:
            return False
    return True
