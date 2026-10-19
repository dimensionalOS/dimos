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

"""Decoding the precision profile the planner encodes in the path's own timestamps.

Per-waypoint ``ts`` deltas = segment length / governor speed at that segment's
clearance. Not a schedule: a follower never chases the clock, only the deltas
carry information, and running slower than the encoding is always legal.
The encoder is the rust planner's ``stamps.rs``.
"""

from __future__ import annotations

from dimos_generated.nav_msgs.msg import Path
import numpy as np
from numpy.typing import NDArray

from dimos.msgs.time import to_seconds
from dimos.navigation.embodiment.base import Embodiment

_FAN_EPS = 1e-6  # segment shorter than this is a rotation, not a move


def decode_ceilings(path: Path, lo: float, hi: float) -> NDArray[np.float64] | None:
    """Per-waypoint speed ceiling (m/s) from the stamps; None if unstamped.

    Unstamped = flat or non-monotone ts. Fan segments inherit the previous
    ceiling; the result is clipped into the consumer's band ``[lo, hi]``.
    """
    n = len(path.poses)
    if n < 2:
        return None
    ts = np.array([to_seconds(p.header.stamp) for p in path.poses])
    dt = np.diff(ts)
    if np.any(dt < 0) or not np.any(dt > 0):
        return None
    xy = np.array([[p.pose.position.x, p.pose.position.y] for p in path.poses]).reshape(-1, 2)
    ds = np.linalg.norm(np.diff(xy, axis=0), axis=1)
    # a config is free to set min above max; ordering here rather than
    # trusting the caller keeps the clip a clip
    lo, hi = min(lo, hi), max(lo, hi)
    out = np.full(n, hi)
    prev = hi
    for i in range(1, n):
        if ds[i - 1] >= _FAN_EPS and dt[i - 1] > 0:
            v = ds[i - 1] / dt[i - 1]
            # a non-finite ratio would propagate straight out through the
            # twist; the encoder cannot produce one, the wire can
            if np.isfinite(v):
                prev = float(np.clip(v, lo, hi))
        out[i] = prev
    out[0] = out[1] if n > 1 else prev
    return out


def ceilings_to_clearance(ceilings: NDArray[np.float64], emb: Embodiment) -> NDArray[np.float64]:
    """Speed ceilings -> the clearance that reproduces them under the governor.

    Exact linear inverse on [min_speed, max_speed]; what the viewer colors a
    plan's room by.
    """
    lo, hi = emb.min_speed, emb.max_speed
    assert hi != lo, "a body with min_speed == max_speed has no governor band to invert"
    frac = (np.clip(ceilings, lo, hi) - lo) / (hi - lo)
    return emb.precision + frac * (emb.speed_clearance - emb.precision)
