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

"""Precision encoded in the path's own timestamps — the wire dialect.

Planner and follower share stock dimos/ROS types; the required-precision
profile rides in per-waypoint ``ts``: dt between consecutive waypoints =
segment length / governor speed for that segment's clearance. Seconds are
the carrier, clearance is the meaning. The stamps are NOT a schedule — a
follower must never chase the clock (catch-up would accelerate exactly in
the tight zones); only the deltas carry information, and running slower
than the encoding is always legal.

Encoding is planner/annotator-owned: when the executor's measured precision
improves, only :func:`governor_speed` moves and every follower keeps
decoding dt/length blindly. Fan segments (rotation in place, ~zero length)
carry dt = yaw span / max yaw rate so the timeline stays monotone.
"""

from __future__ import annotations

import numpy as np

from dimos.msgs.nav_msgs.Path import Path
from dimos.navigation.motion.embodiment import Embodiment

# The governor curve is the embodiment's (max_speed, min_speed, speed_clearance,
# precision, max_yaw_rate): a wire contract between planner and follower, so
# both read it off the body they were configured with.
_FAN_EPS = 1e-6  # segment shorter than this is a rotation, not a move


def governor_speed(clearance: np.ndarray, emb: Embodiment) -> np.ndarray:
    """Clearance (m) -> speed ceiling (m/s): creep at the floor, cruise with room."""
    frac = (np.asarray(clearance, dtype=float) - emb.precision) / (
        emb.speed_clearance - emb.precision
    )
    return emb.min_speed + (emb.max_speed - emb.min_speed) * np.clip(frac, 0.0, 1.0)


def _segments(path: Path) -> tuple[np.ndarray, np.ndarray]:
    xy = np.array([[p.position.x, p.position.y] for p in path.poses]).reshape(-1, 2)
    yaws = np.array([p.yaw for p in path.poses])
    return xy, yaws


def encode_precision(path: Path, clearance: np.ndarray, emb: Embodiment, t0: float = 0.0) -> Path:
    """Stamp the path in place: ts[i+1]-ts[i] = seg length / governor speed.

    ``clearance`` is per waypoint; a segment uses the tighter of its two
    endpoints. Fans get dt from their yaw span. Returns the same Path.
    """
    n = len(path.poses)
    if n == 0:
        return path
    xy, yaws = _segments(path)
    v = governor_speed(clearance, emb) if len(clearance) == n else np.full(n, emb.max_speed)
    t = t0
    path.poses[0].ts = t
    for i in range(1, n):
        ds = float(np.linalg.norm(xy[i] - xy[i - 1]))
        if ds < _FAN_EPS:
            dyaw = abs(float(np.remainder(yaws[i] - yaws[i - 1] + np.pi, 2 * np.pi) - np.pi))
            t += dyaw / emb.max_yaw_rate
        else:
            t += ds / float(min(v[i - 1], v[i]))
        path.poses[i].ts = t
    path.ts = t0
    return path


def decode_ceilings(path: Path, lo: float, hi: float) -> np.ndarray | None:
    """Per-waypoint speed ceiling (m/s) from the stamps; None if unstamped.

    Unstamped = flat or non-monotone ts (a plain path from a producer that
    does not speak this dialect) — the follower then falls back to its own
    clearance source or plain max speed. Fan segments inherit the previous
    ceiling; the result is clipped into ``[lo, hi]`` so a slow upstream
    planner can only ever make the robot more careful, and garbage stamps
    saturate at cruise instead of commanding something absurd.

    ``lo``/``hi`` is the CONSUMER's band: the embodiment's for anyone reading
    the wire as stamped, a controller's own ``ControllerConfig`` band so that a
    non-default config decodes the same ceiling its own governor would have
    produced.
    """
    n = len(path.poses)
    if n < 2:
        return None
    ts = np.array([p.ts for p in path.poses])
    dt = np.diff(ts)
    if np.any(dt < 0) or not np.any(dt > 0):
        return None
    xy, _ = _segments(path)
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


def ceilings_to_clearance(ceilings: np.ndarray, emb: Embodiment) -> np.ndarray:
    """Speed ceilings -> the clearance that reproduces them under the governor.

    Exact linear inverse on [min_speed, max_speed]; lets a stamped path feed
    any controller through its existing clearance seam, so the laws (python
    and rust, parity-locked) stay untouched.
    """
    lo, hi = emb.min_speed, emb.max_speed
    frac = (np.clip(ceilings, lo, hi) - lo) / (hi - lo)
    return emb.precision + frac * (emb.speed_clearance - emb.precision)
