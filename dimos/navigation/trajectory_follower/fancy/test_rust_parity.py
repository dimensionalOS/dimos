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

"""The rust controller is a port, so the python is its oracle.

Agreement is asserted at 1e-9 but the observed spread is exactly zero (`test_parity_headroom`):
same operations, same order, same libm.
"""

from dataclasses import replace
import math

import numpy as np
import pytest

pytest.importorskip("dimos_trajectory_follower")

from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.Path import Path
from dimos.navigation.embodiment.go2 import GO2
from dimos.navigation.local_planner.obstacles import path_clearance as follower_clearance
from dimos.navigation.local_planner.profile import encode_precision
from dimos.navigation.trajectory_follower.fancy.controller import (
    ControllerConfig,
    load_extension,
    path_xy_yaw,
)
from dimos.navigation.trajectory_follower.fancy.laws import hinted, seed

load_extension()  # importorskip above already proved it is there

TOL = 1e-9
CASES = 240


GOVERNOR = GO2.to_json()


def _pose(x: float, y: float, yaw: float = 0.0) -> PoseStamped:
    return PoseStamped(
        frame_id="world",
        position=Vector3(x, y, 0.0),
        orientation=Quaternion.from_euler(Vector3(0, 0, yaw)),
    )


def _path(states: list[tuple[float, float, float]]) -> Path:
    return Path(frame_id="world", poses=[_pose(*s) for s in states])


def _straight(rng: np.random.Generator) -> list[tuple[float, float, float]]:
    n = int(rng.integers(2, 60))
    step = float(rng.uniform(0.02, 0.4))
    yaw = float(rng.uniform(-math.pi, math.pi))
    x0, y0 = rng.uniform(-3.0, 3.0, 2)
    return [(x0 + k * step * math.cos(yaw), y0 + k * step * math.sin(yaw), yaw) for k in range(n)]


def _s_curve(rng: np.random.Generator) -> list[tuple[float, float, float]]:
    n = int(rng.integers(4, 80))
    amp, freq = float(rng.uniform(0.2, 1.5)), float(rng.uniform(0.3, 2.0))
    step = float(rng.uniform(0.03, 0.25))
    out = []
    for k in range(n):
        x = k * step
        y = amp * math.sin(freq * x)
        out.append((x, y, math.atan2(amp * freq * math.cos(freq * x), 1.0)))
    return out


def _with_fans(rng: np.random.Generator) -> list[tuple[float, float, float]]:
    """An S-curve with coincident-waypoint rotations spliced in: fan detection and yaw advance."""
    base = _s_curve(rng)
    out: list[tuple[float, float, float]] = []
    for k, (x, y, yaw) in enumerate(base):
        out.append((x, y, yaw))
        if k and k % int(rng.integers(3, 9)) == 0:
            turn = float(rng.uniform(-math.pi, math.pi))
            steps = int(rng.integers(2, 7))
            for m in range(1, steps + 1):
                # exactly coincident: zero arc, pure yaw step
                out.append((x, y, yaw + turn * m / steps))
    return out


def _degenerate(rng: np.random.Generator) -> list[tuple[float, float, float]]:
    n = int(rng.integers(0, 3))  # 0, 1, or 2 poses
    return [
        (
            float(rng.uniform(-2, 2)),
            float(rng.uniform(-2, 2)),
            float(rng.uniform(-math.pi, math.pi)),
        )
        for _ in range(n)
    ]


GENERATORS = (_straight, _s_curve, _with_fans, _with_fans, _degenerate)


def _cases(seed: int = 20260802, n: int = CASES):  # type: ignore[no-untyped-def]
    """(config, pose, path, clearance) tuples covering every branch."""
    rng = np.random.default_rng(seed)
    # a separate stream for the stamps: drawing from `rng` would shift every later draw
    srng = np.random.default_rng(seed + 1)
    for k in range(n):
        states = GENERATORS[k % len(GENERATORS)](rng)
        path = _path(states)
        if states:
            # on the path, off it, and behind its start
            anchor = states[int(rng.integers(0, len(states)))]
            mode = k % 3
            off = (0.0, 0.0) if mode == 0 else tuple(rng.uniform(-1.5, 1.5, 2))
            base = states[0] if mode == 2 else anchor
            px, py = base[0] + off[0] - (2.0 if mode == 2 else 0.0), base[1] + off[1]
        else:
            px, py = rng.uniform(-2, 2, 2)
        pose = _pose(float(px), float(py), float(rng.uniform(-math.pi, math.pi)))

        clearance = None
        if k % 4:
            clearance = rng.uniform(0.0, 0.8, len(states))
            if k % 8 == 1:  # a wrong-length annotation must be ignored by both
                clearance = clearance[:-1]
        # a share of the paths stamped with the precision profile; k % 8 == 3 leaves nonsense
        if k % 3 and len(states) > 1:
            enc = np.clip(srng.uniform(0.0, 0.8, len(states)), 0.0, None)
            encode_precision(path, enc, GO2, t0=float(srng.uniform(0.0, 1e9)))
            if k % 8 == 3:
                for q in path.poses:
                    q.ts = 5.0  # flat: not the dialect, both must ignore it

        # non-default gains and plant every fifth case: the params tuple order is load-bearing
        emb = GO2
        if k % 5 == 0:
            emb = replace(
                GO2,
                control=GO2.control.model_copy(
                    update={
                        "lookahead": float(rng.uniform(0.1, 1.2)),
                        "k_pos": float(rng.uniform(0.5, 4.0)),
                        "k_yaw": float(rng.uniform(0.5, 4.0)),
                        "fan_yaw_per_m": float(rng.uniform(1.0, 6.0)),
                        "fan_yaw_done": float(rng.uniform(0.05, 0.6)),
                        "speed_lookahead": float(rng.uniform(0.5, 4.0)),
                    }
                ),
                max_speed=float(rng.uniform(0.2, 1.5)),
                min_speed=float(rng.uniform(0.05, 0.3)),
                max_yaw_rate=float(rng.uniform(0.4, 3.0)),
                speed_clearance=float(rng.uniform(0.2, 0.8)),
                precision=float(rng.uniform(0.01, 0.15)),
                walk_gain=float(srng.uniform(0.7, 1.3)),
                walk_slip=float(srng.uniform(0.0, 0.3)),
                walk_slip_ramp=float(srng.uniform(0.02, 0.3)),
            )
        yield emb.control, pose, path, clearance, emb


LAWS = {
    "seed": (seed.make, seed.make_rust),
    "hinted": (hinted.make, hinted.make_rust),
}


def _twists(law, cfg, pose, path, clearance, emb=GO2):  # type: ignore[no-untyped-def]
    make_py, make_rs = LAWS[law]
    py = make_py(emb).update(pose, path, 0.0, clearance)
    rs = make_rs(emb).update(pose, path, 0.0, clearance)
    return (
        (py.linear.x, py.linear.y, py.angular.z),
        (rs.linear.x, rs.linear.y, rs.angular.z),
    )


@pytest.mark.parametrize("law", sorted(LAWS))
def test_rust_matches_python(law: str) -> None:
    seen = {"fan": 0, "governed": 0, "clamped": 0, "held": 0}
    for k, case in enumerate(_cases()):
        a, b = _twists(law, *case)
        for c, (x, y) in enumerate(zip(a, b, strict=True)):
            assert abs(x - y) <= TOL, f"case {k} component {c}: python {x!r} vs rust {y!r}"
        emb = case[4]
        top = emb.max_speed
        if a == (0.0, 0.0, 0.0):
            seen["held"] += 1
        if abs(a[2]) >= emb.max_yaw_rate - 1e-12 or math.hypot(a[0], a[1]) >= top - 1e-12:
            seen["clamped"] += 1
        if case[3] is not None and len(case[3]) == len(case[2]):
            seen["governed"] += 1
        if len(case[2]) > 2 and math.hypot(a[0], a[1]) < 1e-3 and abs(a[2]) > 1e-3:
            seen["fan"] += 1
    # the sweep is only worth its tolerance if it actually reached the branches
    assert all(v > 0 for v in seen.values()), f"unexercised branches: {seen}"


@pytest.mark.parametrize("law", sorted(LAWS))
def test_parity_headroom(law: str) -> None:
    worst = 0.0
    for case in _cases():
        a, b = _twists(law, *case)
        worst = max(worst, max(abs(x - y) for x, y in zip(a, b, strict=True)))
    assert worst <= TOL, f"max component diff {worst:.3e}"
    # libm is shared, so the only expected spread is zero; non-zero is a real divergence
    assert worst == 0.0, f"unexpected non-zero divergence {worst:.3e}"


@pytest.mark.parametrize(
    "yaw", [-math.pi, -math.pi / 2, 0.0, math.pi / 2, math.pi, math.pi - 1e-12]
)
def test_wrap_boundaries(yaw: float) -> None:
    """+-pi is where a `%`-based wrap diverges from IEEE remainder."""
    path = _path([(0.0, 0.0, math.pi), (0.0, 0.0, -math.pi + 0.2), (1.0, 0.0, -math.pi + 0.2)])
    for law in sorted(LAWS):
        a, b = _twists(law, GO2.control, _pose(0.0, 0.0, yaw), path, None)
        assert a == b, f"{law} yaw={yaw!r}: python {a} vs rust {b}"


def test_rust_factories_build() -> None:
    for _, make_rs in LAWS.values():
        assert isinstance(make_rs().config, ControllerConfig)


def test_encode_precision_matches_python() -> None:
    """The producer side of the dialect; a wrong-length annotation must be ignored identically."""
    rs = load_extension()
    worst = 0.0
    for case in _cases():
        path, clearance = case[2], case[3]
        clr = np.asarray([] if clearance is None else clearance, dtype=np.float64)
        # a t0 well off zero: an offset that cancels in the diff would hide a base-stamp divergence
        t0 = 1754212345.75
        want = [p.ts for p in encode_precision(path, clr, GO2, t0=t0).poses]
        got = rs.encode_precision(path_xy_yaw(path), clr if len(clr) else None, t0, GOVERNOR)
        assert len(got) == len(want)
        for k, (x, y) in enumerate(zip(want, got, strict=True)):
            assert abs(x - y) <= TOL, f"waypoint {k}: python {x!r} vs rust {y!r}"
            worst = max(worst, abs(x - y))
    assert worst == 0.0, f"unexpected non-zero divergence {worst:.3e}"


def test_path_clearance_matches_scipy() -> None:
    """The room hint against the python cKDTree; only the distance crosses, so it is exact."""
    rs = load_extension()
    rng = np.random.default_rng(20260803)
    for case in range(60):
        # spreads either side of the rust grid's cell size
        spread = (0.05, 0.5, 5.0, 40.0)[case % 4]
        pts = rng.uniform(-spread, spread, size=(1 + case * 7, 3)).astype(np.float32)
        # z is read by neither side: the model already decided
        pts[:, 2] = rng.uniform(-0.2, 0.7, size=len(pts)).astype(np.float32)
        xy = rng.uniform(-spread, spread, size=(12, 2))

        want = follower_clearance(xy, pts, 0.25)
        got = np.asarray(rs.path_clearance(np.ascontiguousarray(xy), pts, 0.25))
        assert got.shape == want.shape
        for k, (a, b) in enumerate(zip(want, got, strict=True)):
            # equality, not a difference: inf - inf is nan and would pass a tolerance check
            assert a == b, f"case {case} waypoint {k}: python {a!r} vs rust {b!r}"
