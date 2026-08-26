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

"""The rust controller is a PORT, so the python is its oracle.

Every branch of the law gets sampled: straight runs, S-curves, fan clusters
of coincident waypoints, degenerate one/zero-pose paths, clearance annotated
and blind, poses on/off/behind the path, yaws swept across +-pi. Agreement is
asserted per component at 1e-9, but the observed spread is exactly zero
(`test_parity_headroom` pins that separately): the two run the same operations
in the same order against the same libm, so they agree bit for bit, and any
drift off zero is a real divergence rather than accumulated noise.
"""

from dataclasses import replace
import math

import numpy as np
import pytest

from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.Path import Path
from dimos.navigation.motion.control.controller import (
    ControllerConfig,
    load_extension,
    path_xy_yaw,
)
from dimos.navigation.motion.control.laws import blind, hinted, seed
from dimos.navigation.motion.control.profile import (
    ceilings_to_clearance,
    encode_precision,
)
from dimos.navigation.motion.embodiment.go2 import GO2
from dimos.navigation.motion.obstacles import path_clearance as follower_clearance

load_extension()  # the crate under test; the ImportError names the build command

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
    """An S-curve with coincident-waypoint rotations spliced in -- the branch
    that exercises both fan detection and the yaw-progress advance."""
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
    # a SEPARATE stream for the stamps: drawing them from `rng` would shift
    # every later draw and silently replace the sweep this file's tolerances
    # were characterised on
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
        # Stamp a share of the paths with the precision profile: it is the
        # blind law's only governor channel, and an unstamped path exercises
        # its fallback. k % 8 == 3 leaves stamps that are deliberate nonsense.
        if k % 3 and len(states) > 1:
            enc = np.clip(srng.uniform(0.0, 0.8, len(states)), 0.0, None)
            encode_precision(path, enc, GO2, t0=float(srng.uniform(0.0, 1e9)))
            if k % 8 == 3:
                for q in path.poses:
                    q.ts = 5.0  # flat: not the dialect, both must ignore it

        # non-default gains AND plant every fifth case, so the params tuple
        # order across the boundary is load-bearing
        emb = GO2
        if k % 5 == 0:
            emb = replace(
                GO2,
                control=GO2.control.model_copy(
                    update=dict(
                        lookahead=float(rng.uniform(0.1, 1.2)),
                        k_pos=float(rng.uniform(0.5, 4.0)),
                        k_yaw=float(rng.uniform(0.5, 4.0)),
                        fan_yaw_per_m=float(rng.uniform(1.0, 6.0)),
                        fan_yaw_done=float(rng.uniform(0.05, 0.6)),
                        speed_lookahead=float(rng.uniform(0.5, 4.0)),
                    )
                ),
                max_speed=float(rng.uniform(0.2, 1.5)),
                min_speed=float(rng.uniform(0.05, 0.3)),
                gait_band=(float(rng.uniform(0.05, 0.5)), float(rng.uniform(0.5, 1.5))),
                max_yaw_rate=float(rng.uniform(0.4, 3.0)),
                speed_clearance=float(rng.uniform(0.2, 0.8)),
                precision=float(rng.uniform(0.01, 0.15)),
                walk_gain=float(srng.uniform(0.7, 1.3)),
                walk_slip=float(srng.uniform(0.0, 0.3)),
                walk_slip_ramp=float(srng.uniform(0.02, 0.3)),
                command_slew=tuple(float(v) for v in srng.uniform(0.5, 6.0, 3)),
            )
        yield emb.control, pose, path, clearance, emb


# every law and its rust twin; each pair is held to TOL independently
LAWS = {
    "seed": (seed.make, seed.make_rust),
    "blind": (blind.make, blind.make_rust),
    "hinted": (hinted.make, hinted.make_rust),
}

# laws that keep state across ticks, so a single call proves nothing: their
# parity has to be replayed as a SEQUENCE through one controller instance
STATEFUL = {"hinted"}


def _raw_twists(law, cfg, pose, path, clearance, emb=GO2):  # type: ignore[no-untyped-def]
    """The PURE tick of a law, before any state it keeps.

    A stateful law's first tick is the only one a fresh instance can produce,
    and for `hinted` that tick is rate limited from a standing start -- so
    sweeping `update()` would never reach the clamp or the governor's ceiling
    and the branch census below would be measuring the limiter, not the law.
    The limiter has its own gate in `test_stateful_parity_over_a_sequence`.
    """
    if law not in STATEFUL:
        return _twists(law, cfg, pose, path, clearance, emb)
    ext = load_extension()
    clr = None if clearance is None else np.ascontiguousarray(clearance, dtype=np.float64)
    py = hinted.update(pose, path, emb, clearance)
    rs = ext.update_hinted_raw(
        (float(pose.position.x), float(pose.position.y), float(pose.yaw)),
        path_xy_yaw(path),
        clr,
        emb.to_json(),
    )
    return py, rs


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
        a, b = _raw_twists(law, *case)
        for c, (x, y) in enumerate(zip(a, b, strict=True)):
            assert abs(x - y) <= TOL, f"case {k} component {c}: python {x!r} vs rust {y!r}"
        emb = case[4]
        top = emb.gait_band[1] if law == "hinted" else emb.max_speed
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
    """Report the real spread: it must sit far under the asserted tolerance."""
    worst = 0.0
    for case in _cases():
        a, b = _raw_twists(law, *case)
        worst = max(worst, max(abs(x - y) for x, y in zip(a, b, strict=True)))
    assert worst <= TOL, f"max component diff {worst:.3e}"
    # libm hypot/sin/cos are shared between the two, so the only expected
    # spread is zero; a non-zero worst here is a real divergence to explain
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


@pytest.mark.parametrize("law", sorted(STATEFUL))
def test_stateful_parity_over_a_sequence(law: str) -> None:
    """A law with memory has to agree tick after tick, not just once.

    One instance each, fed the whole sweep in order at a realistic 50 Hz clock:
    a rate limiter that diverged by an ulp on tick one would carry that ulp
    forward, so this is the assertion that actually binds it. The tick times
    deliberately include a stall longer than the limiter's MAX_TICK and a
    repeated timestamp, both of which its dt fallback has to handle the same
    way on either side.
    """
    make_py, make_rs = LAWS[law]
    cases = list(_cases())
    py_law, rs_law = make_py(cases[0][4]), make_rs(cases[0][4])
    worst = 0.0
    t = 0.0
    for k, (_cfg, pose, path, clearance, emb) in enumerate(cases):
        # the config is per-case in this sweep, so rebuild on a change and
        # reset BOTH -- a fresh law and a reset law must answer identically
        if k and emb is not cases[k - 1][4]:
            py_law, rs_law = make_py(emb), make_rs(emb)
        t += (0.02, 0.02, 0.0, 0.5)[k % 4]  # nominal, nominal, repeat, stall
        a = py_law.update(pose, path, t, clearance)
        b = rs_law.update(pose, path, t, clearance)
        got = ((a.linear.x, a.linear.y, a.angular.z), (b.linear.x, b.linear.y, b.angular.z))
        for c, (x, y) in enumerate(zip(*got, strict=True)):
            assert abs(x - y) <= TOL, f"tick {k} component {c}: python {x!r} vs rust {y!r}"
            worst = max(worst, abs(x - y))
    assert worst == 0.0, f"unexpected non-zero divergence {worst:.3e}"


@pytest.mark.parametrize("law", sorted(STATEFUL))
def test_reset_clears_every_tick_of_history(law: str) -> None:
    """reset() must make a used law indistinguishable from a fresh one."""
    make_py, make_rs = LAWS[law]
    cfg, pose, path, clearance, emb = next(c for c in _cases() if len(c[2].poses) > 3)
    other = next(c for c in _cases() if len(c[2].poses) > 3 and c[2] is not path)
    for make in (make_py, make_rs):
        fresh, used = make(emb), make(emb)
        for _ in range(5):  # give `used` a foreign history to forget
            used.update(other[1], other[2], 0.02, other[3])
        used.reset()
        a = fresh.update(pose, path, 0.02, clearance)
        b = used.update(pose, path, 0.02, clearance)
        assert (a.linear.x, a.linear.y, a.angular.z) == (b.linear.x, b.linear.y, b.angular.z)


def test_encode_precision_matches_python() -> None:
    """The dialect's producer side, the half the planner module runs.

    Same sweep as the laws, because the encoder sees the same paths: fan
    clusters price by yaw, degenerate paths must not raise, and a
    wrong-length annotation has to be ignored identically on both sides or a
    stamped path would decode to a speed the planner never intended.
    """
    rs = load_extension()
    worst = 0.0
    for case in _cases():  # indexed, not unpacked: `_pose` is taken by the helper
        path, clearance = case[2], case[3]
        clr = np.asarray([] if clearance is None else clearance, dtype=np.float64)
        # a t0 well off zero: only the deltas are the dialect, so an offset
        # that cancels in the diff would hide a divergence in the base stamp
        t0 = 1754212345.75
        want = [p.ts for p in encode_precision(path, clr, GO2, t0=t0).poses]
        got = rs.encode_precision(path_xy_yaw(path), clr if len(clr) else None, t0, GOVERNOR)
        assert len(got) == len(want)
        for k, (x, y) in enumerate(zip(want, got, strict=True)):
            assert abs(x - y) <= TOL, f"waypoint {k}: python {x!r} vs rust {y!r}"
            worst = max(worst, abs(x - y))
    assert worst == 0.0, f"unexpected non-zero divergence {worst:.3e}"


def test_ceilings_to_clearance_matches_python() -> None:
    """The dialect's inverse leg, the half the follower module runs.

    A hinted follower with no cloud of its own has no clearance array, and the
    hinted law takes no stamps -- so the stamps are decoded to ceilings and
    bent back into the clearance that produced them. Both legs are wire
    constants, so a divergence here would silently re-price every waypoint of
    a plan the robot is already following.
    """
    rs = load_extension()
    rng = np.random.default_rng(20260803)
    # in band, either side of both knees, and the values the wire can hand over
    # that the encoder never produces
    ceilings = np.concatenate(
        [
            rng.uniform(0.0, 1.0, 200),
            np.array([GO2.min_speed, GO2.max_speed, 0.0, -1.0, 1e9, np.inf]),
        ]
    )
    want = ceilings_to_clearance(ceilings, GO2)
    got = np.asarray(rs.ceilings_to_clearance(np.ascontiguousarray(ceilings), GOVERNOR))
    assert got.shape == want.shape
    for k, (a, b) in enumerate(zip(want, got, strict=True)):
        assert a == b, f"ceiling {ceilings[k]!r}: python {a!r} vs rust {b!r}"
    assert not len(rs.ceilings_to_clearance(np.zeros(0), GOVERNOR))


def test_path_clearance_matches_scipy() -> None:
    """The room hint, against the cKDTree the python uses.

    Only the DISTANCE crosses this boundary, never which point produced it,
    so the rust is free to use a grid where the python uses a KD-tree: an
    exact nearest-neighbour distance is unique even when the nearest point is
    not. Agreement here is therefore exact, not approximate.
    """
    rs = load_extension()
    rng = np.random.default_rng(20260803)
    for case in range(60):
        # spreads either side of the rust grid's cell size, so both the
        # first-ring hit and the long ring walk are covered
        spread = (0.05, 0.5, 5.0, 40.0)[case % 4]
        pts = rng.uniform(-spread, spread, size=(1 + case * 7, 3)).astype(np.float32)
        # spread over z, which neither side reads: the model already decided
        pts[:, 2] = rng.uniform(-0.2, 0.7, size=len(pts)).astype(np.float32)
        xy = rng.uniform(-spread, spread, size=(12, 2))

        want = follower_clearance(xy, pts, 0.25)
        got = np.asarray(rs.path_clearance(np.ascontiguousarray(xy), pts, 0.25))
        assert got.shape == want.shape
        for k, (a, b) in enumerate(zip(want, got, strict=True)):
            # equality, not a difference: infinite room is a real value here
            # (an empty band means nothing can touch the body) and inf - inf
            # is nan, which would pass a tolerance check by accident
            assert a == b, f"case {case} waypoint {k}: python {a!r} vs rust {b!r}"
