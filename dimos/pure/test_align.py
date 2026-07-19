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

"""T5 alignment tests pinning tasks/t5-align.md (§4 semantics, §7 wiring, §9 errors)."""

from __future__ import annotations

import dataclasses
import itertools
import random
from typing import Any

import pytest

from dimos.pure.align import (
    AlignmentError,
    AlignRule,
    align,
    interpolator_for,
    register_interpolator,
)
from dimos.pure.rows import In, InterpolateSpec, LatestSpec, TickSpec, interpolate, latest, tick

# ── fixtures: stamped payloads ───────────────────────────────────────────────


@dataclasses.dataclass(frozen=True)
class S:
    """Minimal stamped payload (frozen: pins the read-only-property Stamped spelling)."""

    ts: float
    v: float = 0.0


@dataclasses.dataclass(frozen=True)
class Vec:
    """Self-describing Interpolatable payload (protocol method, spec §6.1)."""

    ts: float
    x: float = 0.0

    def interpolate(self, other: Vec, alpha: float) -> Vec:
        return Vec(ts=_lerp(self.ts, other.ts, alpha), x=_lerp(self.x, other.x, alpha))


def _lerp(a: float, b: float, alpha: float) -> float:
    return a + (b - a) * alpha


def s_lerp(a: S, b: S, alpha: float) -> S:
    return S(ts=_lerp(a.ts, b.ts, alpha), v=_lerp(a.v, b.v, alpha))


def _use_s_lerp() -> None:
    """Idempotent per-test registration (last-wins, spec §6.1)."""
    register_interpolator(S, s_lerp)


# ── fixtures: bundles (module level — T1 machinery is real) ──────────────────


class TickOnly(In):
    lidar: S = tick()


class LatestReq(In):  # tick index 0, latest index 1
    img: S = tick()
    pose: S = latest()


class LatestOpt(In):
    img: S = tick()
    pose: S | None = latest(default=None)


class InterpReq(In):  # tick index 0, interpolate index 1
    img: S = tick()
    pose: S = interpolate()


class InterpOpt(In):
    img: S = tick()
    pose: S | None = interpolate(default=None)


class InterpFirst(In):  # interpolate index 0, tick index 1 (equal-ts corner, lower side)
    pose: S = interpolate()
    img: S = tick()


class LatestFirst(In):  # latest index 0, tick index 1 (equal-ts corner, lower side)
    pose: S = latest()
    img: S = tick()


class VecInterp(In):
    img: S = tick()
    vec: Vec = interpolate()


class IntInterp(In):
    img: S = tick()
    n: int = interpolate()


class AnyInterp(In):
    img: S = tick()
    a: Any = interpolate()


class Combined(In):  # tick + optional latest + optional interpolate
    img: S = tick()
    pose: S | None = latest(default=None)
    vel: S | None = interpolate(default=None)


class NoTick(In):
    pose: S = latest()


class BaseIn(In):
    img: S = tick()


class ExtIn(BaseIn):
    pose: S = latest()


def ss(*ts: float) -> list[S]:
    """S items at the given timestamps, v = 10 * ts."""
    return [S(ts=t, v=10 * t) for t in ts]


def _logged(items: list[S], log: list[tuple[str, float]], tag: str) -> Any:
    """Generator that records each pull into log — pins the pull discipline."""

    def gen() -> Any:
        for it in items:
            log.append((tag, it.ts))
            yield it

    return gen()


def _rule_of(err: pytest.ExceptionInfo[AlignmentError]) -> AlignRule | None:
    return err.value.rule


# ── the denotational oracle (spec §4, transcribed) ───────────────────────────


def oracle(in_type: type[Any], streams: dict[str, list[Any]]) -> list[Any]:
    """Reference resolution: materialized kept sequences, L/R per tick (spec §4.4)."""
    fields = in_type.fields()
    kept: dict[str, list[Any]] = {}
    for name in fields:
        seq: list[Any] = []
        last = float("-inf")
        for it in streams.get(name, []):
            if it.ts <= last:
                continue
            seq.append(it)
            last = it.ts
        kept[name] = seq
    tick_name = next(n for n, sp in fields.items() if isinstance(sp, TickSpec))
    rows: list[Any] = []
    for k in kept[tick_name]:
        t = k.ts
        values: dict[str, Any] = {tick_name: k}
        ok = True
        for name, sp in fields.items():
            if name == tick_name:
                continue
            seq = kept[name]
            left = None
            for it in seq:
                if it.ts <= t:
                    left = it
                else:
                    break
            if isinstance(sp, LatestSpec):
                if left is not None:
                    values[name] = left
                elif not sp.required:
                    values[name] = sp.default
                else:
                    ok = False
            elif isinstance(sp, InterpolateSpec):
                right = next((it for it in seq if it.ts >= t), None)
                if left is not None and left.ts == t:
                    values[name] = left
                elif left is not None and right is not None:
                    values[name] = s_lerp(left, right, (t - left.ts) / (right.ts - left.ts))
                elif not sp.required:
                    values[name] = sp.default
                else:
                    ok = False
        if ok:
            rows.append(in_type(ts=t, **values))
    return rows


# ── the trigger ──────────────────────────────────────────────────────────────


def test_tick_only_bundle():
    items = ss(1.0, 2.0, 3.5)
    rows = list(align(TickOnly, {"lidar": items}))
    assert [r.ts for r in rows] == [1.0, 2.0, 3.5]
    assert [r.lidar for r in rows] == items
    assert rows[0].lidar is items[0]  # the tick field is the item itself, verbatim (D2)


def test_empty_tick_stream():
    a = align(LatestOpt, {"img": [], "pose": ss(0.5)})
    assert list(a) == []
    # normative pseudocode (spec §5.2): refill pulls one pose item before termination
    assert a.stats.ports["pose"].accepted == 1
    assert a.stats.ticks_fired == 0


# ── latest resolution ────────────────────────────────────────────────────────


def test_latest_resolution_matrix():
    # spanning: newest-at-or-before wins per tick
    rows = list(align(LatestReq, {"img": ss(1.0, 2.0, 3.0), "pose": ss(0.5, 1.5, 2.5)}))
    assert [(r.ts, r.pose.ts) for r in rows] == [(1.0, 0.5), (2.0, 1.5), (3.0, 2.5)]
    # secondary exhausted mid-run: newest keeps serving later ticks
    rows = list(align(LatestReq, {"img": ss(1.0, 5.0, 9.0), "pose": ss(0.5)}))
    assert [(r.ts, r.pose.ts) for r in rows] == [(1.0, 0.5), (5.0, 0.5), (9.0, 0.5)]
    # empty required secondary: every tick drops, no rows, no error
    a = align(LatestReq, {"img": ss(1.0, 2.0), "pose": []})
    assert list(a) == []
    assert a.stats.ticks_dropped == 2


def test_latest_required_warmup_drops():
    a = align(LatestReq, {"img": ss(1.0, 2.0, 3.0), "pose": ss(2.5)})
    rows = list(a)
    assert [(r.ts, r.pose.ts) for r in rows] == [(3.0, 2.5)]
    assert a.stats.ticks_fired == 3
    assert a.stats.ticks_dropped == 2
    assert a.stats.rows_emitted == 1
    assert a.stats.drops_by_field == {"pose": 2}


def test_latest_optional_defaults():
    marker = S(ts=-1.0, v=99.0)

    class OptMarker(In):
        img: S = tick()
        pose: S = latest(default=marker)

    a = align(OptMarker, {"img": ss(1.0, 2.0), "pose": []})
    rows = list(a)
    assert [r.pose for r in rows] == [marker, marker]
    assert rows[0].pose is marker  # default delivered by reference, uncopied (§4.4)
    assert a.stats.ticks_dropped == 0


def test_optional_port_stream_omitted():
    a = align(Combined, {"img": ss(1.0, 2.0)})  # pose and vel omitted entirely (§7 #5)
    rows = list(a)
    assert [(r.pose, r.vel) for r in rows] == [(None, None), (None, None)]
    assert a.stats.ports["pose"].accepted == 0  # entry exists at zero (§8.1)
    assert a.stats.ports["vel"].accepted == 0


# ── interpolate resolution ───────────────────────────────────────────────────


def test_interpolate_exact_hit_and_bracket():
    _use_s_lerp()
    hit = S(ts=2.0, v=7.0)
    rows = list(align(InterpReq, {"img": ss(1.0, 2.0), "pose": [S(0.0, 1.0), hit, S(4.0, 9.0)]}))
    # bracket (0.0, 2.0) at T=1.0 → alpha exactly 0.5
    assert rows[0].pose == S(ts=1.0, v=4.0)
    # exact hit returns the sample itself — the interpolator is bypassed (§4.4)
    assert rows[1].pose is hit


def test_interpolate_one_sided_no_extrapolation():
    _use_s_lerp()
    # before-only (required): drop
    a = align(InterpReq, {"img": ss(5.0), "pose": ss(1.0, 2.0)})
    assert list(a) == []
    assert a.stats.drops_by_field == {"pose": 1}
    # after-only (required): drop
    a = align(InterpReq, {"img": ss(0.5), "pose": ss(1.0, 2.0)})
    assert list(a) == []
    assert a.stats.drops_by_field == {"pose": 1}
    # both sides optional variant: default instead
    a2 = align(InterpOpt, {"img": ss(0.5, 5.0), "pose": ss(1.0, 2.0)})
    assert [r.pose for r in a2] == [None, None]


def test_interpolate_tail_exhausted():
    _use_s_lerp()
    a = align(InterpReq, {"img": ss(1.0, 3.0), "pose": ss(0.0, 2.0)})
    rows = list(a)
    assert [r.ts for r in rows] == [1.0]  # T=3.0 has no right bracket → final drop (§4.5)
    assert a.stats.ticks_dropped == 1


def test_multiple_ticks_share_bracket():
    _use_s_lerp()
    rows = list(align(InterpReq, {"img": ss(1.0, 2.0, 3.0), "pose": [S(0.0, 0.0), S(4.0, 4.0)]}))
    # resolution never consumes: three ticks resolve against the one bracket (§5.4)
    assert [r.pose.v for r in rows] == [1.0, 2.0, 3.0]


# ── the equal-ts corner (§4.3, §5.4) ─────────────────────────────────────────


def test_equal_ts_secondary_lower_index():
    _use_s_lerp()
    at = S(ts=1.0, v=7.0)
    rows = list(align(LatestFirst, {"img": [S(1.0)], "pose": [at]}))
    assert rows[0].pose is at  # committed before the tick (lower declaration index)
    rows = list(align(InterpFirst, {"img": [S(1.0)], "pose": [at]}))
    assert rows[0].pose is at  # exact hit via newest


def test_equal_ts_secondary_higher_index():
    _use_s_lerp()
    at = S(ts=1.0, v=7.0)
    rows = list(align(LatestReq, {"img": [S(1.0)], "pose": [at]}))
    assert rows[0].pose is at  # the pending-head corner: head.ts == T resolves
    rows = list(align(InterpReq, {"img": [S(1.0)], "pose": [at]}))
    assert rows[0].pose is at  # exact hit via head


# ── ingestion: monotonic filter and ts contract ──────────────────────────────


def test_monotonic_regression_dropped():
    a = align(LatestReq, {"img": ss(1.0, 2.0), "pose": [S(0.5), S(0.4), S(0.5), S(1.5)]})
    rows = list(a)
    assert [(r.ts, r.pose.ts) for r in rows] == [(1.0, 0.5), (2.0, 1.5)]
    assert a.stats.ports["pose"].dropped_nonmonotonic == 2  # regression + duplicate
    assert a.stats.ports["pose"].accepted == 2


def test_tick_port_monotonic_filter():
    a = align(TickOnly, {"lidar": [S(1.0), S(1.0), S(0.5), S(2.0)]})
    assert [r.ts for r in list(a)] == [1.0, 2.0]  # output strictly ts-increasing
    assert a.stats.ports["lidar"].dropped_nonmonotonic == 2
    assert a.stats.ticks_fired == 2


def test_nan_ts_raises():
    a = align(TickOnly, {"lidar": [S(1.0), S(float("nan"))]})
    assert next(a).ts == 1.0
    with pytest.raises(AlignmentError) as ei:
        next(a)  # raised at the pulling next() (§9 D-errors)
    assert _rule_of(ei) is AlignRule.BAD_TS
    a = align(TickOnly, {"lidar": [S(float("inf"))]})
    with pytest.raises(AlignmentError) as ei:
        next(a)
    assert _rule_of(ei) is AlignRule.BAD_TS


def test_unstamped_ts_raises():
    a = align(TickOnly, {"lidar": [S(float("-inf"))]})
    with pytest.raises(AlignmentError) as ei:
        next(a)
    assert _rule_of(ei) is AlignRule.UNSTAMPED_ITEM
    assert "UNSTAMPED" in str(ei.value) and "stamped by its driver" in str(ei.value)


def test_bad_item_raises():
    a = align(TickOnly, {"lidar": [object()]})
    with pytest.raises(AlignmentError) as ei:
        next(a)
    assert _rule_of(ei) is AlignRule.BAD_ITEM
    assert "lidar" in str(ei.value)

    class WeirdTs:
        ts = "not a number"

    a = align(TickOnly, {"lidar": [WeirdTs()]})
    with pytest.raises(AlignmentError) as ei:
        next(a)
    assert _rule_of(ei) is AlignRule.BAD_ITEM


# ── hold = pull order (§4.5, §5.2) ───────────────────────────────────────────


def test_hold_is_pull_order():
    log: list[tuple[str, float]] = []
    a = align(
        LatestReq,
        {
            "img": _logged(ss(1.0, 2.0), log, "img"),
            "pose": _logged(ss(0.5, 1.5, 2.5), log, "pose"),
        },
    )
    assert log == []  # construction pulls nothing (§7: validation only)
    row = next(a)
    assert row.ts == 1.0 and row.pose.ts == 0.5
    # the tick could not fire until pose's frontier passed T=1.0 ...
    assert ("pose", 1.5) in log
    # ... and nothing beyond the frontier was pulled (lazy)
    assert ("pose", 2.5) not in log


def test_lazy_tail():
    log: list[tuple[str, float]] = []
    a = align(
        LatestReq,
        {
            "img": _logged(ss(1.0), log, "img"),
            "pose": _logged(ss(0.5, 5.0, 6.0, 7.0), log, "pose"),
        },
    )
    assert [r.ts for r in a] == [1.0]
    # tick exhaustion terminates without draining the secondary (§5.2 step b)
    assert ("pose", 5.0) in log and ("pose", 6.0) not in log


# ── wiring validation (§7, §9) ───────────────────────────────────────────────


def test_wiring_no_tick():
    with pytest.raises(AlignmentError) as ei:
        align(NoTick, {"pose": ss(1.0)})
    assert _rule_of(ei) is AlignRule.NO_TICK
    assert "tick()" in str(ei.value) and "NoTick" in str(ei.value)


def test_wiring_not_in_row():
    with pytest.raises(AlignmentError) as ei:
        align(int, {})  # type: ignore[type-var]
    assert _rule_of(ei) is AlignRule.NOT_IN_ROW


def test_wiring_unknown_port_and_ts_key():
    with pytest.raises(AlignmentError) as ei:
        align(TickOnly, {"lidar": ss(1.0), "lidarr": ss(1.0)})
    assert _rule_of(ei) is AlignRule.UNKNOWN_PORT
    assert "lidarr" in str(ei.value) and "lidar" in str(ei.value)  # names the ports
    with pytest.raises(AlignmentError) as ei:
        align(TickOnly, {"lidar": ss(1.0), "ts": ss(1.0)})
    assert _rule_of(ei) is AlignRule.UNKNOWN_PORT
    assert "row infrastructure" in str(ei.value)  # the ts teaching note


def test_wiring_missing_tick_and_required_streams():
    with pytest.raises(AlignmentError) as ei:
        align(LatestReq, {"pose": ss(1.0)})
    assert _rule_of(ei) is AlignRule.MISSING_TICK_STREAM
    with pytest.raises(AlignmentError) as ei:
        align(LatestReq, {"img": ss(1.0)})
    assert _rule_of(ei) is AlignRule.MISSING_REQUIRED_STREAM
    assert "default=" in str(ei.value)  # teaches the optional spelling


def test_wiring_shared_iterator():
    shared = iter(ss(1.0, 2.0))
    with pytest.raises(AlignmentError) as ei:
        align(LatestReq, {"img": shared, "pose": shared})
    assert _rule_of(ei) is AlignRule.SHARED_ITERATOR
    # the same re-iterable under two ports is legal (independent iterators)
    items = ss(1.0, 2.0)
    rows = list(align(LatestReq, {"img": items, "pose": items}))
    assert [(r.ts, r.pose.ts) for r in rows] == [(1.0, 1.0), (2.0, 2.0)]


def test_wiring_errors_are_eager():
    log: list[tuple[str, float]] = []
    with pytest.raises(AlignmentError):
        align(NoTick, {"pose": _logged(ss(1.0), log, "pose")})
    assert log == []  # wiring errors precede any pull (§7)


# ── interpolator registry (§6) ───────────────────────────────────────────────


def test_interp_registry_and_protocol():
    # seeded float lerp is directly resolvable (bare-float fields cannot be fed
    # by streams — the seed serves envelope implementations, spec §6.1)
    f = interpolator_for(float)
    assert f is not None and f(0.0, 10.0, 0.25) == 2.5
    # protocol method: Vec self-describes
    rows = list(align(VecInterp, {"img": ss(1.0), "vec": [Vec(0.0, 0.0), Vec(2.0, 8.0)]}))
    assert rows[0].vec == Vec(ts=1.0, x=4.0)
    # registry beats nothing / last-wins
    calls: list[str] = []

    def lerp_a(a: S, b: S, alpha: float) -> S:
        calls.append("a")
        return s_lerp(a, b, alpha)

    def lerp_b(a: S, b: S, alpha: float) -> S:
        calls.append("b")
        return s_lerp(a, b, alpha)

    register_interpolator(S, lerp_a)
    register_interpolator(S, lerp_b)
    list(align(InterpReq, {"img": ss(1.0), "pose": ss(0.0, 2.0)}))
    assert calls == ["b"]


def test_interp_mro_precedence():
    class Base:
        def __init__(self, ts: float) -> None:
            self.ts = ts

    class Sub(Base):
        def interpolate(self, other: Sub, alpha: float) -> Sub:
            return Sub(_lerp(self.ts, other.ts, alpha))

    register_interpolator(Base, lambda a, b, alpha: Base(_lerp(a.ts, b.ts, alpha)))
    # most-derived level wins: Sub's own method beats Base's registry entry
    f = interpolator_for(Sub)
    assert f is not None
    out = f(Sub(0.0), Sub(2.0), 0.5)
    assert type(out) is Sub
    # within one MRO level, an explicit registration beats the method (§6.2)
    register_interpolator(Sub, lambda a, b, alpha: Base(_lerp(a.ts, b.ts, alpha)))
    f = interpolator_for(Sub)
    assert f is not None
    assert type(f(Sub(0.0), Sub(2.0), 0.5)) is Base
    # base lookup unaffected; unregistered unrelated type resolves to None
    assert interpolator_for(Base) is not None
    assert interpolator_for(bytes) is None


def test_interp_optional_none_annotation():
    _use_s_lerp()
    # `S | None` + default=None dispatches on S (§6.3 Optional strip)
    rows = list(align(InterpOpt, {"img": ss(1.0), "pose": [S(0.0, 0.0), S(2.0, 8.0)]}))
    assert rows[0].pose == S(ts=1.0, v=4.0)


def test_interp_not_interpolatable_and_untyped():
    with pytest.raises(AlignmentError) as ei:
        align(IntInterp, {"img": ss(1.0), "n": ss(1.0)})
    assert _rule_of(ei) is AlignRule.NOT_INTERPOLATABLE
    assert "register_interpolator" in str(ei.value) and "latest()" in str(ei.value)
    with pytest.raises(AlignmentError) as ei:
        align(AnyInterp, {"img": ss(1.0), "a": ss(1.0)})
    assert _rule_of(ei) is AlignRule.INTERP_UNTYPED


def test_interpolator_not_called_on_dropped_tick():
    calls: list[float] = []

    def counting(a: S, b: S, alpha: float) -> S:
        calls.append(alpha)
        return s_lerp(a, b, alpha)

    register_interpolator(S, counting)

    class ReqBoth(In):
        img: S = tick()
        pose: S = latest()
        vel: S = interpolate()

    a = align(ReqBoth, {"img": ss(1.0), "pose": [], "vel": ss(0.0, 2.0)})
    assert list(a) == []  # pose (required latest) is missing → tick drops
    assert calls == []  # two-pass: the resolvable interpolate field never materialized
    assert a.stats.drops_by_field == {"pose": 1}


# ── stats (§8) ───────────────────────────────────────────────────────────────


def test_stats_scenario():
    _use_s_lerp()
    a = align(
        Combined,
        {
            "img": [S(1.0), S(2.0), S(3.0), S(3.0), S(4.0)],  # one duplicate
            "pose": ss(0.5, 2.5),
            "vel": ss(1.5),
        },
    )
    rows = list(a)
    assert [r.ts for r in rows] == [1.0, 2.0, 3.0, 4.0]
    st = a.stats
    assert st.ticks_fired == 4 and st.rows_emitted == 4 and st.ticks_dropped == 0
    assert st.ticks_fired == st.rows_emitted + st.ticks_dropped
    assert st.drops_by_field == {}
    assert st.ports["img"].accepted == 4 and st.ports["img"].dropped_nonmonotonic == 1
    assert st.ports["pose"].accepted == 2
    assert st.ports["vel"].accepted == 1


# ── bundle inheritance ───────────────────────────────────────────────────────


def test_inherited_bundle_aligns():
    rows = list(align(ExtIn, {"img": ss(1.0, 2.0), "pose": ss(0.5)}))
    assert type(rows[0]) is ExtIn
    assert [(r.ts, r.pose.ts) for r in rows] == [(1.0, 0.5), (2.0, 0.5)]


# ── determinism (§10) ────────────────────────────────────────────────────────


def _combined_streams() -> dict[str, list[S]]:
    return {
        "img": ss(1.0, 2.0, 3.0, 4.0, 5.0),
        "pose": ss(0.5, 1.5, 2.5, 4.5),
        "vel": ss(0.0, 2.0, 3.5, 6.0),
    }


def test_determinism_chunking_and_mapping_order():
    _use_s_lerp()
    base = _combined_streams()
    runs: list[tuple[list[Any], Any]] = []

    def run(streams: dict[str, Any]) -> None:
        a = align(Combined, streams)
        runs.append((list(a), a.stats))

    run(dict(base))  # lists
    run({k: iter(v) for k, v in base.items()})  # one-shot iterators
    run({k: itertools.chain(v[:1], v[1:3], v[3:]) for k, v in base.items()})  # sharded
    run(dict(reversed(list(base.items()))))  # shuffled mapping order
    first_rows, first_stats = runs[0]
    assert first_rows == oracle(Combined, base)
    for rows, stats in runs[1:]:
        assert rows == first_rows  # byte-identical rows regardless of delivery shape
        assert stats == first_stats  # counters are replay-stable too (§8.1)


def test_determinism_oracle_random_schedules():
    _use_s_lerp()
    for trial in range(25):
        rng = random.Random(1234 + trial)

        def raw(rng: random.Random, n: int) -> list[S]:
            out, t = [], 0.0
            for _ in range(n):
                # occasional zero/negative increments exercise the monotonic filter
                t += rng.choice([-0.2, 0.0, 0.3, 0.7, 1.1])
                out.append(S(ts=round(t, 3), v=rng.random()))
            return out

        streams = {
            "img": raw(rng, rng.randint(0, 12)),
            "pose": raw(rng, rng.randint(0, 12)),
            "vel": raw(rng, rng.randint(0, 12)),
        }
        assert list(align(Combined, dict(streams))) == oracle(Combined, streams), (
            f"trial {trial} diverged from the denotational oracle"
        )


# ── iterator contract ────────────────────────────────────────────────────────


def test_aligner_is_one_shot():
    a = align(TickOnly, {"lidar": ss(1.0)})
    assert iter(a) is a
    assert [r.ts for r in a] == [1.0]
    with pytest.raises(StopIteration):
        next(a)
    assert list(a) == []  # exhausted stays exhausted
