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

"""T11 tf tests (spec: dimos/pure/tasks/t11-tf.md §14).

Definition/build-time behavior is landed and tested live; buffer, align
integration, and tf_out routing run against the skeleton and are skip-gated
until the implementer lands the bodies.
"""

from dataclasses import MISSING

import pytest

from dimos import pure as pm
from dimos.pure.rows import (
    BundleDefinitionError,
    TfOutSpec,
    TfSpec,
    format_frame,
    normalize_frame,
)
from dimos.pure.stepspec import PureModuleDefinitionError

# ── live: specifiers ─────────────────────────────────────────────────────────


class TestSpecifiers:
    def test_tf_required(self) -> None:
        s = pm.tf("world", "base_link")
        assert isinstance(s, TfSpec)
        assert (s.parent, s.child) == ("world", "base_link")
        assert s.kind == "tf" and s.side == "in"
        assert s.default is MISSING and s.required

    def test_tf_optional(self) -> None:
        s = pm.tf("world", "cam", default=None)
        assert isinstance(s, TfSpec)
        assert s.default is None and not s.required

    def test_tf_out_sparse_and_required(self) -> None:
        sparse = pm.tf_out("world", "map", default=None)
        assert isinstance(sparse, TfOutSpec)
        assert sparse.kind == "tf_out" and sparse.side == "out"
        assert sparse.default is None and sparse.min_hz is None
        required = pm.tf_out("world", "map", min_hz=1.0)
        assert required.required and required.min_hz == 1.0

    def test_tf_out_min_hz_validated(self) -> None:
        with pytest.raises(ValueError, match=r"min_hz must be > 0"):
            pm.tf_out("world", "map", min_hz=0.0)

    @pytest.mark.parametrize(
        "parent",
        ["", 3, "{}", "{a.b}", "{a[0]}", "{a!r}", "{a:>3}", "///"],
    )
    def test_bad_template_rejected_at_call(self, parent: object) -> None:
        with pytest.raises(ValueError, match=r"tf\(\): "):
            pm.tf(parent, "base_link")  # type: ignore[arg-type]

    def test_identical_templates_rejected_at_call(self) -> None:
        with pytest.raises(ValueError, match=r"identical"):
            pm.tf("{prefix}/odom", "{prefix}/odom")


# ── live: template machinery ─────────────────────────────────────────────────


class TestFrameTemplates:
    def test_normalize_frame(self) -> None:
        assert normalize_frame("go2a//odom") == "go2a/odom"
        assert normalize_frame("/odom") == "odom"
        assert normalize_frame("///") == ""

    def test_format_frame(self) -> None:
        assert format_frame("{prefix}/odom", {"prefix": "go2a"}) == "go2a/odom"
        assert format_frame("{prefix}/odom", {"prefix": ""}) == "odom"
        assert format_frame("{prefix}/odom", {"prefix": None}) == "odom"
        assert format_frame("robot{n}/base", {"n": 2}) == "robot2/base"
        assert format_frame("odom", {}) == "odom"

    def test_format_frame_unknown_key(self) -> None:
        with pytest.raises(KeyError):
            format_frame("{missing}/odom", {"prefix": "x"})

    def test_format_frame_bad_value(self) -> None:
        with pytest.raises(ValueError, match=r"not usable in a frame name"):
            format_frame("{cfg}/odom", {"cfg": [1, 2]})


# ── live: bundle rules ───────────────────────────────────────────────────────


class TestBundleRules:
    def test_tf_is_in_side_only(self) -> None:
        with pytest.raises(BundleDefinitionError, match=r"tf\(\) is not valid on an Out"):

            class BadOut(pm.Out):
                edge: float = pm.tf("world", "base")

    def test_tf_out_is_out_side_only(self) -> None:
        with pytest.raises(BundleDefinitionError, match=r"tf_out\(\) is not valid on an In"):

            class BadIn(pm.In):
                scan: float = pm.tick()
                edge: float = pm.tf_out("world", "base")

    def test_in_field_named_tf_reserved(self) -> None:
        # Unconditional: 'tf' is the stream key (over(tf=...), m.i.tf).
        with pytest.raises(BundleDefinitionError, match=r"'tf' is reserved"):

            class BadName(pm.In):
                tf: float = pm.tick()

    def test_out_field_named_tf_still_legal(self) -> None:
        class Fine(pm.Out):
            tf: float | None = None

        assert "tf" in Fine.fields()


# ── live: build resolution + introspection ───────────────────────────────────


class Conn(pm.PureModule):
    prefix: str = ""

    class In(pm.In):
        odom: float = pm.tick(expect_hz=50)
        world_to_base: float = pm.tf("world", "{prefix}/base_link")

    class Out(pm.Out):
        pose: float = pm.contract(min_hz=10)
        tf_odom_base: float | None = pm.tf_out("{prefix}/odom", "{prefix}/base_link", default=None)

    def step(self, i: In) -> Out:
        return Conn.Out(pose=i.odom, tf_odom_base=None)


class TestBuildResolution:
    def test_templates_resolve_per_instance(self) -> None:
        a, b = Conn(prefix="go2a"), Conn(prefix="go2b")
        assert a.o.tf_odom_base.frames == ("go2a/odom", "go2a/base_link")
        assert b.o.tf_odom_base.frames == ("go2b/odom", "go2b/base_link")
        assert a.i.world_to_base.frames == ("world", "go2a/base_link")

    def test_empty_prefix_segment_drops(self) -> None:
        m = Conn()
        assert m.o.tf_odom_base.frames == ("odom", "base_link")
        assert m.i.world_to_base.frames == ("world", "base_link")

    def test_fields_carry_tf_specs(self) -> None:
        fields = Conn.In.fields()
        assert isinstance(fields["world_to_base"], TfSpec)
        assert isinstance(Conn.Out.fields()["tf_odom_base"], TfOutSpec)

    def test_frames_on_non_tf_port_raises(self) -> None:
        m = Conn()
        with pytest.raises(NotImplementedError, match=r"not a tf\(\)/tf_out\(\) port"):
            _ = m.o.pose.frames

    def test_template_unknown_names_module_field_template(self) -> None:
        class Bad(pm.PureModule):
            class In(pm.In):
                scan: float = pm.tick()

            class Out(pm.Out):
                edge: float | None = pm.tf_out("{missing}/odom", "base", default=None)

            def step(self, i: In) -> Out:
                return Bad.Out(edge=None)

        with pytest.raises(
            PureModuleDefinitionError,
            match=r"Bad: tf field 'edge' template '\{missing\}/odom' names 'missing'.*"
            r"\[tf-template-unknown\]",
        ):
            Bad()

    def test_resolved_self_edge_rejected(self) -> None:
        class Selfy(pm.PureModule):
            a: str = "x"
            b: str = "x"

            class In(pm.In):
                scan: float = pm.tick()

            class Out(pm.Out):
                edge: float | None = pm.tf_out("{a}", "{b}", default=None)

            def step(self, i: In) -> Out:
                return Selfy.Out(edge=None)

        with pytest.raises(PureModuleDefinitionError, match=r"\[tf-self-edge\]"):
            Selfy()

    def test_resolved_empty_frame_rejected(self) -> None:
        class Empty(pm.PureModule):
            prefix: str = ""

            class In(pm.In):
                scan: float = pm.tick()

            class Out(pm.Out):
                edge: float | None = pm.tf_out("{prefix}", "base", default=None)

            def step(self, i: In) -> Out:
                return Empty.Out(edge=None)

        with pytest.raises(PureModuleDefinitionError, match=r"\[tf-frame-empty\]"):
            Empty()

    def test_duplicate_edge_rejected_reversed_too(self) -> None:
        class Dup(pm.PureModule):
            class In(pm.In):
                scan: float = pm.tick()

            class Out(pm.Out):
                fwd: float | None = pm.tf_out("world", "map", default=None)
                rev: float | None = pm.tf_out("map", "world", default=None)

            def step(self, i: In) -> Out:
                return Dup.Out(fwd=None, rev=None)

        with pytest.raises(PureModuleDefinitionError, match=r"\[tf-duplicate-edge\]"):
            Dup()

    def test_tf_free_module_pays_nothing(self) -> None:
        class Plain(pm.PureModule):
            class In(pm.In):
                x: float = pm.tick()

            class Out(pm.Out):
                y: float = pm.contract(min_hz=1)

            def step(self, i: In) -> Out:
                return Plain.Out(y=i.x)

        m = Plain()
        assert type(m).__pure_tf_templates__ == {}
        assert not hasattr(m, "__pure_tf_frames__")


# ── skip-gated: buffer semantics (spec §4, §5) ───────────────────────────────


def _tfm(parent: str, child: str, ts: float, x: float = 0.0, y: float = 0.0):  # type: ignore[no-untyped-def]
    from dimos.msgs.geometry_msgs.Transform import Transform
    from dimos.msgs.geometry_msgs.Vector3 import Vector3

    return Transform(translation=Vector3(x, y, 0.0), frame_id=parent, child_frame_id=child, ts=ts)


@pytest.mark.skip(reason="T11 impl pending")
class TestTfBuffer:
    def _buffer(self):  # type: ignore[no-untyped-def]
        from dimos.pure.tfbuffer import TfBuffer

        return TfBuffer()

    def test_direct_edge_interpolates(self) -> None:
        buf = self._buffer()
        buf.ingest(_tfm("world", "odom", 0.0, x=0.0))
        buf.ingest(_tfm("world", "odom", 1.0, x=2.0))
        t = buf.resolve("world", "odom", 0.5)
        assert t is not None
        assert t.translation.x == pytest.approx(1.0)
        assert t.ts == pytest.approx(0.5)
        assert (t.frame_id, t.child_frame_id) == ("world", "odom")

    def test_exact_hit_returns_sample(self) -> None:
        buf = self._buffer()
        sample = _tfm("world", "odom", 1.0, x=2.0)
        buf.ingest(_tfm("world", "odom", 0.0))
        buf.ingest(sample)
        assert buf.resolve("world", "odom", 1.0) is sample

    def test_no_extrapolation(self) -> None:
        buf = self._buffer()
        buf.ingest(_tfm("world", "odom", 1.0, x=2.0))
        assert buf.resolve("world", "odom", 0.5) is None  # left of all samples
        assert buf.resolve("world", "odom", 1.5) is None  # right of all samples

    def test_reverse_edge_inverts(self) -> None:
        buf = self._buffer()
        buf.ingest(_tfm("world", "odom", 0.0, x=3.0))
        buf.ingest(_tfm("world", "odom", 2.0, x=3.0))
        t = buf.resolve("odom", "world", 1.0)
        assert t is not None
        assert t.translation.x == pytest.approx(-3.0)
        assert (t.frame_id, t.child_frame_id) == ("odom", "world")

    def test_chain_composition_worked_example(self) -> None:
        # spec §5.3: world->odom is Rz(90deg) + t=(10,0,0); odom->base is t=(1,0,0).
        # T_world_base = T_world_odom + T_odom_base -> translation (10, 1, 0).
        import math

        from dimos.msgs.geometry_msgs.Quaternion import Quaternion
        from dimos.msgs.geometry_msgs.Transform import Transform
        from dimos.msgs.geometry_msgs.Vector3 import Vector3

        buf = self._buffer()
        s = math.sin(math.pi / 4)
        for ts in (0.0, 2.0):
            buf.ingest(
                Transform(
                    translation=Vector3(10.0, 0.0, 0.0),
                    rotation=Quaternion(0.0, 0.0, s, s),
                    frame_id="world",
                    child_frame_id="odom",
                    ts=ts,
                )
            )
            buf.ingest(_tfm("odom", "base", ts, x=1.0))
        t = buf.resolve("world", "base", 1.0)
        assert t is not None
        assert t.translation.x == pytest.approx(10.0)
        assert t.translation.y == pytest.approx(1.0)
        assert (t.frame_id, t.child_frame_id) == ("world", "base")

    def test_identity_fast_path(self) -> None:
        buf = self._buffer()
        t = buf.resolve("world", "world", 5.0)
        assert t is not None and t.ts == 5.0

    def test_static_resolves_at_any_ts(self) -> None:
        buf = self._buffer()
        buf.set_static(_tfm("base", "lidar", 0.0, x=0.2))
        for at in (0.0, 1.0, 1e9):
            t = buf.resolve("base", "lidar", at)
            assert t is not None and t.translation.x == pytest.approx(0.2)

    def test_static_composes_with_dynamic(self) -> None:
        buf = self._buffer()
        buf.set_static(_tfm("base", "lidar", 0.0, x=0.2))
        buf.ingest(_tfm("odom", "base", 0.0, x=1.0))
        buf.ingest(_tfm("odom", "base", 2.0, x=1.0))
        t = buf.resolve("odom", "lidar", 1.0)
        assert t is not None and t.translation.x == pytest.approx(1.2)

    def test_out_of_order_ingest_sorts(self) -> None:
        buf = self._buffer()
        buf.ingest(_tfm("world", "odom", 1.0, x=1.0))
        buf.ingest(_tfm("world", "odom", 0.5, x=0.5))  # late: sorted in, kept
        t = buf.resolve("world", "odom", 0.75)
        assert t is not None and t.translation.x == pytest.approx(0.75)
        assert buf.stats.edges[("world", "odom")].accepted == 2

    def test_duplicate_ts_first_wins(self) -> None:
        buf = self._buffer()
        first = _tfm("world", "odom", 1.0, x=1.0)
        buf.ingest(first)
        buf.ingest(_tfm("world", "odom", 1.0, x=9.9))  # duplicate ts: dropped
        assert buf.resolve("world", "odom", 1.0) is first
        stats = buf.stats.edges[("world", "odom")]
        assert stats.accepted == 1 and stats.dropped_duplicate == 1

    def test_window_eviction_keeps_newest(self) -> None:
        from dimos.pure.tfbuffer import TfBuffer

        buf = TfBuffer(horizon=1.0)
        for ts in (0.0, 0.5, 1.0, 3.0):
            buf.ingest(_tfm("world", "odom", ts))
        stats = buf.stats.edges[("world", "odom")]
        assert stats.evicted == 3  # 0.0, 0.5, 1.0 < 3.0 - 1.0
        assert buf.resolve("world", "odom", 3.0) is not None  # newest survives
        assert buf.resolve("world", "odom", 0.5) is None  # evicted history
        buf.ingest(_tfm("world", "odom", 1.5))  # older than the live window
        assert buf.stats.edges[("world", "odom")].dropped_expired == 1

    def test_claims_single_writer(self) -> None:
        from dimos.pure.tfbuffer import TfError

        buf = self._buffer()
        buf.claim(("world", "map"), "A")
        buf.claim(("world", "map"), "A")  # same owner: idempotent
        with pytest.raises(TfError, match=r"\[tf-multi-writer\]"):
            buf.claim(("world", "map"), "B")
        with pytest.raises(TfError, match=r"\[tf-multi-writer\]"):
            buf.claim(("map", "world"), "B")  # reversed orientation conflicts too
        buf.release("A")
        buf.claim(("world", "map"), "B")  # released edges reclaim freely

    def test_claims_forest(self) -> None:
        from dimos.pure.tfbuffer import TfError

        buf = self._buffer()
        buf.claim(("world", "odom"), "A")
        buf.claim(("odom", "base"), "B")
        with pytest.raises(TfError, match=r"\[tf-frame-cycle\]"):
            buf.claim(("base", "world"), "C")


# ── skip-gated: align integration through over() (spec §6, §8) ───────────────


@pytest.mark.skip(reason="T11 impl pending")
class TestOverIntegration:
    def _consumer(self):  # type: ignore[no-untyped-def]
        from dimos.msgs.geometry_msgs.Transform import Transform

        class Follower(pm.PureModule):
            class In(pm.In):
                odom: float = pm.tick()
                world_to_base: Transform = pm.tf("world", "base")

            class Out(pm.Out):
                x: float = pm.contract(min_hz=1)

            def step(self, i: In) -> Out:
                return Follower.Out(x=i.world_to_base.translation.x)

        return Follower

    def test_required_tf_resolves_at_tick_ts(self) -> None:
        from dimos.pure.test_align import S  # the house stamped fixture

        Follower = self._consumer()
        ticks = [S(ts=0.5, v=1.0)]
        tf_stream = [_tfm("world", "base", 0.0, x=0.0), _tfm("world", "base", 1.0, x=2.0)]
        rows = list(Follower().over(odom=ticks, tf=tf_stream))
        assert len(rows) == 1
        assert rows[0].x == pytest.approx(1.0)  # interpolated at ts=0.5

    def test_required_tf_holds_across_late_bracket(self) -> None:
        from dimos.pure.test_align import S

        Follower = self._consumer()
        # Right bracket for the tick at 0.5 arrives two items later on another
        # edge's cadence: the tick holds (pulls tf forward) and still emits.
        tf_stream = [
            _tfm("world", "base", 0.4, x=0.4),
            _tfm("odom", "base", 0.55, x=9.9),  # frontier passes T on a foreign edge
            _tfm("odom", "base", 0.58, x=9.9),
            _tfm("world", "base", 0.6, x=0.6),
        ]
        rows = list(Follower().over(odom=[S(ts=0.5, v=0.0)], tf=tf_stream))
        assert len(rows) == 1
        assert rows[0].x == pytest.approx(0.5)

    def test_required_tf_drops_at_exhaustion(self) -> None:
        from dimos.pure.test_align import S

        Follower = self._consumer()
        tf_stream = [_tfm("world", "base", 0.0), _tfm("world", "base", 1.0)]
        rows = list(Follower().over(odom=[S(ts=0.5, v=0.0), S(ts=2.0, v=0.0)], tf=tf_stream))
        assert len(rows) == 1  # tick at 2.0 has no right bracket, stream exhausted: drop

    def test_optional_tf_defaults_without_holding(self) -> None:
        from dimos.msgs.geometry_msgs.Transform import Transform
        from dimos.pure.test_align import S

        class Optional(pm.PureModule):
            class In(pm.In):
                odom: float = pm.tick()
                world_to_cam: Transform | None = pm.tf("world", "cam", default=None)

            class Out(pm.Out):
                ok: float = pm.contract(min_hz=1)

            def step(self, i: In) -> Out:
                return Optional.Out(ok=1.0 if i.world_to_cam is None else 2.0)

        rows = list(Optional().over(odom=[S(ts=0.5, v=0.0)], tf=[]))
        assert [r.ok for r in rows] == [1.0]

    def test_statics_only_buffer_resolves(self) -> None:
        from dimos.pure.test_align import S
        from dimos.pure.tfbuffer import TfBuffer

        Follower = self._consumer()
        buf = TfBuffer(statics=[_tfm("world", "base", 0.0, x=7.0)])
        rows = list(Follower().over(odom=[S(ts=0.5, v=0.0)], tf=buf))
        assert [r.x for r in rows] == [pytest.approx(7.0)]

    def test_tf_kwarg_without_tf_fields_rejected(self) -> None:
        from dimos.pure.test_align import S
        from dimos.pure.tfbuffer import TfError

        class Plain(pm.PureModule):
            class In(pm.In):
                x: float = pm.tick()

            class Out(pm.Out):
                y: float = pm.contract(min_hz=1)

            def step(self, i: In) -> Out:
                return Plain.Out(y=i.x)

        with pytest.raises(TfError, match=r"\[tf-unexpected-stream\]"):
            Plain().over(x=[S(ts=0.0, v=0.0)], tf=[])

    def test_required_tf_without_source_rejected(self) -> None:
        from dimos.pure.test_align import S
        from dimos.pure.tfbuffer import TfError

        Follower = self._consumer()
        with pytest.raises(TfError, match=r"\[tf-missing-stream\]"):
            Follower().over(odom=[S(ts=0.0, v=0.0)])

    def test_determinism_chunking(self) -> None:
        import itertools

        from dimos.pure.test_align import S

        Follower = self._consumer()
        ticks = [S(ts=t, v=0.0) for t in (0.2, 0.5, 0.8)]
        tf_items = [_tfm("world", "base", t / 10.0, x=t / 10.0) for t in range(0, 12)]
        a = list(Follower().over(odom=ticks, tf=list(tf_items)))
        b = list(Follower().over(odom=iter(ticks), tf=itertools.chain(tf_items[:5], tf_items[5:])))
        assert [(r.ts, r.x) for r in a] == [(r.ts, r.x) for r in b]


# ── skip-gated: tf_out routing (spec §7) ─────────────────────────────────────


@pytest.mark.skip(reason="T11 impl pending")
class TestTfOutRouting:
    def test_self_ingestion_brackets_with_stream(self) -> None:
        # spec §7.3: a module's own assertion (ingested by the tap at tick 0.0)
        # forms the LEFT bracket for its own later tick at 0.5; the stream's
        # sample at 0.6 — pulled early by advance_past(0.0) — is the RIGHT one.
        # Sorted per-edge insert is what lets the older own-assertion land
        # behind the already-ingested stream sample.
        from dimos.msgs.geometry_msgs.Transform import Transform
        from dimos.pure.test_align import S

        class Loop(pm.PureModule):
            class In(pm.In):
                scan: float = pm.tick()
                world_to_map: Transform | None = pm.tf("world", "map", default=None)

            class Out(pm.Out):
                seen: float = pm.contract(min_hz=1)
                tf_world_map: Transform | None = pm.tf_out("world", "map", default=None)

            def step(self, i: In) -> Out:
                return Loop.Out(
                    seen=-1.0 if i.world_to_map is None else i.world_to_map.translation.x,
                    tf_world_map=_tfm("world", "map", i.ts, x=i.ts),
                )

        rows = list(
            Loop().over(
                scan=[S(ts=0.0, v=0.0), S(ts=0.5, v=0.0)],
                tf=[_tfm("world", "map", 0.6, x=0.6)],
            )
        )
        # tick 0.0: only the stream's 0.6 sample exists — no left bracket — None;
        # tick 0.5: own 0.0 assertion + stream 0.6 bracket -> lerp = 0.5.
        assert [r.seen for r in rows] == [-1.0, pytest.approx(0.5)]

    def test_frame_mismatch_rejected(self) -> None:
        from dimos.msgs.geometry_msgs.Transform import Transform
        from dimos.pure.test_align import S
        from dimos.pure.tfbuffer import TfError

        class Liar(pm.PureModule):
            class In(pm.In):
                scan: float = pm.tick()

            class Out(pm.Out):
                tf_world_map: Transform | None = pm.tf_out("world", "map", default=None)
                ok: float = pm.contract(min_hz=1)

            def step(self, i: In) -> Out:
                return Liar.Out(tf_world_map=_tfm("world", "odom", i.ts), ok=1.0)

        with pytest.raises(TfError, match=r"\[tf-out-frame-mismatch\]"):
            list(Liar().over(scan=[S(ts=0.0, v=0.0)]))

    def test_unstamped_payload_rejected(self) -> None:
        from dimos.msgs.geometry_msgs.Transform import Transform
        from dimos.pure.test_align import S
        from dimos.pure.tfbuffer import TfError

        class Unstamped(pm.PureModule):
            class In(pm.In):
                scan: float = pm.tick()

            class Out(pm.Out):
                tf_world_map: Transform | None = pm.tf_out("world", "map", default=None)
                ok: float = pm.contract(min_hz=1)

            def step(self, i: In) -> Out:
                bad = _tfm("world", "map", 1.0)
                bad.ts = float("-inf")
                return Unstamped.Out(tf_world_map=bad, ok=1.0)

        with pytest.raises(TfError, match=r"\[tf-out-unstamped\]"):
            list(Unstamped().over(scan=[S(ts=0.0, v=0.0)]))
