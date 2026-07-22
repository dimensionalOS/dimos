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

"""T13 graph layer (spec: tasks/t13-graph.md).

Live tests pin what the skeleton already delivers: the §5 error-catalog copy,
Port/feedback semantics, the application/transform dispatch on
``PureModule.__call__``, and ``PureModule.blueprint()`` symmetry. Everything
touching classification, build, ``over()``, and lowering is skip-gated on the
T13 implementation; graph classes live inside test bodies because their bare-In
bundles need the §2.4 T1 amendment to even define.
"""

from collections.abc import Iterator
from itertools import tee

import pytest

from dimos import pure as pm
from dimos.pure import graph as pg

# ── toy payloads / modules (definable under the landed engine) ───────────────


class PayA:
    def __init__(self, ts: float, v: float) -> None:
        self.ts, self.v = ts, v

    def __eq__(self, other):
        return isinstance(other, PayA) and (self.ts, self.v) == (other.ts, other.v)


class PayB:
    def __init__(self, ts: float, v: float) -> None:
        self.ts, self.v = ts, v

    def __eq__(self, other):
        return isinstance(other, PayB) and (self.ts, self.v) == (other.ts, other.v)


class MakeB(pm.PureModule):
    """A -> B, doubling."""

    class In(pm.In):
        a: PayA = pm.tick()

    class Out(pm.Out):
        b: PayB

    def step(self, i: In) -> Out:
        return MakeB.Out(b=PayB(ts=i.ts, v=i.a.v * 2.0))


class JoinB(pm.PureModule):
    """B (+ latest extra) -> total."""

    class In(pm.In):
        b: PayB = pm.tick()
        extra: PayB | None = pm.latest(default=None)

    class Out(pm.Out):
        total: PayB

    def step(self, i: In) -> Out:
        add = i.extra.v if i.extra is not None else 0.0
        return JoinB.Out(total=PayB(ts=i.ts, v=i.b.v + add))


class Foldy(pm.PureModule):
    """Fold twin of MakeB with a finalization receipt (teardown tests)."""

    class In(pm.In):
        a: PayA = pm.tick()

    class Out(pm.Out):
        b: PayB

    def fold(self, rows: Iterator[In]) -> Iterator[Out]:
        try:
            for row in rows:
                yield Foldy.Out(ts=row.ts, b=PayB(ts=row.ts, v=row.a.v * 2.0))
        finally:
            FOLDY_FINALIZED.append(1)


FOLDY_FINALIZED: list[int] = []


class BatchB(pm.PureModule):
    """A -> B, emitting every 2nd tick plus a finish() tail flush (Mealy)."""

    class In(pm.In):
        a: PayA = pm.tick()

    class Out(pm.Out):
        b: PayB

    class State(pm.State):
        n: int = 0

    def step(self, s: State, i: In) -> tuple[State, Out | None]:
        s = s.replace(n=s.n + 1)
        if s.n % 2 == 0:
            return s, BatchB.Out(b=PayB(ts=i.ts, v=float(s.n)))
        return s, None

    def finish(self, s: State) -> Out | None:
        if s.n == 0 or s.n % 2 == 0:
            return None
        return BatchB.Out(b=PayB(ts=float(s.n), v=float(s.n)))


COUNT_CALLS: list[int] = []


class CountUp(pm.PureModule):
    """A -> B, counting every step invocation (compute-once proof)."""

    class In(pm.In):
        a: PayA = pm.tick()

    class Out(pm.Out):
        b: PayB

    def step(self, i: In) -> Out:
        COUNT_CALLS.append(1)
        return CountUp.Out(b=PayB(ts=i.ts, v=i.a.v))


class CollectSink(pm.PureModule):
    """Terminal sink: tick on total, sample latest b; record what it saw."""

    class In(pm.In):
        total: PayB = pm.tick()
        b: PayB | None = pm.latest(default=None)

    class Out(pm.Out):
        n: int

    class State(pm.State):
        n: int = 0

    def step(self, s: State, i: In) -> tuple[State, Out]:
        COLLECT_SAW.append((i.total.v, None if i.b is None else i.b.v))
        return s.replace(n=s.n + 1), CollectSink.Out(n=s.n + 1)


COLLECT_SAW: list[tuple[float, float | None]] = []


class StaleStamp(pm.PureModule):
    """Emits every tick, but its payload ts is FROZEN — mimics a cached/wall-clock ts.

    The engine stamps the Out ROW from the tick (monotonic); the nested payload keeps
    ``ts=999.0``. A consumer ticking on this payload sees a constant ts and drops all
    but the first as ``nonmonotonic`` — unless the edge carries the row's engine ts
    (T13 §0.6).
    """

    class In(pm.In):
        a: PayA = pm.tick()

    class Out(pm.Out):
        b: PayB

    def step(self, i: In) -> Out:
        return StaleStamp.Out(b=PayB(ts=999.0, v=i.a.v))  # frozen payload ts; row ts = i.ts


class FutureStamp(pm.PureModule):
    """Emits every tick with a payload ts far in the future — mimics a wall-clock stamp.

    A ``latest`` sampler downstream pins this off-scale value as a pending head and
    never advances the producer again (starvation) unless the edge carries the row ts.
    """

    class In(pm.In):
        a: PayA = pm.tick()

    class Out(pm.Out):
        b: PayB

    def step(self, i: In) -> Out:
        return FutureStamp.Out(b=PayB(ts=1e12 + i.a.v, v=i.a.v))  # off-scale payload ts


class _StaleFan(pg.PureGraph):
    """StaleStamp fanned to JoinB (tick) + a direct b export — the delivery hot path."""

    class In(pm.In):
        a: PayA

    class Out(pm.Out):
        b: PayB
        total: PayB

    def wire(self, i: "_StaleFan.In") -> "_StaleFan.Out":
        s = StaleStamp()(a=i.a)
        jn = JoinB()(b=s.b)
        return _StaleFan.Out(b=s.b, total=jn.total)


class _Fan(pg.PureGraph):
    """Fan CountUp's output to two exports (b direct, total through JoinB)."""

    class In(pm.In):
        a: PayA

    class Out(pm.Out):
        b: PayB
        total: PayB

    def wire(self, i: "_Fan.In") -> "_Fan.Out":
        c = CountUp()(a=i.a)
        jn = JoinB()(b=c.b)
        return _Fan.Out(b=c.b, total=jn.total)


def _a_stream(n: int = 5) -> list[PayA]:
    return [PayA(ts=float(t + 1), v=float(t)) for t in range(n)]


# ── live: Port / PortRef / feedback ──────────────────────────────────────────


class TestPortLive:
    def test_portref_path(self):
        assert pg.PortRef("voxel_mapper", "global_map").path == "voxel_mapper.global_map"
        assert pg.PortRef(None, "lidar").path == "lidar"

    def test_feedback_closes_once(self):
        prev = pg.feedback()
        producer = pg.Port(pg.PortRef("planner", "cmd_vel"), PayB)
        prev.close(producer)
        with pytest.raises(pg.PureGraphDefinitionError, match=r"\[graph-feedback-reclosed\]"):
            prev.close(producer)

    def test_close_on_ordinary_port(self):
        port = pg.Port(pg.PortRef("m", "x"), PayA)
        with pytest.raises(pg.PureGraphDefinitionError, match=r"\[graph-close-not-feedback\]"):
            port.close(port)

    def test_close_with_non_port(self):
        prev = pg.feedback()
        with pytest.raises(pg.PureGraphDefinitionError, match=r"\[graph-not-a-port\]"):
            prev.close(3.0)

    def test_repr(self):
        assert repr(pg.Port(pg.PortRef("m", "x"), PayA)) == "Port(m.x: PayA)"
        assert repr(pg.feedback()) == "Port(<feedback open>: None)"

    def test_snake_case(self):
        assert pg.snake_case("VoxelMapper") == "voxel_mapper"
        assert pg.snake_case("NavStack") == "nav_stack"
        assert pg.snake_case("GO2Connection") == "go2_connection"


# ── live: §5 catalog copy (helpers are the single source; pinned verbatim) ───


class TestCatalogLive:
    def test_unknown_kwarg_verbatim(self):
        e = pg._e_unknown_kwarg(JoinB, MakeB, ["scan"], ["a"])
        assert str(e) == (
            "dimos.pure.test_graph.JoinB: wire applies dimos.pure.test_graph.MakeB "
            "with unknown port(s) 'scan'; In fields are: 'a'. Application kwargs must "
            "match the member's In field names (tf= is the reserved tf side channel). "
            "[graph-unknown-kwarg]"
        )
        assert e.graph_rule is pg.GraphRule.UNKNOWN_KWARG

    def test_unbound_input_verbatim(self):
        e = pg._e_unbound_input(JoinB, MakeB, ["a"])
        assert str(e) == (
            "dimos.pure.test_graph.JoinB: wire applies dimos.pure.test_graph.MakeB "
            "without required port(s) 'a' — bind each, or give the member field a "
            "default to make it optional. [graph-unbound-input]"
        )

    def test_tick_cycle_verbatim(self):
        e = pg._e_tick_cycle(JoinB, MakeB, "a")
        assert str(e) == (
            "dimos.pure.test_graph.JoinB: feedback closes into MakeB.a, a tick() input "
            "— a back edge through the trigger would define time by itself. Cycles are "
            "legal only through a sampled input (latest/interpolate/tf). "
            "[graph-tick-cycle]"
        )

    def test_unbound_rim_verbatim(self):
        e = pg._e_unbound_rim(JoinB, ["a", "b"])
        assert str(e) == (
            "dimos.pure.test_graph.JoinB.over(): rim input(s) 'a', 'b' have no source "
            "— pass a store carrying them by name, a named stream kwarg, or remap= a "
            "store channel onto them. [graph-unbound-rim]"
        )

    def test_every_helper_carries_its_slug_and_rule(self):
        cases = [
            (pg._e_apply_context(MakeB), pg.GraphRule.APPLY_CONTEXT),
            (pg._e_unknown_kwarg(JoinB, MakeB, ["x"], ["a"]), pg.GraphRule.UNKNOWN_KWARG),
            (pg._e_not_a_port(JoinB, MakeB, "a", 1.0), pg.GraphRule.NOT_A_PORT),
            (pg._e_close_not_a_port(1.0), pg.GraphRule.NOT_A_PORT),
            (pg._e_unbound_input(JoinB, MakeB, ["a"]), pg.GraphRule.UNBOUND_INPUT),
            (
                pg._e_type_mismatch(JoinB, MakeB, "a", "PayA", "i.b", "PayB"),
                pg.GraphRule.TYPE_MISMATCH,
            ),
            (pg._e_tick_cycle(JoinB, MakeB, "a"), pg.GraphRule.TICK_CYCLE),
            (pg._e_tf_unexpected(JoinB, MakeB), pg.GraphRule.TF_UNEXPECTED),
            (pg._e_tf_missing(JoinB, MakeB, ["position"]), pg.GraphRule.TF_MISSING),
            (pg._e_unclosed_feedback(JoinB, 2), pg.GraphRule.UNCLOSED_FEEDBACK),
            (pg._e_feedback_reclosed(), pg.GraphRule.FEEDBACK_RECLOSED),
            (pg._e_close_not_feedback(), pg.GraphRule.CLOSE_NOT_FEEDBACK),
            (pg._e_foreign_port(JoinB), pg.GraphRule.FOREIGN_PORT),
            (pg._e_duplicate_name(JoinB, "front"), pg.GraphRule.DUPLICATE_NAME),
            (pg._e_wire_return(JoinB, None, JoinB.Out), pg.GraphRule.WIRE_RETURN),
            (pg._e_unset_output(JoinB, "total", 1.0), pg.GraphRule.UNSET_OUTPUT),
            (pg._e_export_input(JoinB, "total", "b"), pg.GraphRule.EXPORT_INPUT),
            (pg._e_export_alias(JoinB, "total", "b"), pg.GraphRule.EXPORT_ALIAS),
            (pg._e_unused_input(JoinB, ["extra"]), pg.GraphRule.UNUSED_INPUT),
        ]
        for exc, rule in cases:
            assert str(exc).endswith(f"[{rule.value}]"), str(exc)
            assert exc.graph_rule is rule
            assert isinstance(exc, pm.PureModuleDefinitionError)
        run_cases = [
            (pg._e_unknown_stream(JoinB, ["x"], ["b", "extra"]), pg.GraphRunRule.UNKNOWN_STREAM),
            (pg._e_unbound_rim(JoinB, ["b"]), pg.GraphRunRule.UNBOUND_RIM),
            (pg._e_store_missing(JoinB, "go2_lidar", "lidar"), pg.GraphRunRule.STORE_MISSING),
            (pg._e_unknown_path(JoinB, "nope.b", ["make_b.b"]), pg.GraphRunRule.UNKNOWN_PATH),
            (pg._e_impl_pending("PureGraph.bind()"), pg.GraphRunRule.IMPL_PENDING),
        ]
        for exc, rule in run_cases:
            assert str(exc).endswith(f"[{rule.value}]"), str(exc)
            assert exc.graph_rule is rule
            assert isinstance(exc, pm.PureModuleRunError)


# ── live: the one operator's dispatch + blueprint symmetry ───────────────────


class TestCallDispatchLive:
    def test_keyword_application_reaches_the_graph_seam(self):
        # Keyword ports route to apply_symbolic, never the transformer. Applied
        # outside a build, symbolic application is meaningless: [graph-apply-context].
        with pytest.raises(pg.PureGraphDefinitionError, match=r"\[graph-apply-context\]"):
            MakeB()(a=pg.Port(pg.PortRef(None, "a"), PayA))

    def test_positional_iterator_is_still_the_transformer(self):
        assert list(MakeB()([])) == []  # S5 equivalence face, unchanged

    def test_mixed_call_is_a_type_error(self):
        with pytest.raises(TypeError, match="mixes"):
            MakeB()([], a=1.0)

    def test_module_blueprint_symmetry(self):
        from dimos.pure.legacy import legacy_actor

        bp = MakeB.blueprint()
        assert bp.blueprints[0].module is legacy_actor(MakeB)
        assert bp.blueprints[0].kwargs == {}

    def test_phase_c_stubs_are_loud(self):
        g = object.__new__(pg.PureGraph)  # no subclass is definable pre-impl
        with pytest.raises(pm.PureModuleRunError, match=r"\[graph-impl-pending\]"):
            pg.PureGraph.bind(g, transport=None)
        with pytest.raises(pm.PureModuleRunError, match=r"\[graph-impl-pending\]"):
            pg.PureGraph.partition(g, sensing=["make_b"])


# ── skip-gated: definition-time classification (spec §2) ─────────────────────


class TestGraphDefinition:
    def test_wire_missing(self):
        with pytest.raises(pg.PureGraphDefinitionError, match=r"\[graph-wire-missing\]"):

            class G(pg.PureGraph):
                pass

    def test_step_in_graph(self):
        with pytest.raises(pg.PureGraphDefinitionError, match=r"\[graph-step\]"):

            class G(pg.PureGraph):
                class In(pm.In):
                    a: PayA

                class Out(pm.Out):
                    b: PayB

                def step(self, i):
                    return None

                def wire(self, i: "G.In") -> "G.Out":
                    raise NotImplementedError

    def test_sampler_on_graph_in_field(self):
        with pytest.raises(pg.PureGraphDefinitionError, match=r"\[graph-port-sampler\]"):

            class G(pg.PureGraph):
                class In(pm.In):
                    a: PayA = pm.tick()

                class Out(pm.Out):
                    b: PayB

                def wire(self, i: "G.In") -> "G.Out":
                    raise NotImplementedError

    def test_default_on_graph_in_field(self):
        with pytest.raises(pg.PureGraphDefinitionError, match=r"\[graph-port-default\]"):

            class G(pg.PureGraph):
                class In(pm.In):
                    a: PayA = None  # type: ignore[assignment]

                class Out(pm.Out):
                    b: PayB

                def wire(self, i: "G.In") -> "G.Out":
                    raise NotImplementedError

    def test_contract_on_graph_out_field(self):
        with pytest.raises(pg.PureGraphDefinitionError, match=r"\[graph-port-sampler\]"):

            class G(pg.PureGraph):
                class In(pm.In):
                    a: PayA

                class Out(pm.Out):
                    b: PayB = pm.contract(min_hz=1.0)

                def wire(self, i: "G.In") -> "G.Out":
                    raise NotImplementedError

    def test_bare_in_bundle_defines(self):
        # §2.4 T1 amendment: a bare In field is legal and classifies as "bare".
        class Rim(pm.In):
            lidar: PayA

        assert Rim.fields()["lidar"].kind == "bare"

    def test_module_step_rejects_bare_in_field(self):
        # §2.4: module-side loudness relocates to the module class statement.
        with pytest.raises(pm.PureModuleDefinitionError, match=r"\[in-field-unsampled\]"):

            class Bad(pm.PureModule):
                class In(pm.In):
                    a: PayA  # bare — needs a sampler on a module

                class Out(pm.Out):
                    b: PayB

                def step(self, i: In) -> Out:
                    raise NotImplementedError


def _pipe():
    """The toy 3-edge DAG: rim a -> MakeB -> JoinB; fan-out exports mk.b too."""

    class Pipe(pg.PureGraph):
        class In(pm.In):
            a: PayA

        class Out(pm.Out):
            total: PayB
            b: PayB

        def wire(self, i: "Pipe.In") -> "Pipe.Out":
            mk = MakeB()(a=i.a)
            jn = JoinB()(b=mk.b)
            return Pipe.Out(total=jn.total, b=mk.b)

    return Pipe


# ── skip-gated: the build (spec §4, §5) ──────────────────────────────────────


class TestGraphBuild:
    def test_build_is_pure_and_plans_compare_equal(self):
        Pipe = _pipe()
        assert Pipe().build() == Pipe().build()

    def test_plan_shape(self):
        plan = _pipe()().build()
        assert [path for path, _ in plan.members] == ["make_b", "join_b"]
        assert pg.Binding(pg.PortRef(None, "a"), pg.PortRef("make_b", "a")) in plan.bindings
        assert pg.Binding(pg.PortRef("make_b", "b"), pg.PortRef("join_b", "b")) in plan.bindings
        assert dict(plan.exports) == {
            "total": pg.PortRef("join_b", "total"),
            "b": pg.PortRef("make_b", "b"),
        }

    def test_auto_naming_suffixes_duplicates(self):
        class Two(pg.PureGraph):
            class In(pm.In):
                a: PayA

            class Out(pm.Out):
                b: PayB

            def wire(self, i: "Two.In") -> "Two.Out":
                first = MakeB()(a=i.a)
                second = MakeB()(a=i.a)
                del first
                return Two.Out(b=second.b)

        paths = [p for p, _ in Two().build().members]
        assert paths == ["make_b", "make_b_2"]

    def test_unknown_kwarg(self):
        class G(pg.PureGraph):
            class In(pm.In):
                a: PayA

            class Out(pm.Out):
                b: PayB

            def wire(self, i: "G.In") -> "G.Out":
                mk = MakeB()(scan=i.a)  # wrong port name
                return G.Out(b=mk.b)

        with pytest.raises(pg.PureGraphDefinitionError, match=r"\[graph-unknown-kwarg\]"):
            G().build()

    def test_not_a_port(self):
        class G(pg.PureGraph):
            class In(pm.In):
                a: PayA

            class Out(pm.Out):
                b: PayB

            def wire(self, i: "G.In") -> "G.Out":
                mk = MakeB()(a=PayA(0.0, 0.0))  # a value, not a port
                return G.Out(b=mk.b)

        with pytest.raises(pg.PureGraphDefinitionError, match=r"\[graph-not-a-port\]"):
            G().build()

    def test_unbound_required_input(self):
        class G(pg.PureGraph):
            class In(pm.In):
                a: PayA

            class Out(pm.Out):
                b: PayB

            def wire(self, i: "G.In") -> "G.Out":
                mk = MakeB()()  # tick port left unbound
                return G.Out(b=mk.b)

        with pytest.raises(pg.PureGraphDefinitionError, match=r"\[graph-unbound-input\]"):
            G().build()

    def test_type_mismatch(self):
        class G(pg.PureGraph):
            class In(pm.In):
                b: PayB

            class Out(pm.Out):
                b: PayB

            def wire(self, i: "G.In") -> "G.Out":
                mk = MakeB()(a=i.b)  # PayB into a PayA port
                return G.Out(b=mk.b)

        with pytest.raises(pg.PureGraphDefinitionError, match=r"\[graph-type-mismatch\]"):
            G().build()

    def test_unused_input(self):
        class G(pg.PureGraph):
            class In(pm.In):
                a: PayA
                unused: PayA

            class Out(pm.Out):
                b: PayB

            def wire(self, i: "G.In") -> "G.Out":
                return G.Out(b=MakeB()(a=i.a).b)

        with pytest.warns(UserWarning, match=r"\[graph-unused-input\]"):
            G().build()  # Q2: an unused rim input warns, it does not fail the build

    def test_unset_output(self):
        class G(pg.PureGraph):
            class In(pm.In):
                a: PayA

            class Out(pm.Out):
                b: PayB

            def wire(self, i: "G.In") -> "G.Out":
                MakeB()(a=i.a)
                return G.Out(b=PayB(0.0, 0.0))  # a payload, not a port

        with pytest.raises(pg.PureGraphDefinitionError, match=r"\[graph-unset-output\]"):
            G().build()

    def test_export_input_passthrough(self):
        class G(pg.PureGraph):
            class In(pm.In):
                a: PayA

            class Out(pm.Out):
                echo: PayA

            def wire(self, i: "G.In") -> "G.Out":
                return G.Out(echo=i.a)

        with pytest.raises(pg.PureGraphDefinitionError, match=r"\[graph-export-input\]"):
            G().build()

    def test_unclosed_feedback(self):
        class G(pg.PureGraph):
            class In(pm.In):
                b: PayB

            class Out(pm.Out):
                total: PayB

            def wire(self, i: "G.In") -> "G.Out":
                prev: pg.Port[PayB] = pg.feedback()
                jn = JoinB()(b=i.b, extra=prev)  # never closed
                return G.Out(total=jn.total)

        with pytest.raises(pg.PureGraphDefinitionError, match=r"\[graph-unclosed-feedback\]"):
            G().build()

    def test_feedback_into_tick_is_a_plan_error(self):
        class G(pg.PureGraph):
            class In(pm.In):
                a: PayA

            class Out(pm.Out):
                total: PayB

            def wire(self, i: "G.In") -> "G.Out":
                prev: pg.Port[PayB] = pg.feedback()
                jn = JoinB()(b=prev)  # back edge into the trigger
                prev.close(jn.total)
                return G.Out(total=jn.total)

        with pytest.raises(pg.PureGraphDefinitionError, match=r"\[graph-tick-cycle\]"):
            G().build()

    def test_sampled_feedback_builds(self):
        class G(pg.PureGraph):
            class In(pm.In):
                b: PayB

            class Out(pm.Out):
                total: PayB

            def wire(self, i: "G.In") -> "G.Out":
                prev: pg.Port[PayB] = pg.feedback()
                jn = JoinB()(b=i.b, extra=prev)  # latest() — legal back edge
                prev.close(jn.total)
                return G.Out(total=jn.total)

        plan = G().build()
        assert plan.feedbacks == (
            pg.Binding(pg.PortRef("join_b", "total"), pg.PortRef("join_b", "extra")),
        )

    def test_foreign_port(self):
        stolen: list = []

        class Thief(pg.PureGraph):
            class In(pm.In):
                a: PayA

            class Out(pm.Out):
                b: PayB

            def wire(self, i: "Thief.In") -> "Thief.Out":
                stolen.append(i.a)
                return Thief.Out(b=MakeB()(a=i.a).b)

        Thief().build()

        class G(pg.PureGraph):
            class In(pm.In):
                a: PayA

            class Out(pm.Out):
                b: PayB

            def wire(self, i: "G.In") -> "G.Out":
                return G.Out(b=MakeB()(a=stolen[0]).b)  # another build's port

        with pytest.raises(pg.PureGraphDefinitionError, match=r"\[graph-foreign-port\]"):
            G().build()

    def test_nesting_flattens_by_path(self):
        Pipe = _pipe()

        class Outer(pg.PureGraph):
            class In(pm.In):
                a: PayA

            class Out(pm.Out):
                total: PayB

            def wire(self, i: "Outer.In") -> "Outer.Out":
                inner = Pipe()(a=i.a)
                return Outer.Out(total=inner.total)

        plan = Outer().build()
        assert [p for p, _ in plan.members] == ["pipe.make_b", "pipe.join_b"]


# ── skip-gated: the local runtime (spec §6, Phase A acceptance) ──────────────


class TestGraphOver:
    def test_equivalence_with_hand_chaining(self):
        Pipe = _pipe()
        source = _a_stream(6)
        with Pipe().over(a=source) as run:
            got_totals = run.total.to_list()
            got_bs = run.b.to_list()
        bs = (r.b for r in MakeB().over(a=source))
        b_join, b_out = tee(bs)
        want_totals = [r.total for r in JoinB().over(b=b_join)]
        assert got_totals == want_totals
        assert got_bs == list(b_out)

    def test_member_finish_tail_reaches_downstream(self):
        # a Mealy member's finish() tail flows to its downstream consumer like any
        # member output — the graph delivers the tail row, not just cadence emits.
        class G(pg.PureGraph):
            class In(pm.In):
                a: PayA

            class Out(pm.Out):
                total: PayB

            def wire(self, i: "G.In") -> "G.Out":
                b = BatchB()(a=i.a)
                return G.Out(total=JoinB()(b=b.b).total)

        with G().over(a=_a_stream(5)) as run:
            totals = run.total.to_list()
        # BatchB emits at ticks 2 & 4, finish() flushes the 5th tail → 3 downstream rows
        assert [t.v for t in totals] == [2.0, 4.0, 5.0]

    def test_streams_are_reiterable(self):
        # §0.2: an export re-iterates by re-running (over() is deterministic); no
        # edge log, no buffering. Two fresh runs over the same source agree.
        Pipe = _pipe()
        with Pipe().over(a=_a_stream(3)) as run:
            assert list(run.total) == list(run.total)

    def test_unknown_stream(self):
        Pipe = _pipe()
        with pytest.raises(pm.PureModuleRunError, match=r"\[graph-unknown-stream\]"):
            Pipe().over(nope=_a_stream())

    def test_unbound_rim(self):
        Pipe = _pipe()
        with pytest.raises(pm.PureModuleRunError, match=r"\[graph-unbound-rim\]"):
            Pipe().over()

    def test_early_close_finalizes_members(self):
        class G(pg.PureGraph):
            class In(pm.In):
                a: PayA

            class Out(pm.Out):
                b: PayB

            def wire(self, i: "G.In") -> "G.Out":
                return G.Out(b=Foldy()(a=i.a).b)

        FOLDY_FINALIZED.clear()
        run = G().over(a=_a_stream(100))
        next(iter(run.b))
        run.close()
        assert FOLDY_FINALIZED == [1]
        run.close()  # idempotent
        assert FOLDY_FINALIZED == [1]

    def test_build_runs_no_resources(self):
        FOLDY_FINALIZED.clear()

        class G(pg.PureGraph):
            class In(pm.In):
                a: PayA

            class Out(pm.Out):
                b: PayB

            def wire(self, i: "G.In") -> "G.Out":
                return G.Out(b=Foldy()(a=i.a).b)

        G().build()
        assert FOLDY_FINALIZED == []


RIM_SAW: list[tuple[float, float | None]] = []


class RimSink(pm.PureModule):
    """Ticks on the rim input a; latest-samples the derived b."""

    class In(pm.In):
        a: PayA = pm.tick()
        b: PayB | None = pm.latest(default=None)

    class Out(pm.Out):
        n: int

    class State(pm.State):
        n: int = 0

    def step(self, s: State, i: In) -> tuple[State, Out]:
        RIM_SAW.append((i.a.v, None if i.b is None else i.b.v))
        return s.replace(n=s.n + 1), RimSink.Out(n=s.n + 1)


# ── skip-gated: the terminal drive — compute-once multi-output (spec §0.5) ───


class TestTerminalDrive:
    def test_save_to_store_records_every_output(self):
        from dimos.memory2.store.memory import MemoryStore

        store = MemoryStore()
        COUNT_CALLS.clear()
        returned = _Fan().over(a=_a_stream(5)).save(store)
        assert returned is store
        assert sorted(store.list_streams()) == ["b", "total"]
        # JoinB has no extra source here, so total passes b through (v unchanged).
        assert [o.data.v for o in store.stream("b")] == [0.0, 1.0, 2.0, 3.0, 4.0]
        assert [o.data.v for o in store.stream("total")] == [0.0, 1.0, 2.0, 3.0, 4.0]
        # ts rides the payload's own ts — symmetric with ingestion (§0.3).
        assert [o.ts for o in store.stream("b")] == [1.0, 2.0, 3.0, 4.0, 5.0]

    def test_shared_upstream_computes_once(self):
        # The compute-once contract: CountUp feeds both exports, runs ONCE per tick.
        from dimos.memory2.store.memory import MemoryStore

        COUNT_CALLS.clear()
        _Fan().over(a=_a_stream(5)).save(MemoryStore())
        assert len(COUNT_CALLS) == 5  # not 10 — the voxel-mapper-shaped member ran once

    def test_per_export_pull_reruns_the_shared_member(self):
        # Contrast (§0.5): independent per-export lazy pull re-evaluates the cone.
        COUNT_CALLS.clear()
        with _Fan().over(a=_a_stream(5)) as run:
            run.b.to_list()
            run.total.to_list()
        assert len(COUNT_CALLS) == 10  # each export re-ran CountUp — why .save() exists

    def test_save_to_module_sink_drives_in_one_pass(self):
        COUNT_CALLS.clear()
        COLLECT_SAW.clear()
        frames = _Fan().over(a=_a_stream(5)).save(CollectSink())
        assert frames == 5
        assert len(COUNT_CALLS) == 5  # compute-once through the module sink too
        # tick on total, latest-sampled b — both fed from the one pass.
        assert COLLECT_SAW == [(0.0, 0.0), (1.0, 1.0), (2.0, 2.0), (3.0, 3.0), (4.0, 4.0)]

    def test_module_sink_remap(self):
        # remap a sink In field onto a differently-named output.
        class Sink2(pm.PureModule):
            class In(pm.In):
                x: PayB = pm.tick()

            class Out(pm.Out):
                n: int

            class State(pm.State):
                n: int = 0

            def step(self, s: State, i: In) -> tuple[State, Out]:
                return s.replace(n=s.n + 1), Sink2.Out(n=s.n + 1)

        frames = _Fan().over(a=_a_stream(4)).save(Sink2(), remap={"x": "total"})
        assert frames == 4

    def test_record_interior_edge_by_path(self):
        from dimos.memory2.store.memory import MemoryStore

        class TotalOnly(pg.PureGraph):
            class In(pm.In):
                a: PayA

            class Out(pm.Out):
                total: PayB  # make_b.b stays interior — only reachable by path

            def wire(self, i: "TotalOnly.In") -> "TotalOnly.Out":
                mk = MakeB()(a=i.a)
                jn = JoinB()(b=mk.b)
                return TotalOnly.Out(total=jn.total)

        store = MemoryStore()
        TotalOnly().over(a=_a_stream(3)).save(store, record={"make_b.b": "raw"})
        assert sorted(store.list_streams()) == ["raw", "total"]
        assert [o.data.v for o in store.stream("raw")] == [0.0, 2.0, 4.0]

    def test_unknown_record_path_raises(self):
        from dimos.memory2.store.memory import MemoryStore

        with pytest.raises(pm.PureModuleRunError, match=r"\[graph-unknown-path\]"):
            _pipe()().over(a=_a_stream(2)).save(MemoryStore(), record={"nope.x": "y"})

    def test_bad_sink_type_raises(self):
        with pytest.raises(TypeError, match="neither a mem2 store"):
            _pipe()().over(a=_a_stream(2)).save(object())

    def test_rim_input_records_by_name(self):
        # spec §0.5: a record= path naming a rim input tees the source stream
        from dimos.memory2.store.memory import MemoryStore

        store = MemoryStore()
        _Fan().over(a=_a_stream(3)).save(store, record={"a": "raw_a"})
        assert [o.data.v for o in store.stream("raw_a")] == [0.0, 1.0, 2.0]

    def test_rim_input_feeds_module_sink_port(self):
        # spec §0.5: a sink In port named after a rim input samples the source
        RIM_SAW.clear()
        frames = _Fan().over(a=_a_stream(3)).save(RimSink())
        assert frames == 3
        # tick a and edge b share ts; latest resolves inclusively (newest <= T,
        # spec §5.4), so each tick sees the b derived from that same a.
        assert RIM_SAW == [(0.0, 0.0), (1.0, 1.0), (2.0, 2.0)]

    def test_save_disposes_resources_once(self):
        # The terminal drive owns member drivers; teardown fires once at exhaustion.
        from dimos.memory2.store.memory import MemoryStore

        class G(pg.PureGraph):
            class In(pm.In):
                a: PayA

            class Out(pm.Out):
                b: PayB

            def wire(self, i: "G.In") -> "G.Out":
                return G.Out(b=Foldy()(a=i.a).b)

        FOLDY_FINALIZED.clear()
        G().over(a=_a_stream(5)).save(MemoryStore())
        assert FOLDY_FINALIZED == [1]


# ── progress markers (spec §0.7): a sparse producer must not exhaust the cone ──


NEVER_STEPS: list[int] = []


class NeverB(pm.PureModule):
    """A -> B that never emits — the planner-with-no-reachable-goal twin."""

    class In(pm.In):
        a: PayA = pm.tick()

    class Out(pm.Out):
        b: PayB

    class State(pm.State):
        n: int = 0

    def step(self, s: State, i: In) -> tuple[State, Out | None]:
        NEVER_STEPS.append(1)
        return s.replace(n=s.n + 1), None


class _SparseFan(pg.PureGraph):
    """CountUp -> b; NeverB -> silent (wired, never fires at runtime)."""

    class In(pm.In):
        a: PayA

    class Out(pm.Out):
        b: PayB
        silent: PayB

    def wire(self, i: "_SparseFan.In") -> "_SparseFan.Out":
        c = CountUp()(a=i.a)
        nv = NeverB()(a=i.a)
        return _SparseFan.Out(b=c.b, silent=nv.b)


SPARSE_SAW: list[tuple[float, float | None, int]] = []


class SparseSink(pm.PureModule):
    """Ticks on b; latest-samples silent; records upstream progress per tick."""

    class In(pm.In):
        b: PayB = pm.tick()
        silent: PayB | None = pm.latest(default=None)

    class Out(pm.Out):
        n: int

    class State(pm.State):
        n: int = 0

    def step(self, s: State, i: In) -> tuple[State, Out]:
        SPARSE_SAW.append((i.ts, None if i.silent is None else i.silent.v, len(NEVER_STEPS)))
        return s.replace(n=s.n + 1), SparseSink.Out(n=s.n + 1)


class TestProgressMarkers:
    def test_sparse_output_does_not_exhaust_cone_in_module_sink(self):
        # The rerun-sink incident: a silent planner port used to drive the whole
        # upstream cone to exhaustion on the sink's FIRST tick (burst-at-end).
        NEVER_STEPS.clear()
        SPARSE_SAW.clear()
        frames = _SparseFan().over(a=_a_stream(20)).save(SparseSink())
        assert frames == 20
        assert all(silent is None for _, silent, _ in SPARSE_SAW)  # default resolved
        # Markers bound the pull: at sink tick k, NeverB has ticked ~k times — not 20.
        assert SPARSE_SAW[0][2] <= 2
        assert SPARSE_SAW[9][2] <= 12

    def test_save_to_store_stores_no_markers(self):
        from dimos.memory2.store.memory import MemoryStore

        store = MemoryStore()
        _SparseFan().over(a=_a_stream(5)).save(store)
        assert [o.data.v for o in store.stream("b")] == [0.0, 1.0, 2.0, 3.0, 4.0]
        assert list(store.stream("silent")) == []  # frontier currency, never data

    def test_markers_never_cross_rim_on_export(self):
        class G(pg.PureGraph):
            class In(pm.In):
                a: PayA

            class Out(pm.Out):
                b: PayB

            def wire(self, i: "G.In") -> "G.Out":
                return G.Out(b=BatchB()(a=i.a).b)

        with G().over(a=_a_stream(5)) as run:
            outs = run.b.to_list()
        # cadence emits (n=2, 4) + the finish tail — and ONLY payloads, no markers
        assert all(isinstance(o, PayB) for o in outs)
        assert len(outs) == 3


# ── the edge-ts carry (spec §0.6): offline currency is the tick row's engine ts ──


class LatestSink(pm.PureModule):
    """Ticks on b; latest-samples total — a module sink that aligns on payload ts."""

    class In(pm.In):
        b: PayB = pm.tick()
        total: PayB | None = pm.latest(default=None)

    class Out(pm.Out):
        n: int

    class State(pm.State):
        n: int = 0

    def step(self, s: State, i: In) -> tuple[State, Out]:
        COLLECT_SAW.append((i.b.v, None if i.total is None else i.total.v))
        return s.replace(n=s.n + 1), LatestSink.Out(n=s.n + 1)


class TestEdgeTsCarry:
    """A member whose nested payload ts is off (frozen / wall-clock) must still deliver.

    Regression for the go2 nav bug: the PGO mapper's ``global_map`` carried a stale
    grid ts (dropped at the cost mapper as ``nonmonotonic``) and the planner's ``path``
    carried a wall-clock ts (starving the rerun sink's ``latest`` sampler). The offline
    edge now carries the producer's engine ts, so every emit reaches every consumer.
    """

    def test_over_frozen_payload_ts_reaches_tick_consumer(self):
        # StaleStamp emits b.ts=999 every tick; JoinB ticks on b. Without the ts carry,
        # JoinB drops all but the first (nonmonotonic) — total would be length 1.
        with _StaleFan().over(a=_a_stream(6)) as run:
            totals = run.total.to_list()
        assert len(totals) == 6  # every emit delivered, not collapsed to 1
        assert [t.ts for t in totals] == [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]  # tick ts, monotonic

    def test_save_store_frozen_payload_ts_records_every_edge(self):
        from dimos.memory2.store.memory import MemoryStore

        store = MemoryStore()
        _StaleFan().over(a=_a_stream(6)).save(store)
        # Both the direct b export and the JoinB total record every tick, on the tick ts.
        assert [o.ts for o in store.stream("b")] == [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]
        assert [o.ts for o in store.stream("total")] == [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]
        assert [o.data.v for o in store.stream("total")] == [0.0, 1.0, 2.0, 3.0, 4.0, 5.0]

    def test_save_module_wallclock_latest_reaches_sampler(self):
        # The real planner-starvation shape: the sink ticks on a data-scale stream (b
        # from MakeB) and latest-samples an off-scale one (FutureStamp, ts=1e12+v).
        # Without the ts carry the latest head sits far in the future forever, so the
        # sampler pins its first pull as pending and never advances the producer —
        # every tick samples None. With it, the sampled stream tracks the tick.
        class FutFan(pg.PureGraph):
            class In(pm.In):
                a: PayA

            class Out(pm.Out):
                b: PayB
                fut: PayB

            def wire(self, i: "FutFan.In") -> "FutFan.Out":
                return FutFan.Out(b=MakeB()(a=i.a).b, fut=FutureStamp()(a=i.a).b)

        class BFutSink(pm.PureModule):
            class In(pm.In):
                b: PayB = pm.tick()
                fut: PayB | None = pm.latest(default=None)

            class Out(pm.Out):
                n: int

            class State(pm.State):
                n: int = 0

            def step(self, s: State, i: In) -> tuple[State, Out]:
                COLLECT_SAW.append((i.b.v, None if i.fut is None else i.fut.v))
                return s.replace(n=s.n + 1), BFutSink.Out(n=s.n + 1)

        COLLECT_SAW.clear()
        frames = FutFan().over(a=_a_stream(6)).save(BFutSink())
        assert frames == 6
        # The off-scale stream was sampled on every tick (its producer was not starved).
        sampled = [f for _, f in COLLECT_SAW]
        assert sampled == [0.0, 1.0, 2.0, 3.0, 4.0, 5.0]  # not all-None

    def test_burst_chain_delivers_every_emission_once_in_order(self):
        # A bursts (StaleStamp: dense, frozen payload ts) -> B (JoinB) -> C (JoinB).
        # C must see every B emission exactly once, in tick order.
        class Chain(pg.PureGraph):
            class In(pm.In):
                a: PayA

            class Out(pm.Out):
                out: PayB

            def wire(self, i: "Chain.In") -> "Chain.Out":
                a = StaleStamp()(a=i.a)
                b = pg.named("b", JoinB())(b=a.b)
                c = pg.named("c", JoinB())(b=b.total)
                return Chain.Out(out=c.total)

        with Chain().over(a=_a_stream(8)) as run:
            got = run.out.to_list()
        assert [p.ts for p in got] == [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0]
        assert [p.v for p in got] == [0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0]


# ── data-marked: the hk nav DAG as a real graph (spec §9 acceptance 3) ─


def _hk_data(name):
    from dimos.utils.data import get_data

    try:
        return get_data(name)
    except Exception as exc:  # LFS unavailable / offline — skip, don't fail
        pytest.skip(f"dataset {name} unavailable: {exc}")


class TestNavStackPort:
    """NavStack (the hk nav DAG as a graph) matches the hand wiring."""

    def test_over_matches_hand_chaining(self):
        from itertools import tee as _tee

        from dimos.memory2.store.sqlite import SqliteStore
        from dimos.msgs.geometry_msgs.PointStamped import PointStamped
        from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
        from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
        from dimos.pure.modules.costmapper import PureCostMapper
        from dimos.pure.modules.odom_tf import OdomTf
        from dimos.pure.modules.planner import Planner
        from dimos.pure.modules.voxel_mapper import VoxelMapper

        from .modules.nav_stack import NavStack

        path = _hk_data("go2_hongkong_office.db")
        scans, voxel, every = 24, 0.1, 8

        def _goal():
            g = PointStamped(x=0.0, y=0.0, z=0.0, frame_id="world")
            g.ts = 1.0  # below every recording ts — planner's latest() sees it from tick 1
            return g

        # Hand wiring: the example DAG (tee fan-out), one lazy pass.
        with SqliteStore(path=str(path)) as store:
            lidar = store.stream("lidar", PointCloud2).range_seek(0, scans)
            odom = store.stream("odom", PoseStamped).range_seek(0, scans * 4)
            tf_items = [r.tf for r in OdomTf().over(odom=odom)]
            maps = (
                r.global_map
                for r in VoxelMapper(voxel_size=voxel, emit_every=every).over(scan=lidar)
            )
            map_cost, map_sink = _tee(maps)
            costs = (r.global_costmap for r in PureCostMapper().over(global_map=map_cost))
            cost_plan, cost_sink = _tee(costs)
            paths = [
                r.path for r in Planner().over(costmap=cost_plan, goal_point=[_goal()], tf=tf_items)
            ]
            want_maps = list(map_sink)
            want_costs = list(cost_sink)

        # The graph: same wiring, run in one process by NavStack.over().
        with SqliteStore(path=str(path)) as store:
            lidar = store.stream("lidar", PointCloud2).range_seek(0, scans)
            odom = store.stream("odom", PoseStamped).range_seek(0, scans * 4)
            with NavStack(voxel_size=voxel, emit_every=every).over(
                lidar=lidar, odom=odom, goal_point=[_goal()]
            ) as run:
                got_paths = run.path.to_list()
                got_maps = run.map.to_list()
                got_costs = run.costmap.to_list()

        # Same number of frames on each edge as the hand wiring.
        assert len(got_maps) == len(want_maps)
        assert len(got_costs) == len(want_costs)
        assert len(got_paths) == len(paths)
        assert got_maps, "expected at least one map emit over the slice"
        # Same payloads (deterministic pipeline): repr carries point/occupancy counts + ts.
        assert [repr(m) for m in got_maps] == [repr(m) for m in want_maps]
        assert [repr(c) for c in got_costs] == [repr(c) for c in want_costs]
        assert [len(p.poses) for p in got_paths] == [len(p.poses) for p in paths]

    def test_save_to_store_matches_per_export(self):
        # §0.5 acceptance: .save(store) records every output computed ONCE; the streams
        # match the Phase A per-export pull frame-for-frame.
        from dimos.memory2.store.memory import MemoryStore
        from dimos.memory2.store.sqlite import SqliteStore
        from dimos.msgs.geometry_msgs.PointStamped import PointStamped
        from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
        from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2

        from .modules.nav_stack import NavStack

        path = _hk_data("go2_hongkong_office.db")
        scans, voxel, every = 24, 0.1, 8

        def _goal():
            g = PointStamped(x=0.0, y=0.0, z=0.0, frame_id="world")
            g.ts = 1.0
            return g

        with SqliteStore(path=str(path)) as store:
            lidar = store.stream("lidar", PointCloud2).range_seek(0, scans)
            odom = store.stream("odom", PoseStamped).range_seek(0, scans * 4)
            with NavStack(voxel_size=voxel, emit_every=every).over(
                lidar=lidar, odom=odom, goal_point=[_goal()]
            ) as run:
                want_maps = [repr(m) for m in run.map.to_list()]
                want_costs = [repr(c) for c in run.costmap.to_list()]
                want_paths = [len(p.poses) for p in run.path.to_list()]

        out = MemoryStore()
        with SqliteStore(path=str(path)) as store:
            lidar = store.stream("lidar", PointCloud2).range_seek(0, scans)
            odom = store.stream("odom", PoseStamped).range_seek(0, scans * 4)
            NavStack(voxel_size=voxel, emit_every=every).over(
                lidar=lidar, odom=odom, goal_point=[_goal()]
            ).save(out)

        assert sorted(out.list_streams()) == ["costmap", "map", "path"]
        assert [repr(o.data) for o in out.stream("map")] == want_maps
        assert [repr(o.data) for o in out.stream("costmap")] == want_costs
        assert [len(o.data.poses) for o in out.stream("path")] == want_paths
        assert want_maps, "expected at least one map emit over the slice"


# ── live: blueprint lowering (spec §7, Phase B) ──────────────────────────────


class TestGraphBlueprint:
    def test_lowering_names_remaps_and_exposure(self):
        class Pipe2(pg.PureGraph):
            class In(pm.In):
                a: PayA

            class Out(pm.Out):
                total: PayB  # jn.total exported; mk.b stays interior

            def wire(self, i: "Pipe2.In") -> "Pipe2.Out":
                mk = MakeB()(a=i.a)
                jn = JoinB()(b=mk.b)
                return Pipe2.Out(total=jn.total)

        bp = Pipe2.blueprint()
        assert {atom.name for atom in bp.blueprints} == {"pipe2/make_b", "pipe2/join_b"}
        remap = dict(bp.remapping_map)
        # Interior edge: producer path-qualifies, consumer joins the same topic.
        assert remap[("pipe2/make_b", "b")] == "pipe2/make_b/b"
        assert remap[("pipe2/join_b", "b")] == "pipe2/make_b/b"
        # Rim ports stay bare (exposed) — linked by the autoconnect convention.
        assert remap.get(("pipe2/make_b", "a"), "a") == "a"
        assert remap.get(("pipe2/join_b", "total"), "total") == "total"

    def test_config_and_namespace_override(self):
        # Member config flows into the atom kwargs; namespace= overrides the prefix.
        class Scale(pm.PureModule):
            gain: float = 1.0

            class In(pm.In):
                a: PayA = pm.tick()

            class Out(pm.Out):
                b: PayB

            def step(self, i: In) -> Out:
                return Scale.Out(b=PayB(ts=i.ts, v=i.a.v * self.gain))

        class Cfg(pg.PureGraph):
            gain: float = 5.0

            class In(pm.In):
                a: PayA

            class Out(pm.Out):
                b: PayB

            def wire(self, i: "Cfg.In") -> "Cfg.Out":
                return Cfg.Out(b=Scale(gain=self.gain)(a=i.a).b)

        bp = Cfg.blueprint(gain=9.0, namespace="ns")
        (atom,) = bp.blueprints
        assert atom.name == "ns/scale"
        assert atom.kwargs["gain"] == 9.0  # graph config → member config → atom kwargs

    def test_navstack_lowering_matches_the_worked_table(self):
        # Spec §7.3 asserted against the produced Blueprint (no coordinator). The
        # landed NavStack additionally exports `map`, so voxel_mapper/global_map is
        # exposed rather than path-qualified — the one divergence from the charter
        # table row; the cost->plan name-crossing edge appears exactly as promised.
        from .modules.nav_stack import NavStack

        bp = NavStack.blueprint(voxel_size=0.1)
        assert {atom.name for atom in bp.blueprints} == {
            "nav_stack/odom_tf",
            "nav_stack/voxel_mapper",
            "nav_stack/pure_cost_mapper",
            "nav_stack/planner",
        }
        remap = dict(bp.remapping_map)

        # Rim In ports stay exposed (bare) — the name-crossing scan->lidar included.
        assert remap[("nav_stack/voxel_mapper", "scan")] == "lidar"
        assert remap[("nav_stack/odom_tf", "odom")] == "odom"
        assert remap[("nav_stack/planner", "goal_point")] == "goal_point"

        # The charter's name-crossing edge: cost.global_costmap -> planner.costmap,
        # both joined on the exported `costmap` topic (the export name wins).
        assert remap[("nav_stack/pure_cost_mapper", "global_costmap")] == "costmap"
        assert remap[("nav_stack/planner", "costmap")] == "costmap"

        # Exports stay exposed; voxel_mapper.global_map is exported AND consumed by
        # the cost mapper — one topic, `map`.
        assert remap[("nav_stack/planner", "path")] == "path"
        assert remap[("nav_stack/voxel_mapper", "global_map")] == "map"
        assert remap[("nav_stack/pure_cost_mapper", "global_map")] == "map"

        # An unexported interior Out stream is namespaced, never bare (health merges).
        assert remap[("nav_stack/voxel_mapper", "health")] == "nav_stack/health"
        assert remap[("nav_stack/planner", "health")] == "nav_stack/health"

    def test_combined_with_go2_matches_by_name_and_type(self):
        # Spec §9: the combined blueprint is valid — GO2's lidar/odom outputs link
        # NavStack's exposed rim inputs by (name, type); no topic collisions.
        from dimos.core.coordination.blueprints import autoconnect
        from dimos.core.coordination.module_coordinator import (
            _all_name_types,
            _verify_no_name_conflicts,
        )
        from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
        from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
        from dimos.robot.unitree.go2.connection import GO2Connection

        from .modules.nav_stack import NavStack

        bp = autoconnect(GO2Connection.blueprint(), NavStack.blueprint(voxel_size=0.1))
        pairs = _all_name_types(bp)
        assert ("lidar", PointCloud2) in pairs  # GO2 Out -> NavStack rim In
        assert ("odom", PoseStamped) in pairs
        _verify_no_name_conflicts(bp)  # namespaced interior never collides


class TestTwistUnstampTranslator:
    """The §0.4 cmd_vel seam: a TwistStamped -> Twist translator, not covariance."""

    def test_row_level_unwrap(self):
        from dimos.msgs.geometry_msgs.Twist import Twist
        from dimos.msgs.geometry_msgs.TwistStamped import TwistStamped

        from .modules.translators import TwistUnstamp

        ins = [
            TwistStamped(ts=1.0, linear=[1.0, 0.0, 0.0], angular=[0.0, 0.0, 0.5]),
            TwistStamped(ts=2.0, linear=[0.0, 2.0, 0.0], angular=[0.0, 0.0, 1.0]),
        ]
        outs = [r.cmd_vel for r in TwistUnstamp().over(cmd_vel_stamped=ins)]
        assert len(outs) == 2
        assert all(type(o) is Twist for o in outs)  # the stamp is gone (bare Twist)
        assert outs[0].linear == ins[0].linear and outs[0].angular == ins[0].angular
        assert outs[1].linear == ins[1].linear

    def test_is_bridgeable(self):
        # Register-clean: the legacy factory accepts it (no reserved-name collision),
        # and it lowers to an In[TwistStamped]/Out[Twist] atom for autoconnect.
        from dimos.core.coordination.blueprints import BlueprintAtom
        from dimos.msgs.geometry_msgs.Twist import Twist
        from dimos.msgs.geometry_msgs.TwistStamped import TwistStamped
        from dimos.pure.legacy import legacy_actor

        from .modules.translators import TwistUnstamp

        atom = BlueprintAtom.create(legacy_actor(TwistUnstamp), {})
        refs = {(s.name, s.type, s.direction) for s in atom.streams}
        assert ("cmd_vel_stamped", TwistStamped, "in") in refs
        assert ("cmd_vel", Twist, "out") in refs


# The full north star (spec §7.4) — NavStack.blueprint() autoconnected beside
# GO2Connection under --replay, deployed on the real worker pool — is heavy
# (webrtc import + standup sleep + multi-worker fork) and its END-TO-END payload
# emergence over cross-worker LCM replay is timing-dependent (a Phase A rim/
# transport concern, not the lowering). It is skipped in CI and gated on the
# dataset. The lowering itself is proven deterministically: the combined
# blueprint BUILDS + DEPLOYS every graph member beside GO2, each on its own
# worker, with the rim transports (lidar/odom in, costmap/path out) wired — which
# is what this test asserts. Live data flow THROUGH a lowered graph member is
# proven in-process by test_legacy.TestGraphBlueprintE2E; the NavStack DAG over
# this very dataset is proven by TestNavStackPort (both run by default).
@pytest.mark.skipif_in_ci  # heavy: worker pool + replay + webrtc import + standup sleep
class TestGroundingGo2Replay:
    """North star (spec §7.4): NavStack beside GO2Connection under --replay."""

    def test_lowered_graph_deploys_next_to_go2_replay(self):
        from dimos.core.coordination.blueprints import autoconnect
        from dimos.core.coordination.module_coordinator import ModuleCoordinator
        from dimos.robot.unitree.go2.connection import GO2Connection

        from .modules.nav_stack import NavStack

        path = _hk_data("go2_hongkong_office.db")
        bp = autoconnect(GO2Connection.blueprint(), NavStack.blueprint(voxel_size=0.1))
        mc = ModuleCoordinator.build(
            bp,
            {
                "g": {
                    "viewer": "none",
                    "robot_ip": "replay",  # ip=='replay' selects ReplayConnection
                    "replay_db": str(path),
                    "obstacle_avoidance": False,
                }
            },
        )
        try:
            # Every graph member deploys next to GO2, each on its own worker (P9).
            assert {
                "go2connection",
                "nav_stack/odom_tf",
                "nav_stack/voxel_mapper",
                "nav_stack/pure_cost_mapper",
                "nav_stack/planner",
            } <= set(mc._deployed_modules)
            workers = mc._managers["python"].workers  # type: ignore[attr-defined]
            owners = {w.worker_id for w in workers for m in w.module_names}
            assert len(owners) >= 5  # GO2 + four graph members, dedicated workers each

            # GO2's lidar/odom bridged onto the bus; the graph's exported outputs
            # (costmap/path) are wired transports — the rim exposed, interior namespaced.
            names = {name for name, _ in mc._transport_registry}
            assert {"lidar", "odom", "costmap", "path"} <= names  # rim exposed, bare
            # The name-crossing interior edge never leaks a bare interior topic.
            assert "global_costmap" not in names and "global_map" not in names
        finally:
            mc.stop()
            for _t in list(mc._transport_registry.values()):
                try:
                    _t.stop()
                except Exception:
                    pass
