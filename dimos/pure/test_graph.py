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

SKIP = pytest.mark.skip(reason="T13 impl pending")
SKIP_B = pytest.mark.skip(reason="T13 Phase B impl pending")


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
        # Skeleton pin (Opus replaces with real application): keyword ports must
        # route to apply_symbolic, never to the transformer.
        with pytest.raises(NotImplementedError, match="T13 impl pending"):
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


@SKIP
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


@SKIP
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

        with pytest.raises(pg.PureGraphDefinitionError, match=r"\[graph-unused-input\]"):
            G().build()

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


@SKIP
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

    def test_interior_tap_by_path(self):
        Pipe = _pipe()
        with Pipe().over(a=_a_stream(3)) as run:
            run.total.to_list()
            assert run.stream("make_b.b").to_list() == [
                PayB(ts=t, v=(t - 1) * 2.0) for t in (1.0, 2.0, 3.0)
            ]

    def test_streams_are_reiterable(self):
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

    def test_unknown_path(self):
        Pipe = _pipe()
        with Pipe().over(a=_a_stream(1)) as run:
            with pytest.raises(pm.PureModuleRunError, match=r"\[graph-unknown-path\]"):
                run.stream("nope.b")

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


# ── skip-gated: blueprint lowering (spec §7, Phase B) ────────────────────────


@SKIP_B
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

    def test_grounding_go2_replay(self):
        # North star (spec §7.4): NavStack beside GO2Connection under --replay.
        from dimos.core.coordination.blueprints import autoconnect
        from dimos.pure.modules.costmapper import CostMapper  # noqa: F401 (T12 tree)
        from dimos.robot.unitree.go2.connection import GO2Connection

        from .modules.nav_stack import NavStack  # landed with the NavStack port

        bp = autoconnect(GO2Connection.blueprint(), NavStack.blueprint(voxel_size=0.1))
        assert bp is not None  # coordinator run + on-bus assertions per spec §7.4
