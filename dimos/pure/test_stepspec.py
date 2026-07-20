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

"""T3 classification tests — executable spec for tasks/t3-validation.md §12.

Modules under test are defined inside test functions with nested bundles and
classify() is called directly: no engine imports. Cross-module and
annotation-regime cases use exec-created fixture modules (spec §12 preamble).
"""

from collections.abc import Awaitable, Generator, Iterable, Iterator
import functools
import itertools
import sys
import textwrap
import types
import typing
from typing import NamedTuple, Optional, TypeVar, Union

import pytest

from dimos.pure.rows import In as PmIn, Out as PmOut, contract, interpolate, latest, tick
from dimos.pure.stepspec import (
    PURE_STEP_ATTR,
    PureModuleDefinitionError,
    Rule,
    StepKind,
    StepSpec,
    classify,
    step_spec,
)

# Shared minimal rows for error-path tests where bundle identity is incidental.
# Module-level so they resolve from fn.__globals__ under every annotation regime.


class RowIn(PmIn):
    x: float = tick()


class RowOut(PmOut):
    y: float = 0.0


_seq = itertools.count()


def _mk_module(source, name=None):
    """Exec *source* as a registered module (spec §12 fixture recipe)."""
    name = name or f"_t3_fixture_{next(_seq)}"
    mod = types.ModuleType(name)
    sys.modules[name] = mod
    exec(compile(textwrap.dedent(source), f"<{name}>", "exec"), mod.__dict__)
    return mod


def _raises(cls, rule):
    """classify(cls) must raise the given Rule; returns the message."""
    with pytest.raises(PureModuleDefinitionError) as e:
        classify(cls)
    assert e.value.rule is rule
    msg = str(e.value)
    assert cls.__qualname__ in msg
    assert f"[{rule.value}]" in msg
    return e


# ── §12.1 classification happy paths ─────────────────────────────────────────


def test_tagger_floor_zero_ceremony():
    # The API-floor guard: the sketch's 4-declaration Tagger, zero added ceremony.
    class Tagger:
        class In(PmIn):
            image: float = tick(expect_hz=30)
            pose: float = interpolate()

        class Out(PmOut):
            located: str = contract(min_hz=10)

        def step(self, i: In) -> Out:
            raise NotImplementedError

    spec = classify(Tagger)
    assert spec.kind is StepKind.STATELESS
    assert spec.in_type is Tagger.In
    assert spec.out_type is Tagger.Out
    assert spec.state_type is None
    assert spec.skips is False
    assert spec.owner is Tagger
    assert spec.impl_name == "step"
    assert spec.arity == 1
    assert spec.is_async is False


def test_bare_in_field_rejected_at_module():
    # §2.4 (T13): a specifier-less In field is legal at bundle definition (it
    # classifies as "bare"), but a module step using it fails loudly at the
    # module class statement — the old bundle-time [needs a sampler] check,
    # relocated to stepspec as [in-field-unsampled].
    class Bad:
        class In(PmIn):
            a: float = tick()
            bare: float  # no sampler — legal on a graph rim, not on a module step

        class Out(PmOut):
            y: float = 0.0

        def step(self, i: In) -> Out:
            raise NotImplementedError

    e = _raises(Bad, Rule.IN_FIELD_UNSAMPLED)
    assert "'bare'" in str(e.value)
    assert "needs a sampler specifier" in str(e.value)


def test_bare_in_field_with_default_rejected_at_module():
    # A plain default on an In field is still bare (BareSpec captures the default);
    # a module step rejects it just the same.
    class Bad:
        class In(PmIn):
            a: float = tick()
            flag: bool = False

        class Out(PmOut):
            y: float = 0.0

        def step(self, i: In) -> Out:
            raise NotImplementedError

    _raises(Bad, Rule.IN_FIELD_UNSAMPLED)


def test_classify_stateless_multi_out():
    # QualityGate shape: the sparse Out field is T1's business; T3 sees a normal Out.
    class QualityGate:
        class In(PmIn):
            image: float = tick(expect_hz=30)

        class Out(PmOut):
            quality: float = contract(min_hz=10)
            alert: str | None = None

        def step(self, i: In) -> Out:
            raise NotImplementedError

    spec = classify(QualityGate)
    assert spec.kind is StepKind.STATELESS
    assert spec.out_type is QualityGate.Out
    assert spec.skips is False


def test_classify_mealy():
    # VoxelGridMapper shape: MEALY, nested State, tuple[State, Out | None] skips.
    class VoxelGridMapper:
        class In(PmIn):
            lidar: float = tick()

        class Out(PmOut):
            global_map: float = contract(min_hz=1)

        class State(NamedTuple):
            n: int = 0

        def step(self, state: State, i: In) -> tuple[State, Out | None]:
            raise NotImplementedError

    spec = classify(VoxelGridMapper)
    assert spec.kind is StepKind.MEALY
    assert spec.in_type is VoxelGridMapper.In
    assert spec.out_type is VoxelGridMapper.Out
    assert spec.state_type is VoxelGridMapper.State
    assert spec.skips is True
    assert spec.arity == 2


def test_classify_mealy_no_skip():
    class M:
        class In(PmIn):
            x: float = tick()

        class Out(PmOut):
            y: float = 0.0

        class State(NamedTuple):
            n: int = 0

        def step(self, state: State, i: In) -> tuple[State, Out]:
            raise NotImplementedError

    assert classify(M).skips is False


def test_classify_shape_abstract():
    # CostMapper: a raising body is the abstractness mechanism, invisible to T3.
    class CostMapper:
        class In(PmIn):
            global_map: float = tick()
            relocalized_map: float | None = latest(default=None)

        class Out(PmOut):
            global_costmap: float = contract(min_hz=2)

        def step(self, i: In) -> Out:
            raise NotImplementedError

    spec = classify(CostMapper)
    assert spec.kind is StepKind.STATELESS
    assert spec.in_type is CostMapper.In
    assert spec.owner is CostMapper


def test_classify_shape_impl_inherits():
    # HeightCostMapper pattern: override spelled Shape.In/Shape.Out, own spec.
    class Shape:
        class In(PmIn):
            a: float = tick()

        class Out(PmOut):
            b: float = 0.0

        def step(self, i: In) -> Out:
            raise NotImplementedError

    class Impl(Shape):
        def step(self, i: Shape.In) -> Shape.Out:
            raise NotImplementedError

    spec = classify(Impl)
    assert spec.owner is Impl
    assert spec.in_type is Shape.In
    assert spec.out_type is Shape.Out
    assert classify(Shape).owner is Shape


def test_classify_async_stateless():
    # Captioner shape.
    class Captioner:
        class In(PmIn):
            image: float = tick(expect_hz=2)

        class Out(PmOut):
            caption: str = contract(min_hz=1)

        async def step(self, i: In) -> Out:
            raise NotImplementedError

    spec = classify(Captioner)
    assert spec.kind is StepKind.ASYNC_STATELESS
    assert spec.is_async is True
    assert spec.arity == 1
    assert spec.skips is False


def test_classify_fold_cross_module():
    # ScanBatcher pattern: Iterator[Other.In] -> Iterator[Other.Out], no own bundles.
    a = _mk_module(
        """
        from dimos.pure.rows import In as PmIn, Out as PmOut, tick

        class Voxel:
            class In(PmIn):
                lidar: float = tick()

            class Out(PmOut):
                global_map: float = 0.0
        """
    )
    b = _mk_module(
        f"""
        from __future__ import annotations
        from typing import Iterator
        from {a.__name__} import Voxel

        class ScanBatcher:
            def fold(self, rows: Iterator[Voxel.In]) -> Iterator[Voxel.Out]:
                yield from ()
        """
    )
    spec = classify(b.ScanBatcher)
    assert spec.kind is StepKind.FOLD
    assert spec.in_type is a.Voxel.In
    assert spec.out_type is a.Voxel.Out
    assert spec.skips is False
    assert spec.impl_name == "fold"


def test_classify_fold_plain_function():
    # Non-generator fold returning an iterator expression: the annotation is the
    # contract, not isgeneratorfunction.
    class F:
        def fold(self, rows: Iterator[RowIn]) -> Iterator[RowOut]:
            return iter([])

    spec = classify(F)
    assert spec.kind is StepKind.FOLD
    assert spec.in_type is RowIn


def test_config_only_subclass_inherits_owner():
    # Explicit-string annotations resolve against the *owner's* class body (§4.1.1).
    class Base:
        class In(PmIn):
            x: float = tick()

        class Out(PmOut):
            y: float = 0.0

        def step(self, i: "In") -> "Out":
            raise NotImplementedError

    class Tuned(Base):
        pass

    spec = classify(Tuned)
    assert spec.owner is Base
    assert spec.in_type is Base.In
    assert spec == classify(Base)  # value-equal, recomputed from the inherited step


def test_classify_optional_spellings():
    class A:
        class In(PmIn):
            x: float = tick()

        class Out(PmOut):
            y: float = 0.0

        def step(self, i: In) -> Out | None:
            raise NotImplementedError

    class B:
        class In(PmIn):
            x: float = tick()

        class Out(PmOut):
            y: float = 0.0

        def step(self, i: In) -> Optional[Out]:  # noqa: UP045 — the spelling IS the test
            raise NotImplementedError

    class C:
        class In(PmIn):
            x: float = tick()

        class Out(PmOut):
            y: float = 0.0

        def step(self, i: In) -> Union[Out, None]:
            raise NotImplementedError

    specs = [classify(A), classify(B), classify(C)]
    assert all(s.skips is True for s in specs)
    assert [s.out_type for s in specs] == [A.Out, B.Out, C.Out]
    assert all(s.kind is StepKind.STATELESS for s in specs)


def test_classify_typing_spellings():
    # typing.Tuple / typing.Iterator / typing.Generator normalize to the same origins.
    class M1:
        class In(PmIn):
            x: float = tick()

        class Out(PmOut):
            y: float = 0.0

        class State(NamedTuple):
            n: int = 0

        def step(self, state: State, i: In) -> typing.Tuple[State, Out]:  # noqa: UP006
            raise NotImplementedError

    class M2:
        def fold(self, rows: typing.Iterator[RowIn]) -> typing.Iterator[RowOut]:
            yield from ()

    class M3:
        def fold(self, rows: typing.Iterable[RowIn]) -> typing.Generator[RowOut, None, None]:
            yield from ()

    s1 = classify(M1)
    assert s1.kind is StepKind.MEALY and s1.skips is False and s1.out_type is M1.Out
    s2 = classify(M2)
    assert s2.kind is StepKind.FOLD and s2.in_type is RowIn and s2.out_type is RowOut
    s3 = classify(M3)  # Iterable param accepted (contravariant); Generator return ok
    assert s3.kind is StepKind.FOLD and s3.out_type is RowOut


def test_classify_wrapped_step():
    # functools.wraps copies the annotations; the wrapper is what classifies.
    def logged(fn):
        @functools.wraps(fn)
        def wrapper(*args, **kwargs):
            return fn(*args, **kwargs)

        return wrapper

    class M:
        class In(PmIn):
            x: float = tick()

        class Out(PmOut):
            y: float = 0.0

        @logged
        def step(self, i: In) -> Out:
            raise NotImplementedError

    spec = classify(M)
    assert spec.kind is StepKind.STATELESS
    assert spec.in_type is M.In
    assert spec.out_type is M.Out


def test_classify_generator_fold_with_generator_return():
    class B:
        class In(PmIn):
            x: float = tick()

        class Out(PmOut):
            y: float = 0.0

        def fold(self, rows: Iterator[In]) -> Generator[Out, None, None]:
            yield from ()

    spec = classify(B)
    assert spec.kind is StepKind.FOLD
    assert spec.out_type is B.Out  # element extracted from Generator[Out, S, R]


# ── §12.2 resolution matrix ──────────────────────────────────────────────────


def test_resolve_bare_nested_stringized():
    mod = _mk_module(
        """
        from __future__ import annotations
        from dimos.pure.rows import In as PmIn, Out as PmOut, tick

        class M:
            class In(PmIn):
                x: float = tick()

            class Out(PmOut):
                y: float = 0.0

            def step(self, i: In) -> Out:
                raise NotImplementedError
        """
    )
    spec = classify(mod.M)
    assert spec.in_type is mod.M.In
    assert spec.out_type is mod.M.Out


def test_resolve_bare_nested_eager():
    mod = _mk_module(
        """
        from dimos.pure.rows import In as PmIn, Out as PmOut, tick

        class M:
            class In(PmIn):
                x: float = tick()

            class Out(PmOut):
                y: float = 0.0

            def step(self, i: In) -> Out:
                raise NotImplementedError
        """
    )
    spec = classify(mod.M)
    assert spec.in_type is mod.M.In
    assert spec.out_type is mod.M.Out


def test_resolve_self_qualified_in_own_body():
    # Owner-name injection (§4.1.3): the class name is not yet bound during
    # __init_subclass__, and never bound at all for function-scope classes.
    class Tagger:
        class In(PmIn):
            x: float = tick()

        class Out(PmOut):
            y: float = 0.0

        def step(self, i: "Tagger.In") -> "Tagger.Out":
            raise NotImplementedError

    spec = classify(Tagger)
    assert spec.in_type is Tagger.In
    assert spec.out_type is Tagger.Out


def test_resolve_shape_qualified_in_subclass():
    mod = _mk_module(
        """
        from __future__ import annotations
        from dimos.pure.rows import In as PmIn, Out as PmOut, tick

        class Shape:
            class In(PmIn):
                x: float = tick()

            class Out(PmOut):
                y: float = 0.0

            def step(self, i: In) -> Out:
                raise NotImplementedError

        class Impl(Shape):
            def step(self, i: Shape.In) -> Shape.Out:
                raise NotImplementedError
        """
    )
    spec = classify(mod.Impl)
    assert spec.owner is mod.Impl
    assert spec.in_type is mod.Shape.In


def test_resolve_cross_module_bundle():
    a = _mk_module(
        """
        from dimos.pure.rows import In as PmIn, Out as PmOut, tick

        class Carrier:
            class In(PmIn):
                x: float = tick()

            class Out(PmOut):
                y: float = 0.0
        """
    )
    b = _mk_module(
        f"""
        from __future__ import annotations
        from {a.__name__} import Carrier

        class M:
            def step(self, i: Carrier.In) -> Carrier.Out:
                raise NotImplementedError
        """
    )
    spec = classify(b.M)
    assert spec.in_type is a.Carrier.In
    assert spec.out_type is a.Carrier.Out


def test_resolve_explicit_string_annotations():
    # "Shape.In" literals, no future import.
    mod = _mk_module(
        """
        from dimos.pure.rows import In as PmIn, Out as PmOut, tick

        class Shape:
            class In(PmIn):
                x: float = tick()

            class Out(PmOut):
                y: float = 0.0

            def step(self, i: In) -> Out:
                raise NotImplementedError

        class Impl(Shape):
            def step(self, i: "Shape.In") -> "Shape.Out":
                raise NotImplementedError
        """
    )
    spec = classify(mod.Impl)
    assert spec.in_type is mod.Shape.In
    assert spec.out_type is mod.Shape.Out


def test_resolve_redeclared_bare_in_subclass():
    class Base:
        class In(PmIn):
            x: float = tick()

        class Out(PmOut):
            y: float = 0.0

        def step(self, i: In) -> Out:
            raise NotImplementedError

    class Sub(Base):
        class In(Base.In):
            z: float = latest(default=0.0)

        def step(self, i: In) -> Base.Out:
            raise NotImplementedError

    spec = classify(Sub)  # coherent redeclaration: step reads the redeclared bundle
    assert spec.owner is Sub
    assert spec.in_type is Sub.In
    assert spec.out_type is Base.Out


def test_resolve_class_defined_in_function():
    # The canonical nested-bundle module inside a def — the pattern every other
    # test uses — pinned explicitly (§4.3).
    class M:
        class In(PmIn):
            x: float = tick()

        class Out(PmOut):
            y: float = 0.0

        def step(self, i: In) -> Out:
            raise NotImplementedError

    assert classify(M).kind is StepKind.STATELESS


def test_resolve_bare_in_subclass_fails():
    class Base:
        class In(PmIn):
            x: float = tick()

        class Out(PmOut):
            y: float = 0.0

        def step(self, i: In) -> Out:
            raise NotImplementedError

    class Sub(Base):
        def step(self, i: "In") -> "RowOut":
            raise NotImplementedError

    with pytest.raises(PureModuleDefinitionError) as e:
        classify(Sub)
    assert e.value.rule is Rule.STEP_UNRESOLVABLE
    msg = str(e.value)
    assert "class attributes are not lexically scoped" in msg
    assert "Sub.In" in msg  # the qualified-spelling guidance
    assert isinstance(e.value.__cause__, NameError)


def test_resolve_function_local_bundle_fails():
    # PEP 563's classic gap, documented and not worked around (§4.3).
    class LocalIn(PmIn):
        x: float = tick()

    class M:
        def step(self, i: "LocalIn") -> "RowOut":
            raise NotImplementedError

    with pytest.raises(PureModuleDefinitionError) as e:
        classify(M)
    assert e.value.rule is Rule.STEP_UNRESOLVABLE
    msg = str(e.value)
    assert "function locals" in msg
    assert "module-level runtime import" in msg


def test_resolve_type_checking_import_fails():
    mod = _mk_module(
        """
        from __future__ import annotations
        from typing import TYPE_CHECKING
        from dimos.pure.rows import In as PmIn, Out as PmOut, tick

        if TYPE_CHECKING:
            from _t3_checking_only import GhostIn  # never importable at runtime

        class M:
            class Out(PmOut):
                y: float = 0.0

            def step(self, i: GhostIn) -> Out:
                raise NotImplementedError
        """
    )
    with pytest.raises(PureModuleDefinitionError) as e:
        classify(mod.M)
    assert e.value.rule is Rule.STEP_UNRESOLVABLE
    assert "TYPE_CHECKING" in str(e.value)
    assert isinstance(e.value.__cause__, NameError)


def test_resolve_attribute_typo():
    class Base:
        class In(PmIn):
            x: float = tick()

        class Out(PmOut):
            y: float = 0.0

        def step(self, i: "Base.Inn") -> "Base.Out":
            raise NotImplementedError

    with pytest.raises(PureModuleDefinitionError) as e:
        classify(Base)
    assert e.value.rule is Rule.STEP_UNRESOLVABLE
    assert isinstance(e.value.__cause__, AttributeError)
    assert "Inn" in str(e.value)


# ── §12.3 error catalog — one per slug (variants included) ───────────────────


def test_err_step_missing():
    class M:
        pass

    e = _raises(M, Rule.STEP_MISSING)
    msg = str(e.value)
    assert "defines neither step nor fold" in msg
    assert "NotImplementedError" in msg  # teaches the abstract-shape pattern
    assert msg.endswith("[step-missing]")


def test_err_step_and_fold():
    class M:
        def step(self, i: RowIn) -> RowOut:
            raise NotImplementedError

        def fold(self, rows: Iterator[RowIn]) -> Iterator[RowOut]:
            yield from ()

    e = _raises(M, Rule.STEP_AND_FOLD)
    assert "never both" in str(e.value)
    assert str(e.value).count(M.__qualname__) >= 2  # names both owners


def test_err_step_and_fold_helper():
    # A helper wrongly named fold trips G0; the message teaches the reserved names.
    class M:
        class In(PmIn):
            x: float = tick()

        class Out(PmOut):
            y: float = 0.0

        def step(self, i: In) -> Out:
            raise NotImplementedError

        def fold(self, values):
            return values

    e = _raises(M, Rule.STEP_AND_FOLD)
    assert "reserved" in str(e.value)


def test_err_step_not_function_static():
    class M:
        @staticmethod
        def step(i: RowIn) -> RowOut:
            raise NotImplementedError

    e = _raises(M, Rule.STEP_NOT_FUNCTION)
    assert "not a staticmethod" in str(e.value)
    assert "needs self for config and resources" in str(e.value)


def test_err_step_not_function_classmethod():
    class M:
        @classmethod
        def step(cls, i: RowIn) -> RowOut:
            raise NotImplementedError

    e = _raises(M, Rule.STEP_NOT_FUNCTION)
    assert "not a classmethod" in str(e.value)


def test_err_step_not_function_attr():
    class M:
        step = 3

    e = _raises(M, Rule.STEP_NOT_FUNCTION)
    assert "got int" in str(e.value)
    assert "reserved member names" in str(e.value)


def test_err_generator_step():
    # A sync generator step is permitted mechanically and fails the G5 return
    # rules: -> Iterator[Out] is a parameterized alias, not a row (spec G3 note).
    class M:
        class In(PmIn):
            x: float = tick()

        class Out(PmOut):
            y: float = 0.0

        def step(self, i: In) -> Iterator[Out]:
            yield self.Out()

    e = _raises(M, Rule.OUT_NOT_ROW)
    assert "parameterized alias" in str(e.value)


def test_err_async_generator_step():
    class M:
        class In(PmIn):
            x: float = tick()

        class Out(PmOut):
            y: float = 0.0

        async def step(self, i: In) -> Out:
            yield

    e = _raises(M, Rule.STEP_NOT_FUNCTION)
    assert "async generator" in str(e.value)
    assert "one tick" in str(e.value)


def test_err_step_no_self():
    class M:
        def step() -> None:
            raise NotImplementedError

    e = _raises(M, Rule.STEP_NO_SELF)
    assert "takes no parameters" in str(e.value)


def test_err_step_params_kwonly():
    class M:
        def step(self, *, i: RowIn) -> RowOut:
            raise NotImplementedError

    e = _raises(M, Rule.STEP_PARAMS)
    assert "'i' is keyword-only" in str(e.value)
    assert "The engine calls step positionally" in str(e.value)


def test_err_step_params_varargs():
    class M:
        def step(self, *args: RowIn) -> RowOut:
            raise NotImplementedError

    e = _raises(M, Rule.STEP_PARAMS)
    assert "'args' is *args" in str(e.value)

    class M2:
        def step(self, i: RowIn, **extra: float) -> RowOut:
            raise NotImplementedError

    e2 = _raises(M2, Rule.STEP_PARAMS)
    assert "'extra' is **kwargs" in str(e2.value)


def test_classify_posonly_step():
    # Converted from test_err_step_params_posonly: the orchestrator amended G2
    # after T4 adopted positional-only protocol params (t4-typing.md §2.3), so a
    # `/`-spelled step is LEGAL and must classify identically to the plain twin.
    class Slashed:
        class In(PmIn):
            x: float = tick()

        class Out(PmOut):
            y: float = 0.0

        def step(self, i: In, /) -> Out:
            raise NotImplementedError

    class Plain:
        class In(PmIn):
            x: float = tick()

        class Out(PmOut):
            y: float = 0.0

        def step(self, i: In) -> Out:
            raise NotImplementedError

    a, b = classify(Slashed), classify(Plain)
    assert a.kind is b.kind is StepKind.STATELESS
    assert a.in_type is Slashed.In and a.out_type is Slashed.Out
    assert (a.skips, a.arity) == (b.skips, b.arity)

    class MealySlashed:
        class In(PmIn):
            x: float = tick()

        class Out(PmOut):
            y: float = 0.0

        class State(NamedTuple):
            n: int = 0

        def step(self, state: State, i: In, /) -> tuple[State, Out]:
            raise NotImplementedError

    assert classify(MealySlashed).kind is StepKind.MEALY


def test_err_step_param_default():
    class M:
        def step(self, i: RowIn = None) -> RowOut:
            raise NotImplementedError

    e = _raises(M, Rule.STEP_PARAM_DEFAULT)
    assert "'i' has a default" in str(e.value)
    assert "across Python versions" in str(e.value)


def test_err_step_arity_zero():
    class M:
        def step(self) -> RowOut:
            raise NotImplementedError

    e = _raises(M, Rule.STEP_ARITY)
    assert "takes no input row" in str(e.value)
    assert "did you forget self" not in str(e.value)


def test_err_step_arity_zero_forgot_self():
    class M:
        def step(i: RowIn) -> RowOut:  # noqa: N805
            raise NotImplementedError

    e = _raises(M, Rule.STEP_ARITY)
    assert "(Its first parameter 'i' is a pm.In row — did you forget self?)" in str(e.value)


def test_err_step_arity_many():
    class M:
        def step(self, a: RowIn, b: RowIn, c: RowIn) -> RowOut:
            raise NotImplementedError

    e = _raises(M, Rule.STEP_ARITY)
    assert "takes 3 inputs" in str(e.value)
    assert "one In bundle" in str(e.value)


def test_err_async_mealy():
    class M:
        class In(PmIn):
            x: float = tick()

        class Out(PmOut):
            y: float = 0.0

        class State(NamedTuple):
            n: int = 0

        async def step(self, state: State, i: In) -> tuple[State, Out]:
            raise NotImplementedError

    e = _raises(M, Rule.ASYNC_MEALY)
    assert "race the state thread" in str(e.value)


def test_err_fold_arity():
    class M:
        def fold(self, rows: Iterator[RowIn], extra: int) -> Iterator[RowOut]:
            yield from ()

    e = _raises(M, Rule.FOLD_ARITY)
    assert "takes 2 data parameters" in str(e.value)
    assert "generator locals" in str(e.value)


def test_err_fold_async():
    class M:
        async def fold(self, rows: Iterator[RowIn]) -> Iterator[RowOut]:
            raise NotImplementedError

    e = _raises(M, Rule.FOLD_ASYNC)
    assert "fold owns its loop synchronously" in str(e.value)


def test_err_fold_async_generator():
    class M:
        async def fold(self, rows: Iterator[RowIn]) -> Iterator[RowOut]:
            yield

    _raises(M, Rule.FOLD_ASYNC)


def test_err_step_unannotated_param():
    class M:
        def step(self, i) -> RowOut:
            raise NotImplementedError

    e = _raises(M, Rule.STEP_UNANNOTATED)
    assert "'i' has no annotation" in str(e.value)
    assert "whole contract" in str(e.value)


def test_err_step_unannotated_return():
    class M:
        def step(self, i: RowIn):
            raise NotImplementedError

    e = _raises(M, Rule.STEP_UNANNOTATED)
    assert "no return annotation" in str(e.value)


def test_err_step_unresolvable_bare_in():
    class Base:
        class In(PmIn):
            x: float = tick()

        class Out(PmOut):
            y: float = 0.0

        def step(self, i: In) -> Out:
            raise NotImplementedError

    class Sub(Base):
        def step(self, i: "In") -> "RowOut":
            raise NotImplementedError

    e = _raises(Sub, Rule.STEP_UNRESOLVABLE)
    msg = str(e.value)
    assert "cannot resolve step's annotations" in msg
    assert "resolve at import time" in msg
    assert isinstance(e.value.__cause__, NameError)


def test_err_in_not_row():
    class M:
        def step(self, i: int) -> RowOut:
            raise NotImplementedError

    e = _raises(M, Rule.IN_NOT_ROW)
    assert "step input 'i' is int, not a pm.In row" in str(e.value)


def test_err_in_not_row_union():
    class OtherIn(PmIn):
        z: float = tick()

    class M:
        def step(self, i: RowIn | OtherIn) -> RowOut:
            raise NotImplementedError

    e = _raises(M, Rule.IN_NOT_ROW)
    assert "is a union" in str(e.value)
    assert "latest(default=None)" in str(e.value)


def test_err_in_not_row_typevar():
    T = TypeVar("T")

    class M:
        def step(self, i: T) -> RowOut:
            raise NotImplementedError

    e = _raises(M, Rule.IN_NOT_ROW)
    assert "TypeVar" in str(e.value)
    assert "not generic by design" in str(e.value)


def test_err_out_not_row():
    class M:
        def step(self, i: RowIn) -> int:
            raise NotImplementedError

    e = _raises(M, Rule.OUT_NOT_ROW)
    assert "step returns int, not a pm.Out row" in str(e.value)
    assert "effects are outputs too" in str(e.value)


def test_err_out_union():
    class Out2(PmOut):
        z: float = 0.0

    class M:
        def step(self, i: RowIn) -> RowOut | Out2:
            raise NotImplementedError

    e = _raises(M, Rule.OUT_UNION)
    assert "exactly one Out row type" in str(e.value)

    class M2:
        def step(self, i: RowIn) -> RowOut | Out2 | None:
            raise NotImplementedError

    _raises(M2, Rule.OUT_UNION)  # ≥2 non-None members: still a union error


def test_err_step_returns_nothing():
    class M:
        def step(self, i: RowIn) -> None:
            raise NotImplementedError

    e = _raises(M, Rule.STEP_RETURNS_NOTHING)
    assert "never emits has no outputs to wire" in str(e.value)


def test_err_mealy_returns_nothing():
    class M:
        class State(NamedTuple):
            n: int = 0

        def step(self, state: State, i: RowIn) -> tuple[State, None]:
            raise NotImplementedError

    _raises(M, Rule.STEP_RETURNS_NOTHING)


def test_err_step_returns_awaitable_sync():
    # mypy structurally accepts this against AsyncStateless; runtime rejects (R3).
    class M:
        def step(self, i: RowIn) -> Awaitable[RowOut]:
            raise NotImplementedError

    e = _raises(M, Rule.STEP_RETURNS_AWAITABLE)
    assert "but is not async" in str(e.value)
    assert "One spelling per capability" in str(e.value)


def test_err_step_returns_awaitable_async():
    class M:
        async def step(self, i: RowIn) -> Awaitable[RowOut]:
            raise NotImplementedError

    e = _raises(M, Rule.STEP_RETURNS_AWAITABLE)
    assert "awaited result" in str(e.value)


def test_err_mealy_no_state():
    class Counter:
        pass

    class M:
        def step(self, state: Counter, i: RowIn) -> tuple[Counter, RowOut]:
            raise NotImplementedError

    e = _raises(M, Rule.MEALY_NO_STATE)
    assert "declares no State" in str(e.value)
    assert "a run starts from State()" in str(e.value)


def test_err_state_mismatch():
    class OtherState(NamedTuple):
        n: int = 0

    class M:
        class State(NamedTuple):
            n: int = 0

        def step(self, state: OtherState, i: RowIn) -> tuple[OtherState, RowOut]:
            raise NotImplementedError

    e = _raises(M, Rule.STATE_MISMATCH)
    assert "must be the class's own State" in str(e.value)


def test_err_state_mismatch_swapped():
    class M:
        class State(NamedTuple):
            n: int = 0

        def step(self, i: RowIn, state: State) -> tuple[State, RowOut]:
            raise NotImplementedError

    e = _raises(M, Rule.STATE_MISMATCH)
    assert "swapped" in str(e.value)
    assert "def step(self, state: State, i: In)" in str(e.value)


def test_err_state_mismatch_shadowed():
    class Base:
        class State(NamedTuple):
            n: int = 0

        def step(self, state: State, i: RowIn) -> tuple[State, RowOut]:
            raise NotImplementedError

    class Sub(Base):
        class State(NamedTuple):
            m: int = 0

    e = _raises(Sub, Rule.STATE_MISMATCH)
    msg = str(e.value)
    assert "inherits step from" in msg
    assert "Override step or remove the State declaration" in msg
    assert classify(Base).state_type is Base.State  # the base itself is fine


def test_err_state_is_row():
    class M:
        class State(PmIn):
            s: float = tick()

        def step(self, state: State, i: RowIn) -> tuple[State, RowOut]:
            raise NotImplementedError

    e = _raises(M, Rule.STATE_IS_ROW)
    assert "row bundle" in str(e.value)
    assert "subclass of pm.In" in str(e.value)


def test_err_state_not_default_constructible():
    class M:
        class State(NamedTuple):
            n: int  # no default

        def step(self, state: State, i: RowIn) -> tuple[State, RowOut]:
            raise NotImplementedError

    e = _raises(M, Rule.STATE_NOT_DEFAULT_CONSTRUCTIBLE)
    assert "field 'n' has no default" in str(e.value)


def test_err_mealy_return_shape():
    class M:
        class State(NamedTuple):
            n: int = 0

        def step(self, state: State, i: RowIn) -> RowOut:
            raise NotImplementedError

    e = _raises(M, Rule.MEALY_RETURN)
    assert "a Mealy step returns tuple[State, Out]" in str(e.value)

    class M2:
        class State(NamedTuple):
            n: int = 0

        def step(self, state: State, i: RowIn) -> tuple[State, RowOut, int]:
            raise NotImplementedError

    _raises(M2, Rule.MEALY_RETURN)

    class M3:
        class State(NamedTuple):
            n: int = 0

        def step(self, state: State, i: RowIn) -> tuple[State, ...]:
            raise NotImplementedError

    _raises(M3, Rule.MEALY_RETURN)


def test_err_mealy_return_first_slot():
    class M:
        class State(NamedTuple):
            n: int = 0

        def step(self, state: State, i: RowIn) -> tuple[RowOut, State]:
            raise NotImplementedError

    e = _raises(M, Rule.MEALY_RETURN)
    assert "first, but the state slot must be" in str(e.value)


def test_err_fold_rows_param():
    class M:
        def fold(self, rows: list[RowIn]) -> Iterator[RowOut]:
            yield from ()

    e = _raises(M, Rule.FOLD_ROWS_PARAM)
    assert "expected Iterator[In]" in str(e.value)

    class M2:  # Generator param is narrower than the Iterator the driver passes
        def fold(self, rows: Generator[RowIn, None, None]) -> Iterator[RowOut]:
            yield from ()

    _raises(M2, Rule.FOLD_ROWS_PARAM)

    class M3:  # unparameterized
        def fold(self, rows: Iterator) -> Iterator[RowOut]:
            yield from ()

    _raises(M3, Rule.FOLD_ROWS_PARAM)


def test_err_fold_return():
    class M:
        def fold(self, rows: Iterator[RowIn]) -> list[RowOut]:
            raise NotImplementedError

    e = _raises(M, Rule.FOLD_RETURN)
    assert "must return Iterator[Out]" in str(e.value)


def test_err_fold_return_iterable():
    class M:
        def fold(self, rows: Iterator[RowIn]) -> Iterable[RowOut]:
            yield from ()

    e = _raises(M, Rule.FOLD_RETURN)
    assert "does not promise an Iterator" in str(e.value)
    assert "Declare -> Iterator[RowOut]" in str(e.value)


def test_err_state_unused():
    class M:
        class State(NamedTuple):
            n: int = 0

        def step(self, i: RowIn) -> RowOut:
            raise NotImplementedError

    e = _raises(M, Rule.STATE_UNUSED)
    assert "step is stateless" in str(e.value)
    assert "Take it or delete it" in str(e.value)

    class M2:
        class State(NamedTuple):
            n: int = 0

        async def step(self, i: RowIn) -> RowOut:
            raise NotImplementedError

    e2 = _raises(M2, Rule.STATE_UNUSED)
    assert "async stateless" in str(e2.value)


def test_err_fold_state():
    class M:
        class State(NamedTuple):
            n: int = 0

        def fold(self, rows: Iterator[RowIn]) -> Iterator[RowOut]:
            yield from ()

    e = _raises(M, Rule.FOLD_STATE)
    assert "generator locals" in str(e.value)
    assert "respell as a Mealy step" in str(e.value)


def test_err_bundle_shadowed():
    class M:
        class In(PmIn):  # dead: step reads RowIn
            x: float = tick()

        def step(self, i: RowIn) -> RowOut:
            raise NotImplementedError

    e = _raises(M, Rule.BUNDLE_SHADOWED)
    msg = str(e.value)
    assert "declares a nested In" in msg
    assert "single typing authority" in msg
    assert "dead" in msg


def test_err_bundle_shadowed_inherited():
    class Base:
        class In(PmIn):
            x: float = tick()

        class Out(PmOut):
            y: float = 0.0

        def step(self, i: In) -> Out:
            raise NotImplementedError

    class Sub(Base):
        class In(Base.In):
            z: float = latest(default=0.0)

    e = _raises(Sub, Rule.BUNDLE_SHADOWED)
    msg = str(e.value)
    assert "inherits step from" in msg
    assert "Override step or remove the declaration" in msg


# ── §12.4 seam and accessor ──────────────────────────────────────────────────


def test_seam_recipe():
    # The §8 contract demonstrated without T2: a FakePureModule whose
    # __init_subclass__ classifies; an invalid subclass fails AT the class
    # statement — the import-fails-loudly moment.
    class FakePureModule:
        def __init_subclass__(cls, **kwargs):
            super().__init_subclass__(**kwargs)
            cls.__pure_step__ = classify(cls)

    class Good(FakePureModule):
        class In(PmIn):
            x: float = tick()

        class Out(PmOut):
            y: float = 0.0

        def step(self, i: In) -> Out:
            raise NotImplementedError

    assert PURE_STEP_ATTR in vars(Good)
    assert vars(Good)[PURE_STEP_ATTR].kind is StepKind.STATELESS

    with pytest.raises(PureModuleDefinitionError) as e:

        class Bad(FakePureModule):
            pass

    assert e.value.rule is Rule.STEP_MISSING


def test_step_spec_accessor():
    class M:
        class In(PmIn):
            x: float = tick()

        class Out(PmOut):
            y: float = 0.0

        def step(self, i: In) -> Out:
            raise NotImplementedError

    M.__pure_step__ = classify(M)  # what T2 does (§7.2)
    spec = step_spec(M)
    assert isinstance(spec, StepSpec)
    assert spec is vars(M)[PURE_STEP_ATTR]


def test_step_spec_accessor_unclassified():
    class M:
        pass

    with pytest.raises(PureModuleDefinitionError) as e:
        step_spec(M)
    assert e.value.rule is Rule.NOT_CLASSIFIED
    assert "[not-classified]" in str(e.value)

    # An inherited-only attribute does not satisfy the own-dict rule (§7.3).
    class Base:
        class In(PmIn):
            x: float = tick()

        class Out(PmOut):
            y: float = 0.0

        def step(self, i: In) -> Out:
            raise NotImplementedError

    Base.__pure_step__ = classify(Base)

    class Child(Base):
        pass

    assert Child.__pure_step__ is Base.__pure_step__  # normal attribute inheritance
    with pytest.raises(PureModuleDefinitionError) as e2:
        step_spec(Child)
    assert e2.value.rule is Rule.NOT_CLASSIFIED


# ── §12.5 purity guarantees ──────────────────────────────────────────────────


def test_classify_never_instantiates():
    calls = []

    class Boom:  # resource-like descriptor: must never be executed
        def __get__(self, obj, owner=None):
            calls.append("descriptor")
            raise AssertionError("descriptor executed")

    class M:
        class In(PmIn):
            x: float = tick()

        class Out(PmOut):
            y: float = 0.0

        class State:
            def __init__(self):
                calls.append("state")
                raise AssertionError("State constructed")

        grid = Boom()

        def step(self, state: State, i: In) -> tuple[State, Out]:
            raise NotImplementedError

    for bundle in (M.In, M.Out):  # record any bundle construction
        orig = bundle.__init__

        def spy(self, *args, _orig=orig, **kwargs):
            calls.append("bundle")
            return _orig(self, *args, **kwargs)

        bundle.__init__ = spy

    spec = classify(M)
    assert spec.kind is StepKind.MEALY
    assert spec.state_type is M.State
    assert calls == []  # nothing constructed, no descriptor touched


def test_classify_no_mutation():
    class M:
        class In(PmIn):
            x: float = tick()

        class Out(PmOut):
            y: float = 0.0

        def step(self, i: In) -> Out:
            raise NotImplementedError

    before = dict(vars(M))
    classify(M)
    after = dict(vars(M))
    assert before.keys() == after.keys()
    assert all(before[k] is after[k] for k in before)
