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

"""Unit tests for dimos.pure.rows.

Engine-free by construction: imports are rows.py + stdlib + pytest only.
Bundles are defined inside test bodies so this file collects cleanly against
the T1 skeleton; the implementer deletes the module-level skip to activate.
Three cases need module-level fixtures and are added with the implementation:
nested-bundle pickle round-trip, cross-module bundle inheritance, and the
positive qualified-path resolution case (spec §9.1).
"""

from __future__ import annotations

import dataclasses
import math
from typing import ClassVar, Generic, TypeVar

import pytest

from dimos.pure.rows import (
    UNSTAMPED,
    BundleDefinitionError,
    ContractSpec,
    FieldSpec,
    In,
    InterpolateSpec,
    LatestSpec,
    Out,
    PlainSpec,
    TickSpec,
    _Bundle,
    contract,
    interpolate,
    latest,
    tick,
)


class Image:
    """Fake payload."""


class PoseStamped:
    """Fake payload."""


class PointCloud2:
    """Fake payload."""


def _tag_in():
    class TagIn(In):
        image: Image = tick(expect_hz=30)
        pose: PoseStamped = interpolate()

    return TagIn


def _cost_in():
    class CostIn(In):
        global_map: PointCloud2 = tick()
        relocalized_map: PointCloud2 | None = latest(default=None)

    return CostIn


def _quality_out():
    class QualityOut(Out):
        quality: float = contract(min_hz=10)
        alert: str | None = None  # sparse

    return QualityOut


# ── construction ─────────────────────────────────────────────────────────────


def test_construction_required_and_defaulted():
    TagIn = _tag_in()
    img, pose = Image(), PoseStamped()
    row = TagIn(ts=0.5, image=img, pose=pose)
    assert row.ts == 0.5 and row.image is img and row.pose is pose

    CostIn = _cost_in()
    row2 = CostIn(ts=0.0, global_map=PointCloud2())
    assert row2.relocalized_map is None  # latest(default=None) omissible

    QualityOut = _quality_out()
    row3 = QualityOut(quality=0.9)
    assert row3.alert is None  # plain sparse omissible


def test_constructor_type_errors():
    TagIn = _tag_in()
    with pytest.raises(TypeError):
        TagIn(ts=0.0, pose=PoseStamped())  # missing required specifier field
    with pytest.raises(TypeError):
        TagIn(ts=0.0, image=Image(), pose=PoseStamped(), zz=1)  # unknown kwarg
    with pytest.raises(TypeError):
        TagIn(image=Image(), pose=PoseStamped())  # missing ts


def test_ts_kw_only():
    TagIn = _tag_in()
    with pytest.raises(TypeError):
        TagIn(0.0, Image(), PoseStamped())  # positional construction rejected

    class Single(Out):
        located: str = contract(min_hz=10)

    with pytest.raises(TypeError):
        Single("x")
    assert Single(located="x").located == "x"  # defaulted ts before required field: fine


def test_out_ts_unstamped_and_explicit():
    class VoxelOut(Out):
        global_map: PointCloud2 = contract(min_hz=1)

    o = VoxelOut(global_map=PointCloud2())
    assert o.ts == UNSTAMPED and math.isinf(o.ts) and o.ts < 0
    assert VoxelOut(ts=3.0, global_map=PointCloud2()).ts == 3.0  # fold self-stamp


def test_frozen():
    TagIn = _tag_in()
    row = TagIn(ts=0.0, image=Image(), pose=PoseStamped())
    with pytest.raises(dataclasses.FrozenInstanceError):
        row.ts = 1.0
    with pytest.raises(dataclasses.FrozenInstanceError):
        row.image = Image()


def test_eq_and_replace_stamping():
    QualityOut = _quality_out()
    assert QualityOut(quality=1.0) == QualityOut(quality=1.0)  # unstamped rows equal
    assert QualityOut(quality=1.0) != QualityOut(quality=2.0)
    assert hash(QualityOut(quality=1.0)) == hash(QualityOut(quality=1.0))

    o = QualityOut(quality=1.0)
    stamped = dataclasses.replace(o, ts=7.0)  # the T6 stamping path
    assert stamped.ts == 7.0 and stamped.quality == 1.0
    assert o.ts == UNSTAMPED  # original untouched


# ── introspection ────────────────────────────────────────────────────────────


def test_fields_introspection():
    TagIn = _tag_in()
    f = TagIn.fields()
    assert list(f) == ["image", "pose"]  # declaration order, ts excluded
    assert isinstance(f["image"], TickSpec)
    assert f["image"].expect_hz == 30
    assert f["image"].required and f["image"].name == "image"
    assert f["image"].annotation is Image
    assert isinstance(f["pose"], InterpolateSpec) and f["pose"].required

    CostIn = _cost_in()
    cf = CostIn.fields()
    assert isinstance(cf["relocalized_map"], LatestSpec)
    assert not cf["relocalized_map"].required
    assert cf["relocalized_map"].default is None
    assert cf["relocalized_map"].annotation == (PointCloud2 | None)

    QualityOut = _quality_out()
    qf = QualityOut.fields()
    assert isinstance(qf["quality"], ContractSpec) and qf["quality"].min_hz == 10
    assert qf["quality"].required
    assert isinstance(qf["alert"], PlainSpec) and qf["alert"].default is None
    assert "ts" not in qf


def test_fields_returns_fresh_copy():
    TagIn = _tag_in()
    TagIn.fields().clear()
    assert list(TagIn.fields()) == ["image", "pose"]


def test_fields_lazy_resolution_not_cached_on_failure():
    class Late(In):
        x: LateMsg = tick()

    with pytest.raises(BundleDefinitionError) as ei:
        Late.fields()
    assert "qualified path" in str(ei.value)  # E-UNRESOLVED hint
    globals()["LateMsg"] = Image  # name appears later — failure was not cached
    try:
        assert Late.fields()["x"].annotation is Image
    finally:
        del globals()["LateMsg"]


def test_roots_and_empty_bundle():
    class EmptyIn(In):
        pass

    assert EmptyIn.fields() == {}
    assert EmptyIn(ts=0.0).ts == 0.0
    assert In.fields() == {} and Out.fields() == {}
    assert In(ts=1.0).ts == 1.0 and Out().ts == UNSTAMPED


# ── the never-survive rule ───────────────────────────────────────────────────


def test_no_specifier_leakage():
    TagIn = _tag_in()
    CostIn = _cost_in()
    row = TagIn(ts=0.0, image=Image(), pose=PoseStamped())
    assert set(vars(row)) == {"ts", "image", "pose"}  # exactly the fields
    assert not any(isinstance(v, FieldSpec) for v in vars(row).values())
    assert not hasattr(TagIn, "image")  # required field: no class attr at all
    assert CostIn.relocalized_map is None  # defaulted field: the plain default
    assert not any(isinstance(v, FieldSpec) for v in vars(TagIn).values())


def test_asdict_and_dataclass_fields():
    QualityOut = _quality_out()
    o = QualityOut(quality=1.0)
    assert dataclasses.asdict(o) == {"ts": UNSTAMPED, "quality": 1.0, "alert": None}
    TagIn = _tag_in()
    assert [f.name for f in dataclasses.fields(TagIn)] == ["ts", "image", "pose"]


# ── nesting, naming, inheritance ─────────────────────────────────────────────


def test_nested_class_body_equivalence():
    class Free(In):
        image: Image = tick(expect_hz=30)
        pose: PoseStamped = interpolate()

    class Holder:
        class Nested(In):
            image: Image = tick(expect_hz=30)
            pose: PoseStamped = interpolate()

    assert list(Holder.Nested.fields()) == list(Free.fields())
    for name in ("image", "pose"):
        a, b = Holder.Nested.fields()[name], Free.fields()[name]
        assert type(a) is type(b) and a.required == b.required
    n = Holder.Nested(ts=1.0, image=Image(), pose=PoseStamped())
    assert n.ts == 1.0 and "Nested" in repr(n)


def test_in_out_share_field_names():
    class I1(In):
        image: Image = tick()

    class O1(Out):
        image: Image = contract(min_hz=1)

    assert isinstance(I1.fields()["image"], TickSpec)
    assert isinstance(O1.fields()["image"], ContractSpec)


def test_bundle_inheritance_extends():
    CostIn = _cost_in()

    class Ext(CostIn):
        extra: float = latest()

    assert list(Ext.fields()) == ["global_map", "relocalized_map", "extra"]
    with pytest.raises(TypeError):
        Ext(ts=0.0, global_map=PointCloud2())  # new required field enforced
    row = Ext(ts=0.0, global_map=PointCloud2(), extra=1.0)
    assert row.extra == 1.0


def test_redeclare_required_override():
    CostIn = _cost_in()

    class Strict(CostIn):
        relocalized_map: PointCloud2 = latest()  # defaulted -> required

    assert Strict.fields()["relocalized_map"].required
    assert list(Strict.fields()) == ["global_map", "relocalized_map"]  # position kept
    with pytest.raises(TypeError):
        Strict(ts=0.0, global_map=PointCloud2())


def test_helpers_allowed():
    class WithHelpers(In):
        SCALE: ClassVar[float] = 2.0
        lidar: PointCloud2 = tick()

        def doubled(self) -> float:
            return self.SCALE * 2

    assert list(WithHelpers.fields()) == ["lidar"]
    assert WithHelpers(ts=0.0, lidar=PointCloud2()).doubled() == 4.0


# ── definition-time errors (spec §7) ─────────────────────────────────────────


def _raises(substr, build):
    with pytest.raises(BundleDefinitionError) as ei:
        build()
    assert substr in str(ei.value), str(ei.value)
    return str(ei.value)


def test_error_specifier_without_annotation():
    def build():
        class B(In):
            x = tick()

    msg = _raises("no field annotation", build)
    assert "'x'" in msg


def test_bare_in_field_defines_as_bare():
    # §2.4 T1 amendment (T13): a specifier-less In field is legal at bundle
    # definition (graph rims spell `class In(pm.In)` with bare fields); it
    # classifies as "bare", with or without a plain default. Module-side
    # loudness relocates to stepspec's [in-field-unsampled] (test_stepspec.py).
    class WithDefault(In):
        lidar: PointCloud2 = tick()
        flag: bool = False

    class WithoutDefault(In):
        lidar: PointCloud2 = tick()
        bare: float

    assert WithDefault.fields()["flag"].kind == "bare"
    assert WithDefault.fields()["flag"].default is False
    assert WithoutDefault.fields()["bare"].kind == "bare"
    assert WithoutDefault.fields()["bare"].required


def test_error_wrong_side_specifier():
    def contract_on_in():
        class B(In):
            x: float = contract(min_hz=1)

    def tick_on_out():
        class B(Out):
            x: float = tick()

    _raises("contract() is not valid on an In bundle field", contract_on_in)
    _raises("tick() is not valid on an Out bundle field", tick_on_out)


def test_error_reserved_names():
    def redeclare_ts():
        class B(In):
            ts: float = tick()

    def shadow_fields():
        class B(Out):
            fields: int = 3

    assert "'ts'" in _raises("reserved", redeclare_ts)
    assert "'fields'" in _raises("reserved", shadow_fields)


def test_error_multiple_ticks():
    def direct():
        class B(In):
            a: Image = tick()
            b: Image = tick()

    def inherited():
        CostIn = _cost_in()

        class B(CostIn):
            second: Image = tick()

    _raises("multiple tick() fields", direct)
    _raises("multiple tick() fields", inherited)


def test_error_in_out_mix_and_bare_bundle():
    def mix():
        class B(In, Out):
            pass

    def bare():
        class B(_Bundle):
            pass

    _raises("both In and Out", mix)
    _raises("must subclass pm.In or pm.Out", bare)


def test_error_foreign_dataclass_base():
    @dataclasses.dataclass
    class D:
        x: int = 0

    def build():
        class B(In, D):
            lidar: PointCloud2 = tick()

    _raises("dataclass but not a bundle", build)


def test_error_generic_bundle():
    T = TypeVar("T")

    def build():
        class B(In, Generic[T]):
            lidar: PointCloud2 = tick()

    _raises("generic bundles are not supported", build)


def test_error_property_raw_field_initvar():
    def prop():
        class B(Out):
            x: float = property(lambda self: 1.0)  # type: ignore[assignment]

    def raw():
        class B(Out):
            x: float = dataclasses.field(default=1.0)

    def initvar():
        class B(Out):
            x: dataclasses.InitVar[int] = 3

    _raises("property", prop)
    _raises("dataclasses.field()", raw)
    _raises("InitVar", initvar)


def test_error_classvar_specifier():
    def build():
        class B(Out):
            x: ClassVar[int] = contract(min_hz=1)  # type: ignore[assignment]

    _raises("not fields", build)


def test_error_messages_name_the_bundle():
    def build():
        class Malformed(In):
            x = tick()

    msg = _raises("no field annotation", build)
    assert "Malformed" in msg and __name__ in msg


def test_specifier_arg_validation():
    with pytest.raises(ValueError):
        tick(expect_hz=0)
    with pytest.raises(ValueError):
        tick(expect_hz=-5)
    with pytest.raises(ValueError):
        contract(min_hz=-1)


def test_mutable_default_rejected():
    with pytest.raises(ValueError):

        class B(Out):
            tags: list[str] = []


# ── the sketch floor ─────────────────────────────────────────────────────────


def test_sketch_floor_tagger():
    """Sketch3 §1 bundles + hand-construction, spelled like the sketch.

    Inside the class body the bases line still sees the outer In/Out (the
    nested names bind only after each class statement completes), so the
    sketch's `class In(pm.In)` spelling works with the bare imports too.
    """

    class Tagger:
        class In(In):
            image: Image = tick(expect_hz=30)
            pose: PoseStamped = interpolate()

        class Out(Out):
            located: str = contract(min_hz=10)

    i = Tagger.In(ts=0.0, image=Image(), pose=PoseStamped())
    o = Tagger.Out(located="bright=0.80 @ (1.00, 2.00)")
    assert i.ts == 0.0 and o.ts == UNSTAMPED
    assert o.located == "bright=0.80 @ (1.00, 2.00)"
    assert list(Tagger.In.fields()) == ["image", "pose"]


# ── implementation-time additions (need module-level fixture bundles) ────────
# These three cases were impossible under the skeleton (module-level specifier
# calls raised); they are added now that rows.py is real (spec §9.1 close).


class PickleHolder:
    class Row(In):
        value: int = latest()
        label: str = latest(default="")


def test_nested_pickle_roundtrip():
    """A bundle nested in a module-level class round-trips through pickle."""
    import pickle

    row = PickleHolder.Row(ts=2.0, value=7, label="hi")
    back = pickle.loads(pickle.dumps(row))
    assert type(back) is PickleHolder.Row
    assert back == row
    assert back.ts == 2.0 and back.value == 7 and back.label == "hi"


class BMsg:
    """Payload visible only in this test module's globals."""


def test_cross_module_inheritance():
    """Each MRO layer resolves annotations against its own module's globals."""
    import sys
    import textwrap
    import types

    mod = types.ModuleType("dimos.pure._t1_fixture_mod_a")
    src = textwrap.dedent(
        """
        from __future__ import annotations

        from dimos.pure.rows import In, tick


        class AMsg:
            pass


        class ABundle(In):
            a: AMsg = tick()
        """
    )
    sys.modules[mod.__name__] = mod
    try:
        exec(compile(src, mod.__name__, "exec"), mod.__dict__)
        ABundle = mod.ABundle

        class BBundle(ABundle):  # a resolves in mod_a; b resolves in this module
            b: BMsg = latest()

        f = BBundle.fields()
        assert list(f) == ["a", "b"]
        assert f["a"].annotation is mod.AMsg  # would NameError if resolved here
        assert f["b"].annotation is BMsg
        assert isinstance(f["a"], TickSpec) and isinstance(f["b"], LatestSpec)
    finally:
        del sys.modules[mod.__name__]


class QualOuter:
    class Sib:  # a payload, referenced by qualified path from the nested bundle
        pass

    class Row(In):
        x: QualOuter.Sib = tick()


def test_qualified_path_resolution():
    """A field typed by qualified path (Outer.Sib) resolves in fields()."""
    f = QualOuter.Row.fields()
    assert isinstance(f["x"], TickSpec)
    assert f["x"].annotation is QualOuter.Sib
