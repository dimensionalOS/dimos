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

"""T2 config machinery tests — executable spec for dimos/pure/tasks/t2-config.md.

Engine-free: construct modules, poke config, assert. Example classes are
defined inside test bodies so importing this file never triggers
__init_subclass__. Static-typing regression cases live in the spec (§13.2)
and land with T4's mypy harness.
"""

from typing import ClassVar

from pydantic import ValidationError
import pytest

from dimos.protocol.service.spec import BaseConfig
from dimos.pure.config import (
    ConfigFieldError,
    FrozenModuleError,
    PureModuleConfig,
)
from dimos.pure.module import PureModule
from dimos.pure.rows import In, Out, tick
from dimos.pure.stepspec import PureModuleDefinitionError, StepKind, step_spec

# T3 seam ACTIVE: every PureModule subclass must declare a valid step at its
# class statement. These shared minimal rows + a per-class `def step` keep the
# T2 tests focused on config; step bodies are never read.


class _In(In):
    x: int = tick()


class _Out(Out):
    y: int = 0


def _go2():
    """The sketch §5b/§5c example, minus bundles (T1); minimal step for T3."""

    class Go2Connection(PureModule):
        prefix: str = ""
        robot_ip: str | None = None
        odom_timeout: float = 0.5

        def step(self, i: _In) -> _Out:
            raise NotImplementedError

    return Go2Connection


# ── construction ────────────────────────────────────────────────────────────


def test_typed_construction_happy_path():
    Go2Connection = _go2()
    c = Go2Connection(prefix="go2a", robot_ip="192.168.12.1")
    assert c.prefix == "go2a"
    assert c.robot_ip == "192.168.12.1"
    assert c.odom_timeout == 0.5  # default


def test_flat_access_matches_config():
    c = _go2()(prefix="go2a")
    assert c.config.prefix == c.prefix
    assert c.config.odom_timeout == c.odom_timeout
    assert isinstance(c.config, PureModuleConfig)


def test_canonical_model_dump():
    # Sketch §5c verbatim: THE canonical serialization, declaration-ordered.
    c = _go2()(prefix="go2a", robot_ip="192.168.12.1")
    assert c.config.model_dump() == {
        "prefix": "go2a",
        "robot_ip": "192.168.12.1",
        "odom_timeout": 0.5,
    }


def test_unknown_kwarg_rejected():
    Go2Connection = _go2()
    with pytest.raises(ValidationError) as e:
        Go2Connection(prefx="go2a")
    assert e.value.errors()[0]["type"] == "extra_forbidden"
    assert "Go2ConnectionConfig" in str(e.value)  # error names the module's model


def test_wrong_type_rejected():
    with pytest.raises(ValidationError) as e:
        _go2()(robot_ip=0.5)
    assert e.value.errors()[0]["loc"] == ("robot_ip",)


def test_missing_required_rejected():
    class NeedsIp(PureModule):
        robot_ip: str  # no default -> required

        def step(self, i: _In) -> _Out:
            raise NotImplementedError

    with pytest.raises(ValidationError) as e:
        NeedsIp()
    assert e.value.errors()[0]["type"] == "missing"
    assert NeedsIp(robot_ip="10.0.0.1").robot_ip == "10.0.0.1"


def test_lax_coercion_int_to_float():
    c = _go2()(odom_timeout=1)
    assert c.odom_timeout == 1.0 and isinstance(c.odom_timeout, float)


def test_positional_args_rejected():
    with pytest.raises(TypeError):
        _go2()("go2a")


def test_puremodule_direct_instantiation_rejected():
    with pytest.raises(TypeError):
        PureModule()


# ── what is NOT a field ─────────────────────────────────────────────────────


def test_machinery_members_are_not_fields():
    class Grid: ...

    class Mapper(PureModule):
        voxel_size: float = 0.05
        LIMIT: ClassVar[int] = 3

        class Payload:  # stand-in for a nested bundle class
            pass

        def helper(self) -> int:
            return 1

        def step(self, i: _In) -> _Out:
            raise NotImplementedError

    fields = Mapper.__pure_config_model__.model_fields
    assert set(fields) == {"voxel_size"}
    for bad in ("Payload", "helper", "LIMIT"):
        with pytest.raises(ValidationError):
            Mapper(**{bad: 1})
    assert Mapper.LIMIT == 3  # ClassVar stays a plain class attribute


def test_dunder_pure_step_not_a_field():
    # T3 seam pin: __pure_step__ (set by classify) must never be config.
    Cls = _go2()
    assert "__pure_step__" not in Cls.__pure_config_model__.model_fields
    with pytest.raises(ValidationError):
        Cls(**{"__pure_step__": None})


def test_engine_surface_accessors_not_fields():
    # T4 seam pin: EngineSurface's unannotated i/o accessors never become
    # config, and the names stay reserved against annotated use.
    Cls = _go2()
    fields = Cls.__pure_config_model__.model_fields
    assert "i" not in fields and "o" not in fields and "over" not in fields
    with pytest.raises(ValidationError):
        Cls(i=1)
    with pytest.raises(ConfigFieldError):

        class Bad(PureModule):
            i: int = 1  # annotating the port accessor name is a hard error


# ── frozen at both surfaces ─────────────────────────────────────────────────


def test_frozen_module_surface():
    c = _go2()(prefix="go2a")
    with pytest.raises(FrozenModuleError):
        c.odom_timeout = 1.0
    with pytest.raises(AttributeError):  # FrozenModuleError IS an AttributeError
        c.odom_timeout = 1.0
    with pytest.raises(FrozenModuleError):
        c.some_new_attr = 1  # non-fields frozen too: no instance state, period
    with pytest.raises(FrozenModuleError):
        del c.odom_timeout
    assert c.odom_timeout == 0.5


def test_frozen_config_surface():
    c = _go2()(prefix="go2a")
    with pytest.raises(ValidationError) as e:
        c.config.odom_timeout = 1.0
    assert e.value.errors()[0]["type"] == "frozen_instance"


def test_rebuild_never_mutate_idiom():
    Go2Connection = _go2()
    c = Go2Connection(prefix="go2a", robot_ip="192.168.12.1")
    # The sketch §5c sweep idiom: rebuild with one field overridden.
    c2 = Go2Connection(**{**c.config.model_dump(), "odom_timeout": 1.0})
    assert c2.odom_timeout == 1.0
    assert c2.prefix == c.prefix and c2.robot_ip == c.robot_ip
    assert c.odom_timeout == 0.5  # original untouched


# ── mutable defaults ────────────────────────────────────────────────────────


def test_mutable_default_not_shared():
    class Stages(PureModule):
        layers: list[str] = []

        def step(self, i: _In) -> _Out:
            raise NotImplementedError

    a, b = Stages(), Stages()
    a.layers.append("x")
    assert b.layers == []  # per-instance copy (default_factory semantics)
    assert a.config.model_dump()["layers"] == ["x"]


# ── determinism ─────────────────────────────────────────────────────────────


def test_model_dump_declaration_order_with_inheritance():
    class Base(PureModule):
        alpha: int = 1
        beta: int = 2

        def step(self, i: _In) -> _Out:
            raise NotImplementedError

    class Child(Base):  # config-only subclass: inherits Base's step
        gamma: int = 3
        alpha: int = 99  # override: new default, ORIGINAL position

    assert list(Child().config.model_dump()) == ["alpha", "beta", "gamma"]
    assert Child().config.model_dump() == {"alpha": 99, "beta": 2, "gamma": 3}


def test_shape_subclass_inherits_and_extends_fields():
    class CostMapper(PureModule):
        resolution: float = 0.1

        def step(self, i: _In) -> _Out:
            raise NotImplementedError

    class HeightCostMapper(CostMapper):  # shape impl: inherits the step contract
        max_height: float = 0.35

    m = HeightCostMapper(resolution=0.2)
    assert m.resolution == 0.2 and m.max_height == 0.35
    assert list(m.config.model_dump()) == ["resolution", "max_height"]


def test_voxelgridmapper_style_construction():
    # T2 §14: the sketch _unit_tests construction, satisfiable now that T1+T3
    # landed — config fields collected; nested In/Out/State and step are not
    # fields. (@resource is T7 and not part of this construction.)
    from typing import NamedTuple

    class VoxelGridMapper(PureModule):
        voxel_size: float = 0.05
        emit_every: int = 5

        class In(In):
            lidar: float = tick()

        class Out(Out):
            global_map: float = 0.0

        class State(NamedTuple):
            n: int = 0

        def step(self, state: State, i: In) -> tuple[State, Out | None]:
            raise NotImplementedError

    m = VoxelGridMapper(emit_every=2)
    assert m.voxel_size == 0.05 and m.emit_every == 2
    assert set(VoxelGridMapper.__pure_config_model__.model_fields) == {"voxel_size", "emit_every"}
    spec = step_spec(VoxelGridMapper)
    assert spec.kind is StepKind.MEALY
    assert spec.state_type is VoxelGridMapper.State
    assert spec.skips is True


# ── identity ────────────────────────────────────────────────────────────────


def test_config_round_trip_eq_hash():
    Go2Connection = _go2()
    c = Go2Connection(prefix="go2a", robot_ip="192.168.12.1")
    c2 = Go2Connection(**c.config.model_dump())
    assert c2 == c and hash(c2) == hash(c)
    assert c2.config == c.config
    assert Go2Connection(prefix="other") != c


def test_eq_is_class_scoped():
    class A(PureModule):
        x: int = 1

        def step(self, i: _In) -> _Out:
            raise NotImplementedError

    class B(PureModule):
        x: int = 1

        def step(self, i: _In) -> _Out:
            raise NotImplementedError

    assert A() != B()  # identity = class + config, not shape


def test_repr_shows_config():
    c = _go2()(prefix="go2a")
    assert repr(c) == "Go2Connection(prefix='go2a', robot_ip=None, odom_timeout=0.5)"


def test_reduce_rebuilds_from_config():
    c = _go2()(prefix="go2a")
    fn, args = c.__reduce__()[:2]
    assert fn(*args) == c
    # Full pickle.dumps needs a module-level class (T12 examples cover it).


# ── definition-time errors ──────────────────────────────────────────────────


def test_reserved_name_rejected():
    with pytest.raises(ConfigFieldError):

        class Bad(PureModule):
            config: str = "x"  # type: ignore[assignment]


def test_pydantic_namespace_collision_rejected():
    with pytest.raises(ConfigFieldError):

        class Bad(PureModule):
            model_dump: str = "x"


def test_unannotated_assignment_rejected():
    with pytest.raises(ConfigFieldError):

        class Bad(PureModule):
            emit_every = 5  # missing annotation: field-or-constant ambiguity


def test_underscore_field_rejected():
    with pytest.raises(ConfigFieldError):

        class Bad(PureModule):
            _rate: float = 1.0


def test_final_field_rejected():
    from typing import Final

    with pytest.raises(ConfigFieldError):

        class Bad(PureModule):
            rate: Final[float] = 1.0


def test_config_field_errors_are_type_errors():
    with pytest.raises(TypeError):  # stable across the T3 base flip (spec §10)

        class Bad(PureModule):
            config: str = "x"  # type: ignore[assignment]


def test_config_field_errors_share_definition_error_base():
    # Post-flip (t2-config.md §10.1): one user-facing definition-error type —
    # except PureModuleDefinitionError catches config AND shape violations.
    assert issubclass(ConfigFieldError, PureModuleDefinitionError)
    assert issubclass(ConfigFieldError, TypeError)
    with pytest.raises(PureModuleDefinitionError):

        class Bad(PureModule):
            config: str = "x"  # type: ignore[assignment]


# ── service interop ─────────────────────────────────────────────────────────


def test_service_interop_surface():
    c = _go2()()
    assert isinstance(c.config, BaseConfig)  # Configurable's declared currency
    # T8-filled behavior (t8-rim.md S4): warmup tolerates an unwired module
    # (the bridge wires streams after build, §11.3); start fails LOUD on the
    # unwired trigger (§6.1.1); stop is idempotent on a never-started session.
    assert c.warmup() is None
    with pytest.raises(TypeError, match=r"\[align-missing-tick-stream\]"):
        c.start()
    assert c.stop() is None
