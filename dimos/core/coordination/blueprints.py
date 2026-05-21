# Copyright 2025-2026 Dimensional Inc.
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

from collections.abc import Callable, Mapping
from dataclasses import dataclass, field, replace
from functools import cached_property, reduce
import operator
import sys
import types as types_mod
from types import MappingProxyType
from typing import TYPE_CHECKING, Any, Literal, Union, get_args, get_origin, get_type_hints

from pydantic import create_model

if TYPE_CHECKING:
    from dimos.protocol.service.system_configurator.base import SystemConfigurator

from dimos.core.global_config import GlobalConfig
from dimos.core.module import ModuleBase, is_module_type
from dimos.core.stream import In, Out
from dimos.core.transport import PubSubTransport
from dimos.spec.utils import Spec, is_spec
from dimos.utils.logging_config import setup_logger

if sys.version_info >= (3, 11):
    from typing import Self
else:
    from typing_extensions import Self

logger = setup_logger()


class DisabledModuleProxy:
    def __init__(self, spec_name: str) -> None:
        object.__setattr__(self, "_spec_name", spec_name)

    def __getattr__(self, name: str) -> Any:
        spec = object.__getattribute__(self, "_spec_name")

        def _noop(*_args: Any, **_kwargs: Any) -> None:
            logger.warning(
                "Called on disabled module (no-op)",
                method=name,
                spec=spec,
            )
            return None

        return _noop

    def __reduce__(self) -> tuple[type, tuple[str]]:
        return (DisabledModuleProxy, (self._spec_name,))

    def __repr__(self) -> str:
        return f"<DisabledModuleProxy spec={self._spec_name}>"


@dataclass(frozen=True)
class StreamRef:
    name: str
    type: type
    direction: Literal["in", "out"]


@dataclass(frozen=True)
class ModuleRef:
    name: str
    spec: type[Spec] | type[ModuleBase]
    optional: bool = False


@dataclass(frozen=True)
class BlueprintAtom:
    kwargs: dict[str, Any]
    module: type[ModuleBase]
    streams: tuple[StreamRef, ...]
    module_refs: tuple[ModuleRef, ...]

    @classmethod
    def create(cls, module: type[ModuleBase], kwargs: dict[str, Any]) -> Self:
        streams: list[StreamRef] = []
        module_refs: list[ModuleRef] = []

        # Resolve annotations using namespaces from the full MRO chain so that
        # In/Out behind TYPE_CHECKING + `from __future__ import annotations` work.
        # Iterate reversed MRO so the most specific class's namespace wins when
        # parent modules shadow names (e.g. spec.perception.Image vs sensor_msgs.Image).
        globalns: dict[str, Any] = {}
        for c in reversed(module.__mro__):
            if c.__module__ in sys.modules:
                globalns.update(sys.modules[c.__module__].__dict__)
        try:
            all_annotations = get_type_hints(module, globalns=globalns)
        except Exception:
            # Fallback to raw annotations if get_type_hints fails.
            all_annotations = {}
            for base_class in reversed(module.__mro__):
                if hasattr(base_class, "__annotations__"):
                    all_annotations.update(base_class.__annotations__)

        for name, annotation in all_annotations.items():
            origin = get_origin(annotation)
            # Streams
            if origin in (In, Out):
                direction = "in" if origin == In else "out"
                type_ = get_args(annotation)[0]
                streams.append(
                    StreamRef(name=name, type=type_, direction=direction)  # type: ignore[arg-type]
                )
            # linking to unknown module via Spec
            elif is_spec(annotation):
                module_refs.append(ModuleRef(name=name, spec=annotation))
            # linking to specific/known module directly
            elif is_module_type(annotation):
                module_refs.append(ModuleRef(name=name, spec=annotation))
            # Optional Spec or Module: SomeSpec | None
            elif origin in (Union, types_mod.UnionType):
                args = [a for a in get_args(annotation) if a is not type(None)]
                if len(args) == 1:
                    inner = args[0]
                    if is_spec(inner):
                        module_refs.append(ModuleRef(name=name, spec=inner, optional=True))
                    elif is_module_type(inner):
                        module_refs.append(ModuleRef(name=name, spec=inner, optional=True))

        return cls(
            module=module,
            streams=tuple(streams),
            module_refs=tuple(module_refs),
            kwargs=kwargs,
        )


@dataclass(frozen=True)
class Blueprint:
    blueprints: tuple[BlueprintAtom, ...]
    disabled_modules_tuple: tuple[type[ModuleBase], ...] = field(default_factory=tuple)
    transport_map: Mapping[tuple[str, type], PubSubTransport[Any]] = field(
        default_factory=lambda: MappingProxyType({})
    )
    global_config_overrides: Mapping[str, Any] = field(default_factory=lambda: MappingProxyType({}))
    remapping_map: Mapping[tuple[type[ModuleBase], str], str | type[ModuleBase] | type[Spec]] = (
        field(default_factory=lambda: MappingProxyType({}))
    )
    requirement_checks: tuple[Callable[[], str | None], ...] = field(default_factory=tuple)
    configurator_checks: "tuple[SystemConfigurator, ...]" = field(default_factory=tuple)

    @classmethod
    def create(cls, module: type[ModuleBase], **kwargs: Any) -> "Blueprint":
        blueprint = BlueprintAtom.create(module, kwargs)
        return cls(blueprints=(blueprint,))

    def disabled_modules(self, *modules: type[ModuleBase]) -> "Blueprint":
        return replace(self, disabled_modules_tuple=self.disabled_modules_tuple + modules)

    def config(self) -> type:
        configs = {
            b.module.name: (get_type_hints(b.module)["config"] | None, None)
            for b in self.blueprints
        }
        configs["g"] = (GlobalConfig | None, None)
        return create_model("BlueprintConfig", __config__={"extra": "forbid"}, **configs)  # type: ignore[call-overload,no-any-return]

    def transports(self, transports: dict[tuple[str, type], Any]) -> "Blueprint":
        return replace(self, transport_map=MappingProxyType({**self.transport_map, **transports}))

    def global_config(self, **kwargs: Any) -> "Blueprint":
        return replace(
            self,
            global_config_overrides=MappingProxyType({**self.global_config_overrides, **kwargs}),
        )

    def remappings(
        self,
        remappings: list[tuple[type[ModuleBase], str, str | type[ModuleBase] | type[Spec]]],
    ) -> "Blueprint":
        remappings_dict = dict(self.remapping_map)
        for module, old, new in remappings:
            remappings_dict[(module, old)] = new
        return replace(self, remapping_map=MappingProxyType(remappings_dict))

    def requirements(self, *checks: Callable[[], str | None]) -> "Blueprint":
        return replace(self, requirement_checks=self.requirement_checks + tuple(checks))

    def configurators(self, *checks: "SystemConfigurator") -> "Blueprint":
        return replace(self, configurator_checks=self.configurator_checks + tuple(checks))

    def with_backend(self, backend: str) -> "Blueprint":
        """Swap tagged connection modules for the matching `(robot, backend)` variant.

        For each atom whose module carries a `_connection_tag` with a backend
        different from the requested one, look up the same-robot module for
        `backend` in the connection registry and substitute it. Streams,
        remappings, and disabled-modules entries are rewritten to point at the
        new class.

        If the blueprint has no tagged atoms this is a no-op (with a warning).
        """
        # Lazy import to keep blueprints.py free of robot deps.
        from dimos.robot.connection_registry import backends_for, get_connection

        swap_map: dict[type[ModuleBase], type[ModuleBase]] = {}
        for atom in self.blueprints:
            tag = getattr(atom.module, "_connection_tag", None)
            if tag is None or tag.backend == backend:
                continue
            target = get_connection(tag.robot, backend)
            if target is None:
                available = sorted(backends_for(tag.robot))
                raise ValueError(
                    f"No connection registered for robot={tag.robot!r} "
                    f"backend={backend!r} (have: {available})"
                )
            swap_map[atom.module] = target

        if not swap_map:
            tagged = any(getattr(a.module, "_connection_tag", None) for a in self.blueprints)
            if not tagged:
                logger.warning(
                    "Blueprint.with_backend(%r) had no tagged connection atoms — "
                    "returning blueprint unchanged",
                    backend,
                )
            return self

        new_atoms: list[BlueprintAtom] = []
        for atom in self.blueprints:
            target = swap_map.get(atom.module)
            if target is None:
                new_atoms.append(atom)
                continue
            _check_stream_parity(atom.module, target, atom)
            _check_kwargs_compat(target, atom.kwargs)
            new_atoms.append(BlueprintAtom.create(target, atom.kwargs))

        new_remappings = {
            (swap_map.get(m, m), name): v for (m, name), v in self.remapping_map.items()
        }
        new_disabled = tuple(swap_map.get(m, m) for m in self.disabled_modules_tuple)

        return replace(
            self,
            blueprints=tuple(new_atoms),
            remapping_map=MappingProxyType(new_remappings),
            disabled_modules_tuple=new_disabled,
        )

    @cached_property
    def active_blueprints(self) -> tuple[BlueprintAtom, ...]:
        if not self.disabled_modules_tuple:
            return self.blueprints
        disabled = set(self.disabled_modules_tuple)
        return tuple(bp for bp in self.blueprints if bp.module not in disabled)


def autoconnect(*blueprints: Blueprint) -> Blueprint:
    all_blueprints = tuple(_eliminate_duplicates([bp for bs in blueprints for bp in bs.blueprints]))
    all_transports = dict(  # type: ignore[var-annotated]
        reduce(operator.iadd, [list(x.transport_map.items()) for x in blueprints], [])
    )
    all_config_overrides = dict(  # type: ignore[var-annotated]
        reduce(operator.iadd, [list(x.global_config_overrides.items()) for x in blueprints], [])
    )
    all_remappings = dict(  # type: ignore[var-annotated]
        reduce(operator.iadd, [list(x.remapping_map.items()) for x in blueprints], [])
    )
    all_requirement_checks = tuple(check for bs in blueprints for check in bs.requirement_checks)
    all_configurator_checks = tuple(check for bs in blueprints for check in bs.configurator_checks)

    return Blueprint(
        blueprints=all_blueprints,
        disabled_modules_tuple=tuple(
            module for bp in blueprints for module in bp.disabled_modules_tuple
        ),
        transport_map=MappingProxyType(all_transports),
        global_config_overrides=MappingProxyType(all_config_overrides),
        remapping_map=MappingProxyType(all_remappings),
        requirement_checks=all_requirement_checks,
        configurator_checks=all_configurator_checks,
    )


def _eliminate_duplicates(blueprints: list[BlueprintAtom]) -> list[BlueprintAtom]:
    # The duplicates are eliminated in reverse so that newer blueprints override older ones.
    seen = set()
    unique_blueprints = []
    for bp in reversed(blueprints):
        if bp.module not in seen:
            seen.add(bp.module)
            unique_blueprints.append(bp)
    return list(reversed(unique_blueprints))


def _stream_signature(streams: tuple[StreamRef, ...]) -> set[tuple[str, str]]:
    return {(s.name, s.direction) for s in streams}


def _check_stream_parity(old: type[ModuleBase], new: type[ModuleBase], atom: BlueprintAtom) -> None:
    new_atom = BlueprintAtom.create(new, atom.kwargs)
    old_sig = _stream_signature(atom.streams)
    new_sig = _stream_signature(new_atom.streams)
    if old_sig != new_sig:
        only_old = sorted(old_sig - new_sig)
        only_new = sorted(new_sig - old_sig)
        raise ValueError(
            f"Stream surface drift swapping {old.__name__} -> {new.__name__}: "
            f"only on {old.__name__}={only_old}, only on {new.__name__}={only_new}"
        )


def _check_kwargs_compat(new: type[ModuleBase], kwargs: dict[str, Any]) -> None:
    if not kwargs:
        return
    try:
        config_type = get_type_hints(new).get("config")
    except Exception:
        return
    if config_type is None:
        return
    valid_fields = set(getattr(config_type, "model_fields", {}))
    invalid = set(kwargs) - valid_fields
    if invalid:
        raise ValueError(
            f"Kwargs from blueprint atom are incompatible with {new.__name__}'s "
            f"config ({config_type.__name__}): unknown field(s) {sorted(invalid)}"
        )
