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
