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

from collections.abc import Callable, Iterable, Mapping, Sequence
from dataclasses import dataclass, field, replace
from functools import cached_property, reduce
import operator
import re
import sys
import types as types_mod
from types import MappingProxyType
from typing import TYPE_CHECKING, Any, Literal, Union, get_args, get_origin, get_type_hints

from pydantic import BaseModel, create_model

if TYPE_CHECKING:
    from dimos.protocol.service.system_configurator.base import SystemConfigurator

from dimos.core.global_config import GlobalConfig
from dimos.core.module import ModuleBase, is_module_type
from dimos.core.stream import In, Out, Transport
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
    # Set when the same module class appears more than once in a blueprint
    # (e.g. one per robot). `None` means just one.
    instance_name: str | None = None

    @property
    def name(self) -> str:
        """The key identifying this module instance within a blueprint."""
        return self.instance_name if self.instance_name is not None else self.module.name

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

        instance_name = kwargs.get("instance_name")
        if instance_name is not None and not isinstance(instance_name, str):
            raise TypeError("instance_name must be a string or None")

        return cls(
            module=module,
            streams=tuple(streams),
            module_refs=tuple(module_refs),
            kwargs=kwargs,
            instance_name=instance_name,
        )


@dataclass(frozen=True)
class TransportSpec:
    """Deferred transport construction: a transport class plus its ctor args.

    Blueprint authors declare transports via ``Cls.spec(...)`` so nothing is
    constructed at blueprint-definition time. The coordinator materializes
    specs at build time, once CLI/env/config overrides have resolved.
    """

    cls: type[Transport[Any]]
    args: tuple[Any, ...] = ()
    kwargs: Mapping[str, Any] = field(default_factory=dict)

    @property
    def config_cls(self) -> type[BaseModel] | None:
        # Set by transports that expose a pydantic config override surface
        return self.cls._config_cls

    def build(self, config: BaseModel | None = None) -> Transport[Any]:
        extra = {"config": config} if config is not None else {}
        return self.cls(*self.args, **self.kwargs, **extra)


# These fields cannot be pickled.
_PROXY_FIELDS = ("transport_map", "global_config_overrides", "remapping_map")


@dataclass(frozen=True)
class Blueprint:
    blueprints: tuple[BlueprintAtom, ...]
    disabled_modules_tuple: tuple[type[ModuleBase], ...] = field(default_factory=tuple)
    transport_map: Mapping[tuple[str, type], TransportSpec | Transport[Any]] = field(
        default_factory=lambda: MappingProxyType({})
    )
    global_config_overrides: Mapping[str, Any] = field(default_factory=lambda: MappingProxyType({}))

    # Keyed by (instance name, stream/ref name).
    remapping_map: Mapping[tuple[str, str], str | type[ModuleBase] | type[Spec]] = field(
        default_factory=lambda: MappingProxyType({})
    )

    requirement_checks: tuple[Callable[[], str | None], ...] = field(default_factory=tuple)
    configurator_checks: "tuple[SystemConfigurator, ...]" = field(default_factory=tuple)

    def __getstate__(self) -> dict[str, Any]:
        state = self.__dict__.copy()
        state.pop("active_blueprints", None)  # recomputable cached_property
        for name in _PROXY_FIELDS:
            state[name] = dict(state[name])
        return state

    def __setstate__(self, state: dict[str, Any]) -> None:
        for name in _PROXY_FIELDS:
            state[name] = MappingProxyType(state[name])
        self.__dict__.update(state)

    @classmethod
    def create(cls, module: type[ModuleBase], **kwargs: Any) -> "Blueprint":
        blueprint = BlueprintAtom.create(module, kwargs)
        return cls(blueprints=(blueprint,))

    def disabled_modules(self, *modules: type[ModuleBase]) -> "Blueprint":
        return replace(self, disabled_modules_tuple=self.disabled_modules_tuple + modules)

    def config(self) -> type:
        configs = {}

        for b in self.blueprints:
            key = config_key(b.name)
            if key in configs:
                raise ValueError(
                    f"Config key collision: two module instances map to {key!r}. "
                    f"Rename one of the instances."
                )
            configs[key] = (get_type_hints(b.module)["config"] | None, None)

        configs["g"] = (GlobalConfig | None, None)
        transport_fields: dict[str, Any] = {}
        seen: set[type] = set()
        for spec in self.transport_map.values():
            # Raw transport instances (plain `LCMTransport(...)` pins) have no
            # config override surface — only deferred specs participate.
            cls = spec.config_cls if isinstance(spec, TransportSpec) else None
            if cls is None or cls in seen:
                continue
            seen.add(cls)
            transport_fields[transport_config_name(cls)] = (cls | None, None)
        if transport_fields:
            transports_model = create_model(
                "TransportsConfig", __config__={"extra": "forbid"}, **transport_fields
            )
            configs["transports"] = (transports_model | None, None)
        return create_model("BlueprintConfig", __config__={"extra": "forbid"}, **configs)  # type: ignore[call-overload,no-any-return]

    def transports(
        self, transports: dict[tuple[str, type], TransportSpec | Transport[Any]]
    ) -> "Blueprint":
        return replace(self, transport_map=MappingProxyType({**self.transport_map, **transports}))

    def global_config(self, **kwargs: Any) -> "Blueprint":
        return replace(
            self,
            global_config_overrides=MappingProxyType({**self.global_config_overrides, **kwargs}),
        )

    def remappings(
        self,
        remappings: Sequence[
            tuple[type[ModuleBase] | str, str, str | type[ModuleBase] | type[Spec]]
        ],
    ) -> "Blueprint":
        remappings_dict = dict(self.remapping_map)
        for module, old, new in remappings:
            remappings_dict[(self._instance_key(module), old)] = new
        return replace(self, remapping_map=MappingProxyType(remappings_dict))

    def _instance_key(self, module: type[ModuleBase] | str) -> str:
        if isinstance(module, str):
            return module
        names = [b.name for b in self.blueprints if b.module is module]
        if len(names) > 1:
            raise ValueError(
                f"{module.__name__} has multiple instances in this blueprint "
                f"({', '.join(sorted(names))}). Pass the instance name instead of the class."
            )
        return names[0] if names else module.name

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


def transport_config_name(cls: type) -> str:
    return cls.__name__.removesuffix("Config").lower()


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
        if bp.name not in seen:
            seen.add(bp.name)
            unique_blueprints.append(bp)
    return list(reversed(unique_blueprints))


def config_key(instance_name: str) -> str:
    """Escape an instance name into a valid config/CLI/env identifier."""
    return instance_name.replace("/", "_")


def namespace(prefix: str, *blueprints: Blueprint, expose: Iterable[str] = ()) -> Blueprint:
    """Isolate *blueprints* under a name prefix so several copies can coexist.

    Instance names, stream names (and so their topics), TF frame ids, and RPC
    topics all get the `{prefix}/` prefix, which disconnects the namespaced
    modules from everything outside.

    Stream names listed in *expose* are left unprefixed, so they connect
    globally by the usual (name, type) matching.  That is how data crosses the
    namespace boundary:

        fleet = autoconnect(
            AggregateMapper.blueprint(),  # shared: sees every robot's pointcloud
            *[
                namespace(f"robot{i}", GO2Connection.blueprint(ip=ip), expose={"pointcloud"})
                for i, ip in enumerate(ips)
            ],
        )
    """
    if not re.fullmatch(r"[A-Za-z0-9_]+", prefix):
        raise ValueError(
            f"Invalid namespace prefix {prefix!r}; use letters, digits and underscores "
            f"(nest namespaces by composition, not with '/')."
        )

    merged = autoconnect(*blueprints)
    expose_set = frozenset(expose)

    effective_names = set()
    for atom in merged.blueprints:
        for stream in atom.streams:
            effective = merged.remapping_map.get((atom.name, stream.name), stream.name)
            if isinstance(effective, str):
                effective_names.add(effective)

    unknown = expose_set - effective_names
    if unknown:
        raise ValueError(
            f"expose names {sorted(unknown)} do not match any stream in the "
            f"namespaced blueprint (available: {sorted(effective_names)})"
        )

    new_atoms = []
    new_remap: dict[tuple[str, str], str | type[ModuleBase] | type[Spec]] = {}

    for atom in merged.blueprints:
        new_name = f"{prefix}/{atom.name}"
        kwargs = dict(atom.kwargs)
        kwargs["instance_name"] = new_name
        old_namespace = atom.name.rsplit("/", 1)[0] if "/" in atom.name else ""
        frame_id_prefix = kwargs.get("frame_id_prefix")
        if frame_id_prefix is None:
            kwargs["frame_id_prefix"] = prefix
        elif old_namespace and frame_id_prefix == old_namespace:
            # Auto-set by an inner namespace(); extend it. User-set values are kept.
            kwargs["frame_id_prefix"] = f"{prefix}/{frame_id_prefix}"
        new_atoms.append(replace(atom, kwargs=kwargs, instance_name=new_name))

        for stream in atom.streams:
            effective = merged.remapping_map.get((atom.name, stream.name), stream.name)
            if not isinstance(effective, str):
                continue
            if effective in expose_set:
                if (atom.name, stream.name) in merged.remapping_map:
                    new_remap[new_name, stream.name] = effective
            else:
                new_remap[new_name, stream.name] = f"{prefix}/{effective}"

    # Module-ref remappings (values that are classes/Specs) keep their values.
    for (instance, ref_name), value in merged.remapping_map.items():
        if not isinstance(value, str):
            new_remap[f"{prefix}/{instance}", ref_name] = value

    new_transports = {}
    for (name, type_), transport in merged.transport_map.items():
        if name in expose_set:
            new_transports[name, type_] = transport
        else:
            new_transports[f"{prefix}/{name}", type_] = _reprefix_transport(transport, prefix)

    return replace(
        merged,
        blueprints=tuple(new_atoms),
        remapping_map=MappingProxyType(new_remap),
        transport_map=MappingProxyType(new_transports),
    )


def _reprefix_transport(
    transport: TransportSpec | Transport[Any], prefix: str
) -> TransportSpec | Transport[Any]:
    """Clone a pinned transport with its topic moved under the namespace prefix."""
    if isinstance(transport, TransportSpec):
        cls, args = transport.cls, transport.args
    else:
        cls, args = transport.__reduce__()  # type: ignore[misc]
    if not (isinstance(args, tuple) and args and isinstance(args[0], str)):
        raise ValueError(
            f"Cannot namespace pinned transport {transport!r}; pin it on the "
            f"namespaced blueprint with an explicit topic instead."
        )
    topic = args[0]
    new_topic = f"/{prefix}{topic}" if topic.startswith("/") else f"{prefix}/{topic}"
    if isinstance(transport, TransportSpec):
        return replace(transport, args=(new_topic, *args[1:]))
    return cls(new_topic, *args[1:])  # type: ignore[no-any-return, call-arg]
