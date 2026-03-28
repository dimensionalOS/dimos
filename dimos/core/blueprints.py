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

from collections import defaultdict
from collections.abc import Callable, Mapping
from dataclasses import dataclass, field, replace
from functools import cached_property, reduce
import operator
import sys
import types as types_mod
from types import MappingProxyType
from typing import TYPE_CHECKING, Any, Literal, Union, get_args, get_origin, get_type_hints

if TYPE_CHECKING:
    from dimos.protocol.service.system_configurator.base import SystemConfigurator

from dimos.core.global_config import GlobalConfig, global_config
from dimos.core.module import ModuleBase, ModuleSpec, is_module_type
from dimos.core.module_coordinator import ModuleCoordinator
from dimos.core.stream import In, Out
from dimos.core.transport import LCMTransport, PubSubTransport, pLCMTransport
from dimos.spec.utils import Spec, is_spec, spec_annotation_compliance, spec_structural_compliance
from dimos.utils.generic import short_id
from dimos.utils.logging_config import setup_logger

if sys.version_info >= (3, 11):
    from typing import Self
else:
    from typing_extensions import Self

logger = setup_logger()


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
class _BlueprintAtom:
    kwargs: dict[str, Any]
    module: type[ModuleBase[Any]]
    streams: tuple[StreamRef, ...]
    module_refs: tuple[ModuleRef, ...]

    @classmethod
    def create(cls, module: type[ModuleBase[Any]], kwargs: dict[str, Any]) -> Self:
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
    blueprints: tuple[_BlueprintAtom, ...]
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
        blueprint = _BlueprintAtom.create(module, kwargs)
        return cls(blueprints=(blueprint,))

    def disabled_modules(self, *modules: type[ModuleBase]) -> "Blueprint":
        return replace(self, disabled_modules_tuple=self.disabled_modules_tuple + modules)

    def transports(self, transports: dict[tuple[str, type], Any]) -> "Blueprint":
        return replace(self, transport_map=MappingProxyType({**self.transport_map, **transports}))

    def global_config(self, **kwargs: Any) -> "Blueprint":
        return replace(
            self,
            global_config_overrides=MappingProxyType({**self.global_config_overrides, **kwargs}),
        )

    def remappings(
        self,
        remappings: list[
            tuple[type[ModuleBase[Any]], str, str | type[ModuleBase[Any]] | type[Spec]]
        ],
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
    def _active_blueprints(self) -> tuple[_BlueprintAtom, ...]:
        if not self.disabled_modules_tuple:
            return self.blueprints
        disabled = set(self.disabled_modules_tuple)
        return tuple(bp for bp in self.blueprints if bp.module not in disabled)

    def _get_transport_for(self, name: str, stream_type: type) -> PubSubTransport[Any]:
        transport = self.transport_map.get((name, stream_type), None)
        if transport:
            return transport

        use_pickled = getattr(stream_type, "lcm_encode", None) is None
        topic = f"/{name}" if self._is_name_unique(name) else f"/{short_id()}"
        transport = pLCMTransport(topic) if use_pickled else LCMTransport(topic, stream_type)

        return transport

    @cached_property
    def _all_name_types(self) -> set[tuple[str, type]]:
        # Apply remappings to get the actual names that will be used
        result = set()
        for blueprint in self._active_blueprints:
            for conn in blueprint.streams:
                # Check if this stream should be remapped
                remapped_name = self.remapping_map.get((blueprint.module, conn.name), conn.name)
                if isinstance(remapped_name, str):
                    result.add((remapped_name, conn.type))
        return result

    def _is_name_unique(self, name: str) -> bool:
        return sum(1 for n, _ in self._all_name_types if n == name) == 1

    def _run_configurators(self) -> None:
        from dimos.protocol.service.system_configurator.base import configure_system
        from dimos.protocol.service.system_configurator.lcm_config import lcm_configurators

        configurators = [*lcm_configurators(), *self.configurator_checks]

        try:
            configure_system(configurators)
        except SystemExit:
            labels = [type(c).__name__ for c in configurators]
            print(
                f"Required system configuration was declined: {', '.join(labels)}",
                file=sys.stderr,
            )
            sys.exit(1)

    def _check_requirements(self) -> None:
        errors = []
        red = "\033[31m"
        reset = "\033[0m"

        for check in self.requirement_checks:
            error = check()
            if error:
                errors.append(error)

        if errors:
            for error in errors:
                print(f"{red}Error: {error}{reset}", file=sys.stderr)
            sys.exit(1)

    def _verify_no_name_conflicts(self) -> None:
        name_to_types = defaultdict(set)
        name_to_modules = defaultdict(list)

        for blueprint in self._active_blueprints:
            for conn in blueprint.streams:
                stream_name = self.remapping_map.get((blueprint.module, conn.name), conn.name)
                name_to_types[stream_name].add(conn.type)
                name_to_modules[stream_name].append((blueprint.module, conn.type))

        conflicts = {}
        for conn_name, types in name_to_types.items():
            if len(types) > 1:
                modules_by_type = defaultdict(list)
                for module, conn_type in name_to_modules[conn_name]:
                    modules_by_type[conn_type].append(module)
                conflicts[conn_name] = modules_by_type

        if not conflicts:
            return

        error_lines = ["Blueprint cannot start because there are conflicting streams."]
        for name, modules_by_type in conflicts.items():
            type_entries = []
            for conn_type, modules in modules_by_type.items():
                for module in modules:
                    type_str = f"{conn_type.__module__}.{conn_type.__name__}"
                    module_str = module.__name__
                    type_entries.append((type_str, module_str))
            if len(type_entries) >= 2:
                locations = ", ".join(f"{type_} in {module}" for type_, module in type_entries)
                error_lines.append(f"    - '{name}' has conflicting types. {locations}")

        raise ValueError("\n".join(error_lines))

    def _deploy_all_modules(
        self, module_coordinator: ModuleCoordinator, global_config: GlobalConfig
    ) -> None:
        module_specs: list[ModuleSpec] = []
        for blueprint in self._active_blueprints:
            module_specs.append((blueprint.module, global_config, blueprint.kwargs))

        module_coordinator.deploy_parallel(module_specs)

    def _connect_streams(self, module_coordinator: ModuleCoordinator) -> None:
        # dict when given (final/remapped) stream name+type, provides a list of modules + original (non-remapped) stream names
        streams = defaultdict(list)

        for blueprint in self._active_blueprints:
            for conn in blueprint.streams:
                # Check if this stream should be remapped
                remapped_name = self.remapping_map.get((blueprint.module, conn.name), conn.name)
                if isinstance(remapped_name, str):
                    # Group by remapped name and type
                    streams[remapped_name, conn.type].append((blueprint.module, conn.name))

        # Connect all In/Out streams by remapped name and type.
        for remapped_name, stream_type in streams.keys():
            transport = self._get_transport_for(remapped_name, stream_type)
            for module, original_name in streams[(remapped_name, stream_type)]:
                instance = module_coordinator.get_instance(module)  # type: ignore[assignment]
                instance.set_transport(original_name, transport)  # type: ignore[union-attr]
                logger.info(
                    "Transport",
                    name=remapped_name,
                    original_name=original_name,
                    topic=str(getattr(transport, "topic", None)),
                    type=f"{stream_type.__module__}.{stream_type.__qualname__}",
                    module=module.__name__,
                    transport=transport.__class__.__name__,
                )

    def _connect_module_refs(self, module_coordinator: ModuleCoordinator) -> None:
        # partly fill out the mod_and_mod_ref_to_proxy
        mod_and_mod_ref_to_proxy = {
            (module, name): replacement
            for (module, name), replacement in self.remapping_map.items()
            if is_spec(replacement) or is_module_type(replacement)
        }

        # after this loop we should have an exact module for every module_ref on every blueprint
        for blueprint in self._active_blueprints:
            for each_module_ref in blueprint.module_refs:
                # we've got to find a another module that implements this spec
                spec = mod_and_mod_ref_to_proxy.get(
                    (blueprint.module, each_module_ref.name), each_module_ref.spec
                )

                # if the spec is actually module, use that (basically a user override)
                if is_module_type(spec):
                    mod_and_mod_ref_to_proxy[blueprint.module, each_module_ref.name] = spec
                    continue

                # find all available candidates
                possible_module_candidates = [
                    each_other_blueprint.module
                    for each_other_blueprint in self._active_blueprints
                    if (
                        each_other_blueprint != blueprint
                        and spec_structural_compliance(each_other_blueprint.module, spec)
                    )
                ]
                # we keep valid separate from invalid to provide a better error message for "almost" valid cases
                valid_module_candidates = [
                    each_candidate
                    for each_candidate in possible_module_candidates
                    if spec_annotation_compliance(each_candidate, spec)
                ]
                # none
                if len(possible_module_candidates) == 0:
                    if each_module_ref.optional:
                        continue
                    raise Exception(
                        f"""The {blueprint.module.__name__} has a module reference ({each_module_ref}) which requested a module that fills out the {each_module_ref.spec.__name__} spec. But I couldn't find a module that met that spec.\n"""
                    )
                # exactly one structurally valid candidate
                elif len(possible_module_candidates) == 1:
                    if len(valid_module_candidates) == 0:
                        logger.warning(
                            f"""The {blueprint.module.__name__} has a module reference ({each_module_ref}) which requested a module that fills out the {each_module_ref.spec.__name__} spec. I found a module ({possible_module_candidates[0].__name__}) that met that spec structurally, but it had a mismatch in type annotations.\nPlease either change the {each_module_ref.spec.__name__} spec or the {possible_module_candidates[0].__name__} module.\n"""
                        )
                    mod_and_mod_ref_to_proxy[blueprint.module, each_module_ref.name] = (
                        possible_module_candidates[0]
                    )
                    continue
                # more than one
                elif len(valid_module_candidates) > 1:
                    raise Exception(
                        f"""The {blueprint.module.__name__} has a module reference ({each_module_ref}) which requested a module that fills out the {each_module_ref.spec.__name__} spec. But I found multiple modules that met that spec: {valid_module_candidates}.\nTo fix this use .remappings, for example:\n    autoconnect(...).remappings([ ({blueprint.module.__name__}, {each_module_ref.name!r}, <ModuleThatHasTheRpcCalls>) ])\n"""
                    )
                # structural candidates, but no valid candidates
                elif len(valid_module_candidates) == 0:
                    possible_module_candidates_str = ", ".join(
                        [each_candidate.__name__ for each_candidate in possible_module_candidates]
                    )
                    raise Exception(
                        f"""The {blueprint.module.__name__} has a module reference ({each_module_ref}) which requested a module that fills out the {each_module_ref.spec.__name__} spec. Some modules ({possible_module_candidates_str}) met the spec structurally but had a mismatch in type annotations\n"""
                    )
                # one valid candidate (and more than one structurally valid candidate)
                else:
                    mod_and_mod_ref_to_proxy[blueprint.module, each_module_ref.name] = (
                        valid_module_candidates[0]
                    )

        # now that we know the streams, we mutate the RPCClient objects
        for (base_module, module_ref_name), target_module in mod_and_mod_ref_to_proxy.items():
            base_module_proxy = module_coordinator.get_instance(base_module)
            target_module_proxy = module_coordinator.get_instance(target_module)  # type: ignore[type-var,arg-type]
            setattr(
                base_module_proxy,
                module_ref_name,
                target_module_proxy,
            )
            # Ensure the remote module instance can use the module ref inside its own RPC handlers.
            base_module_proxy.set_module_ref(module_ref_name, target_module_proxy)

    def build(
        self,
        cli_config_overrides: Mapping[str, Any] | None = None,
    ) -> ModuleCoordinator:
        logger.info("Building the blueprint")
        global_config.update(**dict(self.global_config_overrides))
        if cli_config_overrides:
            global_config.update(**dict(cli_config_overrides))

        self._run_configurators()
        self._check_requirements()
        self._verify_no_name_conflicts()

        logger.info("Starting the modules")
        module_coordinator = ModuleCoordinator(g=global_config)
        module_coordinator.start()

        # all module constructors are called here (each of them setup their own)
        self._deploy_all_modules(module_coordinator, global_config)
        self._connect_streams(module_coordinator)
        self._connect_module_refs(module_coordinator)

        module_coordinator.build_all_modules()
        module_coordinator.start_all_modules()

        return module_coordinator


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


def _eliminate_duplicates(blueprints: list[_BlueprintAtom]) -> list[_BlueprintAtom]:
    # The duplicates are eliminated in reverse so that newer blueprints override older ones.
    seen = set()
    unique_blueprints = []
    for bp in reversed(blueprints):
        if bp.module not in seen:
            seen.add(bp.module)
            unique_blueprints.append(bp)
    return list(reversed(unique_blueprints))
