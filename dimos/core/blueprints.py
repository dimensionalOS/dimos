# Copyright 2025 Dimensional Inc.
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

from abc import ABC
from collections import defaultdict
from collections.abc import Callable, Mapping
from dataclasses import dataclass, field
from functools import cached_property, reduce
import inspect
import operator
import sys
from types import MappingProxyType
from typing import Any, Literal, get_args, get_origin, get_type_hints

from dimos.core.global_config import GlobalConfig
from dimos.core.module import Module
from dimos.core.module_coordinator import ModuleCoordinator
from dimos.core.stream import In, Out
from dimos.core.transport import LCMTransport, pLCMTransport
from dimos.utils.generic import short_id
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


@dataclass(frozen=True)
class ModuleConnection:
    name: str
    type: type
    direction: Literal["in", "out"]


@dataclass(frozen=True)
class ModuleBlueprint:
    module: type[Module]
    connections: tuple[ModuleConnection, ...]
    args: tuple[Any]
    kwargs: dict[str, Any]


@dataclass(frozen=True)
class ModuleBlueprintSet:
    blueprints: tuple[ModuleBlueprint, ...]
    # TODO: Replace Any
    transport_map: Mapping[tuple[str, type], Any] = field(
        default_factory=lambda: MappingProxyType({})
    )
    global_config_overrides: Mapping[str, Any] = field(default_factory=lambda: MappingProxyType({}))
    remapping_map: Mapping[tuple[type[Module], str], str] = field(
        default_factory=lambda: MappingProxyType({})
    )
    requirement_checks: tuple[Callable[[], str | None], ...] = field(default_factory=tuple)

    def transports(self, transports: dict[tuple[str, type], Any]) -> "ModuleBlueprintSet":
        return ModuleBlueprintSet(
            blueprints=self.blueprints,
            transport_map=MappingProxyType({**self.transport_map, **transports}),
            global_config_overrides=self.global_config_overrides,
            remapping_map=self.remapping_map,
            requirement_checks=self.requirement_checks,
        )

    def global_config(self, **kwargs: Any) -> "ModuleBlueprintSet":
        return ModuleBlueprintSet(
            blueprints=self.blueprints,
            transport_map=self.transport_map,
            global_config_overrides=MappingProxyType({**self.global_config_overrides, **kwargs}),
            remapping_map=self.remapping_map,
            requirement_checks=self.requirement_checks,
        )

    def remappings(self, remappings: list[tuple[type[Module], str, str]]) -> "ModuleBlueprintSet":
        remappings_dict = dict(self.remapping_map)
        for module, old, new in remappings:
            remappings_dict[(module, old)] = new

        return ModuleBlueprintSet(
            blueprints=self.blueprints,
            transport_map=self.transport_map,
            global_config_overrides=self.global_config_overrides,
            remapping_map=MappingProxyType(remappings_dict),
            requirement_checks=self.requirement_checks,
        )

    def requirements(self, *checks: Callable[[], str | None]) -> "ModuleBlueprintSet":
        return ModuleBlueprintSet(
            blueprints=self.blueprints,
            transport_map=self.transport_map,
            global_config_overrides=self.global_config_overrides,
            remapping_map=self.remapping_map,
            requirement_checks=self.requirement_checks + tuple(checks),
        )

    def _get_transport_for(self, name: str, type: type) -> Any:
        transport = self.transport_map.get((name, type), None)
        if transport:
            return transport

        use_pickled = getattr(type, "lcm_encode", None) is None
        topic = f"/{name}" if self._is_name_unique(name) else f"/{short_id()}"
        transport = pLCMTransport(topic) if use_pickled else LCMTransport(topic, type)

        return transport

    @cached_property
    def _all_name_types(self) -> set[tuple[str, type]]:
        # Apply remappings to get the actual names that will be used
        result = set()
        for blueprint in self.blueprints:
            for conn in blueprint.connections:
                # Check if this connection should be remapped
                remapped_name = self.remapping_map.get((blueprint.module, conn.name), conn.name)
                result.add((remapped_name, conn.type))
        return result

    def _is_name_unique(self, name: str) -> bool:
        return sum(1 for n, _ in self._all_name_types if n == name) == 1

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

        for blueprint in self.blueprints:
            for conn in blueprint.connections:
                connection_name = self.remapping_map.get((blueprint.module, conn.name), conn.name)
                name_to_types[connection_name].add(conn.type)
                name_to_modules[connection_name].append((blueprint.module, conn.type))

        conflicts = {}
        for conn_name, types in name_to_types.items():
            if len(types) > 1:
                modules_by_type = defaultdict(list)
                for module, conn_type in name_to_modules[conn_name]:
                    modules_by_type[conn_type].append(module)
                conflicts[conn_name] = modules_by_type

        if not conflicts:
            return

        error_lines = ["Blueprint cannot start because there are conflicting connections."]
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
        for blueprint in self.blueprints:
            kwargs = {**blueprint.kwargs}
            sig = inspect.signature(blueprint.module.__init__)
            if "global_config" in sig.parameters:
                kwargs["global_config"] = global_config
            module_coordinator.deploy(blueprint.module, *blueprint.args, **kwargs)

    def _connect_transports(self, module_coordinator: ModuleCoordinator) -> None:
        # Gather all the In/Out connections with remapping applied.
        connections = defaultdict(list)
        # Track original name -> remapped name for each module
        module_conn_mapping = defaultdict(dict)  # type: ignore[var-annotated]

        for blueprint in self.blueprints:
            for conn in blueprint.connections:
                # Check if this connection should be remapped
                remapped_name = self.remapping_map.get((blueprint.module, conn.name), conn.name)
                # Store the mapping for later use
                module_conn_mapping[blueprint.module][conn.name] = remapped_name
                # Group by remapped name and type
                connections[remapped_name, conn.type].append((blueprint.module, conn.name))

        # Connect all In/Out connections by remapped name and type.
        for remapped_name, type in connections.keys():
            transport = self._get_transport_for(remapped_name, type)
            for module, original_name in connections[(remapped_name, type)]:
                instance = module_coordinator.get_instance(module)
                instance.set_transport(original_name, transport)  # type: ignore[union-attr]
                logger.info(
                    "Transport",
                    name=remapped_name,
                    original_name=original_name,
                    topic=str(getattr(transport, "topic", None)),
                    type=f"{type.__module__}.{type.__qualname__}",
                    module=module.__name__,
                    transport=transport.__class__.__name__,
                )

    def _connect_rpc_methods(self, module_coordinator: ModuleCoordinator) -> None:
        # Gather all RPC methods.
        rpc_methods = {}
        rpc_methods_dot = {}
        # Track interface methods to detect ambiguity
        interface_methods = defaultdict(list)  # interface_name.method -> [(module_class, method)]

        for blueprint in self.blueprints:
            for method_name in blueprint.module.rpcs.keys():  # type: ignore[attr-defined]
                method = getattr(module_coordinator.get_instance(blueprint.module), method_name)
                # Register under concrete class name (backward compatibility)
                rpc_methods[f"{blueprint.module.__name__}_{method_name}"] = method
                rpc_methods_dot[f"{blueprint.module.__name__}.{method_name}"] = method

                # Also register under any interface names
                for base in blueprint.module.__bases__:
                    # Check if this base is an abstract interface with the method
                    if (
                        base is not Module
                        and issubclass(base, ABC)
                        and hasattr(base, method_name)
                        and getattr(base, method_name, None) is not None
                    ):
                        interface_key = f"{base.__name__}.{method_name}"
                        interface_methods[interface_key].append((blueprint.module, method))

        # Check for ambiguity in interface methods and add non-ambiguous ones
        for interface_key, implementations in interface_methods.items():
            if len(implementations) == 1:
                rpc_methods_dot[interface_key] = implementations[0][1]

        # Fulfil method requests (so modules can call each other).
        for blueprint in self.blueprints:
            instance = module_coordinator.get_instance(blueprint.module)
            for method_name in blueprint.module.rpcs.keys():  # type: ignore[attr-defined]
                if not method_name.startswith("set_"):
                    continue
                linked_name = method_name.removeprefix("set_")
                if linked_name not in rpc_methods:
                    continue
                getattr(instance, method_name)(rpc_methods[linked_name])
            for requested_method_name in instance.get_rpc_method_names():  # type: ignore[union-attr]
                # Check if this is an ambiguous interface method
                if (
                    requested_method_name in interface_methods
                    and len(interface_methods[requested_method_name]) > 1
                ):
                    modules_str = ", ".join(
                        impl[0].__name__ for impl in interface_methods[requested_method_name]
                    )
                    raise ValueError(
                        f"Ambiguous RPC method '{requested_method_name}' requested by "
                        f"{blueprint.module.__name__}. Multiple implementations found: "
                        f"{modules_str}. Please use a concrete class name instead."
                    )

                if requested_method_name not in rpc_methods_dot:
                    continue
                instance.set_rpc_method(  # type: ignore[union-attr]
                    requested_method_name, rpc_methods_dot[requested_method_name]
                )

    def build(
        self,
        global_config: GlobalConfig | None = None,
        cli_config_overrides: Mapping[str, Any] | None = None,
    ) -> ModuleCoordinator:
        if global_config is None:
            global_config = GlobalConfig()
        global_config = global_config.model_copy(update=dict(self.global_config_overrides))
        if cli_config_overrides:
            global_config = global_config.model_copy(update=dict(cli_config_overrides))

        self._check_requirements()
        self._verify_no_name_conflicts()

        module_coordinator = ModuleCoordinator(global_config=global_config)
        module_coordinator.start()

        self._deploy_all_modules(module_coordinator, global_config)
        self._connect_transports(module_coordinator)
        self._connect_rpc_methods(module_coordinator)

        module_coordinator.start_all_modules()

        return module_coordinator


def _make_module_blueprint(
    module: type[Module], args: tuple[Any], kwargs: dict[str, Any]
) -> ModuleBlueprint:
    connections: list[ModuleConnection] = []

    # Use get_type_hints() to properly resolve string annotations.
    try:
        all_annotations = get_type_hints(module)
    except Exception:
        # Fallback to raw annotations if get_type_hints fails.
        all_annotations = {}
        for base_class in reversed(module.__mro__):
            if hasattr(base_class, "__annotations__"):
                all_annotations.update(base_class.__annotations__)

    for name, annotation in all_annotations.items():
        origin = get_origin(annotation)
        if origin not in (In, Out):
            continue
        direction = "in" if origin == In else "out"
        type_ = get_args(annotation)[0]
        connections.append(ModuleConnection(name=name, type=type_, direction=direction))  # type: ignore[arg-type]

    return ModuleBlueprint(module=module, connections=tuple(connections), args=args, kwargs=kwargs)


def create_module_blueprint(module: type[Module], *args: Any, **kwargs: Any) -> ModuleBlueprintSet:
    blueprint = _make_module_blueprint(module, args, kwargs)
    return ModuleBlueprintSet(blueprints=(blueprint,))


def autoconnect(*blueprints: ModuleBlueprintSet) -> ModuleBlueprintSet:
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

    return ModuleBlueprintSet(
        blueprints=all_blueprints,
        transport_map=MappingProxyType(all_transports),
        global_config_overrides=MappingProxyType(all_config_overrides),
        remapping_map=MappingProxyType(all_remappings),
        requirement_checks=all_requirement_checks,
    )


def _eliminate_duplicates(blueprints: list[ModuleBlueprint]) -> list[ModuleBlueprint]:
    # The duplicates are eliminated in reverse so that newer blueprints override older ones.
    seen = set()
    unique_blueprints = []
    for bp in reversed(blueprints):
        if bp.module not in seen:
            seen.add(bp.module)
            unique_blueprints.append(bp)
    return list(reversed(unique_blueprints))
