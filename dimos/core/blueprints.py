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

from dataclasses import dataclass, field
from collections import defaultdict
from functools import cached_property
import inspect
from types import MappingProxyType
from typing import Any, Literal, Mapping, get_origin, get_args

from dimos.core.module_coordinator import ModuleCoordinator
from dimos.core.global_config import GlobalConfig
from dimos.core.module import Module
from dimos.core.stream import In, Out
from dimos.core.transport import LCMTransport, pLCMTransport
from dimos.utils.generic import short_id


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

    def transports(self, transports: dict[tuple[str, type], Any]) -> "ModuleBlueprintSet":
        return ModuleBlueprintSet(
            blueprints=self.blueprints,
            transport_map=MappingProxyType({**self.transport_map, **transports}),
            global_config_overrides=self.global_config_overrides,
            remapping_map=self.remapping_map,
        )

    def global_config(self, **kwargs: Any) -> "ModuleBlueprintSet":
        return ModuleBlueprintSet(
            blueprints=self.blueprints,
            transport_map=self.transport_map,
            global_config_overrides=MappingProxyType({**self.global_config_overrides, **kwargs}),
            remapping_map=self.remapping_map,
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

    def build(self, global_config: GlobalConfig | None = None) -> ModuleCoordinator:
        if global_config is None:
            global_config = GlobalConfig()
        global_config = global_config.model_copy(update=self.global_config_overrides)

        module_coordinator = ModuleCoordinator(global_config=global_config)

        module_coordinator.start()

        # Deploy all modules.
        for blueprint in self.blueprints:
            kwargs = {**blueprint.kwargs}
            sig = inspect.signature(blueprint.module.__init__)
            if "global_config" in sig.parameters:
                kwargs["global_config"] = global_config
            module_coordinator.deploy(blueprint.module, *blueprint.args, **kwargs)

        # Gather all the In/Out connections with remapping applied.
        connections = defaultdict(list)
        # Track original name -> remapped name for each module
        module_conn_mapping = defaultdict(dict)

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
                # Use the remote method to set transport on Dask actors
                instance.set_transport(original_name, transport)

        # Gather all RPC methods.
        rpc_methods = {}
        for blueprint in self.blueprints:
            for method_name in blueprint.module.rpcs.keys():
                method = getattr(module_coordinator.get_instance(blueprint.module), method_name)
                rpc_methods[f"{blueprint.module.__name__}_{method_name}"] = method

        # Fulfil method requests (so modules can call each other).
        for blueprint in self.blueprints:
            for method_name, method in blueprint.module.rpcs.items():
                if not method_name.startswith("set_"):
                    continue
                linked_name = method_name.removeprefix("set_")
                if linked_name not in rpc_methods:
                    continue
                instance = module_coordinator.get_instance(blueprint.module)
                getattr(instance, method_name)(rpc_methods[linked_name])

        module_coordinator.start_all_modules()

        return module_coordinator


def _make_module_blueprint(
    module: type[Module], args: tuple[Any], kwargs: dict[str, Any]
) -> ModuleBlueprint:
    connections: list[ModuleConnection] = []

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
        connections.append(ModuleConnection(name=name, type=type_, direction=direction))

    return ModuleBlueprint(module=module, connections=tuple(connections), args=args, kwargs=kwargs)


def create_module_blueprint(module: type[Module], *args: Any, **kwargs: Any) -> ModuleBlueprintSet:
    blueprint = _make_module_blueprint(module, args, kwargs)
    return ModuleBlueprintSet(blueprints=(blueprint,))


def autoconnect(*blueprints: ModuleBlueprintSet) -> ModuleBlueprintSet:
    all_blueprints = tuple(_eliminate_duplicates([bp for bs in blueprints for bp in bs.blueprints]))
    all_transports = dict(sum([list(x.transport_map.items()) for x in blueprints], []))
    all_config_overrides = dict(
        sum([list(x.global_config_overrides.items()) for x in blueprints], [])
    )
    all_remappings = dict(sum([list(x.remapping_map.items()) for x in blueprints], []))

    return ModuleBlueprintSet(
        blueprints=all_blueprints,
        transport_map=MappingProxyType(all_transports),
        global_config_overrides=MappingProxyType(all_config_overrides),
        remapping_map=MappingProxyType(all_remappings),
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
