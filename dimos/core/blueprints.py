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
from types import MappingProxyType
from typing import Any, Mapping, get_origin, get_args

from dimos.core.dimos import Dimos
from dimos.core.module import Module
from dimos.core.stream import In, Out
from dimos.core.transport import LCMTransport, pLCMTransport
from dimos.utils.generic import short_id


@dataclass(frozen=True)
class ModuleBlueprint:
    module: type[Module]
    incoming: dict[str, type]
    outgoing: dict[str, type]
    args: tuple[Any]
    kwargs: dict[str, Any]


@dataclass(frozen=True)
class ModuleBlueprintSet:
    blueprints: tuple[ModuleBlueprint, ...]
    # TODO: Replace Any
    transports: Mapping[tuple[str, type], Any] = field(default_factory=lambda: MappingProxyType({}))

    def with_transports(self, transports: dict[tuple[str, type], Any]) -> "ModuleBlueprintSet":
        return ModuleBlueprintSet(
            blueprints=self.blueprints,
            transports=MappingProxyType({**self.transports, **transports}),
        )

    def _get_transport_for(self, name: str, type: type) -> Any:
        transport = self.transports.get((name, type), None)
        if transport:
            return transport

        use_pickled = "lcm_encode" not in type.__dict__
        topic = f"/{name}" if self._is_name_unique(name) else f"/{short_id()}"
        transport = pLCMTransport(topic) if use_pickled else LCMTransport(topic, type)

        return transport

    @cached_property
    def _all_name_types(self) -> set[tuple[str, type]]:
        all_name_types = set()
        for blueprint in self.blueprints:
            for name, type in blueprint.incoming.items():
                all_name_types.add((name, type))
            for name, type in blueprint.outgoing.items():
                all_name_types.add((name, type))
        return all_name_types

    def _is_name_unique(self, name: str) -> bool:
        return sum(1 for n, _ in self._all_name_types if n == name) == 1

    def build(self, n: int | None = None) -> Dimos:
        dimos = Dimos(n=n)

        dimos.start()

        # Deploy all modules.
        for blueprint in self.blueprints:
            dimos.deploy(blueprint.module, *blueprint.args, **blueprint.kwargs)

        # Gather all the In/Out connections.
        incoming = defaultdict(list)
        outgoing = defaultdict(list)
        for blueprint in self.blueprints:
            for name, type in blueprint.incoming.items():
                incoming[(name, type)].append(blueprint.module)
            for name, type in blueprint.outgoing.items():
                outgoing[(name, type)].append(blueprint.module)

        # Connect all In/Out connections by name and type.
        for name, type in set(incoming.keys()).union(outgoing.keys()):
            transport = self._get_transport_for(name, type)
            for module in incoming[(name, type)] + outgoing[(name, type)]:
                instance = dimos.get_instance(module)
                getattr(instance, name).transport = transport

        # Gather all RPC methods.
        rpc_methods = {}
        for blueprint in self.blueprints:
            for method_name, method in blueprint.module.rpcs.items():
                rpc_methods[f"{blueprint.module.__name__}_{method_name}"] = method

        # Fulfil method requests (so modules can call each other).
        for blueprint in self.blueprints:
            for method_name, method in blueprint.module.rpcs.items():
                if not method_name.startswith("set_"):
                    continue
                linked_name = method_name.removeprefix("set_")
                if linked_name not in rpc_methods:
                    continue
                instance = dimos.get_instance(blueprint.module)
                getattr(instance, method_name)(rpc_methods[linked_name])

        dimos.start_all_modules()

        return dimos


def make_module_blueprint(
    module: type[Module], args: tuple[Any], kwargs: dict[str, Any]
) -> ModuleBlueprint:
    incoming: dict[str, type] = {}
    outgoing: dict[str, type] = {}

    all_annotations = {}
    for base_class in reversed(module.__mro__):
        if hasattr(base_class, "__annotations__"):
            all_annotations.update(base_class.__annotations__)

    for name, annotation in all_annotations.items():
        origin = get_origin(annotation)
        if origin not in (In, Out):
            continue
        dict_ = incoming if origin == In else outgoing
        dict_[name] = get_args(annotation)[0]

    return ModuleBlueprint(
        module=module, incoming=incoming, outgoing=outgoing, args=args, kwargs=kwargs
    )


def create_module_blueprint(module: type[Module], *args: Any, **kwargs: Any) -> ModuleBlueprintSet:
    blueprint = make_module_blueprint(module, args, kwargs)
    return ModuleBlueprintSet(blueprints=(blueprint,))


def autoconnect(*blueprints: ModuleBlueprintSet) -> ModuleBlueprintSet:
    all_blueprints = tuple(bp for bs in blueprints for bp in bs.blueprints)
    all_transports = dict(sum([list(x.transports.items()) for x in blueprints], []))
    return ModuleBlueprintSet(
        blueprints=all_blueprints, transports=MappingProxyType(all_transports)
    )
