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

from dataclasses import dataclass
from collections import defaultdict
from typing import Any, get_origin, get_args
from uuid import uuid4

from dimos.core.dimos import Dimos
from dimos.core.module import Module
from dimos.core.stream import In, Out
from dimos.core.transport import LCMTransport, pLCMTransport


@dataclass(frozen=True)
class ModuleBlueprint:
    module: type[Module]
    incoming: dict[str, type]
    outgoing: dict[str, type]
    args: tuple[Any]
    kwargs: dict[str, Any]


@dataclass(frozen=True)
class ModuleBlueprintSet:
    blueprints: list[ModuleBlueprint]


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
    return ModuleBlueprintSet(blueprints=[blueprint])


def autoconnect(*blueprints: ModuleBlueprintSet) -> ModuleBlueprintSet:
    all_blueprints = sum([x.blueprints for x in blueprints], [])
    return ModuleBlueprintSet(blueprints=all_blueprints)


def build_blueprint(blueprints: ModuleBlueprintSet, n: int | None = None) -> Dimos:
    dimos = Dimos(n=n)

    dimos.start()

    for blueprint in blueprints.blueprints:
        dimos.deploy(blueprint.module, *blueprint.args, **blueprint.kwargs)

    incoming = defaultdict(list)
    outgoing = defaultdict(list)

    for blueprint in blueprints.blueprints:
        for name, type in blueprint.incoming.items():
            incoming[(name, type)].append(blueprint.module)
        for name, type in blueprint.outgoing.items():
            outgoing[(name, type)].append(blueprint.module)

    for name, type in set(incoming.keys()).union(outgoing.keys()):
        print("-" * 100)
        topic = f"/{uuid4()}"
        use_pickled = "lcm_encode" not in type.__dict__
        for module in incoming[(name, type)] + outgoing[(name, type)]:
            instance = dimos.get_instance(module)
            transport = pLCMTransport(type) if use_pickled else LCMTransport(topic, type)
            getattr(instance, name).transport = transport
            print(module.__name__, name, type.__name__, topic)

    rpc_methods = {}
    for blueprint in blueprints.blueprints:
        for method_name, method in blueprint.module.rpcs.items():
            rpc_methods[f"{blueprint.module.__name__}_{method_name}"] = method

    print("x" * 100)
    for blueprint in blueprints.blueprints:
        for method_name, method in blueprint.module.rpcs.items():
            if not method_name.startswith("set_"):
                continue
            linked_name = method_name.removeprefix("set_")
            if linked_name not in rpc_methods:
                continue
            print("linking", blueprint.module.__name__, linked_name)
            instance = dimos.get_instance(blueprint.module)
            getattr(instance, method_name)(rpc_methods[linked_name])
    print("x" * 100)

    dimos.start_all_modules()

    return dimos
