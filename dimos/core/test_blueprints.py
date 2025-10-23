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

from dimos.core.blueprints import (
    ModuleBlueprint,
    ModuleBlueprintSet,
    ModuleConnection,
    _make_module_blueprint,
)
from dimos.core.blueprints import autoconnect
from dimos.core.core import rpc
from dimos.core.global_config import GlobalConfig
from dimos.core.module import Module
from dimos.core.module_coordinator import ModuleCoordinator
from dimos.core.rpc_client import RpcCall
from dimos.core.stream import In, Out
from dimos.core.transport import LCMTransport
from dimos.protocol import pubsub


class Scratch:
    pass


class Petting:
    pass


class CatModule(Module):
    pet_cat: In[Petting]
    scratches: Out[Scratch]


class Data1:
    pass


class Data2:
    pass


class Data3:
    pass


class ModuleA(Module):
    data1: Out[Data1] = None
    data2: Out[Data2] = None

    @rpc
    def get_name(self) -> str:
        return "A, Module A"


class ModuleB(Module):
    data1: In[Data1] = None
    data2: In[Data2] = None
    data3: Out[Data3] = None

    _module_a_get_name: callable = None

    @rpc
    def set_ModuleA_get_name(self, callable: RpcCall) -> None:
        self._module_a_get_name = callable
        self._module_a_get_name.set_rpc(self.rpc)

    @rpc
    def what_is_as_name(self) -> str:
        if self._module_a_get_name is None:
            return "ModuleA.get_name not set"
        return self._module_a_get_name()


class ModuleC(Module):
    data3: In[Data3] = None


module_a = ModuleA.blueprint
module_b = ModuleB.blueprint
module_c = ModuleC.blueprint


def test_get_connection_set():
    assert _make_module_blueprint(CatModule, args=("arg1"), kwargs={"k": "v"}) == ModuleBlueprint(
        module=CatModule,
        connections=(
            ModuleConnection(name="pet_cat", type=Petting, direction="in"),
            ModuleConnection(name="scratches", type=Scratch, direction="out"),
        ),
        args=("arg1"),
        kwargs={"k": "v"},
    )


def test_autoconnect():
    blueprint_set = autoconnect(module_a(), module_b())

    assert blueprint_set == ModuleBlueprintSet(
        blueprints=(
            ModuleBlueprint(
                module=ModuleA,
                connections=(
                    ModuleConnection(name="data1", type=Data1, direction="out"),
                    ModuleConnection(name="data2", type=Data2, direction="out"),
                ),
                args=(),
                kwargs={},
            ),
            ModuleBlueprint(
                module=ModuleB,
                connections=(
                    ModuleConnection(name="data1", type=Data1, direction="in"),
                    ModuleConnection(name="data2", type=Data2, direction="in"),
                    ModuleConnection(name="data3", type=Data3, direction="out"),
                ),
                args=(),
                kwargs={},
            ),
        )
    )


def test_with_transports():
    custom_transport = LCMTransport("/custom_topic", Data1)
    blueprint_set = autoconnect(module_a(), module_b()).with_transports(
        {("data1", Data1): custom_transport}
    )

    assert ("data1", Data1) in blueprint_set.transports
    assert blueprint_set.transports[("data1", Data1)] == custom_transport


def test_with_global_config():
    blueprint_set = autoconnect(module_a(), module_b()).with_global_config(option1=True, option2=42)

    assert "option1" in blueprint_set.global_config_overrides
    assert blueprint_set.global_config_overrides["option1"] is True
    assert "option2" in blueprint_set.global_config_overrides
    assert blueprint_set.global_config_overrides["option2"] == 42


def test_build_happy_path():
    pubsub.lcm.autoconf()

    blueprint_set = autoconnect(module_a(), module_b(), module_c())

    coordinator = blueprint_set.build(GlobalConfig())

    try:
        assert isinstance(coordinator, ModuleCoordinator)

        module_a_instance = coordinator.get_instance(ModuleA)
        module_b_instance = coordinator.get_instance(ModuleB)
        module_c_instance = coordinator.get_instance(ModuleC)

        assert module_a_instance is not None
        assert module_b_instance is not None
        assert module_c_instance is not None

        assert module_a_instance.data1.transport is not None
        assert module_a_instance.data2.transport is not None
        assert module_b_instance.data1.transport is not None
        assert module_b_instance.data2.transport is not None
        assert module_b_instance.data3.transport is not None
        assert module_c_instance.data3.transport is not None

        assert module_a_instance.data1.transport.topic == module_b_instance.data1.transport.topic
        assert module_a_instance.data2.transport.topic == module_b_instance.data2.transport.topic
        assert module_b_instance.data3.transport.topic == module_c_instance.data3.transport.topic

        assert module_b_instance.what_is_as_name() == "A, Module A"

    finally:
        coordinator.stop()
