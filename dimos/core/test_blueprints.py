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

import pytest

from dimos.core._test_future_annotations_helper import (
    FutureData,
    FutureModuleIn,
    FutureModuleOut,
)
from dimos.core.blueprints import (
    ModuleBlueprint,
    ModuleBlueprintSet,
    ModuleConnection,
    _make_module_blueprint,
    autoconnect,
)
from dimos.core.core import rpc
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
    data1: Out[Data1]
    data2: Out[Data2]

    @rpc
    def get_name(self) -> str:
        return "A, Module A"


class ModuleB(Module):
    data1: In[Data1]
    data2: In[Data2]
    data3: Out[Data3]

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
    data3: In[Data3]


module_a = ModuleA.blueprint
module_b = ModuleB.blueprint
module_c = ModuleC.blueprint


def test_get_connection_set() -> None:
    assert _make_module_blueprint(CatModule, args=("arg1"), kwargs={"k": "v"}) == ModuleBlueprint(
        module=CatModule,
        connections=(
            ModuleConnection(name="pet_cat", type=Petting, direction="in"),
            ModuleConnection(name="scratches", type=Scratch, direction="out"),
        ),
        args=("arg1"),
        kwargs={"k": "v"},
    )


def test_autoconnect() -> None:
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


def test_transports() -> None:
    custom_transport = LCMTransport("/custom_topic", Data1)
    blueprint_set = autoconnect(module_a(), module_b()).transports(
        {("data1", Data1): custom_transport}
    )

    assert ("data1", Data1) in blueprint_set.transport_map
    assert blueprint_set.transport_map[("data1", Data1)] == custom_transport


def test_global_config() -> None:
    blueprint_set = autoconnect(module_a(), module_b()).global_config(option1=True, option2=42)

    assert "option1" in blueprint_set.global_config_overrides
    assert blueprint_set.global_config_overrides["option1"] is True
    assert "option2" in blueprint_set.global_config_overrides
    assert blueprint_set.global_config_overrides["option2"] == 42


def test_build_happy_path() -> None:
    pubsub.lcm.autoconf()

    blueprint_set = autoconnect(module_a(), module_b(), module_c())

    coordinator = blueprint_set.build()

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


def test_name_conflicts_are_reported() -> None:
    class ModuleA(Module):
        shared_data: Out[Data1]

    class ModuleB(Module):
        shared_data: In[Data2]

    blueprint_set = autoconnect(ModuleA.blueprint(), ModuleB.blueprint())

    try:
        blueprint_set._verify_no_name_conflicts()
        pytest.fail("Expected ValueError to be raised")
    except ValueError as e:
        error_message = str(e)
        assert "Blueprint cannot start because there are conflicting connections" in error_message
        assert "'shared_data' has conflicting types" in error_message
        assert "Data1 in ModuleA" in error_message
        assert "Data2 in ModuleB" in error_message


def test_multiple_name_conflicts_are_reported() -> None:
    class Module1(Module):
        sensor_data: Out[Data1]
        control_signal: Out[Data2]

    class Module2(Module):
        sensor_data: In[Data2]
        control_signal: In[Data3]

    blueprint_set = autoconnect(Module1.blueprint(), Module2.blueprint())

    try:
        blueprint_set._verify_no_name_conflicts()
        pytest.fail("Expected ValueError to be raised")
    except ValueError as e:
        error_message = str(e)
        assert "Blueprint cannot start because there are conflicting connections" in error_message
        assert "'sensor_data' has conflicting types" in error_message
        assert "'control_signal' has conflicting types" in error_message


def test_that_remapping_can_resolve_conflicts() -> None:
    class Module1(Module):
        data: Out[Data1]

    class Module2(Module):
        data: Out[Data2]  # Would conflict with Module1.data

    class Module3(Module):
        data1: In[Data1]
        data2: In[Data2]

    # Without remapping, should raise conflict error
    blueprint_set = autoconnect(Module1.blueprint(), Module2.blueprint(), Module3.blueprint())

    try:
        blueprint_set._verify_no_name_conflicts()
        pytest.fail("Expected ValueError due to conflict")
    except ValueError as e:
        assert "'data' has conflicting types" in str(e)

    # With remapping to resolve the conflict
    blueprint_set_remapped = autoconnect(
        Module1.blueprint(), Module2.blueprint(), Module3.blueprint()
    ).remappings(
        [
            (Module1, "data", "data1"),
            (Module2, "data", "data2"),
        ]
    )

    # Should not raise any exception after remapping
    blueprint_set_remapped._verify_no_name_conflicts()


def test_remapping() -> None:
    """Test that remapping connections works correctly."""
    pubsub.lcm.autoconf()

    # Define test modules with connections that will be remapped
    class SourceModule(Module):
        color_image: Out[Data1]  # Will be remapped to 'remapped_data'

    class TargetModule(Module):
        remapped_data: In[Data1]  # Receives the remapped connection

    # Create blueprint with remapping
    blueprint_set = autoconnect(
        SourceModule.blueprint(),
        TargetModule.blueprint(),
    ).remappings(
        [
            (SourceModule, "color_image", "remapped_data"),
        ]
    )

    # Verify remappings are stored correctly
    assert (SourceModule, "color_image") in blueprint_set.remapping_map
    assert blueprint_set.remapping_map[(SourceModule, "color_image")] == "remapped_data"

    # Verify that remapped names are used in name resolution
    assert ("remapped_data", Data1) in blueprint_set._all_name_types
    # The original name shouldn't be in the name types since it's remapped
    assert ("color_image", Data1) not in blueprint_set._all_name_types

    # Build and verify connections work
    coordinator = blueprint_set.build()

    try:
        source_instance = coordinator.get_instance(SourceModule)
        target_instance = coordinator.get_instance(TargetModule)

        assert source_instance is not None
        assert target_instance is not None

        # Both should have transports set
        assert source_instance.color_image.transport is not None
        assert target_instance.remapped_data.transport is not None

        # They should be using the same transport (connected)
        assert (
            source_instance.color_image.transport.topic
            == target_instance.remapped_data.transport.topic
        )

        # The topic should be /remapped_data since that's the remapped name
        assert target_instance.remapped_data.transport.topic == "/remapped_data"

    finally:
        coordinator.stop()


def test_future_annotations_support() -> None:
    """Test that modules using `from __future__ import annotations` work correctly.

    PEP 563 (future annotations) stores annotations as strings instead of actual types.
    This test verifies that _make_module_blueprint properly resolves string annotations
    to the actual In/Out types.
    """

    # Test that connections are properly extracted from modules with future annotations
    out_blueprint = _make_module_blueprint(FutureModuleOut, args=(), kwargs={})
    assert len(out_blueprint.connections) == 1
    assert out_blueprint.connections[0] == ModuleConnection(
        name="data", type=FutureData, direction="out"
    )

    in_blueprint = _make_module_blueprint(FutureModuleIn, args=(), kwargs={})
    assert len(in_blueprint.connections) == 1
    assert in_blueprint.connections[0] == ModuleConnection(
        name="data", type=FutureData, direction="in"
    )


def test_future_annotations_autoconnect() -> None:
    """Test that autoconnect works with modules using `from __future__ import annotations`."""

    blueprint_set = autoconnect(FutureModuleOut.blueprint(), FutureModuleIn.blueprint())

    coordinator = blueprint_set.build()

    try:
        out_instance = coordinator.get_instance(FutureModuleOut)
        in_instance = coordinator.get_instance(FutureModuleIn)

        assert out_instance is not None
        assert in_instance is not None

        # Both should have transports set
        assert out_instance.data.transport is not None
        assert in_instance.data.transport is not None

        # They should be connected via the same transport
        assert out_instance.data.transport.topic == in_instance.data.transport.topic

    finally:
        coordinator.stop()
