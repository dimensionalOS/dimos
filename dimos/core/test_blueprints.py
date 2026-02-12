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

from typing import Protocol

import pytest

from dimos.core._test_future_annotations_helper import (
    FutureData,
    FutureModuleIn,
    FutureModuleOut,
)
from dimos.core.blueprints import (
    Blueprint,
    StreamRef,
    _BlueprintAtom,
    autoconnect,
)
from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.module_coordinator import ModuleCoordinator
from dimos.core.rpc_client import RpcCall
from dimos.core.stream import In, Out
from dimos.core.transport import LCMTransport
from dimos.msgs.sensor_msgs import Image
from dimos.protocol import pubsub
from dimos.spec.utils import Spec

# Disable Rerun for tests (prevents viewer spawn and gRPC flush errors)
_BUILD_WITHOUT_RERUN = {
    "cli_config_overrides": {"viewer_backend": "none"},
}


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
    assert _BlueprintAtom.create(CatModule, args=("arg1",), kwargs={"k": "v"}) == _BlueprintAtom(
        module=CatModule,
        streams=(
            StreamRef(name="pet_cat", type=Petting, direction="in"),
            StreamRef(name="scratches", type=Scratch, direction="out"),
        ),
        module_refs=(),
        args=("arg1",),
        kwargs={"k": "v"},
    )


def test_autoconnect() -> None:
    blueprint_set = autoconnect(module_a(), module_b())

    assert blueprint_set == Blueprint(
        blueprints=(
            _BlueprintAtom(
                module=ModuleA,
                streams=(
                    StreamRef(name="data1", type=Data1, direction="out"),
                    StreamRef(name="data2", type=Data2, direction="out"),
                ),
                module_refs=(),
                args=(),
                kwargs={},
            ),
            _BlueprintAtom(
                module=ModuleB,
                streams=(
                    StreamRef(name="data1", type=Data1, direction="in"),
                    StreamRef(name="data2", type=Data2, direction="in"),
                    StreamRef(name="data3", type=Data3, direction="out"),
                ),
                module_refs=(),
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


@pytest.mark.integration
def test_build_happy_path() -> None:
    pubsub.lcm.autoconf()

    blueprint_set = autoconnect(module_a(), module_b(), module_c())

    coordinator = blueprint_set.build(**_BUILD_WITHOUT_RERUN)

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
        assert "Blueprint cannot start because there are conflicting streams" in error_message
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
        assert "Blueprint cannot start because there are conflicting streams" in error_message
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


@pytest.mark.integration
def test_remapping() -> None:
    """Test that remapping streams works correctly."""
    pubsub.lcm.autoconf()

    # Define test modules with streams that will be remapped
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

    # Build and verify streams work
    coordinator = blueprint_set.build(**_BUILD_WITHOUT_RERUN)

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
    This test verifies that _BlueprintAtom.create properly resolves string annotations
    to the actual In/Out types.
    """

    # Test that streams are properly extracted from modules with future annotations
    out_blueprint = _BlueprintAtom.create(FutureModuleOut, args=(), kwargs={})
    assert len(out_blueprint.streams) == 1
    assert out_blueprint.streams[0] == StreamRef(name="data", type=FutureData, direction="out")

    in_blueprint = _BlueprintAtom.create(FutureModuleIn, args=(), kwargs={})
    assert len(in_blueprint.streams) == 1
    assert in_blueprint.streams[0] == StreamRef(name="data", type=FutureData, direction="in")


@pytest.mark.integration
def test_future_annotations_autoconnect() -> None:
    """Test that autoconnect works with modules using `from __future__ import annotations`."""

    blueprint_set = autoconnect(FutureModuleOut.blueprint(), FutureModuleIn.blueprint())

    coordinator = blueprint_set.build(**_BUILD_WITHOUT_RERUN)

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


# ModuleRef / RPC tests
class CalculatorSpec(Spec, Protocol):
    @rpc
    def compute1(self, a: int, b: int) -> int: ...

    @rpc
    def compute2(self, a: float, b: float) -> float: ...


class Calculator1(Module):
    @rpc
    def compute1(self, a: int, b: int) -> int:
        return a + b

    @rpc
    def compute2(self, a: float, b: float) -> float:
        return a + b

    @rpc
    def start(self) -> None: ...

    @rpc
    def stop(self) -> None: ...


class Calculator2(Module):
    @rpc
    def compute1(self, a: int, b: int) -> int:
        return a * b

    @rpc
    def compute2(self, a: float, b: float) -> float:
        return a * b

    @rpc
    def start(self) -> None: ...

    @rpc
    def stop(self) -> None: ...


# link to a specific module
class Mod1(Module):
    stream1: In[Image]
    calc: Calculator1

    @rpc
    def start(self) -> None:
        _ = self.calc.compute1

    @rpc
    def stop(self) -> None: ...


# link to any module that implements a spec (Autoconnect will handle it)
class Mod2(Module):
    stream1: In[Image]
    calc: CalculatorSpec

    @rpc
    def start(self) -> None:
        _ = self.calc.compute1

    @rpc
    def stop(self) -> None: ...


@pytest.mark.integration
def test_module_ref_direct() -> None:
    coordinator = autoconnect(
        Calculator1.blueprint(),
        Mod1.blueprint(),
    ).build(**_BUILD_WITHOUT_RERUN)

    try:
        mod1 = coordinator.get_instance(Mod1)
        assert mod1 is not None
        assert mod1.calc.compute1(2, 3) == 5
        assert mod1.calc.compute2(1.5, 2.5) == 4.0
    finally:
        coordinator.stop()


@pytest.mark.integration
def test_module_ref_spec() -> None:
    coordinator = autoconnect(
        Calculator1.blueprint(),
        Mod2.blueprint(),
    ).build(**_BUILD_WITHOUT_RERUN)

    try:
        mod2 = coordinator.get_instance(Mod2)
        assert mod2 is not None
        assert mod2.calc.compute1(4, 5) == 9
        assert mod2.calc.compute2(3.0, 0.5) == 3.5
    finally:
        coordinator.stop()


@pytest.mark.integration
def test_module_ref_remap_ambiguous() -> None:
    coordinator = (
        autoconnect(
            Calculator1.blueprint(),
            Calculator2.blueprint(),
            Mod2.blueprint(),
        )
        .remappings(
            [
                (Mod2, "calc", Calculator1),
            ]
        )
        .build(**_BUILD_WITHOUT_RERUN)
    )

    try:
        mod2 = coordinator.get_instance(Mod2)
        assert mod2 is not None
        assert mod2.calc.compute1(2, 3) == 5
        assert mod2.calc.compute2(2.0, 3.0) == 5.0
    finally:
        coordinator.stop()
