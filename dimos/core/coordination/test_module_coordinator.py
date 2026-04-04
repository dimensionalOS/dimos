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
    FutureModuleIn,
    FutureModuleOut,
)
from dimos.core.coordination.blueprints import (
    DisabledModuleProxy,
    autoconnect,
)
from dimos.core.coordination.module_coordinator import (
    ModuleCoordinator,
    _all_name_types,
    _check_requirements,
    _verify_no_conflicts_with_existing,
    _verify_no_name_conflicts,
)
from dimos.core.core import rpc
from dimos.core.global_config import GlobalConfig
from dimos.core.module import Module
from dimos.core.stream import In, Out
from dimos.msgs.sensor_msgs.Image import Image
from dimos.spec.utils import Spec

# Disable Rerun for tests (prevents viewer spawn and gRPC flush errors)
_BUILD_WITHOUT_RERUN = {
    "cli_config_overrides": {"viewer": "none"},
}


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

    module_a: ModuleA

    @rpc
    def what_is_as_name(self) -> str:
        return self.module_a.get_name()


class ModuleC(Module):
    data3: In[Data3]


class SourceModule(Module):
    color_image: Out[Data1]


class TargetModule(Module):
    remapped_data: In[Data1]


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


@pytest.mark.slow
def test_build_happy_path() -> None:
    blueprint_set = autoconnect(ModuleA.blueprint(), ModuleB.blueprint(), ModuleC.blueprint())

    coordinator = ModuleCoordinator.build(blueprint_set, **_BUILD_WITHOUT_RERUN)

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
        _verify_no_name_conflicts(blueprint_set)
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
        _verify_no_name_conflicts(blueprint_set)
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
        _verify_no_name_conflicts(blueprint_set)
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
    _verify_no_name_conflicts(blueprint_set_remapped)


@pytest.mark.slow
def test_remapping() -> None:
    """Test that remapping streams works correctly."""

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
    all_names = _all_name_types(blueprint_set)
    assert ("remapped_data", Data1) in all_names
    # The original name shouldn't be in the name types since it's remapped
    assert ("color_image", Data1) not in all_names

    # Build and verify streams work
    coordinator = ModuleCoordinator.build(blueprint_set, **_BUILD_WITHOUT_RERUN)

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


@pytest.mark.slow
def test_future_annotations_autoconnect() -> None:
    """Test that autoconnect works with modules using `from __future__ import annotations`."""

    blueprint_set = autoconnect(FutureModuleOut.blueprint(), FutureModuleIn.blueprint())

    coordinator = ModuleCoordinator.build(blueprint_set, **_BUILD_WITHOUT_RERUN)

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


@pytest.mark.slow
def test_module_ref_direct() -> None:
    coordinator = ModuleCoordinator.build(
        autoconnect(
            Calculator1.blueprint(),
            Mod1.blueprint(),
        ),
        **_BUILD_WITHOUT_RERUN,
    )

    try:
        mod1 = coordinator.get_instance(Mod1)
        assert mod1 is not None
        assert mod1.calc.compute1(2, 3) == 5
        assert mod1.calc.compute2(1.5, 2.5) == 4.0
    finally:
        coordinator.stop()


@pytest.mark.slow
def test_module_ref_spec() -> None:
    coordinator = ModuleCoordinator.build(
        autoconnect(
            Calculator1.blueprint(),
            Mod2.blueprint(),
        ),
        **_BUILD_WITHOUT_RERUN,
    )

    try:
        mod2 = coordinator.get_instance(Mod2)
        assert mod2 is not None
        assert mod2.calc.compute1(4, 5) == 9
        assert mod2.calc.compute2(3.0, 0.5) == 3.5
    finally:
        coordinator.stop()


@pytest.mark.slow
def test_disabled_modules_are_skipped_during_build() -> None:
    blueprint_set = autoconnect(
        ModuleA.blueprint(), ModuleB.blueprint(), ModuleC.blueprint()
    ).disabled_modules(ModuleC)

    coordinator = ModuleCoordinator.build(blueprint_set, **_BUILD_WITHOUT_RERUN)

    try:
        assert coordinator.get_instance(ModuleA) is not None
        assert coordinator.get_instance(ModuleB) is not None

        assert coordinator.get_instance(ModuleC) is None
    finally:
        coordinator.stop()


@pytest.mark.slow
def test_disabled_module_ref_gets_noop_proxy() -> None:
    blueprint_set = autoconnect(
        Calculator1.blueprint(),
        Mod2.blueprint(),
    ).disabled_modules(Calculator1)

    coordinator = ModuleCoordinator.build(blueprint_set, **_BUILD_WITHOUT_RERUN)

    try:
        mod2 = coordinator.get_instance(Mod2)
        assert mod2 is not None
        # The proxy should be a _DisabledModuleProxy, not a real Calculator.
        assert isinstance(mod2.calc, DisabledModuleProxy)
        # Calling methods on it should return None (no-op).
        assert mod2.calc.compute1(1, 2) is None
    finally:
        coordinator.stop()


@pytest.mark.slow
def test_module_ref_remap_ambiguous() -> None:
    coordinator = ModuleCoordinator.build(
        autoconnect(
            Calculator1.blueprint(),
            Calculator2.blueprint(),
            Mod2.blueprint(),
        ).remappings(
            [
                (Mod2, "calc", Calculator1),
            ]
        ),
        **_BUILD_WITHOUT_RERUN,
    )

    try:
        mod2 = coordinator.get_instance(Mod2)
        assert mod2 is not None
        assert mod2.calc.compute1(2, 3) == 5
        assert mod2.calc.compute2(2.0, 3.0) == 5.0
    finally:
        coordinator.stop()


@pytest.mark.slow
def test_load_blueprint_basic(dynamic_coordinator) -> None:
    """load_blueprint deploys, wires and starts modules the same way build() does."""
    bp = autoconnect(ModuleA.blueprint(), ModuleB.blueprint(), ModuleC.blueprint())
    dynamic_coordinator.load_blueprint(bp)

    assert dynamic_coordinator.get_instance(ModuleA) is not None
    assert dynamic_coordinator.get_instance(ModuleB) is not None
    assert dynamic_coordinator.get_instance(ModuleC) is not None

    a = dynamic_coordinator.get_instance(ModuleA)
    b = dynamic_coordinator.get_instance(ModuleB)
    c = dynamic_coordinator.get_instance(ModuleC)

    # Streams wired.
    assert a.data1.transport is not None
    assert b.data1.transport is not None
    assert a.data1.transport.topic == b.data1.transport.topic
    assert b.data3.transport.topic == c.data3.transport.topic

    # Module ref wired.
    assert b.what_is_as_name() == "A, Module A"


@pytest.mark.slow
def test_load_blueprint_twice(dynamic_coordinator) -> None:
    """Two sequential load_blueprint calls share transports for matching streams."""
    dynamic_coordinator.load_blueprint(ModuleA.blueprint())
    dynamic_coordinator.load_blueprint(autoconnect(ModuleB.blueprint(), ModuleC.blueprint()))

    a = dynamic_coordinator.get_instance(ModuleA)
    b = dynamic_coordinator.get_instance(ModuleB)
    c = dynamic_coordinator.get_instance(ModuleC)

    assert a is not None
    assert b is not None
    assert c is not None

    # A's Out[Data1] and B's In[Data1] should share a transport.
    assert a.data1.transport.topic == b.data1.transport.topic
    assert a.data2.transport.topic == b.data2.transport.topic
    assert b.data3.transport.topic == c.data3.transport.topic


@pytest.mark.slow
def test_load_module_convenience(dynamic_coordinator) -> None:
    """load_module is a shorthand for load_blueprint(cls.blueprint())."""
    dynamic_coordinator.load_module(ModuleA)
    assert dynamic_coordinator.get_instance(ModuleA) is not None
    assert dynamic_coordinator.get_instance(ModuleA).data1.transport is not None


@pytest.mark.slow
def test_load_blueprint_module_ref_to_existing(dynamic_coordinator) -> None:
    """A module loaded in a second blueprint can reference one from the first."""
    dynamic_coordinator.load_blueprint(Calculator1.blueprint())
    dynamic_coordinator.load_blueprint(Mod2.blueprint())

    mod2 = dynamic_coordinator.get_instance(Mod2)
    assert mod2 is not None
    assert mod2.calc.compute1(2, 3) == 5
    assert mod2.calc.compute2(1.5, 2.5) == 4.0


def test_load_blueprint_conflict_with_existing() -> None:
    """Loading a blueprint whose stream name clashes (different type) raises ValueError."""
    from dimos.core.transport import pLCMTransport

    registry: dict[tuple[str, type], object] = {("data1", Data1): pLCMTransport("/data1")}

    class ConflictModule(Module):
        data1: In[Data2]  # same name, different type

    bp = ConflictModule.blueprint()
    with pytest.raises(ValueError, match="data1"):
        _verify_no_conflicts_with_existing(bp, registry)  # type: ignore[arg-type]


@pytest.mark.slow
def test_load_blueprint_duplicate_module_raises(dynamic_coordinator) -> None:
    """Loading a module that is already deployed raises ValueError."""
    dynamic_coordinator.load_blueprint(ModuleA.blueprint())
    with pytest.raises(ValueError, match="already deployed"):
        dynamic_coordinator.load_blueprint(ModuleA.blueprint())


class ModWithOptionalRef(Module):
    stream1: In[Image]
    calc: CalculatorSpec | None = None  # type: ignore[assignment]

    @rpc
    def start(self) -> None: ...

    @rpc
    def stop(self) -> None: ...


@pytest.fixture
def build_coordinator():
    coordinators = []

    def _build(blueprint):
        c = ModuleCoordinator.build(blueprint, **_BUILD_WITHOUT_RERUN)
        coordinators.append(c)
        return c

    yield _build

    for c in reversed(coordinators):
        c.stop()


@pytest.fixture
def dynamic_coordinator():
    mc = ModuleCoordinator(g=GlobalConfig(n_workers=0, viewer="none"))
    mc.start()
    yield mc
    mc.stop()


@pytest.mark.slow
def test_optional_module_ref_with_provider(build_coordinator) -> None:
    """An optional ref resolves normally when a provider is present."""
    coordinator = build_coordinator(
        autoconnect(
            Calculator1.blueprint(),
            ModWithOptionalRef.blueprint(),
        ),
    )

    mod = coordinator.get_instance(ModWithOptionalRef)
    assert mod is not None
    assert mod.calc.compute1(2, 3) == 5


@pytest.mark.slow
def test_optional_module_ref_without_provider(build_coordinator) -> None:
    """An optional ref is silently skipped when no provider is in the blueprint."""
    coordinator = build_coordinator(ModWithOptionalRef.blueprint())

    mod = coordinator.get_instance(ModWithOptionalRef)
    assert mod is not None


@pytest.mark.slow
def test_load_blueprint_auto_scales_empty_pool(dynamic_coordinator) -> None:
    """A coordinator with 0 initial workers auto-adds workers on load_blueprint."""
    dynamic_coordinator.load_blueprint(ModuleA.blueprint())
    assert dynamic_coordinator.get_instance(ModuleA) is not None
    assert dynamic_coordinator.get_instance(ModuleA).data1.transport is not None


def test_check_requirements_failure(mocker) -> None:
    """A failing requirement check causes sys.exit."""
    mocker.patch("dimos.core.coordination.module_coordinator.sys.exit", side_effect=SystemExit(1))

    bp = ModuleA.blueprint().requirements(lambda: "missing GPU driver")

    with pytest.raises(SystemExit):
        _check_requirements(bp)
