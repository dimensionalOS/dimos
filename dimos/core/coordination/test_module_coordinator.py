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

from dataclasses import dataclass
import os
from pathlib import Path
import sys
from types import MappingProxyType
from typing import Protocol

import psutil
import pytest

from dimos.core._test_future_annotations_helper import (
    FutureModuleIn,
    FutureModuleOut,
)
from dimos.core.coordination.blueprints import (
    DisabledModuleProxy,
    autoconnect,
)
from dimos.core.coordination.coordinator_rpc import CoordinatorRPC
from dimos.core.coordination.module_coordinator import (
    ModuleCoordinator,
    _all_name_types,
    _check_requirements,
    _resolve_module_plans,
    _verify_no_conflicts_with_existing,
    _verify_no_name_conflicts,
    _verify_stream_remappings,
)
from dimos.core.coordination.worker_manager_python import WorkerManagerPython
from dimos.core.core import rpc
from dimos.core.global_config import GlobalConfig
from dimos.core.module import Module, ModuleConfig, ModuleIOContract, StreamDecl
from dimos.core.runtime_environment import (
    MissingPythonProjectFileError,
    PythonLaunchMaterial,
    PythonProjectRuntimeEnvironment,
    PythonVenvRuntimeEnvironment,
    RuntimeEnvironment,
)
from dimos.core.stream import In, Out
from dimos.msgs.sensor_msgs.Image import Image
from dimos.spec.utils import Spec

# Disable Rerun for tests (prevents viewer spawn and gRPC flush errors)
_BUILD_WITHOUT_RERUN = MappingProxyType(
    {
        "g": {"viewer": "none"},
    }
)


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


class DynamicIOConfig(ModuleConfig):
    emit_data2: bool = False


class ConfiguredIOModule(Module):
    config: DynamicIOConfig  # type: ignore[assignment]

    @classmethod
    def io_contract(cls, config: DynamicIOConfig) -> ModuleIOContract:
        stream_type = Data2 if config.emit_data2 else Data1
        return ModuleIOContract(
            streams=(StreamDecl(name="configured", type=stream_type, direction="out"),)
        )


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


def test_build_happy_path() -> None:
    blueprint_set = autoconnect(ModuleA.blueprint(), ModuleB.blueprint(), ModuleC.blueprint())

    coordinator = ModuleCoordinator.build(blueprint_set, _BUILD_WITHOUT_RERUN.copy())

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


def test_resolved_module_plans_honor_blueprint_args_before_wiring() -> None:
    blueprint_set = ConfiguredIOModule.blueprint()

    default_plans = _resolve_module_plans(blueprint_set, GlobalConfig(viewer="none"), {})
    override_plans = _resolve_module_plans(
        blueprint_set,
        GlobalConfig(viewer="none"),
        {ConfiguredIOModule.name: {"emit_data2": True}},
    )

    assert default_plans[0].streams == (StreamDecl(name="configured", type=Data1, direction="out"),)
    assert override_plans[0].streams == (
        StreamDecl(name="configured", type=Data2, direction="out"),
    )
    assert override_plans[0].final_kwargs["emit_data2"] is True


def test_verify_stream_remappings_uses_resolved_io_contract() -> None:
    blueprint_set = ConfiguredIOModule.blueprint().remappings(
        [(ConfiguredIOModule, "configured", "renamed")]
    )
    plans = _resolve_module_plans(blueprint_set, GlobalConfig(viewer="none"), {})

    _verify_stream_remappings(blueprint_set, plans)
    assert _all_name_types(blueprint_set, plans) == {("renamed", Data1)}

    stale_blueprint = ConfiguredIOModule.blueprint().remappings(
        [(ConfiguredIOModule, "missing", "renamed")]
    )
    stale_plans = _resolve_module_plans(stale_blueprint, GlobalConfig(viewer="none"), {})
    with pytest.raises(ValueError, match="absent from the resolved IO contract"):
        _verify_stream_remappings(stale_blueprint, stale_plans)


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
    coordinator = ModuleCoordinator.build(blueprint_set, _BUILD_WITHOUT_RERUN.copy())

    try:
        source_instance = coordinator.get_instance(SourceModule)
        target_instance = coordinator.get_instance(TargetModule)

        assert source_instance is not None
        assert target_instance is not None

        # Both should have transports set
        assert source_instance.color_image.transport is not None
        assert target_instance.remapped_data.transport is not None
        assert set(source_instance.outputs) == {"color_image"}
        assert source_instance.outputs["color_image"].name == "color_image"

        # They should be using the same transport (connected)
        assert (
            source_instance.color_image.transport.topic
            == target_instance.remapped_data.transport.topic
        )

        # The topic should be /remapped_data since that's the remapped name
        assert target_instance.remapped_data.transport.topic == "/remapped_data"

    finally:
        coordinator.stop()


def test_future_annotations_autoconnect() -> None:
    """Test that autoconnect works with modules using `from __future__ import annotations`."""

    blueprint_set = autoconnect(FutureModuleOut.blueprint(), FutureModuleIn.blueprint())

    coordinator = ModuleCoordinator.build(blueprint_set, _BUILD_WITHOUT_RERUN.copy())

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


def test_module_ref_direct() -> None:
    coordinator = ModuleCoordinator.build(
        autoconnect(
            Calculator1.blueprint(),
            Mod1.blueprint(),
        ),
        _BUILD_WITHOUT_RERUN.copy(),
    )

    try:
        mod1 = coordinator.get_instance(Mod1)
        assert mod1 is not None
        assert mod1.calc.compute1(2, 3) == 5
        assert mod1.calc.compute2(1.5, 2.5) == 4.0
    finally:
        coordinator.stop()


def test_module_ref_spec() -> None:
    coordinator = ModuleCoordinator.build(
        autoconnect(
            Calculator1.blueprint(),
            Mod2.blueprint(),
        ),
        _BUILD_WITHOUT_RERUN.copy(),
    )

    try:
        mod2 = coordinator.get_instance(Mod2)
        assert mod2 is not None
        assert mod2.calc.compute1(4, 5) == 9
        assert mod2.calc.compute2(3.0, 0.5) == 3.5
    finally:
        coordinator.stop()


def test_disabled_modules_are_skipped_during_build() -> None:
    blueprint_set = autoconnect(
        ModuleA.blueprint(), ModuleB.blueprint(), ModuleC.blueprint()
    ).disabled_modules(ModuleC)

    coordinator = ModuleCoordinator.build(blueprint_set, _BUILD_WITHOUT_RERUN.copy())

    try:
        assert coordinator.get_instance(ModuleA) is not None
        assert coordinator.get_instance(ModuleB) is not None

        assert coordinator.get_instance(ModuleC) is None
    finally:
        coordinator.stop()


def test_disabled_module_ref_gets_noop_proxy() -> None:
    blueprint_set = autoconnect(
        Calculator1.blueprint(),
        Mod2.blueprint(),
    ).disabled_modules(Calculator1)

    coordinator = ModuleCoordinator.build(blueprint_set, _BUILD_WITHOUT_RERUN.copy())

    try:
        mod2 = coordinator.get_instance(Mod2)
        assert mod2 is not None
        # The proxy should be a _DisabledModuleProxy, not a real Calculator.
        assert isinstance(mod2.calc, DisabledModuleProxy)
        # Calling methods on it should return None (no-op).
        assert mod2.calc.compute1(1, 2) is None
    finally:
        coordinator.stop()


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
        _BUILD_WITHOUT_RERUN.copy(),
    )

    try:
        mod2 = coordinator.get_instance(Mod2)
        assert mod2 is not None
        assert mod2.calc.compute1(2, 3) == 5
        assert mod2.calc.compute2(2.0, 3.0) == 5.0
    finally:
        coordinator.stop()


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


def test_load_module_convenience(dynamic_coordinator) -> None:
    """load_module is a shorthand for load_blueprint(cls.blueprint())."""
    dynamic_coordinator.load_module(ModuleA)
    assert dynamic_coordinator.get_instance(ModuleA) is not None
    assert dynamic_coordinator.get_instance(ModuleA).data1.transport is not None


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
        _verify_no_conflicts_with_existing(bp, registry)


def test_load_blueprint_duplicate_module_raises(dynamic_coordinator) -> None:
    """Loading a module that is already deployed raises ValueError."""
    dynamic_coordinator.load_blueprint(ModuleA.blueprint())
    with pytest.raises(ValueError, match="already deployed"):
        dynamic_coordinator.load_blueprint(ModuleA.blueprint())


class ModWithOptionalRef(Module):
    stream1: In[Image]
    calc: CalculatorSpec | None = None

    @rpc
    def start(self) -> None: ...

    @rpc
    def stop(self) -> None: ...


@pytest.fixture
def build_coordinator():
    coordinators = []

    def _build(blueprint):
        c = ModuleCoordinator.build(blueprint, _BUILD_WITHOUT_RERUN.copy())
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


def test_optional_module_ref_without_provider(build_coordinator) -> None:
    """An optional ref is silently skipped when no provider is in the blueprint."""
    coordinator = build_coordinator(ModWithOptionalRef.blueprint())

    mod = coordinator.get_instance(ModWithOptionalRef)
    assert mod is not None


def test_load_blueprint_auto_scales_empty_pool(dynamic_coordinator) -> None:
    """A coordinator with 0 initial workers auto-adds workers on load_blueprint."""
    dynamic_coordinator.load_blueprint(ModuleA.blueprint())
    assert dynamic_coordinator.get_instance(ModuleA) is not None
    assert dynamic_coordinator.get_instance(ModuleA).data1.transport is not None


def _sys_python_env(name: str) -> PythonVenvRuntimeEnvironment:
    return PythonVenvRuntimeEnvironment(name=name, python_executable=Path(sys.executable))


def _prepared_project(tmp_path: Path) -> PythonProjectRuntimeEnvironment:
    project = tmp_path / "project"
    project.mkdir()
    (project / "pyproject.toml").write_text("[project]\nname='demo'\nversion='0.0.0'\n")
    python_path = project / ".venv" / "bin" / "python"
    python_path.parent.mkdir(parents=True)
    python_path.write_text("prepared")
    return PythonProjectRuntimeEnvironment(name="project-env", project=project)


def _write_fake_uv(path: Path) -> None:
    path.write_text(
        "#!/usr/bin/env python3\n"
        "import subprocess, sys\n"
        "assert sys.argv[1:4] == ['run', '--no-sync', 'python']\n"
        f"raise SystemExit(subprocess.call([{sys.executable!r}, *sys.argv[4:]]))\n"
    )
    path.chmod(0o755)


def test_unplaced_modules_use_default_pool(build_coordinator) -> None:
    coordinator = build_coordinator(ModuleA.blueprint())

    assert coordinator._module_manager_keys[ModuleA] == "python"
    assert "python:env-a" not in coordinator._managers


def test_same_named_env_modules_share_venv_pool(build_coordinator) -> None:
    blueprint = (
        autoconnect(ModuleA.blueprint(), ModuleB.blueprint())
        .global_config(n_workers=1)
        .runtime_environments(_sys_python_env("env-a"))
        .runtime_placements({ModuleA: "env-a", ModuleB: "env-a"})
    )

    coordinator = build_coordinator(blueprint)

    assert coordinator._module_manager_keys[ModuleA] == "python:env-a"
    assert coordinator._module_manager_keys[ModuleB] == "python:env-a"
    manager = coordinator._managers["python:env-a"]
    assert isinstance(manager, WorkerManagerPython)
    assert len(manager.workers) == 1


def test_direct_venv_uses_venv_worker_launcher(dynamic_coordinator) -> None:
    blueprint = (
        ModuleA.blueprint()
        .runtime_environments(_sys_python_env("env-a"))
        .runtime_placements({ModuleA: "env-a"})
    )

    dynamic_coordinator.load_blueprint(blueprint)

    manager = dynamic_coordinator._managers["python:env-a"]
    assert isinstance(manager, WorkerManagerPython)
    worker = manager.workers[0]
    assert type(worker._launcher).__name__ == "VenvWorkerLauncher"


def test_current_runtime_placement_uses_python_launcher(dynamic_coordinator) -> None:
    blueprint = ModuleA.blueprint().runtime_placements({ModuleA: "current"})

    dynamic_coordinator.load_blueprint(blueprint)

    manager = dynamic_coordinator._managers["python:current"]
    assert isinstance(manager, WorkerManagerPython)
    worker = manager.workers[0]
    assert type(worker._launcher).__name__ == "VenvWorkerLauncher"


@dataclass(frozen=True)
class CustomPythonRuntimeEnvironment(RuntimeEnvironment):
    name: str = "custom-python"

    def resolve_python(self) -> PythonLaunchMaterial:
        return PythonLaunchMaterial(
            python_executable=Path(sys.executable),
            env={"DIMOS_TEST_CUSTOM_RUNTIME": "1"},
        )


def test_custom_python_capability_runtime_uses_python_launcher(dynamic_coordinator) -> None:
    blueprint = (
        ModuleA.blueprint()
        .runtime_environments(CustomPythonRuntimeEnvironment())
        .runtime_placements({ModuleA: "custom-python"})
    )

    dynamic_coordinator.load_blueprint(blueprint)

    manager = dynamic_coordinator._managers["python:custom-python"]
    assert isinstance(manager, WorkerManagerPython)
    worker = manager.workers[0]
    assert type(worker._launcher).__name__ == "VenvWorkerLauncher"


def test_project_runtime_uses_command_worker_launcher(
    dynamic_coordinator, tmp_path, monkeypatch
) -> None:
    _write_fake_uv(tmp_path / "uv")
    monkeypatch.setenv("PATH", f"{tmp_path}:{os.environ.get('PATH', '')}")
    blueprint = (
        ModuleA.blueprint()
        .runtime_environments(_prepared_project(tmp_path))
        .runtime_placements({ModuleA: "project-env"})
    )

    dynamic_coordinator.load_blueprint(blueprint)

    manager = dynamic_coordinator._managers["python:project-env"]
    assert isinstance(manager, WorkerManagerPython)
    worker = manager.workers[0]
    assert type(worker._launcher).__name__ == "CommandWorkerLauncher"


def test_distinct_named_env_modules_use_distinct_venv_pools(build_coordinator) -> None:
    blueprint = (
        autoconnect(ModuleA.blueprint(), ModuleB.blueprint())
        .runtime_environments(_sys_python_env("env-a"), _sys_python_env("env-b"))
        .runtime_placements({ModuleA: "env-a", ModuleB: "env-b"})
    )

    coordinator = build_coordinator(blueprint)

    assert coordinator._module_manager_keys[ModuleA] == "python:env-a"
    assert coordinator._module_manager_keys[ModuleB] == "python:env-b"
    assert coordinator._managers["python:env-a"] is not coordinator._managers["python:env-b"]


def test_placed_modules_preserve_stream_refs_and_rpc(build_coordinator) -> None:
    blueprint = (
        autoconnect(ModuleA.blueprint(), ModuleB.blueprint(), ModuleC.blueprint())
        .runtime_environments(_sys_python_env("env-a"))
        .runtime_placements({ModuleA: "env-a", ModuleB: "env-a"})
    )

    coordinator = build_coordinator(blueprint)
    module_a = coordinator.get_instance(ModuleA)
    module_b = coordinator.get_instance(ModuleB)
    module_c = coordinator.get_instance(ModuleC)

    assert module_a.data1.transport.topic == module_b.data1.transport.topic
    assert module_b.data3.transport.topic == module_c.data3.transport.topic
    assert module_b.what_is_as_name() == "A, Module A"


def test_dynamic_load_unload_restart_preserves_placement(dynamic_coordinator) -> None:
    blueprint = (
        ModuleA.blueprint()
        .runtime_environments(_sys_python_env("env-a"))
        .runtime_placements({ModuleA: "env-a"})
    )

    dynamic_coordinator.load_blueprint(blueprint)
    assert dynamic_coordinator._module_manager_keys[ModuleA] == "python:env-a"

    dynamic_coordinator.restart_module(ModuleA, reload_source=False)
    assert dynamic_coordinator._module_manager_keys[ModuleA] == "python:env-a"

    dynamic_coordinator.unload_module(ModuleA)
    assert ModuleA not in dynamic_coordinator._module_manager_keys


def test_unload_clears_placement_for_later_unplaced_load(dynamic_coordinator) -> None:
    blueprint = (
        ModuleA.blueprint()
        .runtime_environments(_sys_python_env("env-a"))
        .runtime_placements({ModuleA: "env-a"})
    )

    dynamic_coordinator.load_blueprint(blueprint)
    assert dynamic_coordinator._module_manager_keys[ModuleA] == "python:env-a"

    dynamic_coordinator.unload_module(ModuleA)
    dynamic_coordinator.load_blueprint(ModuleA.blueprint())

    assert dynamic_coordinator._module_manager_keys[ModuleA] == "python"


def test_unloading_last_venv_module_leaves_health_check_healthy(dynamic_coordinator) -> None:
    blueprint = (
        autoconnect(ModuleA.blueprint(), ModuleC.blueprint())
        .runtime_environments(_sys_python_env("env-a"))
        .runtime_placements({ModuleA: "env-a"})
    )

    dynamic_coordinator.load_blueprint(blueprint)
    dynamic_coordinator.unload_module(ModuleA)

    assert "python:env-a" not in dynamic_coordinator._managers
    assert dynamic_coordinator.health_check() is True


def test_unloading_last_venv_module_removes_idle_worker_manager() -> None:
    coordinator = ModuleCoordinator(g=GlobalConfig(n_workers=2, viewer="none"))
    coordinator.start()
    try:
        blueprint = (
            ModuleA.blueprint()
            .runtime_environments(_sys_python_env("env-a"))
            .runtime_placements({ModuleA: "env-a"})
        )

        coordinator.load_blueprint(blueprint)
        manager = coordinator._managers["python:env-a"]
        assert isinstance(manager, WorkerManagerPython)
        assert len(manager.workers) == 2

        coordinator.unload_module(ModuleA)

        assert "python:env-a" not in coordinator._managers
        assert coordinator.health_check() is True
    finally:
        coordinator.stop()


def test_absent_placement_does_not_affect_later_unplaced_load(dynamic_coordinator) -> None:
    blueprint = (
        ModuleA.blueprint()
        .runtime_environments(_sys_python_env("env-a"))
        .runtime_placements({ModuleB: "env-a"})
    )

    dynamic_coordinator.load_blueprint(blueprint)
    dynamic_coordinator.load_blueprint(ModuleB.blueprint())

    assert dynamic_coordinator._module_manager_keys[ModuleB] == "python"
    assert "python:env-a" not in dynamic_coordinator._managers


def test_disabled_placement_does_not_affect_later_unplaced_load(dynamic_coordinator) -> None:
    blueprint = (
        autoconnect(ModuleA.blueprint(), ModuleB.blueprint())
        .disabled_modules(ModuleB)
        .runtime_environments(_sys_python_env("env-a"))
        .runtime_placements({ModuleB: "env-a"})
    )

    dynamic_coordinator.load_blueprint(blueprint)
    dynamic_coordinator.load_blueprint(ModuleB.blueprint())

    assert dynamic_coordinator._module_manager_keys[ModuleB] == "python"
    assert "python:env-a" not in dynamic_coordinator._managers


def test_placed_module_unknown_env_raises_clear_error(dynamic_coordinator) -> None:
    blueprint = ModuleA.blueprint().runtime_placements({ModuleA: "missing-env"})

    with pytest.raises(RuntimeError, match="Register a Python runtime environment"):
        dynamic_coordinator.load_blueprint(blueprint)


def test_missing_venv_executable_error_includes_env_and_does_not_linger(
    dynamic_coordinator, tmp_path
) -> None:
    missing_python = tmp_path / "missing-python"
    blueprint = (
        ModuleA.blueprint()
        .runtime_environments(
            PythonVenvRuntimeEnvironment(name="bad-env", python_executable=missing_python)
        )
        .runtime_placements({ModuleA: "bad-env"})
    )

    with pytest.raises(RuntimeError) as exc_info:
        dynamic_coordinator.load_blueprint(blueprint)

    message = str(exc_info.value)
    assert "bad-env" in message
    assert str(missing_python) in message
    assert "Python capability" in message
    assert "python:bad-env" not in dynamic_coordinator._managers


def test_missing_project_prepared_python_fails_before_manager_and_popen(
    dynamic_coordinator, tmp_path, mocker
) -> None:
    project = tmp_path / "project"
    project.mkdir()
    (project / "pyproject.toml").write_text("[project]\nname='demo'\nversion='0.0.0'\n")
    env = PythonProjectRuntimeEnvironment(name="project-env", project=project)
    popen = mocker.patch("dimos.core.coordination.worker_launcher.subprocess.Popen")
    blueprint = (
        ModuleA.blueprint().runtime_environments(env).runtime_placements({ModuleA: "project-env"})
    )

    with pytest.raises(Exception) as exc_info:
        dynamic_coordinator.load_blueprint(blueprint)

    message = str(exc_info.value)
    assert "project-env" in message
    assert str(project) in message
    assert str(project / ".venv" / "bin" / "python") in message
    assert "dimos runtime prepare <blueprint> --runtime project-env" in message
    assert "python:project-env" not in dynamic_coordinator._managers
    popen.assert_not_called()


def test_missing_project_pyproject_fails_actionably_before_manager_and_popen(
    dynamic_coordinator, tmp_path, mocker
) -> None:
    project = tmp_path / "project-without-pyproject"
    project.mkdir()
    env = PythonProjectRuntimeEnvironment(name="project-env", project=project)
    popen = mocker.patch("dimos.core.coordination.worker_launcher.subprocess.Popen")
    blueprint = (
        ModuleA.blueprint().runtime_environments(env).runtime_placements({ModuleA: "project-env"})
    )

    with pytest.raises(MissingPythonProjectFileError) as exc_info:
        dynamic_coordinator.load_blueprint(blueprint)

    message = str(exc_info.value)
    assert "project-env" in message
    assert str(project) in message
    assert "pyproject.toml" in message
    assert "python:project-env" not in dynamic_coordinator._managers
    popen.assert_not_called()


@pytest.mark.skipif_macos_bug
def test_build_missing_project_prepared_python_stops_default_workers_and_no_popen(
    tmp_path, mocker
) -> None:
    project = tmp_path / "project"
    project.mkdir()
    (project / "pyproject.toml").write_text("[project]\nname='demo'\nversion='0.0.0'\n")
    env = PythonProjectRuntimeEnvironment(name="project-env", project=project)
    blueprint = (
        ModuleA.blueprint()
        .global_config(n_workers=1)
        .runtime_environments(env)
        .runtime_placements({ModuleA: "project-env"})
    )
    constructed: list[ModuleCoordinator] = []
    original_init = ModuleCoordinator.__init__

    def track_init(self: ModuleCoordinator, g: GlobalConfig) -> None:
        original_init(self, g)
        constructed.append(self)

    mocker.patch.object(ModuleCoordinator, "__init__", track_init)
    popen = mocker.patch("dimos.core.coordination.worker_launcher.subprocess.Popen")

    with pytest.raises(Exception) as exc_info:
        ModuleCoordinator.build(blueprint, _BUILD_WITHOUT_RERUN.copy())

    assert "project-env" in str(exc_info.value)
    popen.assert_not_called()
    assert len(constructed) == 1
    assert "python:project-env" not in constructed[0]._managers
    python_manager = constructed[0]._managers["python"]
    assert isinstance(python_manager, WorkerManagerPython)
    worker_pids = [worker.pid for worker in python_manager.workers if worker.pid is not None]
    assert all(not psutil.pid_exists(pid) for pid in worker_pids)


def test_disabled_unused_project_runtime_placement_does_not_allocate_pool(
    dynamic_coordinator, tmp_path
) -> None:
    blueprint = (
        autoconnect(ModuleA.blueprint(), ModuleB.blueprint())
        .disabled_modules(ModuleB)
        .runtime_environments(_prepared_project(tmp_path))
        .runtime_placements({ModuleB: "project-env"})
    )

    dynamic_coordinator.load_blueprint(blueprint)

    assert "python:project-env" not in dynamic_coordinator._managers


def test_check_requirements_failure(mocker) -> None:
    """A failing requirement check causes sys.exit."""
    mocker.patch("dimos.core.coordination.module_coordinator.sys.exit", side_effect=SystemExit(1))

    bp = ModuleA.blueprint().requirements(lambda: "missing GPU driver")

    with pytest.raises(SystemExit):
        _check_requirements(bp)


def test_restart_module_basic(dynamic_coordinator) -> None:
    """restart_module replaces the deployed proxy with a fresh one."""
    dynamic_coordinator.load_module(ModuleA)
    old_proxy = dynamic_coordinator.get_instance(ModuleA)
    assert old_proxy is not None

    new_proxy = dynamic_coordinator.restart_module(ModuleA, reload_source=False)

    assert new_proxy is not None
    assert new_proxy is not old_proxy
    assert dynamic_coordinator.get_instance(ModuleA) is new_proxy
    assert new_proxy.get_name() == "A, Module A"
    assert "g" in dynamic_coordinator._resolved_module_plans[ModuleA].final_kwargs
    assert "g" not in dynamic_coordinator._deployed_atoms[ModuleA].kwargs


def test_restart_module_preserves_stream_wiring(dynamic_coordinator) -> None:
    """Streams stay on the same transport after restart so consumers keep receiving data."""
    dynamic_coordinator.load_blueprint(autoconnect(ModuleA.blueprint(), ModuleC.blueprint()))

    c = dynamic_coordinator.get_instance(ModuleC)
    assert c is not None
    topic_before = c.data3.transport.topic
    registry_before = dynamic_coordinator._transport_registry[("data3", Data3)]

    dynamic_coordinator.restart_module(ModuleC, reload_source=False)

    # Transport in the registry is the same parent-side object.
    assert dynamic_coordinator._transport_registry[("data3", Data3)] is registry_before

    c_after = dynamic_coordinator.get_instance(ModuleC)
    assert c_after is not None
    assert c_after is not c
    # The restarted module's stream is wired to the same topic.
    assert c_after.data3.transport.topic == topic_before


def test_restart_module_rewires_module_refs(dynamic_coordinator) -> None:
    """After restart, modules that reference the restarted class see the new proxy."""
    dynamic_coordinator.load_blueprint(autoconnect(ModuleA.blueprint(), ModuleB.blueprint()))

    b = dynamic_coordinator.get_instance(ModuleB)
    assert b is not None
    assert b.what_is_as_name() == "A, Module A"

    dynamic_coordinator.restart_module(ModuleA, reload_source=False)

    assert b.what_is_as_name() == "A, Module A"


def test_restart_consumer_rewires_outbound_refs(dynamic_coordinator) -> None:
    """Restarting a consumer re-injects its refs to existing target modules."""
    dynamic_coordinator.load_blueprint(autoconnect(ModuleA.blueprint(), ModuleB.blueprint()))

    dynamic_coordinator.restart_module(ModuleB, reload_source=False)

    b_after = dynamic_coordinator.get_instance(ModuleB)
    assert b_after is not None
    # The new ModuleB must still reach ModuleA through its outbound module_ref.
    assert b_after.what_is_as_name() == "A, Module A"


def test_restart_module_shuts_down_empty_worker(dynamic_coordinator) -> None:
    """Restart shuts down the old worker (when empty) and spawns a new one."""

    dynamic_coordinator.load_module(ModuleA)
    python_wm = dynamic_coordinator._managers["python"]
    assert isinstance(python_wm, WorkerManagerPython)

    old_worker_ids = {w.worker_id for w in python_wm.workers}
    assert len(old_worker_ids) == 1

    dynamic_coordinator.restart_module(ModuleA, reload_source=False)

    new_worker_ids = {w.worker_id for w in python_wm.workers}
    assert len(new_worker_ids) == 1
    assert new_worker_ids.isdisjoint(old_worker_ids)


def test_restart_module_calls_importlib_reload(dynamic_coordinator, mocker) -> None:
    """reload_source=True invokes importlib.reload on the module's source file."""
    dynamic_coordinator.load_module(ModuleA)

    # Stub reload so it's a no-op. Actually reloading this test module would
    # re-execute test definitions and corrupt later tests.
    mock_reload = mocker.patch(
        "dimos.core.coordination.module_coordinator.importlib.reload",
        side_effect=lambda m: m,
    )

    dynamic_coordinator.restart_module(ModuleA, reload_source=True)

    mock_reload.assert_called_once()
    reloaded_module = mock_reload.call_args.args[0]
    assert reloaded_module.__name__ == ModuleA.__module__


def _mock_reload_producing_new_class(original_class):
    """Return a reload side-effect that replaces the original class with a fresh copy."""
    new_class = type(
        original_class.__name__, original_class.__bases__, dict(original_class.__dict__)
    )
    new_class.__module__ = original_class.__module__
    new_class.__qualname__ = original_class.__qualname__

    def side_effect(mod):
        setattr(mod, original_class.__name__, new_class)
        return mod

    return side_effect, new_class


def test_get_instance_after_reload_restart(dynamic_coordinator, mocker) -> None:
    """get_instance with the original class still works after a reload restart."""
    dynamic_coordinator.load_module(ModuleA)

    side_effect, _new_class = _mock_reload_producing_new_class(ModuleA)
    mocker.patch(
        "dimos.core.coordination.module_coordinator.importlib.reload",
        side_effect=side_effect,
    )

    new_proxy = dynamic_coordinator.restart_module(ModuleA, reload_source=True)

    assert dynamic_coordinator.get_instance(ModuleA) is new_proxy


def test_double_restart_with_reload(dynamic_coordinator, mocker) -> None:
    """A second restart via the original class works after a reload restart."""
    dynamic_coordinator.load_module(ModuleA)

    side_effect1, new_class1 = _mock_reload_producing_new_class(ModuleA)
    mocker.patch(
        "dimos.core.coordination.module_coordinator.importlib.reload",
        side_effect=side_effect1,
    )
    proxy1 = dynamic_coordinator.restart_module(ModuleA, reload_source=True)

    side_effect2, _new_class2 = _mock_reload_producing_new_class(new_class1)
    mocker.patch(
        "dimos.core.coordination.module_coordinator.importlib.reload",
        side_effect=side_effect2,
    )
    proxy2 = dynamic_coordinator.restart_module(ModuleA, reload_source=True)

    assert proxy2 is not proxy1
    assert dynamic_coordinator.get_instance(ModuleA) is proxy2


def test_unload_after_reload_restart(dynamic_coordinator, mocker) -> None:
    """unload_module with the original class works after a reload restart."""
    dynamic_coordinator.load_module(ModuleA)

    side_effect, _new_class = _mock_reload_producing_new_class(ModuleA)
    mocker.patch(
        "dimos.core.coordination.module_coordinator.importlib.reload",
        side_effect=side_effect,
    )
    dynamic_coordinator.restart_module(ModuleA, reload_source=True)

    dynamic_coordinator.unload_module(ModuleA)
    assert dynamic_coordinator.get_instance(ModuleA) is None


def test_placed_reload_restart_unload_clears_old_class_placement(
    dynamic_coordinator, mocker
) -> None:
    """A reload restart must not leave stale placement on the old class handle."""
    original_class = ModuleA
    blueprint = (
        original_class.blueprint()
        .runtime_environments(_sys_python_env("env-a"))
        .runtime_placements({original_class: "env-a"})
    )
    dynamic_coordinator.load_blueprint(blueprint)

    side_effect, new_class = _mock_reload_producing_new_class(original_class)
    mocker.patch(
        "dimos.core.coordination.module_coordinator.importlib.reload",
        side_effect=side_effect,
    )

    dynamic_coordinator.restart_module(original_class, reload_source=True)
    assert all(cls is not original_class for cls in dynamic_coordinator._runtime_placement_map)
    assert dynamic_coordinator._runtime_placement_map[new_class] == "env-a"

    dynamic_coordinator.unload_module(original_class)
    assert all(cls is not original_class for cls in dynamic_coordinator._runtime_placement_map)
    assert all(cls is not new_class for cls in dynamic_coordinator._runtime_placement_map)

    setattr(sys.modules[original_class.__module__], original_class.__name__, original_class)
    dynamic_coordinator.load_blueprint(original_class.blueprint())
    assert dynamic_coordinator._module_manager_keys[original_class] == "python"


def test_restart_preserves_remapped_streams(dynamic_coordinator) -> None:
    """Restart reconnects streams that were remapped during initial load."""
    bp = autoconnect(
        SourceModule.blueprint(),
        TargetModule.blueprint(),
    ).remappings(
        [(SourceModule, "color_image", "remapped_data")],
    )
    dynamic_coordinator.load_blueprint(bp)

    target = dynamic_coordinator.get_instance(TargetModule)
    registry_before = dynamic_coordinator._transport_registry[("remapped_data", Data1)]

    dynamic_coordinator.restart_module(SourceModule, reload_source=False)

    # The coordinator-side transport object in the registry is unchanged.
    assert dynamic_coordinator._transport_registry[("remapped_data", Data1)] is registry_before
    # The restarted proxy sees the same topic as the target.
    source_after = dynamic_coordinator.get_instance(SourceModule)
    assert source_after.color_image.transport.topic == target.remapped_data.transport.topic


def test_start_rpc_service_responds_to_ping(dynamic_coordinator) -> None:
    dynamic_coordinator.start_rpc_service()
    client = CoordinatorRPC.connect(timeout=2.0)
    try:
        assert client.call("ping") == "pong"
    finally:
        client.stop()


def test_list_module_names(dynamic_coordinator) -> None:
    assert dynamic_coordinator.list_module_names() == []
    dynamic_coordinator.load_module(ModuleA)
    dynamic_coordinator.load_module(ModuleC)
    assert set(dynamic_coordinator.list_module_names()) == {"ModuleA", "ModuleC"}
