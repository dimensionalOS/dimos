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

from __future__ import annotations

import builtins
import importlib
from pathlib import Path
import sys
from types import ModuleType
from typing import Protocol, cast

import pytest

import dimos.hardware.manipulators as manipulators_pkg
from dimos.hardware.manipulators.mock.adapter import MockAdapter
from dimos.hardware.manipulators.registry import AdapterRegistry, adapter_registry

EXPECTED_ADAPTER_KEYS = {
    "a750",
    "mock",
    "openarm",
    "openarm_rs",
    "piper",
    "sim_mujoco",
    "xarm",
}


class _HasKwargs(Protocol):
    kwargs: dict[str, object]


def _write_package(
    root: Path,
    name: str,
    registry_source: str,
    adapter_source: str = "",
) -> None:
    package = root / name
    package.mkdir()
    _ = (package / "__init__.py").write_text("", encoding="utf-8")
    _ = (package / "__registry__.py").write_text(registry_source, encoding="utf-8")
    if adapter_source:
        _ = (package / "adapter.py").write_text(adapter_source, encoding="utf-8")


def _clear_fake_modules() -> None:
    for module_name in list(sys.modules):
        if module_name.startswith("dimos.hardware.manipulators.fake"):
            del sys.modules[module_name]


@pytest.fixture(autouse=True)
def clear_fake_modules() -> None:
    _clear_fake_modules()
    importlib.invalidate_caches()


def test_available_does_not_import_adapter_implementation(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    _write_package(
        tmp_path,
        "fake_lazy",
        'ADAPTER_FACTORIES = {"fake": "dimos.hardware.manipulators.fake_lazy.adapter:Fake"}\n',
        'raise AssertionError("adapter implementation imported during discovery")\n',
    )
    monkeypatch.setattr(manipulators_pkg, "__path__", [str(tmp_path)])

    registry = AdapterRegistry()
    registry.discover()

    assert registry.available() == ["fake"]
    assert "dimos.hardware.manipulators.fake_lazy.adapter" not in sys.modules


def test_create_imports_selected_adapter_and_passes_kwargs(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    _write_package(
        tmp_path,
        "fake_selected",
        'ADAPTER_FACTORIES = {"fake": "dimos.hardware.manipulators.fake_selected.adapter:Fake"}\n',
        "\n".join(
            [
                "class Fake:",
                "    def __init__(self, **kwargs: object) -> None:",
                "        self.kwargs = kwargs",
                "",
            ]
        ),
    )
    monkeypatch.setattr(manipulators_pkg, "__path__", [str(tmp_path)])

    registry = AdapterRegistry()
    registry.discover()
    adapter = registry.create("fake", address="can0", dof=7)

    assert adapter.__class__.__name__ == "Fake"
    assert cast("_HasKwargs", adapter).kwargs == {"address": "can0", "dof": 7}
    assert "dimos.hardware.manipulators.fake_selected.adapter" in sys.modules


def test_direct_registration_still_creates_adapter() -> None:
    registry = AdapterRegistry()
    registry.register("mock", MockAdapter)

    adapter = registry.create("mock", dof=3)

    assert isinstance(adapter, MockAdapter)
    assert adapter.get_dof() == 3


def test_manifest_validation_rejects_bad_mapping(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    _write_package(tmp_path, "fake_bad", "ADAPTER_FACTORIES = {'fake': 1}\n")
    monkeypatch.setattr(manipulators_pkg, "__path__", [str(tmp_path)])

    registry = AdapterRegistry()
    with pytest.raises(TypeError, match="must map strings to strings"):
        registry.discover()


def test_register_path_rejects_duplicate_and_invalid_paths() -> None:
    registry = AdapterRegistry()
    registry.register_path("fake", "pkg.mod:Factory")

    with pytest.raises(ValueError, match="Duplicate adapter"):
        registry.register_path("fake", "pkg.other:Factory")
    with pytest.raises(ValueError, match="Invalid adapter factory path"):
        registry.register_path("bad", "pkg.mod.Factory")
    with pytest.raises(ValueError, match="Invalid adapter factory path"):
        registry.register_path("bad", "pkg.mod:")


def test_create_reports_missing_selected_module_and_attribute() -> None:
    registry = AdapterRegistry()
    registry.register_path(
        "missing_module", "dimos.hardware.manipulators.fake_missing.adapter:Fake"
    )
    registry.register_path("missing_attr", "dimos.hardware.manipulators.registry:MissingFactory")

    with pytest.raises(ImportError, match="missing_module.*missing module"):
        _ = registry.create("missing_module")
    with pytest.raises(ImportError, match="missing_attr.*missing factory"):
        _ = registry.create("missing_attr")
    with pytest.raises(KeyError, match="Unknown adapter: unknown"):
        _ = registry.create("unknown")


def test_builtin_registry_preserves_adapter_keys() -> None:
    assert EXPECTED_ADAPTER_KEYS.issubset(set(adapter_registry.available()))


def test_discovery_does_not_import_can_motor_control(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    real_import = builtins.__import__

    def fail_can_motor_control(
        name: str,
        globals: dict[str, object] | None = None,
        locals: dict[str, object] | None = None,
        fromlist: tuple[str, ...] = (),
        level: int = 0,
    ) -> ModuleType:
        if name.startswith("can_motor_control"):
            raise AssertionError("can_motor_control imported during discovery")
        module_obj = cast("object", real_import(name, globals, locals, fromlist, level))
        if not isinstance(module_obj, ModuleType):
            raise TypeError(f"expected module import for {name}")
        return module_obj

    monkeypatch.delitem(sys.modules, "can_motor_control", raising=False)
    monkeypatch.delitem(sys.modules, "can_motor_control.damiao", raising=False)
    monkeypatch.setattr(builtins, "__import__", fail_can_motor_control)

    registry = AdapterRegistry()
    registry.discover()

    assert "openarm_rs" in registry.available()
