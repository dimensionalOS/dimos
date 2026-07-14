# Copyright 2026 Dimensional Inc.
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

"""CI guards: hardware adapters must never silently vanish from their registries."""

from __future__ import annotations

import importlib
import importlib.util
from pathlib import Path
import sys

import pytest

from dimos.hardware.drive_trains.registry import twist_base_adapter_registry
from dimos.hardware.manipulators.registry import AdapterRegistry, adapter_registry
from dimos.hardware.whole_body.registry import whole_body_adapter_registry

# Vendor SDKs that are legitimately absent in some environments. A factory
# path failing on one of these still passes IF the SDK is not installed;
# anything else (a typo in a manifest, an internal dimos import breaking,
# a bad import inside an installed SDK) fails CI.
OPTIONAL_VENDOR_MODULES = {
    "can",
    "cyclonedds",
    "mujoco",
    "piper_sdk",
    "rclpy",
    "unitree_sdk2py",
    "xarm",
}

# Subpackages containing an adapter.py that intentionally register nothing.
UNREGISTERED_ADAPTER_DIRS: set[str] = set()

# Every name each registry must declare. Removing a name from a manifest is a
# conscious change: update this set in the same PR.
EXPECTED_NAMES = {
    "manipulators": {"a1z", "a750", "mock", "openarm", "piper", "sim_mujoco", "xarm"},
    "drive_trains": {
        "flowbase",
        "mock_twist_base",
        "transport_lcm",
        "transport_ros",
        "unitree_go2",
    },
    "whole_body": {"sim_mujoco_g1", "transport_lcm", "transport_ros"},
}

FAMILIES = [
    pytest.param(adapter_registry, "manipulators", id="manipulators"),
    pytest.param(twist_base_adapter_registry, "drive_trains", id="drive_trains"),
    pytest.param(whole_body_adapter_registry, "whole_body", id="whole_body"),
]


def _adapter_dirs(pkg_name: str):
    """Yield (subpackage, dir, depth) for every adapter.py under the root."""
    pkg = importlib.import_module(pkg_name)
    for root in pkg.__path__:
        root_path = Path(root)
        for adapter_py in sorted(root_path.rglob("adapter.py")):
            parts = adapter_py.relative_to(root_path).parts[:-1]
            if not parts or any(p.startswith(("_", ".")) for p in parts):
                continue
            yield ".".join((pkg_name, *parts)), adapter_py.parent, len(parts)


def _vendor_missing(exc: ModuleNotFoundError) -> bool:
    root = (exc.name or "").partition(".")[0]
    return root in OPTIONAL_VENDOR_MODULES and importlib.util.find_spec(root) is None


@pytest.mark.parametrize(("registry", "family"), FAMILIES)
def test_declared_names_match_golden_set(registry, family) -> None:
    assert set(registry._factory_paths) == EXPECTED_NAMES[family]


@pytest.mark.parametrize(("registry", "family"), FAMILIES)
def test_every_adapter_dir_has_a_manifest(registry, family) -> None:
    checked = 0
    for pkg_name, max_depth in registry.manifest_roots:
        for sub_pkg, dir_path, depth in _adapter_dirs(pkg_name):
            if sub_pkg in UNREGISTERED_ADAPTER_DIRS:
                continue
            assert depth <= max_depth, (
                f"{dir_path} is deeper than the registry scan depth ({max_depth}); "
                f"its manifest would never be discovered"
            )
            manifest = dir_path / "_registry.py"
            assert manifest.exists(), (
                f"{dir_path} contains adapter.py but no _registry.py; "
                f"its adapters would silently vanish from the registry"
            )
            manifest_mod = importlib.import_module(f"{sub_pkg}._registry")
            names = set(manifest_mod.ADAPTER_FACTORIES)
            assert names, f"{manifest} declares no adapters"
            missing = names - set(registry.available())
            assert not missing, f"{manifest} declares {missing} missing from available()"
            checked += 1
    assert checked > 0


def test_every_sim_whole_body_module_is_declared() -> None:
    """Sim whole-body adapters are flat modules; each must appear in the root manifest."""
    pkg = importlib.import_module("dimos.simulation.adapters.whole_body")
    manifest = importlib.import_module("dimos.simulation.adapters.whole_body._registry")
    names = set(manifest.ADAPTER_FACTORIES)
    assert "sim_mujoco_g1" in names
    assert names <= set(whole_body_adapter_registry.available())
    declared_modules = {path.split(":", 1)[0] for path in manifest.ADAPTER_FACTORIES.values()}
    for root in pkg.__path__:
        for mod_file in sorted(Path(root).glob("*.py")):
            if mod_file.name.startswith(("_", ".")):
                continue
            mod_name = f"dimos.simulation.adapters.whole_body.{mod_file.stem}"
            assert mod_name in declared_modules, (
                f"{mod_file} is not referenced by _registry.py; "
                f"its adapter would silently vanish from the registry"
            )


@pytest.mark.parametrize(("registry", "family"), FAMILIES)
def test_declared_factory_paths_resolve(registry, family) -> None:
    for name, factory_path in sorted(registry._factory_paths.items()):
        module_name, attr = factory_path.split(":", maxsplit=1)
        try:
            module = importlib.import_module(module_name)
        except ModuleNotFoundError as exc:
            if _vendor_missing(exc):
                continue
            pytest.fail(f"{name}: importing {module_name!r} failed: {exc}")
        factory = getattr(module, attr, None)
        assert factory is not None, f"{name}: {module_name!r} has no attribute {attr!r}"
        assert callable(factory), f"{name}: {factory_path!r} is not callable"


def test_missing_vendor_sdk_fails_loud_at_create(monkeypatch) -> None:
    registry = AdapterRegistry()
    assert "xarm" in registry.available()
    for mod_name in [m for m in sys.modules if m == "xarm" or m.startswith("xarm.")]:
        monkeypatch.delitem(sys.modules, mod_name)
    monkeypatch.delitem(sys.modules, "dimos.hardware.manipulators.xarm.adapter", raising=False)
    monkeypatch.setitem(sys.modules, "xarm", None)
    with pytest.raises(ImportError, match="xarm"):
        registry.create("xarm")
    assert "xarm" in registry.available()


def test_unknown_adapter_raises_keyerror() -> None:
    with pytest.raises(KeyError, match="Unknown adapter"):
        adapter_registry.create("no_such_adapter")


class _DirectAdapter:
    def __init__(self, **kwargs: object) -> None:
        self.kwargs = kwargs


def test_direct_register_still_works() -> None:
    registry = AdapterRegistry()
    registry.register("direct_test", _DirectAdapter)
    assert "direct_test" in registry.available()
    assert isinstance(registry.create("direct_test", dof=6), _DirectAdapter)


def test_conflicting_manifest_paths_raise() -> None:
    registry = AdapterRegistry()
    with pytest.raises(ValueError, match="Duplicate"):
        registry.register_path("xarm", "somewhere.else:Thing")
