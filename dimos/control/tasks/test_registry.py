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

"""CI guards: control tasks must never silently vanish from the registry."""

from __future__ import annotations

import importlib
import importlib.util
from pathlib import Path

import pytest

from dimos.control.tasks.registry import control_task_registry

# Heavy optional dependencies; a task factory failing on one of these still
# passes IF the dependency is not installed. Anything else (path typo,
# internal breakage) fails CI.
OPTIONAL_TASK_MODULES = {"onnxruntime", "pinocchio"}

# Task dirs that intentionally register nothing.
UNREGISTERED_TASK_DIRS: set[str] = set()


def test_every_task_dir_has_a_manifest() -> None:
    pkg = importlib.import_module("dimos.control.tasks")
    checked = 0
    for root in pkg.__path__:
        for child in sorted(Path(root).iterdir()):
            if not child.is_dir() or child.name.startswith(("_", ".")):
                continue
            if not any(not f.name.startswith("_") for f in child.rglob("*.py")):
                continue
            if child.name in UNREGISTERED_TASK_DIRS:
                continue
            manifest = child / "_registry.py"
            assert manifest.exists(), (
                f"{child} contains task code but no _registry.py; discover() would silently skip it"
            )
            manifest_mod = importlib.import_module(f"dimos.control.tasks.{child.name}._registry")
            names = set(manifest_mod.TASK_FACTORIES)
            assert names, f"{manifest} declares no tasks"
            missing = names - set(control_task_registry.available())
            assert not missing, f"{manifest} declares {missing} missing from available()"
            checked += 1
    assert checked > 0


def test_declared_task_factory_paths_resolve() -> None:
    for name, factory_path in sorted(control_task_registry._factory_paths.items()):
        module_name, attr = factory_path.split(":", maxsplit=1)
        try:
            module = importlib.import_module(module_name)
        except ModuleNotFoundError as exc:
            root = (exc.name or "").partition(".")[0]
            if root in OPTIONAL_TASK_MODULES and importlib.util.find_spec(root) is None:
                continue
            pytest.fail(f"{name}: importing {module_name!r} failed: {exc}")
        factory = getattr(module, attr, None)
        assert factory is not None, f"{name}: {module_name!r} has no attribute {attr!r}"
        assert callable(factory), f"{name}: {factory_path!r} is not callable"
