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

"""The launcher must stay light: no runtime framework, no heavy native libraries."""

import subprocess
import sys

import pytest

MODULES = (
    "dimos.deps.predicates",
    "dimos.deps.imports",
    "dimos.deps.import_map",
    "dimos.deps.lock",
    "dimos.deps.requires",
    "dimos.deps.selectors",
    "dimos.deps.rules",
    "dimos.deps.analysis",
    "dimos.deps.catalog",
    "dimos.deps.environment",
    "dimos.deps.probe",
    "dimos.deps.profiles",
    "dimos.deps.uv",
    "dimos.deps.planning",
    "dimos.deps.policy",
    "dimos.deps.lease",
    "dimos.deps.managed",
    "dimos.deps.launch",
)
BANNED = ("cv2", "open3d", "rerun", "torch", "scipy", "numpy", "pydantic_settings", "zenoh")
BANNED_DIMOS = ("dimos.core.global_config", "dimos.core.module", "dimos.core.transport")


@pytest.mark.parametrize("module", MODULES)
def test_module_imports_only_bootstrap_dependencies(module: str) -> None:
    script = (
        f"import importlib, sys; importlib.import_module({module!r}); "
        f"loaded = set(sys.modules); "
        f"bad = [m for m in {BANNED!r} if m in loaded] + [m for m in {BANNED_DIMOS!r} if m in loaded]; "
        "assert not bad, bad"
    )
    result = subprocess.run([sys.executable, "-c", script], capture_output=True, text=True)
    assert result.returncode == 0, result.stderr
