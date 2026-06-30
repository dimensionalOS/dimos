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

"""Optional real LIBERO-PRO runtime demo coverage."""

from __future__ import annotations

import importlib.util
import os
from pathlib import Path
import sys
from types import ModuleType
from typing import Protocol, cast

import pytest

REPO_ROOT = Path(__file__).resolve().parents[3]
DEMO_SCRIPT = REPO_ROOT / "scripts" / "benchmarks" / "demo_libero_pro_runtime.py"
CONFIG_PATH = (
    REPO_ROOT / "dimos" / "benchmark" / "runtime" / "configs" / "libero_pro_goal_task0.json"
)

_DEMO_SPEC = importlib.util.spec_from_file_location("demo_libero_pro_runtime", DEMO_SCRIPT)
assert _DEMO_SPEC is not None
assert _DEMO_SPEC.loader is not None
_DEMO_MODULE = importlib.util.module_from_spec(_DEMO_SPEC)
sys.modules[_DEMO_SPEC.name] = _DEMO_MODULE
_DEMO_SPEC.loader.exec_module(_DEMO_MODULE)


class _RunDemo(Protocol):
    def __call__(self, config_path: Path) -> Path: ...


run_demo = cast("_RunDemo", cast("ModuleType", _DEMO_MODULE).__dict__["run_demo"])


@pytest.mark.self_hosted_large
def test_real_libero_pro_runtime_demo_with_prepared_assets() -> None:
    if os.environ.get("DIMOS_RUN_LIBERO_PRO_MANUAL_TEST") != "1":
        pytest.skip("set DIMOS_RUN_LIBERO_PRO_MANUAL_TEST=1 in a prepared LIBERO-PRO env")

    artifact_dir = run_demo(CONFIG_PATH)

    assert (artifact_dir / "score.json").exists()
    assert (artifact_dir / "protocol_trace_summary.json").exists()
    assert (artifact_dir / "motor_trace.json").exists()
