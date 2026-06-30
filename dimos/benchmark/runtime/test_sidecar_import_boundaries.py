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

"""Import-boundary tests for runtime sidecar packages."""

from __future__ import annotations

from pathlib import Path
import subprocess
import sys

REPO_ROOT = Path(__file__).resolve().parents[3]
PROTOCOL_SRC = REPO_ROOT / "packages" / "dimos-runtime-protocol" / "src"
ROBOSUITE_SIDECAR_SRC = REPO_ROOT / "packages" / "dimos-robosuite-sidecar" / "src"
LIBERO_PRO_SIDECAR_SRC = REPO_ROOT / "packages" / "dimos-libero-pro-sidecar" / "src"


def test_robosuite_sidecar_import_does_not_import_heavy_backends_or_dimos() -> None:
    script = """
import importlib
import sys

importlib.import_module('dimos_robosuite_sidecar.server')
for name in ('dimos', 'robosuite', 'libero', 'omnigibson'):
    if name in sys.modules:
        raise SystemExit(f'unexpected import: {name}')
"""
    result = subprocess.run(
        [sys.executable, "-c", script],
        check=False,
        capture_output=True,
        text=True,
        env={
            "PYTHONPATH": f"{PROTOCOL_SRC}:{ROBOSUITE_SIDECAR_SRC}",
        },
    )
    assert result.returncode == 0, result.stderr or result.stdout


def test_libero_pro_sidecar_import_does_not_import_heavy_backends_or_dimos() -> None:
    script = """
import importlib
import sys

importlib.import_module('dimos_libero_pro_sidecar.server')
for name in ('dimos', 'robosuite', 'libero', 'torch'):
    if name in sys.modules:
        raise SystemExit(f'unexpected import: {name}')
"""
    result = subprocess.run(
        [sys.executable, "-c", script],
        check=False,
        capture_output=True,
        text=True,
        env={
            "PYTHONPATH": f"{PROTOCOL_SRC}:{LIBERO_PRO_SIDECAR_SRC}",
        },
    )
    assert result.returncode == 0, result.stderr or result.stdout
