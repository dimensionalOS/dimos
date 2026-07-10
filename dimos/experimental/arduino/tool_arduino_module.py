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

"""Registry-vs-disk checks for dimos.experimental.arduino.arduino_module.

These resolve the real Arduino message headers through ``nix build`` and so
are kept out of the unit-test file — they need nix on PATH and the
``dimos_arduino_tools`` flake output.

Run with::

    uv run pytest dimos/experimental/arduino/tool_arduino_module.py -v
"""

from __future__ import annotations

from pathlib import Path
import shutil
import subprocess

import pytest

from dimos.experimental.arduino.arduino_module import (
    _ARDUINO_HW_DIR,
    _KNOWN_TYPE_HEADERS,
)


def _arduino_common_dir() -> Path:
    """Resolve Arduino message headers (in dimos-lcm) via nix; skips if nix is absent."""
    if shutil.which("nix") is None:
        pytest.skip("nix not available — cannot resolve Arduino message headers")

    result = subprocess.run(
        ["nix", "build", ".#dimos_arduino_tools", "--print-out-paths", "--no-link"],
        cwd=str(_ARDUINO_HW_DIR),
        capture_output=True,
        text=True,
        timeout=120,
    )
    if result.returncode != 0:
        pytest.skip(f"nix build failed: {result.stderr[:200]}")

    out_paths = [line for line in result.stdout.splitlines() if line.strip()]
    if not out_paths:
        pytest.skip("nix build returned no paths")

    msgs_dir = Path(out_paths[-1]) / "share" / "arduino_msgs"
    if not msgs_dir.is_dir():
        pytest.skip(f"Arduino message headers not found at {msgs_dir}")

    return msgs_dir


def test_registry_headers_exist_on_disk() -> None:
    common = _arduino_common_dir()
    missing = [
        (msg_name, header)
        for msg_name, header in _KNOWN_TYPE_HEADERS.items()
        if not (common / header).is_file()
    ]
    assert not missing, (
        f"Every entry in _KNOWN_TYPE_HEADERS must point to an existing "
        f"arduino_msgs header, but these are missing: {missing}"
    )


def test_registry_headers_cover_all_arduino_msgs_files() -> None:
    """Every header referenced by _KNOWN_TYPE_HEADERS must exist on disk.
    Extra generated headers (from dimos-lcm codegen) and infrastructure
    headers (lcm_coretypes_arduino.h, dimos_lcm_pubsub.h) are allowed
    without registry entries since they are auto-generated dependencies,
    not user-facing message types."""
    common = _arduino_common_dir()
    on_disk = {str(p.relative_to(common)) for p in common.rglob("*.h")}
    referenced = set(_KNOWN_TYPE_HEADERS.values())
    missing = referenced - on_disk
    assert not missing, (
        f"These headers are referenced by _KNOWN_TYPE_HEADERS but missing "
        f"on disk: {sorted(missing)}"
    )
