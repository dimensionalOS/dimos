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

"""The exported per-bundle lockfiles are current and agree with one another."""

import os
from pathlib import Path
import re
import subprocess

import pytest

from dimos.constants import DIMOS_PROJECT_ROOT
from dimos.deps.bundles import LOCKS_DIR, load_assignments, lock_path
from dimos.deps.export_locks import BACKENDS, main


def test_lock_exports_are_current() -> None:
    """Regenerates locally (like all_blueprints.py); CI only compares."""
    if "CI" in os.environ:
        assert main(["--check"]) == 0, "run `python -m dimos.deps.export_locks` and commit"
        return
    assert main([]) == 0
    changed = subprocess.run(
        ["git", "diff", "--quiet", "--", str(LOCKS_DIR)], cwd=DIMOS_PROJECT_ROOT
    ).returncode
    if changed:
        pytest.fail("dimos/deps/locks was regenerated and has uncommitted changes; commit them")


def _blocks(path: Path) -> dict[tuple[str, str], str]:
    """One text block per locked package, keyed by (name, marker)."""
    blocks: dict[tuple[str, str], str] = {}
    for block in path.read_text().split("\n[[packages]]\n")[1:]:
        name = re.search(r'^name = "([^"]+)"$', block, re.MULTILINE)
        marker = re.search(r'^marker = "([^"]*)"$', block, re.MULTILINE)
        assert name is not None, f"{path.name}: package block without a name"
        blocks[(name.group(1), marker.group(1) if marker else "")] = block
    return blocks


@pytest.mark.parametrize("backend", BACKENDS)
def test_lock_exports_of_one_backend_agree_on_shared_packages(backend: str) -> None:
    """Sequential installs of several bundles are only valid when their exports agree."""
    bundles = sorted(set(load_assignments().values()))
    exports = {bundle: _blocks(lock_path(bundle, backend)) for bundle in bundles}

    disagreements = []
    for bundle, blocks in exports.items():
        for other, other_blocks in exports.items():
            for key, block in blocks.items():
                if key in other_blocks and other_blocks[key] != block:
                    disagreements.append((key[0], bundle, other))

    assert disagreements == []
    assert all(("dimos", marker) not in blocks for blocks in exports.values() for marker in [""])


def test_every_bundle_has_both_backend_exports() -> None:
    expected = {
        lock_path(bundle, backend).name
        for bundle in set(load_assignments().values())
        for backend in BACKENDS
    }

    assert {path.name for path in LOCKS_DIR.glob("pylock.*.toml")} == expected


def test_cuda_exports_carry_the_gpu_onnxruntime_and_cpu_exports_do_not() -> None:
    cpu = _blocks(lock_path("runtime-common", "cpu"))
    cuda = _blocks(lock_path("runtime-common", "cuda"))

    assert not any(name == "onnxruntime-gpu" for name, _ in cpu)
    assert any(name == "onnxruntime-gpu" for name, _ in cuda)
    assert any(name == "onnxruntime" for name, _ in cuda), "chromadb still pulls the CPU build in"
