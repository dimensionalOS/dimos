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

"""The vendored menagerie snapshot is the pinned bytes, or nothing is.

Every fitted knob in this package is a DELTA on the go2 scene at
``MENAGERIE_COMMIT``; if the vendored tree drifts, every acceptance number
means something else. These tests FAIL (never skip) when the vendored assets
are absent or altered — the assets ship with the repo, so their absence is a
real failure, unlike the optional recordings.
"""

import hashlib
from pathlib import Path

import pytest

pytestmark = pytest.mark.go2sim


def _vendored_root() -> Path:
    # Deliberately NOT scene_path(): that honours the MUJOCO_MENAGERIE
    # developer override, and this test is about the vendored bytes even when
    # the environment points elsewhere.
    from dimos.utils.data import get_data

    return Path(get_data("go2_menagerie"))


def _tree_sha256(root: Path) -> str:
    lines = []
    for f in sorted(p for p in root.rglob("*") if p.is_file() and p.name != "PROVENANCE.md"):
        digest = hashlib.sha256(f.read_bytes()).hexdigest()
        lines.append(f"{digest}  {f.relative_to(root).as_posix()}")
    return hashlib.sha256(("\n".join(lines) + "\n").encode()).hexdigest()


def test_the_vendored_tree_is_byte_identical_to_the_pinned_menagerie_commit():
    from dimos.robot.unitree.go2.sim.engines.model import MENAGERIE_TREE_SHA256

    root = _vendored_root()
    assert (root / "unitree_go2" / "scene.xml").is_file()
    assert _tree_sha256(root) == MENAGERIE_TREE_SHA256, (
        "vendored menagerie assets differ from the pinned tree: every fitted "
        "number in this package is a delta on those bytes — re-vendor "
        "deliberately (data/go2_menagerie/PROVENANCE.md), never silently"
    )


def test_the_vendored_snapshot_carries_its_license_and_provenance():
    from dimos.robot.unitree.go2.sim.engines.model import MENAGERIE_COMMIT

    root = _vendored_root()
    # Menagerie's LICENSE is an aggregate, one section per model directory;
    # the section that licenses what we redistribute is Unitree's (BSD-3).
    text = (root / "LICENSE").read_text()
    assert "License for contents in the directory 'unitree_go2/'" in text
    assert "Unitree Robotics" in text
    assert MENAGERIE_COMMIT in (root / "PROVENANCE.md").read_text()


def test_the_vendored_scene_compiles_without_the_environment_override():
    mujoco = pytest.importorskip("mujoco")
    from dimos.robot.unitree.go2.sim.engines.model import load

    model, _ = load(_vendored_root())
    assert model.opt.timestep == pytest.approx(0.002)
    assert mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "imu") >= 0
