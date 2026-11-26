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

from __future__ import annotations

from collections.abc import Iterable
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
import threading

import pytest

from dimos.sim2 import usd_layer

pytest.importorskip("pxr")


def test_threads_writing_the_same_layer_at_once_all_get_it() -> None:
    writers, turns = 8, 50
    start = threading.Barrier(writers)

    def write(turn: int) -> Path:
        start.wait()
        return usd_layer.layer_file(f"#usda 1.0\n# {turn}\n", called="race")

    with ThreadPoolExecutor(writers) as pool:
        for turn in range(turns):
            layers = list(pool.map(write, [turn] * writers))
            assert len(set(layers)) == 1
            assert layers[0].read_text() == f"#usda 1.0\n# {turn}\n"


def test_a_mesh_the_layer_names_that_is_a_git_lfs_pointer_is_fetched_before_it_is_read(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    from dimos.robot.assets.model import UsdModel
    from dimos.robot.assets.usd_utils import lfs
    from dimos.utils.data import _is_lfs_pointer_file

    meshes = tmp_path / "robot" / "meshes"
    meshes.mkdir(parents=True)
    (meshes / "arm.usdc").write_text("version https://git-lfs.github.com/spec/v1\noid sha256:0\n")
    (meshes / "leg.usdc").write_bytes(b"PXR-USDC already fetched")
    robot = tmp_path / "robot" / "robot.usda"
    references = "".join(
        f'def Xform "{name}" (references = @./meshes/{name}.usdc@) {{}}\n'
        for name in ("arm", "leg")
    )
    robot.write_text(f"#usda 1.0\n{references}")
    pulled: list[list[Path]] = []

    def pull(paths: Iterable[Path]) -> None:
        pulled.append(sorted(path for path in paths if _is_lfs_pointer_file(path)))
        for path in pulled[-1]:
            path.write_bytes(b"PXR-USDC fetched")

    monkeypatch.setattr(lfs, "pull_lfs_files", pull)

    usd_layer.usd_layer(UsdModel.from_file(robot))

    # The layer itself is checked too; it is no pointer, so only the mesh is fetched.
    assert [paths for paths in pulled if paths] == [[meshes / "arm.usdc"]]
    assert (meshes / "arm.usdc").read_bytes() == b"PXR-USDC fetched"
