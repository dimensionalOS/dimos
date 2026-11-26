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


from pathlib import Path

import pytest
from pytest_mock import MockerFixture

from dimos.robot.assets.model import UsdModel
from dimos.robot.assets.usd_utils import lfs

_POINTER = "version https://git-lfs.github.com/spec/v1\noid sha256:0\n"


def test_only_the_pointers_are_fetched_in_one_pull(mocker: MockerFixture, tmp_path: Path) -> None:
    pointer, fetched = tmp_path / "a.usdc", tmp_path / "b.usdc"
    pointer.write_text(_POINTER)
    fetched.write_bytes(b"mesh")
    mocker.patch.object(lfs, "get_project_root", return_value=tmp_path)
    mocker.patch.object(lfs, "_initialize_git_lfs")
    run = mocker.patch(
        "dimos.robot.assets.usd_utils.lfs.subprocess.run",
        side_effect=lambda *_a, **_k: pointer.write_bytes(b"mesh"),
    )

    lfs.pull_lfs_files([pointer, fetched])
    lfs.pull_lfs_files([pointer, fetched])

    run.assert_called_once_with(
        ["git", "lfs", "pull", "--include", "a.usdc", "--exclude="],
        cwd=tmp_path.resolve(),
        check=True,
    )


def test_a_pointer_it_could_not_fetch_is_named(mocker: MockerFixture, tmp_path: Path) -> None:
    pointer = tmp_path / "a.usdc"
    pointer.write_text(_POINTER)
    mocker.patch.object(lfs, "get_project_root", return_value=tmp_path / "elsewhere")
    run = mocker.patch("dimos.robot.assets.usd_utils.lfs.subprocess.run")

    with pytest.raises(RuntimeError, match="a.usdc"):
        lfs.pull_lfs_files([pointer])
    run.assert_not_called()


def test_a_layers_relative_asset_names_are_fetched_from_beside_it(
    mocker: MockerFixture, tmp_path: Path
) -> None:
    (tmp_path / "meshes").mkdir()
    hull = tmp_path / "meshes" / "hull.usdc"
    hull.write_text(_POINTER)
    scene = tmp_path / "scene.usda"
    scene.write_text('#usda 1.0\ndef Xform "room" (prepend references = @meshes/hull.usdc@) {}\n')
    pull = mocker.patch.object(lfs, "pull_lfs_files")

    lfs.pull_model_assets(UsdModel.from_file(scene))

    assert set(pull.call_args.args[0]) == {hull}
