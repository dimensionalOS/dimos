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

from pathlib import Path
from types import SimpleNamespace

from pytest_mock import MockerFixture

from dimos.robot.assets.git_cache import GitAssetCache
from dimos.simulation.mujoco import model


def test_model_reads_meshes_from_the_pinned_checkout(tmp_path: Path, mocker: MockerFixture) -> None:
    mesh = tmp_path / "unitree_go1/assets/hip.stl"
    mesh.parent.mkdir(parents=True)
    mesh.write_bytes(b"hip mesh")
    mocker.patch.object(model, "get_data", return_value=tmp_path)
    resolve = mocker.patch.object(GitAssetCache, "resolve", return_value=tmp_path)
    mocker.patch(
        "importlib.util.find_spec",
        return_value=SimpleNamespace(submodule_search_locations=[str(tmp_path)]),
    )

    assert model.get_assets().get("hip.stl") == b"hip mesh"
    resolve.assert_called_once_with(
        "https://github.com/google-deepmind/mujoco_menagerie.git",
        "1b86ece576591213e2b666ebf59508454200ca97",
    )
