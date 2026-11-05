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

"""Legacy quadruped simulation no longer loads the removed G1 implementation."""

import pytest

from dimos.simulation.mujoco import model
from dimos.simulation.mujoco.input_controller import InputController


def test_assets_keep_quadruped_meshes_without_loading_g1(tmp_path, mocker):
    for robot, data in [("unitree_go1", b"quadruped"), ("unitree_g1", b"humanoid")]:
        meshes = tmp_path / robot / "assets"
        meshes.mkdir(parents=True)
        (meshes / "body.stl").write_bytes(data)
    mocker.patch.object(model, "_get_data_dir", return_value=tmp_path)
    mocker.patch.object(model, "menagerie_path", return_value=tmp_path)
    mocker.patch.object(model, "get_data", return_value=tmp_path)

    assert model.get_assets() == {"body.stl": b"quadruped"}


def test_removed_robot_policy_fails_before_loading_assets(mocker):
    assets = mocker.patch.object(model, "get_assets")
    controller = mocker.Mock(spec=InputController)

    with pytest.raises(ValueError, match="Unknown robot policy: unitree_g1"):
        model.load_model(controller, "unitree_g1", "")

    assets.assert_not_called()
