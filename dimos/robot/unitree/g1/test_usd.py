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

pytest.importorskip("pxr")


def test_the_layer_drives_every_joint_the_controller_commands() -> None:
    from dimos.control.components import make_humanoid_joints
    from dimos.robot.unitree.g1.sim2 import G1_USD, _usd_joint
    from dimos.sim2.mujoco.drives import usd_drives

    assert set(usd_drives(G1_USD.usd)) == {_usd_joint(name) for name in make_humanoid_joints("g1")}


@pytest.mark.self_hosted  # usd_layer fetches the G1's Git LFS meshes
def test_the_lidar_mount_is_a_frame_and_the_pelvis_imu_is_a_site() -> None:
    from dimos.robot.unitree.g1.sim2 import G1_USD, IMU_FRAME, G1Sim
    from dimos.sim2.mujoco.world import compile_world
    from dimos.sim2.usd_layer import usd_layer

    assert set(G1Sim.mounts) <= G1_USD.frame_names()
    assert IMU_FRAME not in G1_USD.frame_names()
    assert IMU_FRAME in {site.name for site in compile_world(usd_layer(G1_USD)).sites}


def test_every_g1_asset_resolves_to_a_file() -> None:
    import xml.etree.ElementTree as ET

    from dimos.robot.assets.usd_utils.assets import (
        LFS_POINTER,
        resolve_usd_assets,
        unopenable_assets,
    )
    from dimos.robot.unitree.g1.config import G1_JOINT_MODEL
    from dimos.robot.unitree.g1.sim2 import G1_USD

    assert G1_USD.source_path is not None
    root = str(G1_USD.source_path.parent)
    unopenable = unopenable_assets(resolve_usd_assets(G1_USD.usd, root), root)
    # A pointer is the file at the right path; hosted runners never fetch LFS, and usd_layer does.
    assert {asset: why for asset, why in unopenable.items() if why != LFS_POINTER} == {}
    meshes = [
        mesh.get("filename") or "" for mesh in ET.fromstring(G1_JOINT_MODEL.load().xml).iter("mesh")
    ]
    assert meshes
    assert [mesh for mesh in meshes if not (Path(mesh).is_absolute() and Path(mesh).exists())] == []
