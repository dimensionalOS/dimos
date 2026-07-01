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

from dimos.simulation.scene_assets.spec import (
    MujocoComposedBinary,
    SceneMeshAlignment,
    ScenePackage,
    load_scene_package,
)


def test_scene_package_roundtrips_mujoco_binaries(tmp_path: Path) -> None:
    package_dir = tmp_path / "store"
    mujoco_scene_path = package_dir / "mujoco" / "hash" / "wrapper.xml"
    mujoco_binary_path = package_dir / "mujoco" / "hash" / "wrapper.mjb"
    composed_path = package_dir / "mujoco" / "composed" / "g1_static.mjb"
    for path in (mujoco_scene_path, mujoco_binary_path, composed_path):
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(b"asset")

    package = ScenePackage(
        package_dir=package_dir,
        source_path=tmp_path / "source.blend",
        alignment=SceneMeshAlignment(),
        mujoco_scene_path=mujoco_scene_path,
        mujoco_binary_path=mujoco_binary_path,
        mujoco_composed_binaries={
            "g1_static": MujocoComposedBinary(
                key="g1_static",
                path=composed_path,
                robot="unitree-g1-groot-wbc",
                entity_policy="static-only",
                spawn={"xy": [9.2, 11.8], "yaw": -1.57},
                metadata={"mujoco_version": "3.3.0"},
            )
        },
    )

    metadata_path = package.write_metadata()
    loaded = load_scene_package(metadata_path)

    assert loaded.mujoco_scene_path == mujoco_scene_path.resolve()
    assert loaded.mujoco_binary_path == mujoco_binary_path.resolve()
    binary = loaded.mujoco_composed_binary(
        key="g1_static",
        robot="unitree-g1-groot-wbc",
        entity_policy="static-only",
    )
    assert binary is not None
    assert binary.path == composed_path.resolve()
    assert binary.spawn == {"xy": [9.2, 11.8], "yaw": -1.57}
    assert binary.metadata == {"mujoco_version": "3.3.0"}
    assert (
        loaded.mujoco_composed_binary_path(
            robot="unitree-g1-groot-wbc",
            entity_policy="static-only",
        )
        == composed_path.resolve()
    )
    assert loaded.mujoco_composed_binary(key="g1_static", robot="other") is None
