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

from collections.abc import Iterator
from pathlib import Path

import numpy as np
import pytest

pytest.importorskip("mujoco")

import mujoco

from dimos.simulation.perception.mujoco_surface import (
    sample_body_surface,
    sample_scene_surface,
)
from dimos.simulation.perception.sim_scene_registration import SimSceneRegistrationModule

pytestmark = pytest.mark.mujoco

_SCENE = """
<mujoco model="three-body">
  <compiler angle="radian"/>
  <worldbody>
    <geom name="floor" type="plane" size="5 5 0.1"/>
    <body name="work_table" pos="0 0 0.35">
      <geom name="table_top" type="box" size="0.6 0.4 0.02"/>
    </body>
    <body name="cube_red" pos="0.2 0.1 0.45">
      <freejoint name="cube_red_joint"/>
      <geom name="cube_red_geom" type="box" size="0.03 0.03 0.03" mass="0.2"/>
    </body>
    <body name="mug" pos="-0.2 -0.1 0.45">
      <freejoint name="mug_joint"/>
      <geom name="mug_geom" type="cylinder" size="0.04 0.05" mass="0.3"/>
    </body>
  </worldbody>
</mujoco>
"""


class _FakeSim:
    """Stands in for MujocoSimModule's geometry RPCs, against a real model."""

    def __init__(self, model: mujoco.MjModel, data: mujoco.MjData) -> None:
        self.model, self.data = model, data

    def list_body_names(self) -> list[str]:
        names = [
            mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY, i)
            for i in range(1, int(self.model.nbody))
        ]
        return [n for n in names if n]

    def get_body_poses(self, names: list[str]) -> dict[str, list[float]]:
        raise NotImplementedError

    def get_body_geoms(self, name: str) -> list:
        raise NotImplementedError

    def sample_body_surface(self, name: str, count: int = 512) -> list[list[float]]:
        return sample_body_surface(self.model, self.data, name, count).tolist()

    def sample_scene_surface(
        self, exclude: list[str] | None = None, voxel_size: float = 0.01, count: int = 20000
    ) -> list[list[float]]:
        return sample_scene_surface(
            self.model, self.data, tuple(exclude or ()), voxel_size=voxel_size, count=count
        ).tolist()


@pytest.fixture
def model_data(tmp_path: Path) -> tuple[mujoco.MjModel, mujoco.MjData]:
    path = tmp_path / "three_body.xml"
    path.write_text(_SCENE.strip())
    model = mujoco.MjModel.from_xml_path(str(path))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    return model, data


@pytest.fixture
def module(
    model_data: tuple[mujoco.MjModel, mujoco.MjData],
) -> Iterator[SimSceneRegistrationModule]:
    mod = SimSceneRegistrationModule(aliases={"red block": "cube_red"})
    mod._sim = _FakeSim(*model_data)
    yield mod
    mod.stop()


def test_scan_matches_prompts_to_bodies_and_skips_the_scenery(
    module: SimSceneRegistrationModule,
) -> None:
    detections = module.scan_scene(["cube_red", "mug", "red block", "work_table"])

    by_id = {d.id: d.results[0].hypothesis.class_id for d in detections.detections}
    # work_table is scenery, so no prompt of that name resolves to an object.
    assert set(by_id) == {"cube_red", "mug"}
    # "red block" resolves through the alias onto the same body as "cube_red";
    # one body cannot be two objects, so the later prompt wins the id.
    assert by_id["mug"] == "mug"
    assert detections.header.frame_id == "world"
    for detection in detections.detections:
        assert detection.bbox.center.position.z == pytest.approx(0.45, abs=0.02)


def test_object_cloud_lies_on_the_requested_body(
    module: SimSceneRegistrationModule,
) -> None:
    module.scan_scene(["cube_red"])
    cloud = module.get_object_pointcloud_by_object_id("cube_red")

    assert cloud is not None
    points, _ = cloud.as_numpy()
    assert cloud.frame_id == "world"
    assert len(points) > 100
    # Every sample sits on the 6 cm cube centred at (0.2, 0.1, 0.45).
    offsets = np.abs(points - np.array([0.2, 0.1, 0.45]))
    assert offsets.max() == pytest.approx(0.03, abs=1e-3)
    assert np.isclose(offsets.max(axis=1), 0.03, atol=1e-3).all()
    assert module.get_object_pointcloud_by_name("cube_red") is not None
    assert module.get_object_pointcloud_by_object_id("nope") is None


def test_full_scene_cloud_drops_the_excluded_object(
    module: SimSceneRegistrationModule,
) -> None:
    module.scan_scene(["cube_red", "mug"])
    everything = module.get_full_scene_pointcloud(voxel_size=0.01)
    without_cube = module.get_full_scene_pointcloud(exclude_object_id="cube_red", voxel_size=0.01)

    assert everything is not None and without_cube is not None
    all_points, _ = everything.as_numpy()
    kept, _ = without_cube.as_numpy()
    cube_centre = np.array([0.2, 0.1, 0.45])
    assert (np.linalg.norm(all_points - cube_centre, axis=1) < 0.06).any()
    assert not (np.linalg.norm(kept - cube_centre, axis=1) < 0.06).any()
    # The table and the mug survive the exclusion.
    assert (np.linalg.norm(kept - np.array([-0.2, -0.1, 0.45]), axis=1) < 0.08).any()
