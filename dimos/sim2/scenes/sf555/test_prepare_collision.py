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

"""Offline conversion must preserve walkable openings and metric placement."""

import xml.etree.ElementTree as ET

import mujoco
import numpy as np
import trimesh

from dimos.robot.assets.model import RobotModel
from dimos.sim2.geometry.mujoco import compile_scene
from dimos.sim2.scenes.sf555.prepare_collision import description, prepare_object
from dimos.sim2.scenes.urdf import Component, prepare_scene


def test_concave_doorway_stays_open_and_has_no_fictitious_floor(tmp_path):
    pieces = []
    for size, center in [
        ([0.2, 0.2, 2.2], [10, -0.6, 1.1]),
        ([0.2, 0.2, 2.2], [10, 0.6, 1.1]),
        ([0.2, 1.4, 0.2], [10, 0, 2.3]),
    ]:
        mesh = trimesh.creation.box(size)
        mesh.vertices += center
        pieces.append(mesh)
    source = trimesh.util.concatenate(pieces)
    np.savez(tmp_path / "door.npz", vertices=source.vertices, faces=source.faces)
    result = prepare_object((tmp_path, tmp_path, {"name": "door", "file": "door.npz"}))
    urdf = tmp_path / "door.urdf"
    urdf.write_text(ET.tostring(description({"objects": [result]}), encoding="unicode"))
    scene = prepare_scene({"door": Component(RobotModel.from_file(urdf))}, ground_plane=False)
    model = compile_scene(scene, 2_000_000)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    groups = np.array([0, 0, 0, 1, 1, 1], dtype=np.uint8)
    # URDF collision geoms are group 0; use contype to select them explicitly.
    model.geom_group[:] = (model.geom_contype > 0).astype(np.int32)
    groups[:] = [0, 1, 0, 0, 0, 0]
    hit = np.zeros(1, dtype=np.int32)
    assert (
        mujoco.mj_ray(
            model, data, np.array([9.0, 0, 1.0]), np.array([1.0, 0, 0]), groups, 1, -1, hit
        )
        == -1
    )
    assert (
        abs(
            mujoco.mj_ray(
                model, data, np.array([9.0, 0.6, 1.0]), np.array([1.0, 0, 0]), groups, 1, -1, hit
            )
            - 0.9
        )
        < 0.02
    )
    assert (
        mujoco.mj_ray(
            model, data, np.array([0.0, 0, 1.0]), np.array([0.0, 0, -1]), groups, 1, -1, hit
        )
        == -1
    )
