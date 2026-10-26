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

import mujoco
import numpy as np
import pytest

from dimos.simulation.mujoco.raycast_lidar import EmbreeLidar


@pytest.mark.parametrize(
    "geom,atol",
    [
        ('type="mesh" mesh="shape"', 1e-4),
        ('type="plane" size="2 2 0.1"', 1e-4),
        ('type="plane" size="0 0 0.1"', 1e-4),
        ('type="box" size="0.3 0.4 0.5"', 1e-4),
        ('type="sphere" size="0.5"', 1e-3),
        ('type="cylinder" size="0.5 0.6"', 1e-3),
        ('type="capsule" size="0.4 0.4"', 1e-3),
        ('type="ellipsoid" size="0.4 0.5 0.6"', 1e-3),
    ],
    ids=["mesh", "plane", "infinite_plane", "box", "sphere", "cylinder", "capsule", "ellipsoid"],
)
@pytest.mark.parametrize("moving", [False, True], ids=["static", "mocap"])
def test_geom_hits_match_mj_ray(geom: str, atol: float, moving: bool) -> None:
    body = 'mocap="true"' if moving else ""
    model = mujoco.MjModel.from_xml_string(f"""
        <mujoco>
          <compiler angle="radian"/>
          <asset>
            <mesh name="shape" scale="0.8 1.1 0.9"
              vertex="-0.4 -0.4 -0.4  0.4 -0.4 -0.4  0.4 0.4 -0.4  -0.4 0.4 -0.4
                      -0.4 -0.4 0.4   0.4 -0.4 0.4   0.4 0.4 0.4   -0.4 0.4 0.4"
              face="0 2 1  0 3 2  4 5 6  4 6 7  0 1 5  0 5 4
                    1 2 6  1 6 5  2 3 7  2 7 6  3 0 4  3 4 7"/>
          </asset>
          <worldbody>
            <body name="robot" pos="0 0 1">
              <freejoint/><geom type="sphere" size="0.01" group="3"/>
              <site name="lidar"/>
            </body>
            <body pos="1.8 0 1" {body}>
              <geom name="target" {geom} euler="0.2 -1.1 0.3" pos="0.1 0.05 0.05"/>
            </body>
          </worldbody>
        </mujoco>
    """)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    site_id = model.site("lidar").id
    origin = data.site_xpos[site_id]
    # Off-axis directions hit caps, curved sides, mesh faces and triangle interiors.
    targets = data.geom_xpos[model.geom("target").id] + np.array(
        [[0, 0, 0], [0, 0.11, 0.09], [0, -0.12, -0.1], [0, 0.21, -0.11], [0, -0.08, 0.23]]
    )
    directions = targets - origin
    directions /= np.linalg.norm(directions, axis=1, keepdims=True)
    lidar = EmbreeLidar(model, data, site_id, directions)
    geom_id = np.empty(1, dtype=np.int32)
    groups = np.array([1, 1, 1, 0, 0, 0], dtype=np.uint8)
    distances = np.array(
        [mujoco.mj_ray(model, data, origin, ray, groups, 1, -1, geom_id) for ray in directions]
    )
    assert np.all(distances > 0)
    expected = origin + directions * distances[:, None]

    actual = lidar.cast(data, 3.0)

    np.testing.assert_allclose(actual, expected, atol=atol, rtol=0)


@pytest.mark.parametrize("geom_type", [mujoco.mjtGeom.mjGEOM_HFIELD, mujoco.mjtGeom.mjGEOM_SDF])
def test_unsupported_geometry_raises_at_construction(geom_type: mujoco.mjtGeom) -> None:
    model = mujoco.MjModel.from_xml_string("""
        <mujoco><worldbody>
          <geom name="unsupported" type="box" size="1 1 1"/>
          <body><freejoint/><geom type="sphere" size="0.01"/>
            <site name="lidar"/>
          </body>
        </worldbody></mujoco>
    """)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    # Avoid needing an SDF plugin or heightfield asset just to exercise rejection.
    model.geom_type[model.geom("unsupported").id] = geom_type

    with pytest.raises(ValueError, match=f"Unsupported lidar geom type {geom_type.name}"):
        EmbreeLidar(model, data, model.site("lidar").id, np.array([[1.0, 0, 0]]))
