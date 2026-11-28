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

import time

import mujoco
import numpy as np
import pytest

from dimos.sim2.mujoco.ray_scene import RayScene

_TETRA = "0 0 0  0.4 0 0  0 0.4 0  0 0 0.4"


def _mujoco_cast(model, data, origin, directions, excluded=-1):
    n = len(directions)
    dist, geom, normals = np.full(n, -1.0), np.full(n, -1, dtype=np.int32), np.zeros(3 * n)
    mujoco.mj_multiRay(
        model, data, origin, directions.ravel(), None, 1, excluded, geom, dist, normals, n, 20.0
    )
    dist[geom < 0] = -1.0
    return dist, normals.reshape(n, 3)


def _seconds(cast) -> float:
    cast()
    start = time.perf_counter()
    cast()
    return time.perf_counter() - start


def test_the_cast_hits_what_mujoco_hits_as_bodies_move_and_beats_it_in_a_crowd():
    crowd = "".join(
        f'<geom type="box" size="0.05 0.05 0.05" pos="{x:.3f} {y:.3f} {z:.3f}"/>'
        for x, y, z in np.random.default_rng(3).uniform([-8, -8, 0], [8, 8, 3], (4000, 3))
    )
    model = mujoco.MjModel.from_xml_string(f"""
        <mujoco><asset><mesh name="tetra" vertex="{_TETRA}"/></asset><worldbody>
          <geom type="plane" size="10 10 1"/><geom type="box" size="0.5 0.5 1" pos="3 0 1" euler="0 0 30"/>
          <geom type="mesh" mesh="tetra" pos="-2 1 0.5"/><geom type="cylinder" size="0.2 1" pos="0 -3 1"/>
          {crowd}
          <body name="walker" pos="1 2 1">
            <freejoint/><geom type="mesh" mesh="tetra"/><geom type="sphere" size="0.2" pos="0 0 0.6"/>
            <body name="arm" pos="0.5 0 0">
              <joint type="hinge" axis="0 0 1"/><geom type="capsule" size="0.05 0.3" euler="0 90 0"/>
              <geom type="box" size="0.1 0.1 0.1" pos="0.4 0 0"/>
            </body>
          </body>
        </worldbody></mujoco>""")
    data = mujoco.MjData(model)
    mujoco.mj_kinematics(model, data)
    walker = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "walker")
    directions = np.random.default_rng(0).normal(size=(20_000, 3))
    directions /= np.linalg.norm(directions, axis=1, keepdims=True)
    origin = np.array([0.0, 0.0, 1.5])
    # A body it is blind to lets its rays through, as mj_multiRay's excluded body does.
    blind, _ = RayScene(model, data, (0,), {walker}).cast(origin, directions, 20.0)
    blind_theirs, _ = _mujoco_cast(model, data, origin, directions, excluded=walker)
    np.testing.assert_array_equal(blind > 0, blind_theirs > 0)

    scene = RayScene(model, data, (0,))
    data.qpos[:8] = [1.5, 1.0, 1.2, np.cos(0.4), 0.0, np.sin(0.4), 0.0, 1.0]
    mujoco.mj_kinematics(model, data)
    ours, our_normals = scene.cast(origin, directions, 20.0)
    theirs, their_normals = _mujoco_cast(model, data, origin, directions)
    # The BVH casts in float32 and MuJoCo in float64, so a ray grazing an edge may land on either face.
    agree = (ours > 0) == (theirs > 0)
    agree[ours > 0] &= np.isclose(ours[ours > 0], theirs[ours > 0], atol=1e-4)
    assert np.count_nonzero(~agree) <= len(directions) // 1000
    both = agree & (ours > 0)
    assert np.sum(our_normals[both] * their_normals[both], axis=1) == pytest.approx(1.0, abs=1e-4)

    # A cast does not slow with every geom in the world.
    ours_s = _seconds(lambda: scene.cast(origin, directions, 20.0))
    theirs_s = _seconds(lambda: _mujoco_cast(model, data, origin, directions))
    assert ours_s < theirs_s / 5, (
        f"{ours_s * 1e3:.1f} ms against mj_multiRay's {theirs_s * 1e3:.1f} ms"
    )
