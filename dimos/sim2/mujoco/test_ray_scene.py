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


def _world(boxes: int = 0) -> mujoco.MjModel:
    rng = np.random.default_rng(3)
    crowd = "".join(
        f'<geom type="box" size="0.05 0.05 0.05" pos="{x:.3f} {y:.3f} {z:.3f}"/>'
        for x, y, z in rng.uniform([-8, -8, 0], [8, 8, 3], (boxes, 3))
    )
    return mujoco.MjModel.from_xml_string(
        f"""
        <mujoco>
          <asset><mesh name="tetra" vertex="{_TETRA}"/></asset>
          <worldbody>
            <geom type="plane" size="10 10 1"/>
            <geom type="box" size="0.5 0.5 1" pos="3 0 1" euler="0 0 30"/>
            <geom type="mesh" mesh="tetra" pos="-2 1 0.5"/>
            <geom type="cylinder" size="0.2 1" pos="0 -3 1"/>
            {crowd}
            <body name="walker" pos="1 2 1">
              <freejoint/>
              <geom type="mesh" mesh="tetra"/>
              <geom type="sphere" size="0.2" pos="0 0 0.6"/>
              <body name="arm" pos="0.5 0 0">
                <joint type="hinge" axis="0 0 1"/>
                <geom type="capsule" size="0.05 0.3" euler="0 90 0"/>
                <geom type="box" size="0.1 0.1 0.1" pos="0.4 0 0"/>
              </body>
            </body>
          </worldbody>
        </mujoco>
        """
    )


def _rays(count: int) -> np.ndarray:
    directions = np.random.default_rng(0).normal(size=(count, 3))
    return directions / np.linalg.norm(directions, axis=1, keepdims=True)


def _mujoco_cast(model, data, origin, directions, excluded=-1):
    n = len(directions)
    dist, geom = np.full(n, -1.0), np.full(n, -1, dtype=np.int32)
    normals = np.zeros(3 * n)
    mujoco.mj_multiRay(
        model, data, origin, directions.ravel(), None, 1, excluded, geom, dist, normals, n, 20.0
    )
    dist[geom < 0] = -1.0
    return dist, normals.reshape(n, 3)


def test_the_cast_hits_what_mujoco_hits_as_the_moving_bodies_move():
    model = _world()
    data = mujoco.MjData(model)
    mujoco.mj_kinematics(model, data)
    scene = RayScene(model, data, (0,))
    origin, directions = np.array([0.0, 0.0, 1.5]), _rays(20_000)
    data.qpos[:7] = [1.5, 1.0, 1.2, np.cos(0.4), 0.0, np.sin(0.4), 0.0]
    data.qpos[7] = 1.0
    mujoco.mj_kinematics(model, data)

    ours, our_normals = scene.cast(origin, directions, 20.0)
    theirs, their_normals = _mujoco_cast(model, data, origin, directions)

    np.testing.assert_array_equal(ours > 0, theirs > 0)
    both = ours > 0
    np.testing.assert_allclose(ours[both], theirs[both], atol=1e-4)
    assert np.sum(our_normals[both] * their_normals[both], axis=1) == pytest.approx(1.0, abs=1e-4)


def test_a_body_it_is_blind_to_lets_its_rays_through():
    model = _world()
    data = mujoco.MjData(model)
    mujoco.mj_kinematics(model, data)
    walker = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "walker")
    origin, directions = np.array([0.0, 0.0, 1.5]), _rays(20_000)

    ours, _ = RayScene(model, data, (0,), {walker}).cast(origin, directions, 20.0)
    theirs, _ = _mujoco_cast(model, data, origin, directions, excluded=walker)

    np.testing.assert_array_equal(ours > 0, theirs > 0)


def test_a_cast_does_not_slow_with_every_geom_in_the_world():
    model = _world(boxes=4000)
    data = mujoco.MjData(model)
    mujoco.mj_kinematics(model, data)
    scene = RayScene(model, data, (0,))
    origin, directions = np.array([0.0, 0.0, 1.5]), _rays(20_000)

    def seconds(cast) -> float:
        cast()
        start = time.perf_counter()
        cast()
        return time.perf_counter() - start

    ours = seconds(lambda: scene.cast(origin, directions, 20.0))
    theirs = seconds(lambda: _mujoco_cast(model, data, origin, directions))
    assert ours < theirs / 5, f"{ours * 1e3:.1f} ms against mj_multiRay's {theirs * 1e3:.1f} ms"
