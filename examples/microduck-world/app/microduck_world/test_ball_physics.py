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

"""Check compiled contacts in the six-robot scene, including the benchmark ball."""

import mujoco
import numpy as np
import pytest
from dimos.robot.pollen.microduck.assets_fetch import ensure_assets
from microduck_world.ball_physics import FLOOR_NAMES
from microduck_world.football import BALL_NAMES, FootballMatch
from microduck_world.scene import load_world
from microduck_world.visual_scene import export_scene
from microduck_world.world_sim import WorldSimModule


@pytest.fixture(scope="module")
def world():
    assets = ensure_assets()
    module = WorldSimModule(
        scene_xml=load_world()[0].mujoco_scene_path,
        robot_mjcf=str(assets.robot_mjcf("default")),
        headless=True,
        auto_stand=False,
    )
    spec = module._compose_spec()
    return spec, spec.compile()


def test_threejs_receives_the_compiled_ball_radius(world):
    _, model = world
    scene = export_scene(model, {})
    geoms = {g["name"]: g for g in scene["geoms"]}
    for name in BALL_NAMES:
        assert geoms[name + "_geom"]["kind"] == "sphere"
        assert geoms[name + "_geom"]["size"] == [0.05, 0, 0]


@pytest.mark.parametrize("name", BALL_NAMES)
def test_every_ball_has_upstream_mass_inertia_contacts_and_clear_reset_height(world, name):
    spec, model = world
    geom, body = model.geom(name + "_geom"), model.body(name)
    np.testing.assert_allclose(geom.size, [0.05, 0, 0])
    np.testing.assert_allclose(body.mass, [0.03])
    np.testing.assert_allclose(body.inertia, [0.00003] * 3)
    np.testing.assert_allclose(geom.friction, [0.4, 0.01, 0.003])
    np.testing.assert_allclose(geom.solref, [0.03, 0.4])
    np.testing.assert_allclose(geom.solimp, [0.9, 0.95, 0.001, 0.5, 2])
    assert geom.condim == 6
    adr = int(model.joint(name + "_freejoint").qposadr[0])
    assert model.qpos0[adr + 2] == pytest.approx(0.051)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    assert all(geom.id not in c.geom or c.dist >= 0 for c in data.contact)
    data.qpos[adr + 2] = 1
    mujoco.mj_resetData(model, data)
    assert data.qpos[adr + 2] == pytest.approx(0.051)
    for patch in spec.geoms:
        if patch.name.startswith(name + "_patch_"):
            assert patch.mass == 0 and patch.contype == 0 and patch.conaffinity == 0
            # Scaling the paint must leave it at the enlarged ball's surface.
            assert 0.05 < np.linalg.norm(patch.pos) + patch.size[2] < 0.051


@pytest.mark.parametrize("ball", BALL_NAMES)
@pytest.mark.parametrize("floor", FLOOR_NAMES)
def test_compiled_ball_floor_contact_mixing(world, ball, floor):
    _, model = world
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    ground = model.geom(floor)
    sphere = model.geom(ball + "_geom")
    np.testing.assert_allclose(ground.friction, [1, 0.005, 0.0001])
    np.testing.assert_allclose(ground.solref, [0.02, 1])
    np.testing.assert_allclose(ground.solimp, [0.9, 0.95, 0.001, 0.5, 2])
    assert ground.condim == 3
    for geom in (ground, sphere):
        assert geom.priority == 0 and geom.solmix == 1
        assert geom.margin == 0 and geom.gap == 0
    adr = int(model.joint(ball + "_freejoint").qposadr[0])
    # A slight overlap only in this fixture exposes the compiled contact pair.
    data.qpos[adr : adr + 3] = data.geom_xpos[ground.id] + [0, 0, ground.size[2] + 0.0499]
    mujoco.mj_forward(model, data)
    contacts = [c for c in data.contact if set(c.geom) == {ground.id, sphere.id}]
    assert len(contacts) == 1
    contact = contacts[0]
    assert contact.dim == 6
    np.testing.assert_allclose(contact.friction, [1, 1, 0.01, 0.003, 0.003])
    np.testing.assert_allclose(contact.solref, [0.025, 0.7])
    np.testing.assert_allclose(contact.solimp, [0.9, 0.95, 0.001, 0.5, 2])
    assert model.opt.timestep == 0.005
    np.testing.assert_allclose(model.opt.gravity, [0, 0, -9.81])


@pytest.mark.parametrize("ball", BALL_NAMES)
@pytest.mark.parametrize("sign, expected", [(1, [1, 0]), (-1, [0, 1])])
def test_each_actual_scene_ball_physically_scores_without_reset_or_impulse(
    world, ball, sign, expected
):
    _, model = world
    data = mujoco.MjData(model)
    match = FootballMatch(model)
    joint = model.joint(ball + "_freejoint")
    adr, dof = int(joint.qposadr[0]), int(joint.dofadr[0])
    # Initial rolling shot is a test fixture; match.update cannot alter physics.
    data.qpos[adr : adr + 3] = [sign * 0.85, 4.3, 0.05]
    data.qvel[dof] = sign
    data.qvel[dof + 4] = sign / 0.05
    for _ in range(400):
        mujoco.mj_step(model, data)
        before = data.qpos.copy(), data.qvel.copy()
        match.update(data)
        np.testing.assert_array_equal(data.qpos, before[0])
        np.testing.assert_array_equal(data.qvel, before[1])
    assert match.scores == expected
    assert 1.356 < sign * data.qpos[adr] < 1.74
