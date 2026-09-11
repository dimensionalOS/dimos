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

"""Validate six real policies, passage clearance and native scene views offline."""

import json
import time
from uuid import uuid4

import mujoco
import numpy as np
from dimos.robot.pollen.microduck.policies import PolicyBank
from microduck_world.physics_robots import WorldRobots
from microduck_world.robot_io import RobotCommand
from microduck_world.roster import ROBOT_IDS, ROSTER, SETTINGS
from microduck_world.scene import PROJECT_ROOT, load_world
from microduck_world.world_sim import WorldSimModule
from PIL import Image

out = PROJECT_ROOT / "logs/club-checkpoint"
out.mkdir(parents=True, exist_ok=True)
module = WorldSimModule(
    scene_xml=load_world()[0].mujoco_scene_path,
    robot_mjcf=str(PROJECT_ROOT / "assets/microduck/robot/robot_allcollisions.xml"),
    headless=True,
    auto_stand=False,
)
model = module._compose_model()
data = mujoco.MjData(model)
bank = PolicyBank(PROJECT_ROOT / "assets/microduck/policies", model)
robots = WorldRobots(model, bank, SETTINGS, module.config)
leases = {id: str(uuid4()) for id in ROBOT_IDS}
start = time.monotonic()
for step in range(800):
    if step % 20 == 0:
        robots.leases(leases)
        for id in ROBOT_IDS:
            robots.command(
                id, RobotCommand(leases[id], "twist", (0.15 if step >= 400 else 0, 0, 0))
            )
    robots.step(data)
    mujoco.mj_step(model, data)
wall = time.monotonic() - start
positions = {
    id: data.qpos[r.bank.root_qpos_adr : r.bank.root_qpos_adr + 3].tolist()
    for id, r in robots.robots.items()
}
assert len(robots.states(data)) == 6
for id, r in robots.robots.items():
    assert r.active and r.bank.projected_gravity(data)[2] < -0.85, id
    assert np.linalg.norm(np.array(positions[id][:2]) - ROSTER[id]["spawn"]) > 0.035, (
        id,
        positions[id],
    )
print(
    json.dumps({"six_policy_sim_seconds": data.time, "wall_seconds": wall, "positions": positions}),
    flush=True,
)
# Physical collision ray checks sample complete passage widths above the ground.
checks = []


def passage(name, a, b, width=0.32):
    a = np.array([*a, 0.2])
    b = np.array([*b, 0.2])
    delta = b - a
    length = np.linalg.norm(delta)
    direction = delta / length
    side = np.array([-direction[1], direction[0], 0])
    for offset in (-width / 2, 0, width / 2):
        geom = np.zeros(1, dtype=np.int32)
        distance = mujoco.mj_ray(
            model,
            data,
            a + side * offset,
            direction,
            np.array([1, 0, 0, 0, 0, 0], dtype=np.uint8),
            1,
            -1,
            geom,
        )
        assert distance < 0 or distance >= length - 0.01, (
            name,
            offset,
            distance,
            length,
            model.geom(int(geom[0])).name,
        )
    checks.append(name)


# Park robots before fixed-route checks; this tests scene clearance, not player traffic.
for id, r in robots.robots.items():
    data.qpos[r.bank.root_qpos_adr : r.bank.root_qpos_adr + 3] = [20, 20, -5]
mujoco.mj_forward(model, data)
passage("red lockers to player tunnel", (-1.6, 1.25), (0, 1.25))
passage("blue lockers to player tunnel", (1.6, 1.25), (0, 1.25))
passage("player tunnel to pitch", (0, 1.25), (0, 3.05))
passage("tunnel to corridor", (0, 1.25), (0, -0.125))
passage("side corridor to benchmark living room", (0, -0.125), (7.2, -0.125))
# Rebuild positions for the native overview.
robots.leases({})
robots.step(data)
robots.leases(leases)
for step in range(400):
    if step % 20 == 0:
        robots.leases(leases)
    robots.step(data)
    mujoco.mj_step(model, data)
option = mujoco.MjvOption()
option.geomgroup[:] = [1, 1, 1, 0, 0, 0]
with mujoco.Renderer(model, height=720, width=1280) as renderer:
    for name, lookat, distance, azimuth, elevation in [
        ("club", [2.5, 1, 0.15], 15, 120, -60),
        ("lockers", [0, 1.15, 0.15], 7, 95, -60),
        ("field", [0, 4.3, 0.15], 8, 110, -55),
    ]:
        camera = mujoco.MjvCamera()
        camera.lookat[:] = lookat
        camera.distance = distance
        camera.azimuth = azimuth
        camera.elevation = elevation
        renderer.update_scene(data, camera=camera, scene_option=option)
        Image.fromarray(renderer.render()).save(out / f"{name}.png")
report = {
    "six_policies": True,
    "sim_seconds": 4,
    "compute_seconds": wall,
    "clear_passages": checks,
    "positions": positions,
}
(out / "report.json").write_text(json.dumps(report, indent=2))
print(json.dumps(report), flush=True)
