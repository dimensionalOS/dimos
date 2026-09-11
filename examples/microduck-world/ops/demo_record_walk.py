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

"""Bake the shipped walking policy into a browser-only lobby preview."""

import json
import math
from copy import deepcopy
from uuid import uuid4

import mujoco
import numpy as np
from dimos.robot.pollen.microduck.policies import PolicyBank
from microduck_world.physics_robots import WorldRobots
from microduck_world.robot_io import RobotCommand
from microduck_world.roster import SETTINGS
from microduck_world.scene import PROJECT_ROOT, load_world
from microduck_world.world_sim import WorldSimModule

module = WorldSimModule(
    scene_xml=load_world()[0].mujoco_scene_path,
    robot_mjcf=str(PROJECT_ROOT / "assets/microduck/robot/robot_allcollisions.xml"),
    headless=True,
    auto_stand=False,
)
model = module._compose_spec().compile()
data = mujoco.MjData(model)
bank = PolicyBank(PROJECT_ROOT / "assets/microduck/policies", model)
preview_settings = deepcopy(SETTINGS)
# Use a clear lane on the pitch so a compact locker wall cannot clip the gait.
preview_settings["robots"]["duck1"]["spawn"] = [-1.8, 3.2]
robots = WorldRobots(model, bank, preview_settings, module.config)
generation = str(uuid4())
root = model.body("trunk_base").id
ids = []
for body in range(model.nbody):
    ancestor = body
    while ancestor and ancestor != root:
        ancestor = int(model.body_parentid[ancestor])
    if ancestor == root:
        ids.append(body)
frames = []
# The candidate walks from its spawn using the unchanged ONNX gait.
q = np.array([math.cos(-math.pi / 4), 0, 0, math.sin(-math.pi / 4)])
rotation = np.empty(9)
mujoco.mju_quat2Mat(rotation, q)
rotation = rotation.reshape(3, 3)


def pose():
    offset = data.xpos[root].copy()
    offset[2] = 0
    result = []
    for body in ids:
        position = rotation @ (data.xpos[body] - offset)
        quat = np.empty(4)
        mujoco.mju_mulQuat(quat, q, data.xquat[body])
        result.append([*position.tolist(), *quat.tolist()])
    return result


standing = None
for step in range(1600):
    if step % 50 == 0:
        robots.leases({"duck1": generation})
        robots.command(
            "duck1", RobotCommand(generation, "twist", (0.15 if step >= 400 else 0, 0, 0))
        )
    if step == 400:
        robots.command(
            "duck1",
            RobotCommand(generation, "policy", json.dumps({"policy": "walk", "action": "start"})),
        )
    robots.step(data)
    mujoco.mj_step(model, data)
    if step == 399:
        standing = pose()
    if 800 <= step < 1600 and step % 4 == 0:
        frames.append(pose())
# Select a near-periodic interval from the measured gait, excluding trivial repeats.
a = np.asarray(frames)
features = a[:, :, 0:3].reshape(len(a), -1)
start, end = min(
    ((i, j) for i in range(40) for j in range(i + 25, min(i + 100, len(a)))),
    key=lambda pair: float(np.mean((features[pair[0]] - features[pair[1]]) ** 2)),
)
clip = frames[start:end]
asset = {
    "version": 1,
    "source": "Pollen walking policy, baked in isolated MuJoCo",
    "bodyIds": ids,
    "standing": standing,
    "walk": {"fps": 50, "frames": clip},
}
path = PROJECT_ROOT / "web/public/duck-preview.json"
path.parent.mkdir(parents=True, exist_ok=True)
motion = float(np.max(np.ptp(np.asarray(clip)[:, :, :3], axis=0)))
assert motion > 0.01, f"Preview must actually walk, measured {motion} m"
path.write_text(json.dumps(asset, separators=(",", ":")))
print(
    json.dumps(
        {
            "frames": len(clip),
            "seconds": len(clip) / 50,
            "bytes": path.stat().st_size,
            "motion_range_m": float(np.max(np.ptp(np.asarray(clip)[:, :, :3], axis=0))),
            "root_xy": data.qpos[bank.root_qpos_adr : bank.root_qpos_adr + 2].tolist(),
            "loop_position_rms": float(np.sqrt(np.mean((features[start] - features[end]) ** 2))),
        }
    )
)
