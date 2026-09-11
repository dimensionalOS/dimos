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

"""Offline validation of the published kicking policies in the actual hosted world."""

import json
from copy import deepcopy
from pathlib import Path
from uuid import uuid4

import mujoco
import numpy as np
from dimos.robot.pollen.microduck.assets_fetch import ensure_assets
from dimos.robot.pollen.microduck.policies import KICK_BALL_OFFSETS, PolicyBank
from microduck_world.ball_physics import BALL_SPAWN_HEIGHT
from microduck_world.camera import HEAD_CAMERA
from microduck_world.football import FootballMatch, Scoreboard
from microduck_world.physics_robots import WorldRobots
from microduck_world.robot_io import RobotCommand
from microduck_world.roster import SETTINGS
from microduck_world.scene import PROJECT_ROOT, load_world
from microduck_world.world_sim import WorldSimModule
from PIL import Image


def main():
    directory = PROJECT_ROOT / "logs/football-checkpoint"
    directory.mkdir(parents=True, exist_ok=True)
    assets = ensure_assets()
    robot_mjcf = assets.robot_mjcf("default")
    module = WorldSimModule(
        scene_xml=load_world()[0].mujoco_scene_path,
        robot_mjcf=str(robot_mjcf),
        camera_name=HEAD_CAMERA,
        headless=True,
        auto_stand=False,
        spawn_xy=(2.0, 4.3),
    )
    # Composition without start() creates no server, worker, or network publisher.
    model = module._compose_spec().compile()
    bank = PolicyBank(assets.policy_dir, model, missing=assets.missing)
    results = []
    original_colors = model.geom_rgba.copy()
    for policy in ("kick_left", "kick_right"):
        model.geom_rgba[:] = original_colors
        data = mujoco.MjData(model)
        settings = deepcopy(SETTINGS)
        settings["robots"]["duck1"].update(spawn=[2.0, 4.3], yaw=0)
        robots = WorldRobots(model, bank, settings, module.config)
        generation = str(uuid4())
        match = FootballMatch(model)
        for _ in range(400):
            robots.leases({"duck1": generation})
            robots.step(data)
            mujoco.mj_step(model, data)
            match.update(data)
        duck = robots.robots["duck1"]
        ball = model.joint("football_ball_1_freejoint")
        adr, vel = int(ball.qposadr[0]), int(ball.dofadr[0])
        yaw = duck.bank.root_yaw(data)
        dx, dy = KICK_BALL_OFFSETS[policy]
        # Fixture setup only. The live policy path never places a ball.
        data.qpos[adr : adr + 3] = data.qpos[
            duck.bank.root_qpos_adr : duck.bank.root_qpos_adr + 3
        ] + [np.cos(yaw) * dx - np.sin(yaw) * dy, np.sin(yaw) * dx + np.cos(yaw) * dy, 0]
        data.qpos[adr + 2] = BALL_SPAWN_HEIGHT
        data.qvel[vel : vel + 6] = 0
        mujoco.mj_forward(model, data)
        start = data.qpos[adr : adr + 3].copy()
        robots.command(
            "duck1",
            RobotCommand(generation, "policy", json.dumps({"policy": policy, "action": "start"})),
        )
        robots.step(data)
        np.testing.assert_array_equal(data.qpos[adr : adr + 3], start)
        contact_bodies = set()
        peak_speed = 0.0
        ball_geom = model.geom("football_ball_1_geom").id
        frames = []
        option = mujoco.MjvOption()
        option.geomgroup[:] = [1, 1, 1, 0, 0, 0]
        camera = mujoco.MjvCamera()
        camera.lookat[:] = (2.2, 4.3, 0.18)
        camera.distance = 1.4
        camera.azimuth = 155
        camera.elevation = -25
        board = Scoreboard(model)
        with mujoco.Renderer(model, height=540, width=960) as renderer:
            for step in range(400):
                robots.leases({"duck1": generation})
                robots.step(data)
                mujoco.mj_step(model, data)
                match.update(data)
                peak_speed = max(peak_speed, float(np.linalg.norm(data.qvel[vel : vel + 3])))
                for contact in data.contact:
                    if ball_geom in contact.geom:
                        other = contact.geom[1] if contact.geom[0] == ball_geom else contact.geom[0]
                        contact_bodies.add(model.body(int(model.geom_bodyid[other])).name)
                if step % 20 == 0:
                    board.apply(model, match.lit)
                    renderer.update_scene(data, camera=camera, scene_option=option)
                    frames.append(Image.fromarray(renderer.render().copy()))
            frames[0].save(
                directory / (policy + ".gif"),
                save_all=True,
                append_images=frames[1:],
                duration=100,
                loop=0,
            )
        distance = float(np.linalg.norm(data.qpos[adr : adr + 2] - start[:2]))
        result = dict(
            policy=policy,
            robotModel=Path(robot_mjcf).name,
            contactBodies=sorted(contact_bodies),
            ballTravel=distance,
            peakBallSpeed=peak_speed,
            scores=match.scores,
            upright=float(duck.bank.projected_gravity(data)[2]),
        )
        results.append(result)
        assert ("ankle_left" if policy == "kick_left" else "ankle_right") in contact_bodies, result
        assert distance > 0.1, result
        assert peak_speed > 0.2, result
    # A clean static room image, including the native scoreboard.
    data = mujoco.MjData(model)
    settings = deepcopy(SETTINGS)
    settings["robots"]["duck1"].update(spawn=[2.0, 4.3], yaw=0)
    robots = WorldRobots(model, bank, settings, module.config)
    generation = str(uuid4())
    for _ in range(100):
        robots.leases({"duck1": generation})
        robots.step(data)
        mujoco.mj_step(model, data)
    model.geom_rgba[:] = original_colors
    scoreboard = Scoreboard(model)
    scoreboard.apply(model, scoreboard.lamps([0, 0]))
    camera = mujoco.MjvCamera()
    camera.lookat[:] = (0, 4.3, 0.15)
    camera.distance = 7.2
    camera.azimuth = 105
    camera.elevation = -47
    with mujoco.Renderer(model, height=720, width=1280) as renderer:
        renderer.update_scene(data, camera=camera, scene_option=option)
        Image.fromarray(renderer.render()).save(directory / "native-pitch.png")
    (directory / "physics.json").write_text(json.dumps(results, indent=2) + "\n")
    print(json.dumps(results, indent=2))


if __name__ == "__main__":
    main()
