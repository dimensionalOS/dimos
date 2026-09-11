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

"""Microduck simulation with a read-only visual state stream for the web world."""

import json
import threading
import time
from pathlib import Path
from typing import Any

import mujoco
import requests
from dimos.core.core import rpc
from dimos.core.stream import In, Out
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.pollen.microduck.sim_module import (
    LIDAR_CAMERA_SPECS,
    POV_CAMERA_NAME,
    MicroduckSimModule,
)
from dimos.simulation.engines.mujoco_engine import MujocoEngine
from dimos.simulation.engines.robot_sim_binding import RobotSimSpec
from dimos.web.codecs import web_encoder
from microduck_world.ball_physics import BALL_SPAWN_HEIGHT
from microduck_world.camera import HEAD_CAMERA, configure_clipping, configure_head_camera
from microduck_world.comparison import JpegComparison
from microduck_world.football import BALL_BODY, BALL_NAMES, FootballMatch, add_footballs
from microduck_world.gateway import GatewayConfig
from microduck_world.physics_robots import WorldRobots
from microduck_world.robot_io import ROBOT_IDS, VISITOR_IDS, RobotCommand, RobotState, RobotVision
from microduck_world.scene import PROJECT_ROOT
from microduck_world.scorers import ScorerLedger
from microduck_world.sensors import RobotSensors
from microduck_world.visual_scene import body_snapshot, export_scene, write_scene

WORLD_FPS = 30.0
WORLD_ENCODING = "world.json.v1"


@web_encoder(WORLD_ENCODING)
def encode_world_state(message: str) -> bytes:
    return message.encode("utf-8")


class WorldSimModule(MicroduckSimModule):
    world_state: Out[str]
    world_compare_image: Out[Image]
    duck1_command: In[RobotCommand]
    duck2_command: In[RobotCommand]
    duck3_command: In[RobotCommand]
    duck4_command: In[RobotCommand]
    duck5_command: In[RobotCommand]
    duck6_command: In[RobotCommand]
    duck1_state: Out[RobotState]
    duck2_state: Out[RobotState]
    duck3_state: Out[RobotState]
    duck4_state: Out[RobotState]
    duck5_state: Out[RobotState]
    duck6_state: Out[RobotState]
    duck1_vision: Out[RobotVision]
    duck2_vision: Out[RobotVision]
    duck3_vision: Out[RobotVision]
    duck4_vision: Out[RobotVision]
    duck5_vision: Out[RobotVision]
    duck6_vision: Out[RobotVision]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._football: FootballMatch | None = None
        self._visual_body_ids: list[int] = []
        self._visual_asset = ""
        self._last_world_publish = 0.0
        self._comparison: JpegComparison | None = None
        self._robots: WorldRobots | None = None
        self._scorer_identities = {}
        self._drop_lock = threading.Lock()
        self._ball_drops = {}
        self._sensors: RobotSensors | None = None
        self._lobby_stop = threading.Event()
        self._lobby_thread: threading.Thread | None = None
        self._visitor_settings = json.loads(
            (PROJECT_ROOT / "assets/scenes/apartment/multiplayer.json").read_text()
        )

    @rpc
    def start(self) -> None:
        super().start()
        settings = GatewayConfig.model_validate_json(
            (PROJECT_ROOT / "config/tailnet.json").read_text()
        )
        assert self._engine is not None
        assert self._bank is not None
        self._robots = WorldRobots(
            self._engine.model, self._bank, self._visitor_settings, self.config
        )
        for id in ROBOT_IDS:

            def drive(command: RobotCommand, robot: str = id) -> None:
                assert self._robots is not None
                if command.kind == "drop_ball":
                    if isinstance(command.value, str) and command.value in BALL_NAMES:
                        with self._drop_lock:
                            self._ball_drops[command.value] = (
                                robot,
                                command.generation,
                                time.monotonic(),
                            )
                else:
                    self._robots.command(robot, command)

            self._subscribe(getattr(self, id + "_command"), drive)
        self._sensors = RobotSensors(self._engine.model, self._publish_vision)
        self._sensors.start()
        self._lobby_stop.clear()
        self._lobby_thread = threading.Thread(
            target=self._poll_lobby,
            args=(settings.upstream_url,),
            daemon=True,
            name="visitor-assignments",
        )
        self._lobby_thread.start()
        self._comparison = JpegComparison(
            self._engine.model,
            f"{settings.upstream_url}/api/stats",
            self.world_compare_image.publish,
        )

    @rpc
    def stop(self) -> None:
        self._lobby_stop.set()
        if self._lobby_thread is not None:
            self._lobby_thread.join(timeout=3)
        comparison, self._comparison = self._comparison, None
        try:
            if self._sensors is not None:
                self._sensors.close()
            if comparison is not None:
                comparison.close()
        finally:
            super().stop()
            if self._football and self._football.ledger:
                self._football.ledger.flush()

    def _poll_lobby(self, origin: str) -> None:
        with requests.Session() as client:
            client.trust_env = False
            while not self._lobby_stop.is_set():
                try:
                    response = client.get(origin + "/internal/assignments", timeout=1)
                    response.raise_for_status()
                    occupants = response.json()
                    if isinstance(occupants, dict) and all(
                        k in VISITOR_IDS and isinstance(v, str) and v for k, v in occupants.items()
                    ):
                        assert self._robots is not None
                        self._robots.leases(occupants)
                        identities = client.get(origin + "/internal/scorers", timeout=1)
                        identities.raise_for_status()
                        values = identities.json()
                        self._scorer_identities = {
                            robot: value
                            for robot, value in values.items()
                            if robot in ROBOT_IDS
                            and isinstance(value, dict)
                            and all(
                                isinstance(value.get(k), str) and value[k]
                                for k in ("generation", "userId", "handle")
                            )
                            and value["generation"] == occupants.get(robot)
                        }
                        if self._football and self._football.ledger:
                            self._football.ledger.flush()
                except (requests.RequestException, ValueError):
                    pass  # The physics mailbox expires assignments if discovery stays unavailable.
                self._lobby_stop.wait(0.5)

    def _gait_pre_step(self, engine: MujocoEngine) -> None:
        if self._robots is not None:
            self._robots.step(engine.data)
            with self._drop_lock:
                for ball, (robot_id, generation, queued) in list(self._ball_drops.items()):
                    robot = self._robots.robots[robot_id]
                    valid = (
                        robot.active
                        and robot.generation == generation
                        and time.monotonic() - queued < 5
                    )
                    if not valid or (
                        self._football and self._football.drop_ball(engine.data, ball)
                    ):
                        del self._ball_drops[ball]
            host = self._robots.robots["duck1"]
            engine.write_joint_command(
                JointState(
                    position=engine.data.ctrl[host.actuators][self._engine_target_perm].tolist()
                )
            )

    def _publish_vision(self, robot: str, vision: RobotVision) -> None:
        getattr(self, robot + "_vision").publish(vision)

    def _compose_spec(self) -> mujoco.MjSpec:
        spec = super()._compose_spec()
        assert self.config.robot_mjcf is not None
        # Remove the inherited trunk-mounted POV workaround. Every view uses
        # the actual head mount, including the private agent RGB-D renderer.
        spec.delete(spec.camera(POV_CAMERA_NAME))
        configure_head_camera(spec)
        # Bind the existing host engine to its own 14 actuators before guests are attached.
        host_joints = tuple(a.target for a in spec.actuators)
        self.config.robot_sim_spec = RobotSimSpec(
            robot_id="duck1",
            hardware_joints=host_joints,
            model_joint_names=host_joints,
            root_body_names=("trunk_base",),
            root_joint_names=("trunk_base_freejoint",),
            require_floating_base=True,
        )
        for id in ROBOT_IDS[1:]:
            guest = mujoco.MjSpec.from_file(str(self.config.robot_mjcf))
            configure_head_camera(guest)
            for camera_name, _ in LIDAR_CAMERA_SPECS:
                camera = spec.camera(camera_name)
                guest.body("trunk_base").add_camera(
                    name=camera_name,
                    pos=list(camera.pos),
                    quat=list(camera.quat),
                    fovy=float(camera.fovy),
                )
            spec.attach(guest, prefix=id + "_", frame=spec.worldbody.add_frame())
        spec.body(BALL_BODY).pos = [9.8, 0.075, BALL_SPAWN_HEIGHT]
        add_footballs(spec)
        return spec

    def _compose_model(self) -> mujoco.MjModel:
        model = super()._compose_model()
        configure_clipping(model)
        # Initial data starts with unoccupied robots outside the arena. Keep the
        # compiled scene extent unchanged so the host POV clipping stays correct.
        for index, id in enumerate(ROBOT_IDS):
            adr = int(
                model.joint(("" if id == "duck1" else id + "_") + "trunk_base_freejoint").qposadr[0]
            )
            model.qpos0[adr : adr + 3] = (10 + index, 10, -5)
        # This hook runs during startup, before the engine starts its physics thread.
        if self.config.scene_xml is None:
            raise ValueError("WorldSimModule requires a scene XML")
        appearance_path = Path(self.config.scene_xml).parent / "viewer.json"
        appearance = json.loads(appearance_path.read_text()) if appearance_path.exists() else {}
        scene = export_scene(model, appearance, self.config.camera_name)
        scene["actors"] = []
        for id in ROBOT_IDS:
            prefix = "" if id == "duck1" else id + "_"
            root = model.body(prefix + "trunk_base").id
            body_ids = []
            for body in scene["bodyIds"]:
                ancestor = body
                while ancestor and ancestor != root:
                    ancestor = int(model.body_parentid[ancestor])
                if ancestor == root:
                    body_ids.append(body)
            color = self._visitor_settings["colors"][id]
            rgb = [int(color[i : i + 2], 16) / 255 for i in (1, 3, 5)]
            # Materials are scoped by each attached robot's prefix in MuJoCo.
            tinted_materials: set[int] = set()
            for geom in range(model.ngeom):
                if int(model.geom_bodyid[geom]) not in body_ids:
                    continue
                material = int(model.geom_matid[geom])
                rgba = model.mat_rgba[material] if material >= 0 else model.geom_rgba[geom]
                if (
                    model.geom_type[geom] == mujoco.mjtGeom.mjGEOM_MESH
                    and max(rgba[:3]) > 0.5
                    and material not in tinted_materials
                ):
                    rgba[:3] = [
                        float(base) * 0.22 + tint * 0.78
                        for base, tint in zip(rgba[:3], rgb, strict=True)
                    ]
                    if material >= 0:
                        tinted_materials.add(material)
            scene["actors"].append(
                {
                    "id": id,
                    "bodyIds": body_ids,
                    "focusBody": root,
                    "camera": {
                        **scene["camera"],
                        "body": int(model.camera(prefix + HEAD_CAMERA).bodyid[0]),
                    },
                    "color": color,
                }
            )
        # Export the same colors the native cameras see.
        for geom in scene["geoms"]:
            index = geom["id"]
            material = int(model.geom_matid[index])
            geom["rgba"] = (
                model.mat_rgba[material] if material >= 0 else model.geom_rgba[index]
            ).tolist()
        self._visual_body_ids = scene["bodyIds"]
        name = write_scene(scene, PROJECT_ROOT / "state/viewer")
        self._visual_asset = f"/world-assets/{name}"
        self._football = FootballMatch(model)
        self._football.ledger = ScorerLedger(PROJECT_ROOT / "state/football-scorers.sqlite3")
        return model

    def _after_step(self, engine: MujocoEngine) -> None:
        assert self._football is not None
        if self._robots is not None:
            self._football.touches(engine.data, self._robots.robots, self._scorer_identities)
        self._football.update(engine.data)
        now = time.monotonic()
        if now - self._last_world_publish < 1.0 / WORLD_FPS:
            return
        self._last_world_publish = now
        if self._robots is not None:
            for robot, state in self._robots.states(engine.data).items():
                getattr(self, robot + "_state").publish(state)
            if self._sensors is not None:
                self._sensors.snapshot(
                    engine.data.qpos, self._robots.sensor_assignments(), self._football.lit
                )
        comparison = self._comparison
        if comparison is not None:
            comparison.snapshot(engine.data.qpos, self._football.lit)
        self.world_state.publish(
            json.dumps(
                {
                    "model": self._visual_asset,
                    "t": time.time(),
                    "simTime": float(engine.data.time),
                    "football": self._football.snapshot(),
                    "actors": self._robots.snapshot() if self._robots is not None else [],
                    "sensorError": self._sensors.error if self._sensors is not None else None,
                    "comparison": comparison.status() if comparison is not None else None,
                    "poses": body_snapshot(engine.data, self._visual_body_ids),
                },
                separators=(",", ":"),
                allow_nan=False,
            )
        )
