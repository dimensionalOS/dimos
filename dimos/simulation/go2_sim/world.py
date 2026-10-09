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

"""The simulated world: a generated scene, the legged Go2 and its Mid-360.

Stands in for the Go2 driver and PointLio, publishing PointLio's output contract from
ground truth.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
from threading import Event, Thread
import time

import mujoco
import mujoco.viewer
import numpy as np
from numpy.typing import NDArray
from pydantic import Field
from reactivex.disposable import Disposable
from scipy.spatial.transform import Rotation

from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.LineSegments3D import LineSegments3D
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.sim_msgs.Contacts import Contact, Contacts, Part
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.protocol.tf.static_tf_publisher import frames_to_edge_transforms
from dimos.robot.unitree.go2.constants import CMD_VEL_TIMEOUT
from dimos.robot.unitree.go2.go2_mid360_static_transforms import FRAMES
from dimos.simulation.go2_legged.policy import Go2Policy, load_policy
from dimos.simulation.go2_legged.robot import CONTROL_DT, LeggedGo2, apply_fitted_physics, go2_spec
from dimos.simulation.scenes.mjcf import CEILING_GROUP, SCENE_GROUP, add_boxes, geom_name
from dimos.simulation.scenes.procedural import Family, Scene, generate
from dimos.simulation.sensors.mid360.lidar import SimMid360
from dimos.simulation.sensors.mid360.pattern import POINT_RATE
from dimos.simulation.sensors.mujoco_raycaster import MujocoRaycaster
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

FRAME_DT = 0.1
TICKS_PER_FRAME = round(FRAME_DT / CONTROL_DT)
LIDAR_HALF_EXTENTS = (0.0325, 0.0325, 0.03)
ROBOT_VISUAL_GROUP = 2
COLLISION_GROUP = 3
# Collision primitives are left out.
LIDAR_GROUPS = np.isin(np.arange(6), (SCENE_GROUP, CEILING_GROUP, ROBOT_VISUAL_GROUP)).astype(
    np.uint8
)
# Looks only. The lidar and the contacts never read color or light.
BOX_RGBA = {
    "wall": (0.86, 0.84, 0.78, 1.0),
    "ceiling": (0.95, 0.95, 0.95, 1.0),
    "clutter": (0.45, 0.55, 0.70, 1.0),
}
FLOOR_CHECKER = ((0.62, 0.60, 0.56), (0.70, 0.68, 0.64))
SCENE_PUBLISH_DT = 2.0
# contacts go out on change, and on this period so a recording always holds the current set
CONTACTS_HEARTBEAT_DT = 1.0
ODOM_FRAME_ID = "odom"
SENSOR_FRAME_ID = "mid360_link"
STILL = np.zeros(3)

BOX_EDGES = np.array(
    [
        [[-1, -1, -1], [1, -1, -1]],
        [[-1, 1, -1], [1, 1, -1]],
        [[-1, -1, 1], [1, -1, 1]],
        [[-1, 1, 1], [1, 1, 1]],
        [[-1, -1, -1], [-1, 1, -1]],
        [[1, -1, -1], [1, 1, -1]],
        [[-1, -1, 1], [-1, 1, 1]],
        [[1, -1, 1], [1, 1, 1]],
        [[-1, -1, -1], [-1, -1, 1]],
        [[1, -1, -1], [1, -1, 1]],
        [[-1, 1, -1], [-1, 1, 1]],
        [[1, 1, -1], [1, 1, 1]],
    ],
    dtype=np.float64,
)


def _mount() -> tuple[NDArray[np.float64], NDArray[np.float64]]:
    """base_link -> mid360_link as a translation and rotation matrix, from the rig's frame tree."""
    edges = {t.child_frame_id: t for t in frames_to_edge_transforms(FRAMES)}
    matrix = edges["front_camera"].to_matrix() @ edges["mid360_link"].to_matrix()
    return matrix[:3, 3].copy(), matrix[:3, :3].copy()


MOUNT_XYZ, MOUNT_R = _mount()


@dataclass
class LidarFrame:
    """One frame: sensor-frame points at the frame-end pose, like PointLio's output."""

    t: float
    points: NDArray[np.float32]


class GroundTruthLio:
    """PointLio's deskew with the true motion: substep returns in, one frame-end cloud out."""

    def __init__(self) -> None:
        self._world: list[NDArray[np.float64]] = []

    def add(
        self,
        points: NDArray[np.float32],
        position: NDArray[np.float64],
        rotation: NDArray[np.float64],
    ) -> None:
        """Returns cast from one sensor pose, placed in the world with that pose."""
        self._world.append(position + points.astype(np.float64) @ rotation.T)

    def frame(
        self, t: float, position: NDArray[np.float64], rotation: NDArray[np.float64]
    ) -> LidarFrame:
        """Everything added since the last frame, re-expressed at the frame-end pose."""
        world = np.concatenate(self._world) if self._world else np.zeros((0, 3))
        self._world = []
        points = ((world - position) @ rotation).astype(np.float32)
        return LidarFrame(t, points)

    def clear(self) -> None:
        self._world = []


def _dress(spec: mujoco.MjSpec) -> None:
    """A checkered floor material and a sun, since the Go2 model brings no light of its own."""
    texture = spec.add_texture()
    texture.name = "floor"
    texture.type = mujoco.mjtTexture.mjTEXTURE_2D
    texture.builtin = mujoco.mjtBuiltin.mjBUILTIN_CHECKER
    texture.rgb1, texture.rgb2 = FLOOR_CHECKER
    texture.width = texture.height = 256
    material = spec.add_material()
    material.name = "floor"
    material.textures[mujoco.mjtTextureRole.mjTEXROLE_RGB] = "floor"
    material.texrepeat = (1.0, 1.0)
    material.texuniform = True
    material.reflectance = 0.1
    light = spec.worldbody.add_light()
    light.type = mujoco.mjtLightType.mjLIGHT_DIRECTIONAL
    light.pos = (0.0, 0.0, 10.0)
    light.dir = (0.3, 0.2, -1.0)
    light.castshadow = True
    light.diffuse = (0.6, 0.6, 0.6)
    light.specular = (0.1, 0.1, 0.1)
    spec.visual.headlight.ambient = (0.35, 0.35, 0.35)
    spec.visual.headlight.diffuse = (0.3, 0.3, 0.3)


def build_model(scene: Scene) -> mujoco.MjModel:
    """The scene's boxes, the Go2 and the Mid-360 housing as a contact box, in one model."""
    spec = go2_spec()
    _dress(spec)
    add_boxes(spec, scene)
    for i, box in enumerate(scene.boxes):
        geom = spec.geom(geom_name(i, box))
        if box.kind == "floor":
            geom.material = "floor"
        else:
            geom.rgba = BOX_RGBA[box.kind]
    lidar = spec.body("base").add_geom()
    lidar.name = "mid360"
    lidar.type = mujoco.mjtGeom.mjGEOM_BOX
    lidar.size = LIDAR_HALF_EXTENTS
    lidar.pos = MOUNT_XYZ
    q = Rotation.from_matrix(MOUNT_R).as_quat()
    lidar.quat = (q[3], q[0], q[1], q[2])
    lidar.group = COLLISION_GROUP
    model = spec.compile()
    apply_fitted_physics(model)
    return model


def scene_edges(scene: Scene) -> NDArray[np.float64]:
    """The 12 edges of every box, as (N, 2, 3) segments."""
    centers = np.array([box.center for box in scene.boxes])[:, None, None, :]
    halves = np.array([box.half for box in scene.boxes])[:, None, None, :]
    edges: NDArray[np.float64] = (centers + BOX_EDGES[None] * halves).reshape(-1, 2, 3)
    return edges


class Go2Sim:
    """The compiled scene with the Go2 and its Mid-360, ticked from a velocity command."""

    def __init__(self, scene: Scene, seed: int, policy: Go2Policy) -> None:
        self.scene = scene
        self.model = build_model(scene)
        self.data = mujoco.MjData(self.model)
        self.robot = LeggedGo2(self.model, self.data, policy)
        self.lidar = SimMid360.go2(MujocoRaycaster(self.model, self.data, LIDAR_GROUPS), seed)
        self.lio = GroundTruthLio()
        self.t = 0.0
        self._tick = 0
        self._points_per_step = int(POINT_RATE * self.model.opt.timestep)
        geom = mujoco.mjtObj.mjOBJ_GEOM
        self._kind = {
            mujoco.mj_name2id(self.model, geom, geom_name(i, box)): box.kind
            for i, box in enumerate(scene.boxes)
        }
        self._lidar_geom = mujoco.mj_name2id(self.model, geom, "mid360")

    def reset(self, x: float, y: float, z_feet: float, yaw: float) -> None:
        self.robot.reset(x, y, z_feet, yaw)
        self.t = 0.0
        self._tick = 0
        self.lio.clear()

    def sensor_pose(self) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
        position, rotation = self.robot.base_pose()
        return position + rotation @ MOUNT_XYZ, rotation @ MOUNT_R

    def base_pose(self) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
        return self.robot.base_pose()

    def sensor_velocity(self) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
        """Linear velocity in the odom frame and angular velocity in the sensor frame, as PointLio reports."""
        linear, angular = self.robot.base_velocity()
        _, rotation = self.robot.base_pose()
        return linear + rotation @ np.cross(angular, MOUNT_XYZ), MOUNT_R.T @ angular

    def contacts(self) -> list[Contact]:
        """Every (robot part, scene kind) pair in contact right now, sorted."""
        m, d = self.model, self.data
        found: set[Contact] = set()
        for i in range(d.ncon):
            c = d.contact[i]
            for scene_geom, robot_geom in ((c.geom1, c.geom2), (c.geom2, c.geom1)):
                kind = self._kind.get(scene_geom)
                if kind is not None and m.geom_bodyid[robot_geom] != 0:
                    found.add(Contact(self._part(robot_geom), kind))
        return sorted(found)

    def _part(self, geom: int) -> Part:
        if geom == self._lidar_geom:
            return "lidar"
        if geom in self.robot.feet:
            return "foot"
        if self.model.geom_bodyid[geom] == self.robot.trunk:
            return "trunk"
        return "leg"

    def tick(self, command: NDArray[np.float64]) -> LidarFrame | None:
        """Advance one policy tick. Returns the lidar frame that completed on this tick, if any."""

        def cast() -> None:
            position, rotation = self.sensor_pose()
            self.lio.add(
                self.lidar.cast(position, rotation, self._points_per_step), position, rotation
            )

        self.robot.tick(command, cast)
        self._tick += 1
        self.t = self._tick * CONTROL_DT
        if self._tick % TICKS_PER_FRAME:
            return None
        return self.lio.frame(self.t, *self.sensor_pose())


class CommandHold:
    """The latest velocity command, or still once it is older than the driver's timeout."""

    def __init__(self, timeout: float = CMD_VEL_TIMEOUT) -> None:
        self._timeout = timeout
        self._latest: tuple[NDArray[np.float64], float] = (STILL, -math.inf)

    def update(self, command: NDArray[np.float64], now: float) -> bool:
        """Take a finite command. A non-finite one is refused and the previous one kept."""
        if not np.all(np.isfinite(command)):
            return False
        self._latest = (command, now)
        return True

    def current(self, now: float) -> NDArray[np.float64]:
        command, received = self._latest
        return command if now - received < self._timeout else STILL


class SimGo2WorldConfig(ModuleConfig):
    family: Family = "office"
    seed: int = 1
    scene_params: dict[str, bool | int | float] = {}
    real_time_factor: float = Field(default=1.0, gt=0.0)
    mujoco_viewer: bool = False
    policy: Path | None = None


class SimGo2World(Module):
    """A generated scene with the legged Go2 and its simulated Mid-360, paced to the wall clock."""

    config: SimGo2WorldConfig

    cmd_vel: In[Twist]

    lidar: Out[PointCloud2]
    odometry: Out[Odometry]
    tf: Out[TFMessage]
    ground_truth: Out[PoseStamped]
    contacts: Out[Contacts]
    scene: Out[LineSegments3D]

    _thread: Thread | None = None
    _reset: tuple[float, float, float, float] | None = None

    @rpc
    def start(self) -> None:
        super().start()
        self._policy = load_policy(self.config.policy)
        self._sim = self._load(self.config.family, self.config.seed, self.config.scene_params)
        self._hold = CommandHold()
        self._stop_event = Event()
        self.register_disposable(Disposable(self.cmd_vel.subscribe(self._on_cmd_vel)))
        self._thread = Thread(target=self._run, daemon=True)
        self._thread.start()

    @rpc
    def stop(self) -> None:
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)
        super().stop()

    @rpc
    def reset_pose(self, x: float, y: float, z: float, yaw: float) -> None:
        """Stand the robot at rest at the pose with its feet on z, before the sim's next tick."""
        self._reset = (x, y, z, yaw)

    def _on_cmd_vel(self, msg: Twist) -> None:
        command = np.array([msg.linear.x, msg.linear.y, msg.angular.z])
        if not self._hold.update(command, time.monotonic()):
            logger.warning("Ignored non-finite cmd_vel", command=command.tolist())

    def _load(self, family: Family, seed: int, params: dict[str, bool | int | float]) -> Go2Sim:
        scene = generate(family, seed, **params)
        sim = Go2Sim(scene, seed, self._policy)
        sim.reset(*scene.start, 0.0)
        blocked = [c for c in sim.contacts() if c.kind != "floor"]
        if blocked:
            raise RuntimeError(f"robot starts inside scene geometry in {scene.name}: {blocked}")
        logger.info("Sim world ready", scene=scene.name)
        return sim

    def _open_viewer(self, sim: Go2Sim) -> mujoco.viewer.Handle | None:
        if not self.config.mujoco_viewer:
            return None
        viewer = mujoco.viewer.launch_passive(
            sim.model, sim.data, show_left_ui=False, show_right_ui=False
        )
        viewer.opt.geomgroup[CEILING_GROUP] = 0
        # a free camera, so the mouse can pan away from the robot. Starts above the walls,
        # looking down over the robot's shoulder.
        viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FREE
        viewer.cam.lookat[:] = sim.base_pose()[0]
        viewer.cam.distance = 4.0
        viewer.cam.elevation = -55
        viewer.cam.azimuth = 225
        return viewer

    def _run(self) -> None:
        # open3d loads on the first cloud, about a second
        PointCloud2.from_numpy(np.zeros((0, 3), np.float32), frame_id=SENSOR_FRAME_ID)
        viewer = self._open_viewer(self._sim)
        try:
            self._simulate(self._sim, viewer)
        except Exception:
            logger.exception("Sim thread failed")
        finally:
            if viewer is not None:
                viewer.close()

    def _simulate(self, sim: Go2Sim, viewer: mujoco.viewer.Handle | None) -> None:
        t0 = time.time()
        last_contacts: list[Contact] | None = None
        next_contacts_publish = 0.0
        next_scene_publish = 0.0
        while not self._stop_event.is_set():
            if (pose := self._reset) is not None:
                self._reset = None
                sim.reset(*pose)
                t0 = time.time()
            frame = sim.tick(self._hold.current(time.monotonic()))
            if viewer is not None:
                viewer.sync()
            stamp = t0 + sim.t / self.config.real_time_factor
            self._publish_poses(sim, stamp)
            contacts = sim.contacts()
            if contacts != last_contacts or sim.t >= next_contacts_publish:
                self.contacts.publish(Contacts(contacts, ts=stamp))
                last_contacts = contacts
                next_contacts_publish = sim.t + CONTACTS_HEARTBEAT_DT
            if frame is not None:
                self.lidar.publish(
                    PointCloud2.from_numpy(frame.points, frame_id=SENSOR_FRAME_ID, timestamp=stamp)
                )
            if sim.t >= next_scene_publish:
                self.scene.publish(
                    LineSegments3D(
                        ts=stamp, frame_id=ODOM_FRAME_ID, segments=scene_edges(sim.scene)
                    )
                )
                next_scene_publish = sim.t + SCENE_PUBLISH_DT
            delay = stamp - time.time()
            if delay > 0:
                self._stop_event.wait(delay)
            elif delay < -1.0:
                logger.warning("Sim running slower than real time", behind_s=round(-delay, 1))
                t0 -= delay

    def _publish_poses(self, sim: Go2Sim, stamp: float) -> None:
        position, rotation = sim.sensor_pose()
        linear, angular = sim.sensor_velocity()
        q = Rotation.from_matrix(rotation).as_quat()
        sensor = Pose(*map(float, position), *map(float, q))
        self.odometry.publish(
            Odometry(
                ts=stamp,
                frame_id=ODOM_FRAME_ID,
                child_frame_id=SENSOR_FRAME_ID,
                pose=sensor,
                twist=Twist(Vector3(linear.tolist()), Vector3(angular.tolist())),
            )
        )
        self.tf.publish(
            TFMessage(
                Transform(
                    translation=Vector3(sensor.position),
                    rotation=Quaternion(sensor.orientation),
                    frame_id=ODOM_FRAME_ID,
                    child_frame_id=SENSOR_FRAME_ID,
                    ts=stamp,
                )
            )
        )
        base_position, base_rotation = sim.base_pose()
        base_q = Rotation.from_matrix(base_rotation).as_quat()
        self.ground_truth.publish(
            PoseStamped(
                *map(float, base_position),
                *map(float, base_q),
                ts=stamp,
                frame_id=ODOM_FRAME_ID,
            )
        )
