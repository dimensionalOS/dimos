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


"""The simulated world: a generated scene, a robot and its Mid-360s.

Stands in for the robot's driver and PointLio, publishing PointLio's output contract from
ground truth.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from collections.abc import Callable
from dataclasses import dataclass
import math
from pathlib import Path
from threading import Event, Lock, Thread
import time
from typing import Any, ClassVar

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
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.sim_msgs.Contacts import Contact, Contacts, Part
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.simulation.scenes.mjcf import SCENE_GROUP, add_boxes, geom_name
from dimos.simulation.scenes.procedural import Family, Params, Scene, generate
from dimos.simulation.sensors.mid360.pattern import POINT_RATE
from dimos.simulation.sensors.mujoco_raycaster import MujocoRaycaster
from dimos.simulation.sim_world.body import Body, BodyShape, Rig, Sensor
from dimos.simulation.sim_world.premap import premap_cloud, without_objects
from dimos.utils.data import get_data, get_data_dir
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

FRAME_DT = 0.1
ROBOT_VISUAL_GROUP = 2
COLLISION_GROUP = 3
# Collision primitives are left out.
LIDAR_GROUPS = np.isin(np.arange(6), (SCENE_GROUP, ROBOT_VISUAL_GROUP)).astype(np.uint8)
# Looks only. The lidar and the contacts never read color or light.
BOX_RGBA = {
    "wall": (0.86, 0.84, 0.78, 1.0),
    "clutter": (0.45, 0.55, 0.70, 1.0),
}
FLOOR_CHECKER = ((0.62, 0.60, 0.56), (0.70, 0.68, 0.64))
SCENE_PUBLISH_DT = 2.0
# contacts go out on change, and on this period so a recording always holds the current set
CONTACTS_HEARTBEAT_DT = 1.0
START_WAIT_S = 30.0
ROOT_QUAT = slice(3, 7)
BASE_FRAME_ID = "base_link"
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
    """A checkered floor material and a sun, since the robot models bring no light of their own."""
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


def _with_scene(spec: mujoco.MjSpec, scene: Scene) -> mujoco.MjSpec:
    _dress(spec)
    add_boxes(spec, scene)
    for i, box in enumerate(scene.boxes):
        geom = spec.geom(geom_name(i, box))
        if box.kind == "floor":
            geom.material = "floor"
        else:
            geom.rgba = BOX_RGBA[box.kind]
    return spec


def scene_model(scene: Scene, robot: mujoco.MjSpec) -> mujoco.MjModel:
    """The scene's boxes and a robot spec, in one model for drawing."""
    model: mujoco.MjModel = _with_scene(robot, scene).compile()
    return model


def display_xml(rig: Rig) -> str:
    """The robot alone as MJCF, naming its assets from the data root so the file can change machines."""
    spec = rig.spec()
    spec.meshdir = str(Path(spec.modelfiledir) / spec.meshdir)
    spec.texturedir = str(Path(spec.modelfiledir) / spec.texturedir)  # type: ignore[attr-defined]
    xml: str = spec.to_xml()
    return xml.replace(f'"{get_data_dir()}/', '"')


def load_display_spec(path: Path) -> mujoco.MjSpec:
    """A robot display_xml saved, its assets found under this machine's data root."""
    spec = mujoco.MjSpec.from_file(str(path))
    root = get_data(Path(spec.meshdir).parts[0]).parent
    spec.meshdir = f"{root}/{spec.meshdir}"
    spec.texturedir = f"{root}/{spec.texturedir}"  # type: ignore[attr-defined]
    return spec


def build_model(scene: Scene, rig: Rig) -> mujoco.MjModel:
    """The scene's boxes, the robot and each Mid-360 housing as a contact box, in one model."""
    spec = _with_scene(rig.spec(), scene)
    for sensor in rig.sensors:
        housing = spec.body(rig.base_body).add_geom()
        housing.name = sensor.name
        housing.type = mujoco.mjtGeom.mjGEOM_BOX
        housing.size = sensor.housing_half_extents
        housing.pos = sensor.mount_xyz
        q = Rotation.from_matrix(sensor.mount_r).as_quat()
        housing.quat = (q[3], q[0], q[1], q[2])
        housing.group = COLLISION_GROUP
    model = spec.compile()
    rig.tune(model)
    return model


def scene_edges(scene: Scene) -> NDArray[np.float64]:
    """The 12 edges of every box, as (N, 2, 3) segments."""
    centers = np.array([box.center for box in scene.boxes])[:, None, None, :]
    halves = np.array([box.half for box in scene.boxes])[:, None, None, :]
    turns = Rotation.from_euler("z", [[box.yaw] for box in scene.boxes]).as_matrix()
    corners = np.einsum("nij,nabj->nabi", turns, BOX_EDGES[None] * halves)
    edges: NDArray[np.float64] = (centers + corners).reshape(-1, 2, 3)
    return edges


def open_viewer(
    model: mujoco.MjModel, data: mujoco.MjData, lookat: NDArray[np.float64]
) -> mujoco.viewer.Handle:
    """A passive viewer on the scene with a free camera over the robot."""
    viewer = mujoco.viewer.launch_passive(model, data, show_left_ui=False, show_right_ui=False)
    viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    viewer.cam.lookat[:] = lookat
    viewer.cam.distance = 4.0
    viewer.cam.elevation = -55
    viewer.cam.azimuth = 225
    return viewer


class Sim:
    """The compiled scene with a robot and its Mid-360s, ticked from a velocity command."""

    def __init__(
        self,
        scene: Scene,
        seed: int,
        rig: Rig,
        body: Callable[[mujoco.MjModel, mujoco.MjData], Body],
    ) -> None:
        self.scene = scene
        self.rig = rig
        self.model = build_model(scene, rig)
        self.data = mujoco.MjData(self.model)
        self.robot = body(self.model, self.data)
        self.lidars = [
            sensor.lidar(MujocoRaycaster(self.model, self.data, LIDAR_GROUPS), seed + i)
            for i, sensor in enumerate(rig.sensors)
        ]
        self.lio = GroundTruthLio()
        self.ticks_per_frame = round(FRAME_DT / self.robot.dt)
        self.t = 0.0
        self._tick = 0
        self._points_per_step = int(POINT_RATE * self.model.opt.timestep)
        geom = mujoco.mjtObj.mjOBJ_GEOM
        self._kind = {
            mujoco.mj_name2id(self.model, geom, geom_name(i, box)): box.kind
            for i, box in enumerate(scene.boxes)
        }
        self._housings = {mujoco.mj_name2id(self.model, geom, s.name) for s in rig.sensors}

    def reset(self, x: float, y: float, z_feet: float, yaw: float) -> None:
        self.robot.reset(x, y, z_feet, yaw)
        self.t = 0.0
        self._tick = 0
        self.lio.clear()

    def sensor_pose(self, sensor: Sensor) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
        position, rotation = self.robot.base_pose()
        return position + rotation @ sensor.mount_xyz, rotation @ sensor.mount_r

    def base_pose(self) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
        return self.robot.base_pose()

    def odom_pose(self) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
        """The pose odometry reports, of the rig's one sensor or of its base."""
        if self.rig.cloud_frame == "base":
            return self.robot.base_pose()
        return self.sensor_pose(self.rig.sensors[0])

    def odom_velocity(self) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
        """Linear velocity in the world frame and angular velocity in the odometry child frame, as PointLio reports."""
        linear, angular = self.robot.base_velocity()
        if self.rig.cloud_frame == "base":
            return linear, angular
        sensor = self.rig.sensors[0]
        _, rotation = self.robot.base_pose()
        return linear + rotation @ np.cross(angular, sensor.mount_xyz), sensor.mount_r.T @ angular

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
        if geom in self._housings:
            return "lidar"
        return self.robot.part(geom)

    def tick(self, command: NDArray[np.float64]) -> LidarFrame | None:
        """Advance one control tick. Returns the lidar frame that completed on this tick, if any."""

        def cast() -> None:
            for sensor, lidar in zip(self.rig.sensors, self.lidars, strict=True):
                position, rotation = self.sensor_pose(sensor)
                points = lidar.cast(position, rotation, self._points_per_step)
                kept = points[np.linalg.norm(points, axis=1) >= sensor.lio_blind_range]
                self.lio.add(kept, position, rotation)

        self.robot.tick(command, cast)
        self._tick += 1
        self.t = self._tick * self.robot.dt
        if self._tick % self.ticks_per_frame:
            return None
        return self.lio.frame(self.t, *self.odom_pose())


class CommandHold:
    """The latest velocity command, or still once it is older than the driver's timeout."""

    def __init__(self, timeout: float) -> None:
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


class SimWorldConfig(ModuleConfig):
    family: Family = "office"
    seed: int = 1
    scene_params: Params = {}
    real_time_factor: float = Field(default=1.0, gt=0.0)
    mujoco_viewer: bool = False
    # idle until load_scene, so nothing from the configured scene reaches the stack
    wait_for_scene: bool = False


class SimWorld(Module, ABC):
    """A generated scene with a simulated robot and its Mid-360s, paced to the wall clock."""

    config: SimWorldConfig
    rig: ClassVar[Rig]

    cmd_vel: In[Twist]

    lidar: Out[PointCloud2]
    odometry: Out[Odometry]
    tf: Out[TFMessage]
    ground_truth: Out[PoseStamped]
    joint_state: Out[JointState]
    contacts: Out[Contacts]
    scene: Out[LineSegments3D]
    loaded_map: Out[PointCloud2]

    _thread: Thread | None = None
    _reset: tuple[float, float, float, float] | None = None

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._started = Event()
        self._load_lock = Lock()

    @rpc
    def start(self) -> None:
        super().start()
        self._hold = CommandHold(self.rig.command_timeout)
        self._stop_event = Event()
        self.register_disposable(Disposable(self.cmd_vel.subscribe(self._on_cmd_vel)))
        if not self.config.wait_for_scene:
            self._begin(self._load(self.config.family, self.config.seed, self.config.scene_params))
        self._started.set()

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

    @rpc
    def load_scene(self, family: Family, seed: int, params: Params) -> None:
        """Start a world that is waiting for a scene, with the robot at the scene's start."""
        if not self._started.wait(START_WAIT_S):
            raise TimeoutError("the world did not start")
        with self._load_lock:
            if self._thread is not None:
                raise RuntimeError("the world already runs a scene, set wait_for_scene to load one")
            self._begin(self._load(family, seed, params))

    @rpc
    def shape(self) -> BodyShape:
        return self.rig.shape

    @rpc
    def publish_premap(self, route: NDArray[np.float64], objects: bool) -> int:
        """Publish the Mid-360 returns walking the route on loaded_map, and count them."""
        sim = self._sim
        scene = sim.scene if objects else without_objects(sim.scene)
        cloud = premap_cloud(scene, sim.rig, np.asarray(route, dtype=np.float64))
        self.loaded_map.publish(
            PointCloud2.from_numpy(cloud, frame_id=sim.rig.world_frame, timestamp=time.time())
        )
        return len(cloud)

    @rpc
    def export_model(self, path: Path) -> None:
        """Write the robot alone as MJCF, for drawing it outside the sim."""
        path.write_text(display_xml(self.rig))

    def _on_cmd_vel(self, msg: Twist) -> None:
        command = np.array([msg.linear.x, msg.linear.y, msg.angular.z])
        if not self._hold.update(command, time.monotonic()):
            logger.warning("Ignored non-finite cmd_vel", command=command.tolist())

    @abstractmethod
    def _build(self, scene: Scene, seed: int) -> Sim:
        """The scene compiled around this world's robot."""

    def _load(self, family: Family, seed: int, params: Params) -> Sim:
        scene = generate(family, seed, **params)
        sim = self._build(scene, seed)
        sim.reset(*scene.start, 0.0)
        blocked = [c for c in sim.contacts() if c.kind != "floor"]
        if blocked:
            raise RuntimeError(f"robot starts inside scene geometry in {scene.name}: {blocked}")
        logger.info("Sim world ready", scene=scene.name)
        return sim

    def _begin(self, sim: Sim) -> None:
        self._sim = sim
        self._thread = Thread(target=self._run, daemon=True)
        self._thread.start()

    def _open_viewer(self, sim: Sim) -> mujoco.viewer.Handle | None:
        if not self.config.mujoco_viewer:
            return None
        return open_viewer(sim.model, sim.data, sim.base_pose()[0])

    def _run(self) -> None:
        # open3d loads on the first cloud, about a second
        PointCloud2.from_numpy(np.zeros((0, 3), np.float32), frame_id=self.rig.odom_child_frame)
        viewer = self._open_viewer(self._sim)
        try:
            self._simulate(self._sim, viewer)
        except Exception:
            logger.exception("Sim thread failed")
        finally:
            if viewer is not None:
                viewer.close()

    def _simulate(self, sim: Sim, viewer: mujoco.viewer.Handle | None) -> None:
        t0 = time.time()
        last_contacts: list[Contact] | None = None
        next_contacts_publish = 0.0
        next_scene_publish = 0.0
        while not self._stop_event.is_set():
            if (pose := self._reset) is not None:
                self._reset = None
                self._hold = CommandHold(sim.rig.command_timeout)
                sim.reset(*pose)
                t0 = time.time()
                last_contacts = None
                next_contacts_publish = next_scene_publish = 0.0
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
                    PointCloud2.from_numpy(
                        frame.points, frame_id=sim.rig.odom_child_frame, timestamp=stamp
                    )
                )
            if sim.t >= next_scene_publish:
                self.scene.publish(
                    LineSegments3D(
                        ts=stamp, frame_id=sim.rig.world_frame, segments=scene_edges(sim.scene)
                    )
                )
                next_scene_publish = sim.t + SCENE_PUBLISH_DT
            delay = stamp - time.time()
            if delay > 0:
                self._stop_event.wait(delay)
            elif delay < -1.0:
                logger.warning("Sim running slower than real time", behind_s=round(-delay, 1))
                t0 -= delay

    def _publish_poses(self, sim: Sim, stamp: float) -> None:
        world, child = sim.rig.world_frame, sim.rig.odom_child_frame
        position, rotation = sim.odom_pose()
        linear, angular = sim.odom_velocity()
        q = Rotation.from_matrix(rotation).as_quat()
        odom = Pose(*map(float, position), *map(float, q))
        self.odometry.publish(
            Odometry(
                ts=stamp,
                frame_id=world,
                child_frame_id=child,
                pose=odom,
                twist=Twist(Vector3(linear.tolist()), Vector3(angular.tolist())),
            )
        )
        self.tf.publish(
            TFMessage(
                Transform(
                    translation=Vector3(odom.position),
                    rotation=Quaternion(odom.orientation),
                    frame_id=world,
                    child_frame_id=child,
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
                frame_id=world,
            )
        )
        positions, velocities = sim.robot.joint_state()
        self.joint_state.publish(
            JointState(
                ts=stamp,
                frame_id=BASE_FRAME_ID,
                name=list(sim.robot.joint_names),
                position=positions.tolist(),
                velocity=velocities.tolist(),
            )
        )
