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

"""Compose native scene and robot MJCFs once, with named device attachments."""

from __future__ import annotations

from pathlib import Path

import mujoco
import numpy as np
from numpy.typing import NDArray
from scipy.spatial.transform import Rotation

from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.sim2.sensors.camera.spec import Camera
from dimos.sim2.sensors.composition import Sensor, mounted_sensors
from dimos.sim2.sensors.imu.spec import Imu
from dimos.sim2.sensors.spec import Mount
from dimos.sim2.spec import RobotConfig, RobotInstance, WorldConfig
from dimos.sim2.world.scene_types import SceneDescription
from dimos.utils.data import LfsPath


def quaternion(rpy: tuple[float, float, float]) -> NDArray[np.float64]:
    return np.asarray(Rotation.from_euler("xyz", rpy).as_quat(scalar_first=True), dtype=np.float64)


def load_scene(config: WorldConfig, description: SceneDescription | None = None) -> mujoco.MjModel:
    world = mujoco.MjSpec.from_file(str(config.scene))
    description = description if description is not None else describe_scene(config.scene)
    for geom in world.geoms:
        if geom.group in description.hidden_geom_groups:
            geom.rgba[3] = 0
    world.option.timestep = config.timestep
    world.option.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST
    for robot_id, instance in config.robots.items():
        robot = mujoco.MjSpec.from_file(str(instance.config.model))
        robot.option.integrator = world.option.integrator
        if instance.config.meshdir is not None:
            robot.meshdir = str(instance.config.meshdir)
        root = robot.body(instance.config.root_body)
        if root is None:
            raise ValueError(f"{robot_id}: unknown robot root {instance.config.root_body!r}")
        # Root placement belongs to the instance, not to the source asset.
        root.pos = (0.0, 0.0, 0.0)
        root.quat = (1.0, 0.0, 0.0, 0.0)
        if not instance.config.floating:
            root.mocap = True
        for key in list(robot.keys):
            robot.delete(key)
        _attach_sensors(robot, instance.config.mounted_sensors, robot_id)
        frame = world.worldbody.add_frame(pos=instance.xyz, quat=quaternion(instance.rpy))
        world.attach(robot, prefix=robot_id + "/", frame=frame)
    _attach_sensors(world, mounted_sensors(config.sensors), "world")
    cameras = [
        s
        for sensors in (config.sensors, *(r.config.sensors for r in config.robots.values()))
        for s in sensors
        if isinstance(s, Camera)
    ]
    world.visual.global_.offwidth = max(640, *(s.width for s in cameras)) if cameras else 640
    world.visual.global_.offheight = max(480, *(s.height for s in cameras)) if cameras else 480
    model = world.compile()
    for robot_id, instance in config.robots.items():
        if instance.config.configure_physics is not None:
            instance.config.configure_physics(model, robot_id + "/")
    return model


def _attach_sensors(spec: mujoco.MjSpec, sensors: tuple[Sensor, ...], owner: str) -> None:
    for sensor in sensors:
        attachment = sensor.camera if isinstance(sensor, Camera) else sensor.site
        if isinstance(attachment, Mount):
            body = spec.body(attachment.link)
            if body is None:
                raise ValueError(f"{owner}/{sensor.name}: unknown body {attachment.link!r}")
            if isinstance(sensor, Camera):
                body.add_camera(
                    name=sensor.model_name,
                    pos=attachment.xyz,
                    quat=quaternion(attachment.rpy),
                    fovy=60.0 if sensor.fovy is None else sensor.fovy,
                )
            else:
                body.add_site(
                    name=sensor.model_name,
                    pos=attachment.xyz,
                    quat=quaternion(attachment.rpy),
                    size=(0.001, 0.001, 0.001),
                    rgba=(0, 0, 0, 0),
                )
        else:
            kind = "camera" if isinstance(sensor, Camera) else "site"
            target = (
                spec.camera(attachment) if isinstance(sensor, Camera) else spec.site(attachment)
            )
            if target is None:
                raise ValueError(f"{owner}/{sensor.name}: unknown {kind} {attachment!r}")
        if isinstance(sensor, Imu):
            for suffix, kind_id in (
                ("gyro", mujoco.mjtSensor.mjSENS_GYRO),
                ("accel", mujoco.mjtSensor.mjSENS_ACCELEROMETER),
                ("quat", mujoco.mjtSensor.mjSENS_FRAMEQUAT),
            ):
                spec.add_sensor(
                    name=f"sensor/{sensor.name}/{suffix}",
                    type=kind_id,
                    objtype=mujoco.mjtObj.mjOBJ_SITE,
                    objname=sensor.model_name,
                )


def scene_path(value: str | None, default: str) -> Path:
    if value is None or value == "none":
        return LfsPath("sim2/scenes") / default
    path = Path(value).expanduser()
    if not path.exists() and len(path.parts) == 1:
        path = LfsPath("sim2/scenes") / path
    if path.is_dir():
        path = path / "scene.xml"
    if not path.is_file():
        names = ", ".join(list_scenes())
        raise FileNotFoundError(f"sim2 scene {value!r} is not installed; available: {names}")
    return path.resolve()


def list_scenes() -> list[str]:
    """List installed native scene names without loading or compiling them."""
    root = Path(str(LfsPath("sim2/scenes")))
    return sorted(
        [p.name for p in root.iterdir() if p.is_dir() and (p / "scene.xml").is_file()]
        + [p.name for p in root.glob("*.xml")]
    )


def describe_scene(path: Path) -> SceneDescription:
    """Read the optional semantic sidecar; raw MJCF remains runnable without it."""
    metadata = path.with_suffix(".json")
    if not metadata.is_file():
        return SceneDescription(id=path.parent.name if path.name == "scene.xml" else path.stem)
    return SceneDescription.model_validate_json(metadata.read_text())


def scene_robot(
    path: Path, config: RobotConfig, spawn: str = "default", *, default: tuple[float, float, float]
) -> RobotInstance:
    """Place a robot above a named support, or an explicit support default.

    The scene supplies location/orientation; the robot supplies root height.
    Direct RobotInstance placement and live pose edits remain absolute.
    """
    description = describe_scene(path)
    if description.spawns and spawn not in description.spawns:
        raise ValueError(
            f"scene {description.id!r} has no authored {spawn!r} support; "
            f"available: {', '.join(description.spawns)}"
        )
    pose = description.spawns.get(spawn, Pose(*default)) + Pose(0, 0, config.spawn_height)
    return RobotInstance(
        config, xyz=pose.position.to_tuple(), rpy=pose.orientation.to_euler().to_tuple()
    )
