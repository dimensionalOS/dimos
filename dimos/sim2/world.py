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

"""Compile a scene, its robots and their sensor mounts into one MuJoCo model."""

from collections.abc import Iterable
from pathlib import Path
from typing import Any

import mujoco
import numpy as np
from numpy.typing import NDArray
from scipy.spatial.transform import Rotation

from dimos.sim2.spec import Simulated, SimulatedRobot, SimulatedSensor, Vector, validate

# Sites only mark a pose; keep them invisible and negligible.
_SITE_SIZE = (0.001, 0.001, 0.001)
_IMU_SENSORS = {
    "gyro": mujoco.mjtSensor.mjSENS_GYRO,
    "accelerometer": mujoco.mjtSensor.mjSENS_ACCELEROMETER,
    "orientation": mujoco.mjtSensor.mjSENS_FRAMEQUAT,
}


def compile_world(scene: Path, simulated: Iterable[Simulated]) -> mujoco.MjModel:
    """Robots are prefixed "<robot_id>/"; scene-mounted sensors are prefixed "world/"."""
    descriptions = list(simulated)
    robots = [d for d in descriptions if isinstance(d, SimulatedRobot)]
    sensors = [d for d in descriptions if isinstance(d, SimulatedSensor)]
    validate(descriptions)

    world = mujoco.MjSpec.from_file(str(scene))
    for robot in robots:
        spec = mujoco.MjSpec.from_file(str(robot.model))
        root = spec.body(robot.root_body)
        if root is None:
            raise ValueError(f"{robot.robot_id}: no body {robot.root_body!r} in {robot.model}")
        # The description places the robot, not the source asset.
        root.pos = (0.0, 0.0, 0.0)
        root.quat = (1.0, 0.0, 0.0, 0.0)
        if not robot.floating:
            root.mocap = True
        for key in list(spec.keys):
            spec.delete(key)
        for sensor in sensors:
            if sensor.robot_id == robot.robot_id:
                _mount(spec, sensor, robot.robot_id)
        frame = world.worldbody.add_frame(pos=robot.xyz, quat=_quaternion(robot.rpy))
        world.attach(spec, prefix=f"{robot.robot_id}/", frame=frame)
    for sensor in sensors:
        if sensor.robot_id is None:
            _mount(world, sensor, "world", prefix="world/")
    return world.compile()


def _mount(spec: mujoco.MjSpec, sensor: SimulatedSensor, owner: str, prefix: str = "") -> None:
    body: Any = spec.worldbody if sensor.mount.body == "world" else spec.body(sensor.mount.body)
    if body is None:
        raise ValueError(f"{owner}/{sensor.name}: no body {sensor.mount.body!r}")
    pose = {"pos": sensor.mount.xyz, "quat": _quaternion(sensor.mount.rpy)}
    if sensor.kind == "camera":
        # The mount is the optical frame (z forward, y down); a MuJoCo camera looks down -z, y up.
        rotation = Rotation.from_euler("xyz", sensor.mount.rpy) * Rotation.from_euler("x", np.pi)
        pose["quat"] = rotation.as_quat(scalar_first=True)
        body.add_camera(name=prefix + sensor.name, fovy=sensor.fovy, **pose)
    else:
        body.add_site(name=prefix + sensor.name, size=_SITE_SIZE, rgba=(0, 0, 0, 0), **pose)
    if sensor.kind == "imu":
        for suffix, kind in _IMU_SENSORS.items():
            spec.add_sensor(
                name=f"{prefix}{sensor.name}/{suffix}",
                type=kind,
                objtype=mujoco.mjtObj.mjOBJ_SITE,
                objname=prefix + sensor.name,
            )


def _quaternion(rpy: Vector) -> NDArray[np.float64]:
    return np.asarray(Rotation.from_euler("xyz", rpy).as_quat(scalar_first=True), dtype=np.float64)
