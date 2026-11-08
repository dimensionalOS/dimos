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


# USD suffixes MuJoCo will read, once its reader is registered.
_USD_SUFFIXES = frozenset({".usd", ".usda", ".usdc", ".usdz"})


def _register_usd_reader(scene: Path) -> None:
    """MuJoCo reads USD, but only once `mujoco._usd` has been imported.

    Without it `MjSpec.from_file` falls back to the MJCF parser and a USD layer comes back as
    "syntax error: line 1, column 0" -- XML's complaint about a file that was never XML. The
    import is what registers the reader, so it has to happen in whichever process loads the
    scene, not merely somewhere in the application.
    """
    if scene.suffix.lower() not in _USD_SUFFIXES:
        return
    try:
        import mujoco._usd  # noqa: F401
    except ImportError as error:
        raise ImportError(
            f"{scene} is a USD layer and this MuJoCo cannot read one: {error}"
        ) from error


def compile_world(scene: Path, simulated: Iterable[Simulated]) -> mujoco.MjModel:
    """Robots are prefixed "<robot_id>/"; scene-mounted sensors are prefixed "world/"."""
    _register_usd_reader(Path(scene))
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
    """At its frame: a site of the model (on that site's body, at its pose), a body (at its origin), or "world"."""
    frame = sensor.frame_id
    site, body, pos, quat = None, spec.worldbody, np.zeros(3), np.array([1.0, 0.0, 0.0, 0.0])
    if frame != "world":
        site, body = spec.site(frame), spec.body(frame)
        if site is not None:
            # Compiled, so a site oriented any way (euler, axisangle, ...) reads as pos and quat.
            compiled = spec.copy().compile()
            index = compiled.site(frame).id
            body, pos, quat = site.parent, compiled.site_pos[index], compiled.site_quat[index]
        elif body is None:
            raise ValueError(f"{owner}: no site or body {frame!r} for its {sensor.kind}")
    name = prefix + frame
    if sensor.kind == "camera":
        # The frame is the optical frame (z forward, y down); a MuJoCo camera looks down -z, y up.
        optical = Rotation.from_quat(quat, scalar_first=True) * Rotation.from_euler("x", np.pi)
        body.add_camera(
            name=name, fovy=sensor.fovy, pos=pos, quat=optical.as_quat(scalar_first=True)
        )
    elif site is None or prefix:
        body.add_site(name=name, size=_SITE_SIZE, rgba=(0, 0, 0, 0), pos=pos, quat=quat)
    if sensor.kind == "imu":
        for suffix, kind in _IMU_SENSORS.items():
            spec.add_sensor(
                name=f"{name}/{suffix}", type=kind, objtype=mujoco.mjtObj.mjOBJ_SITE, objname=name
            )


def _quaternion(rpy: Vector) -> NDArray[np.float64]:
    return np.asarray(Rotation.from_euler("xyz", rpy).as_quat(scalar_first=True), dtype=np.float64)
