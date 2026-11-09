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
from dimos.sim2.usd_layer import usd_layer
from dimos.sim2.usd_names import resolve, resolved, shorten

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
    """Robots are prefixed "<robot_id>/"; scene-mounted sensors are prefixed "world/".

    A robot the world already holds is configured where it stands. A composed scene arrives with
    its robots grafted in -- that is what `get_scene` answers with, and the layer the viewer draws
    -- and attaching one on top of that would put a second copy of the robot in the room. Where
    the world does not hold it, its own layer is read and attached, which is how a scene that is
    only a room gets a robot at all.
    """
    _register_usd_reader(Path(scene))
    descriptions = list(simulated)
    robots = [d for d in descriptions if isinstance(d, SimulatedRobot)]
    sensors = [d for d in descriptions if isinstance(d, SimulatedSensor)]
    validate(descriptions)

    world = mujoco.MjSpec.from_file(str(scene))
    # Before anything looks for a robot in it: a composed world names everything by prim path,
    # and what a robot's parts are called leaves the simulation as entity ids and tf frames.
    shorten(world, ids=[robot.robot_id for robot in robots])
    for robot in robots:
        prefix = f"{robot.robot_id}/"
        standing = resolved([body.name for body in world.bodies], prefix + robot.root_body)
        if standing is not None:
            _configure(world, robot, sensors, root=world.body(standing), prefix=prefix)
            continue
        layer = usd_layer(robot.model)
        _register_usd_reader(layer)
        spec = mujoco.MjSpec.from_file(str(layer))
        # The same reason as above, for a layer that is one robot and nothing else.
        shorten(spec)
        robot_geoms = {geom.name for geom in spec.geoms}
        root_name = resolved([body.name for body in spec.bodies], robot.root_body)
        if root_name is None:
            raise ValueError(f"{robot.robot_id}: no body {robot.root_body!r} in {layer}")
        root = spec.body(root_name)
        # The description places the robot, not the source asset.
        root.pos = (0.0, 0.0, 0.0)
        root.quat = (1.0, 0.0, 0.0, 0.0)
        _configure(spec, robot, sensors, root=root, prefix="", layer=layer)
        frame = world.worldbody.add_frame(pos=robot.xyz, quat=_quaternion(robot.rpy))
        world.attach(spec, prefix=prefix, frame=frame)
        # Contact pairs naming a geom the robot lacks (its floor) mean the scene's geom of that name.
        for pair in world.pairs:
            for side in ("geomname1", "geomname2"):
                name = getattr(pair, side).removeprefix(prefix)
                if getattr(pair, side) != name and name not in robot_geoms:
                    setattr(pair, side, name)
    for sensor in sensors:
        if sensor.robot_id is None:
            _mount(world, sensor, "world", prefix="world/")
    return world.compile()


def _configure(
    spec: mujoco.MjSpec,
    robot: SimulatedRobot,
    sensors: Iterable[SimulatedSensor],
    *,
    root: Any,
    prefix: str,
    layer: Path | None = None,
) -> None:
    """Turn the described robot's actuators into servos and mount its sensors, in ``spec``.

    Called once per robot, on its own layer before it is attached or on the world it already
    stands in. ``prefix`` is how that robot's parts are named in ``spec``: nothing on its own
    layer, "<robot_id>/" in a world.
    """
    if not robot.floating:
        root.mocap = True
    for key in list(spec.keys):
        spec.delete(key)
    actuators = {actuator.target: actuator for actuator in spec.actuators}
    for servo in robot.servos:
        try:
            actuator = actuators[resolve(actuators, prefix + servo.joint)]
        except KeyError as error:
            # Naming where it looked, because "no 'left_hip_pitch_joint' among " with nothing
            # after it says only that the model arrived without its actuators, not which model
            # or from where.
            where = layer or "the world it stands in"
            raise ValueError(
                f"{robot.robot_id}: no actuator on {servo.joint!r} in {where} "
                f"({len(actuators)} actuator(s) there): {error}"
            ) from error
        # A motor's control range, if it has one, is its torque limit; a position actuator keeps its own (or its joint's).
        motor = (
            actuator.biastype == mujoco.mjtBias.mjBIAS_NONE
            and actuator.ctrlrange[0] < actuator.ctrlrange[1]
        )
        torque_limit = actuator.ctrlrange.copy()
        actuator.set_to_position(kp=servo.kp, kv=servo.kd)
        actuator.ctrllimited = mujoco.mjtLimited.mjLIMITED_FALSE
        if motor:
            actuator.forcelimited = mujoco.mjtLimited.mjLIMITED_TRUE
            actuator.forcerange = torque_limit
    for sensor in sensors:
        if sensor.robot_id == robot.robot_id:
            _mount(spec, sensor, robot.robot_id, prefix=prefix, look=prefix)


def _mount(
    spec: mujoco.MjSpec,
    sensor: SimulatedSensor,
    owner: str,
    prefix: str = "",
    look: str = "",
) -> None:
    """At its frame: a site of the model (on that site's body, at its pose), a body (at its origin), or "world".

    A frame read out of USD is named by its prim path, so the name the description uses is
    resolved against what the model actually calls things; a scene written as an MJCF names it
    the same either way.

    Two prefixes, because where a frame is found and what the thing mounted at it is called are
    not always the same. ``look`` is how that frame is named *in this spec*: nothing on a robot's
    own layer, "<robot_id>/" in a world the robot already stands in. ``prefix`` is what the site
    or camera is named, which for a scene-mounted sensor is "world/" and for a robot is whatever
    its parts are already called.
    """
    frame = sensor.frame_id
    site, body, pos, quat = None, spec.worldbody, np.zeros(3), np.array([1.0, 0.0, 0.0, 0.0])
    if frame != "world":
        site_name = resolved([s.name for s in spec.sites], look + frame)
        body_name = resolved([b.name for b in spec.bodies], look + frame)
        site = spec.site(site_name) if site_name else None
        body = spec.body(body_name) if body_name else None
        if site is not None:
            # Compiled, so a site oriented any way (euler, axisangle, ...) reads as pos and quat.
            compiled = spec.copy().compile()
            index = compiled.site(site_name).id
            body, pos, quat = site.parent, compiled.site_pos[index], compiled.site_quat[index]
        elif body is None:
            raise ValueError(f"{owner}: no site or body {look + frame!r} for its {sensor.kind}")
    name = prefix + frame
    if sensor.kind == "camera":
        # The frame is the optical frame (z forward, y down); a MuJoCo camera looks down -z, y up.
        optical = Rotation.from_quat(quat, scalar_first=True) * Rotation.from_euler("x", np.pi)
        body.add_camera(
            name=name, fovy=sensor.fovy, pos=pos, quat=optical.as_quat(scalar_first=True)
        )
    # A site of its own unless the frame already *is* a site under that very name, in which case
    # there is nothing to add and adding it would be a second site of the same name.
    elif resolved([s.name for s in spec.sites], name) != name:
        body.add_site(name=name, size=_SITE_SIZE, rgba=(0, 0, 0, 0), pos=pos, quat=quat)
    if sensor.kind == "imu":
        for suffix, kind in _IMU_SENSORS.items():
            spec.add_sensor(
                name=f"{name}/{suffix}", type=kind, objtype=mujoco.mjtObj.mjOBJ_SITE, objname=name
            )


def _quaternion(rpy: Vector) -> NDArray[np.float64]:
    return np.asarray(Rotation.from_euler("xyz", rpy).as_quat(scalar_first=True), dtype=np.float64)
