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

"""What a module sends Simulation, as {"simulated": ...} from a @shared_config(Simulation) method."""

from collections.abc import Iterable, Mapping
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Any, Literal
import xml.etree.ElementTree as ET

import numpy as np
from numpy.typing import NDArray
from scipy.spatial.transform import Rotation

Vector = tuple[float, float, float]


@dataclass(frozen=True)
class SimulatedRobot:
    robot_id: str
    root_body: str
    # None: the model is the robot_description of the same name (a UrdfManager's), in the format the engine loads.
    model: Path | None = None
    xyz: Vector = (0.0, 0.0, 0.0)
    rpy: Vector = (0.0, 0.0, 0.0)
    floating: bool = True
    # Its controller acts every this many sim seconds: the world sends it an "events/act" then, and waits for its actions.
    control_period: float | None = None


@dataclass(frozen=True)
class SimulatedSensor:
    """A sensor at a frame: one in a robot's URDF, or a body of the scene. The engine decides what each kind becomes.

    Simulation fills in where the frame sits: the robot and body it is fixed to, and its pose on that body.
    A camera's frame is its optical frame (z forward, x right, y down).
    """

    kind: Literal["camera", "lidar", "imu"]
    frame_id: str
    fovy: float = 60.0
    robot_id: str | None = None
    body: str = "world"
    xyz: Vector = (0.0, 0.0, 0.0)
    rpy: Vector = (0.0, 0.0, 0.0)


Simulated = SimulatedRobot | SimulatedSensor


def resolve(
    simulated: Iterable[Simulated], descriptions: Mapping[str, Mapping[str, Any]], model_format: str
) -> list[Simulated]:
    """Each robot's model from its robot_description, and each sensor placed at its frame in the robots' URDFs.

    A frame is placed on the nearest link above it that a moving joint (or nothing) holds: a body of the model, as
    a URDF made from it (mjcf_to_urdf) names them. A frame in no URDF is a body of the scene.
    """
    descriptions = dict(descriptions)
    resolved: list[Simulated] = []
    robots = {d.robot_id: d for d in simulated if isinstance(d, SimulatedRobot)}
    frames: dict[str, tuple[str, str, NDArray[np.float64]]] = {}
    for robot_id, robot in robots.items():
        robot_description = descriptions.get(robot_id)
        if robot_description is None:
            continue
        frames.update(_frames(robot_id, robot.root_body, robot_description["urdf"]))
        if robot.model is None:
            model = robot_description["formats"].get(model_format)
            if model is None:
                raise ValueError(f"{robot_id}'s robot_description has no {model_format!r} format")
            robot = replace(robot, model=Path(model))
            robots[robot_id] = robot
    for description in simulated:
        if isinstance(description, SimulatedRobot):
            robot = robots[description.robot_id]
            if robot.model is None:
                raise ValueError(f"{robot.robot_id} has no model and no robot_description names it")
            resolved.append(robot)
        elif description.frame_id in frames:
            robot_id, body, transform = frames[description.frame_id]
            x, y, z = (float(v) for v in transform[:3, 3])
            roll, pitch, yaw = (
                float(v) for v in Rotation.from_matrix(transform[:3, :3]).as_euler("xyz")
            )
            xyz, rpy = (x, y, z), (roll, pitch, yaw)
            resolved.append(replace(description, robot_id=robot_id, body=body, xyz=xyz, rpy=rpy))
        else:
            resolved.append(replace(description, robot_id=None, body=description.frame_id))
    return resolved


def _frames(
    robot_id: str, root_body: str, urdf: str
) -> dict[str, tuple[str, str, NDArray[np.float64]]]:
    """Every URDF link: (robot, the body it is fixed to, its pose on that body)."""
    joints: dict[str, tuple[str, str, NDArray[np.float64]]] = {}
    root = ET.fromstring(urdf)
    for joint in root.findall("joint"):
        element = joint.find("origin")
        origin = np.eye(4)
        if element is not None:
            origin[:3, :3] = Rotation.from_euler("xyz", _triple(element.get("rpy"))).as_matrix()
            origin[:3, 3] = _triple(element.get("xyz"))
        child = joint.find("child").get("link", "")  # type: ignore[union-attr]
        parent = joint.find("parent").get("link", "")  # type: ignore[union-attr]
        joints[child] = (parent, joint.get("type", "fixed"), origin)
    frames: dict[str, tuple[str, str, NDArray[np.float64]]] = {}
    for link in root.findall("link"):
        frame = name = link.get("name", "")
        transform = np.eye(4)
        while name in joints and joints[name][1] == "fixed":
            parent, _, origin = joints[name]
            name, transform = parent, origin @ transform
        frames[frame] = (robot_id, name if name in joints else root_body, transform)
    return frames


def _triple(text: str | None) -> list[float]:
    return [float(v) for v in text.split()] if text else [0.0, 0.0, 0.0]


def validate(simulated: Iterable[Simulated]) -> None:
    """Robot ids are unique and every robot-mounted sensor names one of them."""
    descriptions = list(simulated)
    robot_ids = [d.robot_id for d in descriptions if isinstance(d, SimulatedRobot)]
    if len(set(robot_ids)) != len(robot_ids):
        raise ValueError(f"duplicate simulated robot ids: {sorted(robot_ids)}")
    mounted_on = {
        d.robot_id
        for d in descriptions
        if isinstance(d, SimulatedSensor) and d.robot_id is not None
    }
    unknown = mounted_on - set(robot_ids)
    if unknown:
        raise ValueError(f"sensors are mounted on unknown robots: {sorted(unknown)}")
