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

from collections.abc import Iterable
from dataclasses import dataclass, replace
from functools import cache
from pathlib import Path
from typing import Literal
import xml.etree.ElementTree as ET

Vector = tuple[float, float, float]


@dataclass(frozen=True)
class SimulatedRobot:
    robot_id: str
    # Its MJCF: its sensors' frames are sites (or bodies) in it (mjcf_with_frames adds them).
    model: Path
    root_body: str
    xyz: Vector = (0.0, 0.0, 0.0)
    rpy: Vector = (0.0, 0.0, 0.0)
    floating: bool = True
    # Its controller acts every this many sim seconds: the world sets "act" on it then, and waits for its actions.
    control_period: float | None = None


@dataclass(frozen=True)
class SimulatedSensor:
    """A sensor at a frame: a site or body of a robot's model, or of the scene. The engine decides what each kind becomes.

    Simulation fills in robot_id: the robot whose model names the frame (None: the scene's).
    A camera's frame is its optical frame (z forward, x right, y down).
    """

    kind: Literal["camera", "lidar", "imu"]
    frame_id: str
    fovy: float = 60.0
    robot_id: str | None = None


Simulated = SimulatedRobot | SimulatedSensor


def resolve(simulated: Iterable[Simulated]) -> list[Simulated]:
    """Each sensor with the robot whose model names its frame; one named by no robot's model is the scene's."""
    descriptions = list(simulated)
    sensed = {d.frame_id for d in descriptions if isinstance(d, SimulatedSensor)}
    frames: dict[str, str] = {}
    for robot in descriptions:
        if not isinstance(robot, SimulatedRobot):
            continue
        for frame in sensed & frame_names(robot.model):
            if frame in frames:
                raise ValueError(
                    f"frame {frame!r} is in both {frames[frame]}'s and {robot.robot_id}'s model"
                )
            frames[frame] = robot.robot_id
    return [
        replace(d, robot_id=frames.get(d.frame_id)) if isinstance(d, SimulatedSensor) else d
        for d in descriptions
    ]


@cache
def frame_names(mjcf: Path) -> frozenset[str]:
    """The names of an MJCF's bodies and sites."""
    root = ET.parse(mjcf).getroot()
    return frozenset(
        name
        for tag in ("body", "site")
        for element in root.iter(tag)
        if (name := element.get("name"))
    )


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
