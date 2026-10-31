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
from dataclasses import dataclass
from pathlib import Path
from typing import Literal

Vector = tuple[float, float, float]


@dataclass(frozen=True)
class Mount:
    """A pose on a body: a robot body name for robot sensors, a scene body or "world" otherwise."""

    body: str
    xyz: Vector = (0.0, 0.0, 0.0)
    rpy: Vector = (0.0, 0.0, 0.0)


@dataclass(frozen=True)
class SimulatedRobot:
    robot_id: str
    model: Path
    root_body: str
    xyz: Vector = (0.0, 0.0, 0.0)
    rpy: Vector = (0.0, 0.0, 0.0)
    floating: bool = True
    # Its controller acts every this many sim seconds: the world sends it an "events/act" then, and waits for its actions.
    control_period: float | None = None


@dataclass(frozen=True)
class SimulatedSensor:
    """Where a sensor sits; the engine decides what each kind becomes."""

    name: str
    kind: Literal["camera", "lidar", "imu"]
    mount: Mount
    robot_id: str | None = None
    fovy: float = 60.0


Simulated = SimulatedRobot | SimulatedSensor


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
