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

"""What the blueprint tells a world about each robot in it, as `sim_module(robots=[...])`.

A robot and very little of that: a sensor builds its own world out of the scene, and a robot's
gains live in its layer (`usd_with_drives`), where whatever runs it reads them from the same
place a real one would. What is left is what a layer cannot say -- the name the world calls
this robot, and how often its controller acts.
"""

from collections.abc import Iterable
from dataclasses import dataclass

from dimos.robot.assets.model import UsdModel

Vector = tuple[float, float, float]


@dataclass(frozen=True)
class SimulatedRobot:
    robot_id: str
    # Its USD: its sensors' frames are frames in it (UsdModel.with_frames adds them), and the
    # files it names travel with it, so there is no second place to say where its meshes are.
    model: UsdModel
    xyz: Vector = (0.0, 0.0, 0.0)
    rpy: Vector = (0.0, 0.0, 0.0)
    floating: bool = True
    # Its controller acts every this many sim seconds: the world sets "act" on it then, and waits for its actions.
    control_period: float | None = None

    @property
    def root_body(self) -> str:
        """The body everything else on this robot hangs off, as its own layer names it.

        Read from the description rather than stated beside it: a layer already says which of its
        links is the root, so a description that said so again could disagree with the robot it is
        describing.
        """
        return self.model.root_link


Simulated = SimulatedRobot


def validate(simulated: Iterable[Simulated]) -> None:
    """Robot ids are unique, because an id is what everything downstream calls that robot."""
    robot_ids = [d.robot_id for d in simulated]
    if len(set(robot_ids)) != len(robot_ids):
        raise ValueError(f"duplicate simulated robot ids: {sorted(robot_ids)}")
