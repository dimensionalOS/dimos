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


from collections.abc import Iterable
from dataclasses import dataclass

from dimos.robot.assets.model import UsdModel

# The key a world announces a robot's ``odom_frame`` under, on the robot's entity.
ODOM_FRAME = "odom_frame"
# The key a world marks each robot's entity with.
ROBOT = "robot"


@dataclass(frozen=True)
class SimulatedRobot:
    """A robot's entity in the world: its id (the robot's namespace) and its own USD."""

    robot_id: str
    model: UsdModel
    # The world holds still until this robot is commanded: a robot held up by a policy falls before it is driven.
    held_until_commanded: bool = False
    # The tf frame the robot reports its odometry in, which sensors riding it map in.
    odom_frame: str | None = None
    # Free to move on its own (a floating base the world gives it); a robot driven on its own joints off its mount is not.
    free: bool = True

    @property
    def root_body(self) -> str:
        return self.model.root_link


def validate(simulated: Iterable[SimulatedRobot]) -> None:
    robot_ids = [d.robot_id for d in simulated]
    if len(set(robot_ids)) != len(robot_ids):
        raise ValueError(f"duplicate simulated robot ids: {sorted(robot_ids)}")
    unnamed = [d for d in simulated if not d.robot_id]
    if unnamed:
        raise ValueError(
            "a simulated robot is named by its namespace: put it in one, e.g. .namespace('g1')"
        )
