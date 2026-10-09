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

"""The navigation stack, as ports: a stream in, a stream out, and tf.

``NavigationInterfaceSpec`` is the one goal contract: the calls skills make on
whichever global planner the blueprint holds, and the ``nav_status`` stream that
planner reports each goal on.

A local planner is blind or map-aware; both hand the follower the same ``path``.

A ``path`` with one pose means "hold, no safe route"; with none, "stop".

tf is ``IO`` where the implementers are dimos-module natives (``#[tf]`` both
subscribes and publishes) and their python twins; ``In`` on the global planner,
whose native only reads it.
"""

from enum import Enum
from typing import Protocol

from dimos_lcm.actionlib_msgs import GoalID, GoalStatus
from dimos_lcm.std_msgs import Time

from dimos.core.stream import IO, In, Out
from dimos.msgs.geometry_msgs.PointStamped import PointStamped
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.nav_msgs.Path import Path
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.spec.utils import Spec
from dimos.types.timestamped import Timestamped


class NavigationState(Enum):
    IDLE = "idle"
    FOLLOWING_PATH = "following_path"


def goal_id(goal: Timestamped) -> str:
    """The id nav_status reports carry for a goal, which is its stamp."""
    sec, nsec = goal.ros_timestamp()
    return f"{sec}.{nsec:09d}"


def goal_status(goal: Timestamped, status: int, text: str) -> GoalStatus:
    """A nav_status report for a goal."""
    sec, nsec = goal.ros_timestamp()
    return GoalStatus(
        goal_id=GoalID(stamp=Time(sec=sec, nsec=nsec), id=goal_id(goal)),
        status=status,
        text=text,
    )


def goal_ended(goal: Timestamped, msg: GoalStatus) -> GoalStatus | None:
    """The report that ended a goal, or None when this one does not show it over."""
    if msg.goal_id.id == goal_id(goal):
        return None if msg.status in (GoalStatus.PENDING, GoalStatus.ACTIVE) else msg
    # A newer goal means the report that ended this one was missed.
    if [msg.goal_id.stamp.sec, msg.goal_id.stamp.nsec] > goal.ros_timestamp():
        return goal_status(goal, GoalStatus.PREEMPTED, "replaced")
    return None


class GlobalPlanner(Protocol):
    tf: In[TFMessage]
    goal: In[PointStamped]

    path: Out[Path]


class NavigationInterfaceSpec(Spec, Protocol):
    """The goal RPCs of a global planner and the stream it reports goals on."""

    nav_status: Out[GoalStatus]
    """Each goal's status, under the id of the message that set it."""

    def set_goal(self, goal: PoseStamped) -> bool:
        """Set a new goal without blocking. True if it was accepted.

        The goal's stamp identifies it, so stamp each goal afresh.
        """

    def get_status(self) -> GoalStatus | None:
        """The newest goal's latest status. None before any goal."""

    def cancel_goal(self) -> bool:
        """Cancel the current goal. False if none was held."""


class BlindLocalPlanner(Protocol):
    """Shapes the global route without looking: no map."""

    planner_path: In[Path]
    tf: In[TFMessage]

    path: Out[Path]


class MapLocalPlanner(Protocol):
    """Routes around what the local map says is there."""

    planner_path: In[Path]
    local_map: In[PointCloud2]
    tf: IO[TFMessage]

    path: Out[Path]


class TrajectoryFollower(Protocol):
    tf: IO[TFMessage]
    path: In[Path]

    nav_cmd_vel: Out[Twist]
