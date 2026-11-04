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

"""The Go2's agent skills behind an MCP server: navigation (Go2Navigation puts the MLS stack behind the skills' interface), observe, person following, sport commands and move_to,
speech, web text input, and spatial memory with its perceive loop, driven by the MCP client's LLM, as the old
unitree-go2-agentic was.

Everything is imported when the blueprint is built, so the Go2 runs without the agent extras installed."""

from __future__ import annotations

import time

from dimos_lcm.std_msgs import Bool as LcmBool

from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.PointStamped import PointStamped
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.std_msgs.Bool import Bool
from dimos.navigation.base import NavigationState
from dimos.robot.unitree.go2.config import front_camera_info
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class Go2Navigation(Module):
    """The navigation skills' set_goal / get_state / is_goal_reached / cancel_goal, over MLS planning and the
    trajectory follower: a goal goes to the planner, arrival comes back on goal_reached."""

    goal: Out[PointStamped]
    # MovementManager's type for it
    stop_movement: Out[LcmBool]
    goal_reached: In[Bool]

    _state: NavigationState = NavigationState.IDLE
    _reached: bool = False

    @rpc
    def start(self) -> None:
        super().start()
        self.register_disposable(self.goal_reached.observable().subscribe(self._on_goal_reached))  # type: ignore[no-untyped-call]

    def _on_goal_reached(self, msg: Bool) -> None:
        if msg.data and self._state == NavigationState.FOLLOWING_PATH:
            self._reached = True
            self._state = NavigationState.IDLE

    @rpc
    def set_goal(self, goal: PoseStamped) -> bool:
        self._reached = False
        self._state = NavigationState.FOLLOWING_PATH
        position = goal.position
        self.goal.publish(
            PointStamped(
                ts=time.time(), frame_id=goal.frame_id, x=position.x, y=position.y, z=position.z
            )
        )
        return True

    @rpc
    def get_state(self) -> NavigationState:
        return self._state

    @rpc
    def is_goal_reached(self) -> bool:
        return self._reached

    @rpc
    def cancel_goal(self) -> bool:
        self.stop_movement.publish(LcmBool(data=True))
        # a NaN goal clears the planner's, as MovementManager's cancel does
        nan = float("nan")
        self.goal.publish(PointStamped(ts=time.time(), frame_id="map", x=nan, y=nan, z=nan))
        self._state = NavigationState.IDLE
        return True


def agentic() -> Blueprint:
    """The skills, the MCP server and its client, or nothing (logged) when their dependencies aren't installed."""
    try:
        from dimos.agents.mcp.mcp_client import McpClient
        from dimos.agents.mcp.mcp_server import McpServer
        from dimos.agents.skills.navigation import NavigationSkillContainer
        from dimos.agents.skills.observe_skill import ObserveSkill
        from dimos.agents.skills.person_follow import PersonFollowSkillContainer
        from dimos.agents.skills.speak_skill import SpeakSkill
        from dimos.agents.web_human_input import WebInput
        from dimos.perception.experimental.perceive_loop_skill import PerceiveLoopSkill
        from dimos.perception.experimental.spatial_perception import SpatialMemory
        from dimos.robot.unitree.unitree_skill_container import UnitreeSkillContainer
    except ImportError as error:
        logger.warning(f"Go2 agent skills off, their dependencies aren't installed: {error}")
        return autoconnect()
    return autoconnect(
        McpServer.blueprint(),
        McpClient.blueprint(),
        Go2Navigation.blueprint(),
        NavigationSkillContainer.blueprint(),
        ObserveSkill.blueprint(),
        PersonFollowSkillContainer.blueprint(camera_info=front_camera_info()),
        UnitreeSkillContainer.blueprint(),
        WebInput.blueprint(),
        SpeakSkill.blueprint(),
        SpatialMemory.blueprint(),
        PerceiveLoopSkill.blueprint(),
    )
