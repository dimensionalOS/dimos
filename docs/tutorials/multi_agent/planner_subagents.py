# Copyright 2025 Dimensional Inc.
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

"""Multi-agent tutorial: Planner + Subagents pattern.

This file contains the module and agent classes for the multi-agent tutorial.

Note: can't use both @skill and @rpc decorators together for a method:
    Methods decorated with both @skill() and @rpc cannot be referenced via
    RPC from other modules. The @skill() decorator wraps the method in a local
    function that cannot be pickled for LCM transport. Workarounds:

    1. Use only @skill() if the method is called by agents as a tool
    2. Use only @rpc if the method is called via RPC from other modules
    3. If both are needed, create separate methods or have the calling module
       implement the functionality directly
"""

import time

from langchain_core.messages import HumanMessage

from dimos.agents2.agent import LlmAgent
from dimos.agents2.spec import AnyMessage
from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.rpc_client import RpcCall, RPCClient
from dimos.protocol.skill.skill import skill
from dimos.protocol.skill.type import Return
from dimos.utils.logging_config import setup_logger

# Metadata keys for tracking message flow between agents.
# From = where the message originated; To = which agent is processing it.
FROM_AGENT_KEY = "from_agent"
TO_AGENT_KEY = "to_agent"

# TEMPORARY WORKAROUND: The base Agent class doesn't track which agent sent a query
# via RPC. To show proper From/To in the tutorial's agentspy display, we encode
# the source agent in the query string itself (e.g., "FROM:PlannerAgent|actual query").
# The receiving agent parses this out before processing. This should eventually be
# replaced by proper metadata support in the RPC/Agent infrastructure.
FROM_PREFIX = "FROM:"
FROM_DELIMITER = "|"

logger = setup_logger("multi_agent_tutorial")


def get_from_to(msg) -> tuple[str, str]:
    """Extract (from_agent, to_agent) from message additional_kwargs."""
    return (
        msg.additional_kwargs.get(FROM_AGENT_KEY, "?"),
        msg.additional_kwargs.get(TO_AGENT_KEY, "?"),
    )


# =============================================================================
# Robot Capabilities - Physical actions the robot can perform
# =============================================================================


class RobotCapabilities(Module):
    """Low-level physical capabilities for the robot."""

    rpc_calls = []

    @skill()
    def speak(self, text: str) -> str:
        """Speak text through the robot's speakers.

        Args:
            text: The text to speak.

        Returns:
            Status message.

        Note:
            This method uses only @skill (not @rpc) because combining both
            decorators on a method that's referenced via RPC causes pickle
            errors - the skill wrapper is a local function that can't be
            serialized. The agent calls this directly as a tool.
        """
        time.sleep(0.1)
        logger.info(f"[Robot] Speaking: {text}")
        return f"Spoke: {text}"

    @skill()
    def approach_user(self) -> str:
        """Move to the user's location.

        Returns:
            Status message.
        """
        time.sleep(0.2)
        logger.info("[Robot] Approaching user")
        return "Approached user"

    @rpc
    def set_PlannerAgent_register_skills(self, register_skills: RpcCall) -> None:
        """Auto-register skills with the PlannerAgent."""
        register_skills.set_rpc(self.rpc)
        register_skills(RPCClient(self, self.__class__))

    @rpc
    def start(self) -> None:
        super().start()

    @rpc
    def stop(self) -> None:
        super().stop()

    def __getstate__(self):
        return {}

    def __setstate__(self, state):
        pass


# =============================================================================
# Agents
# =============================================================================


class AgentWithFromToMetadata(LlmAgent):
    """Mixin that adds from/to metadata to published messages for observability."""

    _current_from: str = "Human"  # Tracks 'from' for current query

    async def agent_loop(self, first_query: str = "") -> str:
        """Override to parse 'from' agent from query string prefix."""
        if first_query.startswith(FROM_PREFIX):
            from_part, first_query = first_query.split(FROM_DELIMITER, 1)
            self._current_from = from_part.replace(FROM_PREFIX, "")
        else:
            self._current_from = "Human"
        return await super().agent_loop(first_query)

    def publish(self, msg: AnyMessage) -> None:
        # For HumanMessage (queries): from=tracked source, to=this agent
        # For AIMessage with tool_calls: from=this agent, to=Tools (internal action)
        # For AIMessage without tool_calls: from=this agent, to=whoever sent us the query
        if isinstance(msg, HumanMessage):
            msg.additional_kwargs[FROM_AGENT_KEY] = self._current_from
            msg.additional_kwargs[TO_AGENT_KEY] = self.__class__.__name__
        else:
            msg.additional_kwargs[FROM_AGENT_KEY] = self.__class__.__name__
            has_tool_calls = getattr(msg, "tool_calls", None)
            msg.additional_kwargs[TO_AGENT_KEY] = "Tools" if has_tool_calls else self._current_from
        super().publish(msg)


class PlannerAgent(AgentWithFromToMetadata):
    """Coordinator agent that delegates to specialist subagents.

    Receives user requests, consults subagents for analysis,
    and uses action skills to help the user.
    """

    pass


class WellbeingAgent(AgentWithFromToMetadata):
    """Subagent specializing in mood and environmental context analysis."""

    pass


class ScheduleManagementAgent(AgentWithFromToMetadata):
    """Subagent specializing in calendar reasoning and reminders."""

    pass


# Convenience blueprint factories
planner_agent = PlannerAgent.blueprint
wellbeing_agent = WellbeingAgent.blueprint
schedule_management_agent = ScheduleManagementAgent.blueprint


# =============================================================================
# Delegation Skills - Bridge planner to subagents
# =============================================================================


class DelegationSkills(Module):
    """Skills that let the planner consult specialist subagents.

    We need `ret=Return.call_agent` for two reasons:
    - it notifies the planner when a response arrives
    - it keeps the planner's agent loop alive (the loop terminates when no running skills have this setting)
    """

    rpc_calls = [
        "WellbeingAgent.query",
        "ScheduleManagementAgent.query",
    ]

    @skill(ret=Return.call_agent)
    def consult_wellbeing_specialist(self, situation: str) -> str:
        """Consult the wellbeing specialist for mood/environmental analysis.

        Args:
            situation: Description of the situation to analyze.

        Returns:
            The specialist's analysis.
        """
        query = self.get_rpc_calls("WellbeingAgent.query")
        # Prefix so WellbeingAgent knows this came from PlannerAgent
        prefixed = f"{FROM_PREFIX}PlannerAgent{FROM_DELIMITER}{situation}"
        return f"[Wellbeing]: {query(prefixed)}"

    @skill(ret=Return.call_agent)
    def consult_schedule_specialist(self, question: str) -> str:
        """Ask the schedule specialist about events, timing, travel, or preparation needs.

        Args:
            question: Question about schedule, timing, or preparation needs.

        Returns:
            The specialist's schedule/timing analysis.
        """
        query = self.get_rpc_calls("ScheduleManagementAgent.query")
        # Prefix so ScheduleManagementAgent knows this came from PlannerAgent
        prefixed = f"{FROM_PREFIX}PlannerAgent{FROM_DELIMITER}{question}"
        return f"[Schedule Management]: {query(prefixed)}"

    @rpc
    def set_PlannerAgent_register_skills(self, register_skills: RpcCall) -> None:
        """Auto-register skills with the PlannerAgent."""
        register_skills.set_rpc(self.rpc)
        register_skills(RPCClient(self, self.__class__))

    def __getstate__(self):
        return {}

    def __setstate__(self, state):
        pass


delegation_skills = DelegationSkills.blueprint
