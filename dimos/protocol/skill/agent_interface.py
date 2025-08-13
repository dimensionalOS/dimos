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

from copy import copy
from dataclasses import dataclass
from enum import Enum
from pprint import pformat
from typing import Any, Callable, Optional

from dimos.protocol.skill.comms import AgentMsg, LCMSkillComms, MsgType, SkillCommsSpec
from dimos.protocol.skill.skill import SkillConfig, SkillContainer
from dimos.protocol.skill.types import Reducer, Return, Stream
from dimos.types.timestamped import TimestampedCollection
from dimos.utils.logging_config import setup_logger

logger = setup_logger("dimos.protocol.skill.agent_interface")


@dataclass
class AgentInputConfig:
    agent_comms: type[SkillCommsSpec] = LCMSkillComms


class SkillStateEnum(Enum):
    pending = 0
    running = 1
    returned = 2
    error = 3


# TODO pending timeout, running timeout, etc.
class SkillState(TimestampedCollection):
    name: str
    state: SkillStateEnum
    skill_config: SkillConfig

    def __init__(self, name: str, skill_config: Optional[SkillConfig] = None) -> None:
        super().__init__()
        self.skill_config = skill_config or SkillConfig(
            name=name, stream=Stream.none, ret=Return.none, reducer=Reducer.none
        )

        self.state = SkillStateEnum.pending
        self.name = name

    # returns True if the agent should be called for this message
    def handle_msg(self, msg: AgentMsg) -> bool:
        self.add(msg)

        if msg.type == MsgType.stream:
            if (
                self.skill_config.stream == Stream.none
                or self.skill_config.stream == Stream.passive
            ):
                return False
            if self.skill_config.stream == Stream.call_agent:
                return True

        if msg.type == MsgType.ret:
            self.state = SkillStateEnum.returned
            if self.skill_config.ret == Return.call_agent:
                return True
            return False

        if msg.type == MsgType.error:
            self.state = SkillStateEnum.error
            return True

        if msg.type == MsgType.start:
            self.state = SkillStateEnum.running
            return False

        return False

    def __str__(self) -> str:
        head = f"SkillState(state={self.state}"

        if self.state == SkillStateEnum.returned or self.state == SkillStateEnum.error:
            head += ", ran for="
        else:
            head += ", running for="

        head += f"{self.duration():.2f}s"

        if len(self):
            return head + f", messages={list(self._items)})"
        return head + ", No Messages)"


class AgentInterface(SkillContainer):
    _static_containers: list[SkillContainer]
    _dynamic_containers: list[SkillContainer]
    _skill_state: dict[str, SkillState]
    _skills: dict[str, SkillConfig]
    _agent_callback: Optional[Callable[[dict[str, SkillState]], Any]] = None

    # Agent callback is called with a state snapshot once system decides
    # that agents needs to be woken up, according to inputs from active skills
    def __init__(
        self, agent_callback: Optional[Callable[[dict[str, SkillState]], Any]] = None
    ) -> None:
        super().__init__()
        self._agent_callback = agent_callback
        self._static_containers = []
        self._dynamic_containers = []
        self._skills = {}
        self._skill_state = {}

    def start(self) -> None:
        self.agent_comms.start()
        self.agent_comms.subscribe(self.handle_message)

    def stop(self) -> None:
        self.agent_comms.stop()

    # This is used by agent to call skills
    def execute_skill(self, skill_name: str, *args, **kwargs) -> None:
        skill_config = self.get_skill_config(skill_name)
        if not skill_config:
            logger.error(
                f"Skill {skill_name} not found in registered skills, but agent tried to call it (did a dynamic skill expire?)"
            )
            return

        # This initializes the skill state if it doesn't exist
        self._skill_state[skill_name] = SkillState(name=skill_name, skill_config=skill_config)
        return skill_config.call(*args, **kwargs)

    # Receives a message from active skill
    # Updates local skill state (appends to streamed data if needed etc)
    #
    # Checks if agent needs to be called (if ToolConfig has Return=call_agent or Stream=call_agent)
    def handle_message(self, msg: AgentMsg) -> None:
        logger.info(f"{msg.skill_name} - {msg}")

        if self._skill_state.get(msg.skill_name) is None:
            logger.warn(
                f"Skill state for {msg.skill_name} not found, (skill not called by our agent?) initializing. (message received: {msg})"
            )
            self._skill_state[msg.skill_name] = SkillState(name=msg.skill_name)

        should_call_agent = self._skill_state[msg.skill_name].handle_msg(msg)
        if should_call_agent:
            self.call_agent()

    # Returns a snapshot of the current state of skill runs.
    #
    # If clear=True, it will assume the snapshot is being sent to an agent
    # and will clear the finished skill runs from the state
    def state_snapshot(self, clear: bool = True) -> dict[str, SkillState]:
        if not clear:
            return self._skill_state

        ret = copy(self._skill_state)

        to_delete = []
        # Since state is exported, we can clear the finished skill runs
        for skill_name, skill_run in self._skill_state.items():
            if skill_run.state == SkillStateEnum.returned:
                logger.info(f"Skill {skill_name} finished")
                to_delete.append(skill_name)
            if skill_run.state == SkillStateEnum.error:
                logger.error(f"Skill run error for {skill_name}")
                to_delete.append(skill_name)

        for skill_name in to_delete:
            logger.debug(f"{skill_name} finished, removing from state")
            del self._skill_state[skill_name]

        return ret

    def call_agent(self) -> None:
        """Call the agent with the current state of skill runs."""
        logger.info(f"Calling agent with current skill state: {self.state_snapshot(clear=False)}")

        state = self.state_snapshot(clear=True)

        if self._agent_callback:
            self._agent_callback(state)

    def __str__(self):
        # Convert objects to their string representations
        def stringify_value(obj):
            if isinstance(obj, dict):
                return {k: stringify_value(v) for k, v in obj.items()}
            elif isinstance(obj, list):
                return [stringify_value(item) for item in obj]
            else:
                return str(obj)

        ret = stringify_value(self._skill_state)

        return f"AgentInput({pformat(ret, indent=2, depth=3, width=120, compact=True)})"

    # Given skillcontainers can run remotely, we are
    # Caching available skills from static containers
    #
    # Dynamic containers will be queried at runtime via
    # .skills() method
    def register_skills(self, container: SkillContainer):
        if not container.dynamic_skills:
            logger.info(f"Registering static skill container, {container}")
            self._static_containers.append(container)
            for name, skill_config in container.skills().items():
                self._skills[name] = skill_config.bind(getattr(container, name))
        else:
            logger.info(f"Registering dynamic skill container, {container}")
            self._dynamic_containers.append(container)

    def get_skill_config(self, skill_name: str) -> Optional[SkillConfig]:
        skill_config = self._skills.get(skill_name)
        if not skill_config:
            skill_config = self.skills().get(skill_name)
        return skill_config

    def skills(self) -> dict[str, SkillConfig]:
        # Static container skilling is already cached
        all_skills: dict[str, SkillConfig] = {**self._skills}

        # Then aggregate skills from dynamic containers
        for container in self._dynamic_containers:
            for skill_name, skill_config in container.skills().items():
                all_skills[skill_name] = skill_config.bind(getattr(container, skill_name))

        return all_skills
