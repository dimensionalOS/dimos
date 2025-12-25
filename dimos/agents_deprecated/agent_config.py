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


from dimos.agents_deprecated.agent import Agent


class AgentConfig:
    def __init__(self, agents: list[Agent] | None = None) -> None:
        """
        Initialize an AgentConfig with a list of agents.

        Args:
            agents (List[Agent], optional): List of Agent instances. Defaults to empty list.
        """
        self.agents = agents if agents is not None else []

    def add_agent(self, agent: Agent) -> None:
        """
        Add an agent to the configuration.

        Args:
            agent (Agent): Agent instance to add
        """
        self.agents.append(agent)

    def remove_agent(self, agent: Agent) -> None:
        """
        Remove an agent from the configuration.

        Args:
            agent (Agent): Agent instance to remove
        """
        if agent in self.agents:
            self.agents.remove(agent)

    def get_agents(self) -> list[Agent]:
        """
        Get the list of configured agents.

        Returns:
            List[Agent]: List of configured agents
        """
        return self.agents
