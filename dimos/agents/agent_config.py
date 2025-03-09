from typing import List
from dimos.agents.agent import Agent

class AgentConfig:
    def __init__(self, agents: List[Agent] = None):
        """
        Initialize an AgentConfig with a list of agents.

        Args:
            agents (List[Agent], optional): List of Agent instances. Defaults to empty list.
        """
        self.agents = agents if agents is not None else []

    def add_agent(self, agent: Agent):
        """
        Add an agent to the configuration.

        Args:
            agent (Agent): Agent instance to add
        """
        self.agents.append(agent)

    def remove_agent(self, agent: Agent):
        """
        Remove an agent from the configuration.

        Args:
            agent (Agent): Agent instance to remove
        """
        if agent in self.agents:
            self.agents.remove(agent)

    def get_agents(self) -> List[Agent]:
        """
        Get the list of configured agents.

        Returns:
            List[Agent]: List of configured agents
        """
        return self.agents
