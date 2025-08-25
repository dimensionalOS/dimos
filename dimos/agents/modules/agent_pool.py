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

"""Agent pool module for managing multiple agents."""

import logging
from typing import Any, Dict, List, Optional, Union

from reactivex import operators as ops
from reactivex.disposable import CompositeDisposable
from reactivex.subject import Subject

from dimos.core import Module, In, Out, rpc
from dimos.agents.modules.base_agent import BaseAgentModule
from dimos.agents.modules.unified_agent import UnifiedAgentModule
from dimos.skills.skills import SkillLibrary
from dimos.utils.logging_config import setup_logger

logger = setup_logger("dimos.agents.modules.agent_pool")


class AgentPoolModule(Module):
    """Lightweight agent pool for managing multiple agents.

    This module enables:
    - Multiple agent deployment with different configurations
    - Query routing based on agent ID or capabilities
    - Load balancing across agents
    - Response aggregation from multiple agents
    """

    # Module I/O
    query_in: In[Dict[str, Any]] = None  # {agent_id: str, query: str, ...}
    response_out: Out[Dict[str, Any]] = None  # {agent_id: str, response: str, ...}

    def __init__(self, agents_config: Dict[str, Dict[str, Any]], default_agent: str = None):
        """Initialize agent pool.

        Args:
            agents_config: Configuration for each agent
                {
                    "agent_id": {
                        "model": "openai::gpt-4o",
                        "skills": SkillLibrary(),
                        "system_prompt": "...",
                        ...
                    }
                }
            default_agent: Default agent ID to use if not specified
        """
        super().__init__()

        self._config = agents_config
        self._default_agent = default_agent or next(iter(agents_config.keys()))
        self._agents = {}
        self._disposables = CompositeDisposable()

        # Response routing
        self._response_subject = Subject()

    @rpc
    def start(self):
        """Deploy and start all agents."""
        logger.info(f"Starting agent pool with {len(self._config)} agents")

        # Deploy agents based on config
        for agent_id, config in self._config.items():
            logger.info(f"Deploying agent: {agent_id}")

            # Determine agent type
            agent_type = config.pop("type", "unified")

            if agent_type == "base":
                agent = BaseAgentModule(**config)
            else:
                agent = UnifiedAgentModule(**config)

            # Start the agent
            agent.start()

            # Store agent with metadata
            self._agents[agent_id] = {"module": agent, "config": config, "type": agent_type}

            # Subscribe to agent responses
            self._setup_agent_routing(agent_id, agent)

        # Subscribe to incoming queries
        if self.query_in:
            self._disposables.add(self.query_in.observable().subscribe(self._route_query))

        # Connect response subject to output
        if self.response_out:
            self._disposables.add(self._response_subject.subscribe(self.response_out.publish))

        logger.info("Agent pool started")

    @rpc
    def stop(self):
        """Stop all agents."""
        logger.info("Stopping agent pool")

        # Stop all agents
        for agent_id, agent_info in self._agents.items():
            try:
                agent_info["module"].stop()
            except Exception as e:
                logger.error(f"Error stopping agent {agent_id}: {e}")

        # Dispose subscriptions
        self._disposables.dispose()

        # Clear agents
        self._agents.clear()

    @rpc
    def add_agent(self, agent_id: str, config: Dict[str, Any]):
        """Add a new agent to the pool."""
        if agent_id in self._agents:
            logger.warning(f"Agent {agent_id} already exists")
            return

        # Deploy and start agent
        agent_type = config.pop("type", "unified")

        if agent_type == "base":
            agent = BaseAgentModule(**config)
        else:
            agent = UnifiedAgentModule(**config)

        agent.start()

        # Store and setup routing
        self._agents[agent_id] = {"module": agent, "config": config, "type": agent_type}
        self._setup_agent_routing(agent_id, agent)

        logger.info(f"Added agent: {agent_id}")

    @rpc
    def remove_agent(self, agent_id: str):
        """Remove an agent from the pool."""
        if agent_id not in self._agents:
            logger.warning(f"Agent {agent_id} not found")
            return

        # Stop and remove agent
        agent_info = self._agents[agent_id]
        agent_info["module"].stop()
        del self._agents[agent_id]

        logger.info(f"Removed agent: {agent_id}")

    @rpc
    def list_agents(self) -> List[Dict[str, Any]]:
        """List all agents and their configurations."""
        return [
            {"id": agent_id, "type": info["type"], "model": info["config"].get("model", "unknown")}
            for agent_id, info in self._agents.items()
        ]

    @rpc
    def broadcast_query(self, query: str, exclude: List[str] = None):
        """Send query to all agents (except excluded ones)."""
        exclude = exclude or []

        for agent_id, agent_info in self._agents.items():
            if agent_id not in exclude:
                agent_info["module"].query_in.publish(query)

        logger.info(f"Broadcasted query to {len(self._agents) - len(exclude)} agents")

    def _setup_agent_routing(
        self, agent_id: str, agent: Union[BaseAgentModule, UnifiedAgentModule]
    ):
        """Setup response routing for an agent."""

        # Subscribe to agent responses and tag with agent_id
        def tag_response(response: str) -> Dict[str, Any]:
            return {
                "agent_id": agent_id,
                "response": response,
                "type": self._agents[agent_id]["type"],
            }

        self._disposables.add(
            agent.response_out.observable()
            .pipe(ops.map(tag_response))
            .subscribe(self._response_subject.on_next)
        )

    def _route_query(self, msg: Dict[str, Any]):
        """Route incoming query to appropriate agent(s)."""
        # Extract routing info
        agent_id = msg.get("agent_id", self._default_agent)
        query = msg.get("query", "")
        broadcast = msg.get("broadcast", False)

        if broadcast:
            # Send to all agents
            exclude = msg.get("exclude", [])
            self.broadcast_query(query, exclude)
        elif agent_id == "round_robin":
            # Simple round-robin routing
            agent_ids = list(self._agents.keys())
            if agent_ids:
                # Use query hash for consistent routing
                idx = hash(query) % len(agent_ids)
                selected_agent = agent_ids[idx]
                self._agents[selected_agent]["module"].query_in.publish(query)
                logger.debug(f"Routed to {selected_agent} (round-robin)")
        elif agent_id in self._agents:
            # Route to specific agent
            self._agents[agent_id]["module"].query_in.publish(query)
            logger.debug(f"Routed to {agent_id}")
        else:
            logger.warning(f"Unknown agent ID: {agent_id}, using default: {self._default_agent}")
            if self._default_agent in self._agents:
                self._agents[self._default_agent]["module"].query_in.publish(query)

        # Handle additional routing options
        if "image" in msg and hasattr(self._agents.get(agent_id, {}).get("module"), "image_in"):
            self._agents[agent_id]["module"].image_in.publish(msg["image"])

        if "data" in msg and hasattr(self._agents.get(agent_id, {}).get("module"), "data_in"):
            self._agents[agent_id]["module"].data_in.publish(msg["data"])
