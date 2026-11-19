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

"""Reach a running agent's streams from outside its blueprint."""

from typing import Any

from dimos.core.transport import PubSubTransport
from dimos.porcelain.dimos import Dimos


def running_agents(app: Dimos) -> list[str]:
    """Instance names of the agents in the running system."""
    return sorted(
        {
            rpc.module_name
            for rpc in app.list_rpcs()
            if rpc.name == "agent_transports" and rpc.module_name is not None
        }
    )


def agent_transports(agent: str | None = None) -> dict[str, PubSubTransport[Any]]:
    """Started transports for an agent's streams, keyed by stream name.

    `agent` is an instance name; it may be left out when one agent is running.
    The caller stops the transports.
    """
    app = Dimos.connect()
    try:
        if agent is None:
            agents = running_agents(app)
            if len(agents) != 1:
                raise LookupError(f"Name the agent to reach; running agents: {agents}")
            agent = agents[0]
        module: Any = app.get_module(agent)
        transports: dict[str, PubSubTransport[Any]] = module.agent_transports()
    finally:
        app.stop()
    for transport in transports.values():
        transport.start()
    return transports
