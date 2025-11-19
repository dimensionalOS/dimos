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

"""DimOS skill tutorial: agent-enabled greeter.

This file extends the Greeter from skill_basics with LLM agent auto-registration.
"""

from dimos.core.core import rpc
from dimos.core.rpc_client import RpcCall, RPCClient
from docs.tutorials.skill_basics.greeter import Greeter, RobotCapabilities

__all__ = ["Greeter", "GreeterForAgents", "RobotCapabilities"]


# --8<-- [start:GreeterForAgents]
class GreeterForAgents(Greeter):
    """Greeter with automatic LLM agent registration.

    Extends Greeter to enable skill auto-discovery by agents.
    When composed with llm_agent() via autoconnect(), the framework calls
    set_LlmAgent_register_skills to register this module's skills.
    """

    @rpc
    def set_LlmAgent_register_skills(self, register_skills: RpcCall) -> None:
        """Called by framework when composing with llm_agent().

        This method is discovered by convention during blueprint.build().
        It receives a callback to register this module's skills with the agent.

        Args:
            register_skills: Callback to register this module's skills with the agent.
        """
        register_skills.set_rpc(self.rpc)
        register_skills(RPCClient(self, self.__class__))


# --8<-- [end:GreeterForAgents]
