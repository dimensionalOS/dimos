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

from dimos.core.module import Module
from dimos.core.rpc_client import RpcCall, RPCClient
from dimos.protocol.skill.skill import rpc


class SkillModule(Module):
    """Use this module if you want to auto-register skills to an LlmAgent."""

    @rpc
    def set_LlmAgent_register_skills(self, callable: RpcCall) -> None:
        callable.set_rpc(self.rpc)  # type: ignore[arg-type]
        callable(RPCClient(self, self.__class__))

    def __getstate__(self) -> None:
        pass

    def __setstate__(self, _state) -> None:  # type: ignore[no-untyped-def]
        pass
