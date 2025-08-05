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

from dataclasses import dataclass

from dimos.protocol.service import Service
from dimos.protocol.tool.comms import AgentMsg, LCMToolComms, ToolCommsSpec


@dataclass
class AgentInputConfig:
    comms: ToolCommsSpec = LCMToolComms


class AgentInput(Service[AgentInputConfig]):
    default_config: type[AgentInputConfig] = AgentInputConfig

    def __init__(self, **kwargs) -> None:
        super().__init__(**kwargs)
        self.comms = self.config.comms()

    def start(self) -> None:
        self.comms.start()
        self.comms.subscribe(self.handle_message)

    def stop(self) -> None:
        self.comms.stop()

    def handle_message(self, msg: AgentMsg) -> None:
        print(f"Received message: {msg}")
