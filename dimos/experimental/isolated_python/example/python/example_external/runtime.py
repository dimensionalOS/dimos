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

from typing import Any

from dimos.agents.annotation import skill
from dimos.core.core import rpc
from dimos.experimental.isolated_python.example.contract import ExampleExternal
from dimos.msgs.std_msgs.Int32 import Int32


class ExampleExternalRuntime(ExampleExternal):
    """Concrete implementation loaded in the sibling environment."""

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._multiplier = self.config.initial_multiplier

    @rpc
    def start(self) -> None:
        super().start()
        self.value.subscribe(self._publish_doubled)

    def _publish_doubled(self, message: Int32) -> None:
        self.doubled.publish(Int32(message.data * self._multiplier))

    @rpc
    def get_multiplier(self) -> int:
        return self._multiplier

    @rpc
    def get_adjusted_multiplier(self) -> int:
        return self._multiplier + self._offset.get_offset()

    @skill
    def set_multiplier(self, multiplier: int) -> str:
        self._multiplier = multiplier
        return f"External multiplier set to {multiplier}"
