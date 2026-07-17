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

from __future__ import annotations

from dataclasses import dataclass
import re
from typing import Any

from dimos.msgs.helpers import resolve_msg_type
from dimos.msgs.protocol import DimosMsg
from dimos.protocol.pubsub.patterns import Glob


@dataclass(slots=True)
class Topic:
    topic: str | re.Pattern[str] | Glob
    lcm_type: type[DimosMsg] | None = None
    qos: Any = None

    @property
    def is_pattern(self) -> bool:
        return isinstance(self.topic, (re.Pattern, Glob))

    @property
    def pattern(self) -> str:
        if isinstance(self.topic, str):
            return self.topic
        return self.topic.pattern

    @property
    def key_expr(self) -> str:
        suffix = "" if self.lcm_type is None else f"/{self.lcm_type.msg_name}"
        return f"{self.pattern}{suffix}"

    def __str__(self) -> str:
        suffix = "" if self.lcm_type is None else f"#{self.lcm_type.msg_name}"
        return f"{self.pattern}{suffix}"

    @classmethod
    def from_channel_str(
        cls, channel: str, default_lcm_type: type[DimosMsg] | None = None
    ) -> Topic:
        if "#" not in channel:
            return cls(channel, default_lcm_type)
        name, type_name = channel.rsplit("#", 1)
        return cls(name, resolve_msg_type(type_name) or default_lcm_type)
