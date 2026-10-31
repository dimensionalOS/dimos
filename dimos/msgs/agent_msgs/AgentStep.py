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

"""One entry of an agent conversation, republished so a recording holds what the agent did."""

from __future__ import annotations

import json

from dimos_lcm.std_msgs import String as LCMString

from dimos.types.timestamped import Timestamped


class AgentStep(Timestamped):
    msg_name = "agent_msgs.AgentStep"

    def __init__(self, conversation: int, entry: str, kind: str, text: str, ts: float) -> None:
        super().__init__(ts)
        self.conversation = conversation
        """The conversation id supplied by the client."""
        self.entry = entry
        """The entry's id in the conversation store."""
        self.kind = kind
        """The entry's kind in the conversation store, e.g. a user input or a tool call."""
        self.text = text

    def lcm_encode(self) -> bytes:
        payload = {
            "conversation": self.conversation,
            "entry": self.entry,
            "kind": self.kind,
            "text": self.text,
            "ts": self.ts,
        }
        return LCMString(data=json.dumps(payload)).lcm_encode()  # type: ignore[no-any-return]

    @classmethod
    def lcm_decode(cls, data: bytes) -> AgentStep:
        payload = json.loads(LCMString.lcm_decode(data).data)
        return cls(
            payload["conversation"],
            payload["entry"],
            payload["kind"],
            payload["text"],
            payload["ts"],
        )
