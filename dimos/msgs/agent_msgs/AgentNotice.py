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

"""A notice from an agent module, carried as JSON in an LCM String."""

from __future__ import annotations

import json
import time

from dimos_lcm.std_msgs import String as LCMString

from dimos.types.timestamped import Timestamped


class AgentNotice(Timestamped):
    msg_name = "agent_msgs.AgentNotice"

    def __init__(
        self,
        module: str,
        text: str,
        data: str = "{}",
        exports: str = "[]",
        suppressed: int = 0,
        ts: float | None = None,
    ) -> None:
        super().__init__(time.time() if ts is None else ts)
        self.module = module
        """The agent module's name, as the harness saved it."""
        self.text = text
        self.data = data
        """The notify() keyword arguments as a JSON object, exported values replaced by paths."""
        self.exports = exports
        """The files that notify() exported, as a JSON list of export records."""
        self.suppressed = suppressed
        """How many notices the module's rate limit dropped since the previous one."""

    def lcm_encode(self) -> bytes:
        payload = {
            "module": self.module,
            "text": self.text,
            "data": self.data,
            "exports": self.exports,
            "suppressed": self.suppressed,
            "ts": self.ts,
        }
        return LCMString(data=json.dumps(payload)).lcm_encode()  # type: ignore[no-any-return]

    @classmethod
    def lcm_decode(cls, data: bytes) -> AgentNotice:
        payload = json.loads(LCMString.lcm_decode(data).data)
        return cls(
            payload["module"],
            payload["text"],
            payload["data"],
            payload["exports"],
            payload["suppressed"],
            payload["ts"],
        )
