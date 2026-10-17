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

"""Which scene geometry a simulated robot touches, by robot part and geometry kind."""

from __future__ import annotations

from dataclasses import dataclass
import json
import time
from typing import Literal

from dimos_lcm.std_msgs import String as LCMString

from dimos.types.timestamped import Timestamped

Part = Literal["trunk", "lidar", "leg", "foot"]
Kind = Literal["floor", "wall", "ceiling", "clutter"]


@dataclass(frozen=True, order=True)
class Contact:
    part: Part
    kind: Kind


class Contacts(Timestamped):
    """The set of contacts at one instant, published whenever it changes."""

    msg_name = "sim_msgs.Contacts"

    def __init__(self, contacts: list[Contact] | None = None, ts: float | None = None) -> None:
        self.contacts = contacts or []
        self.ts = time.time() if ts is None else ts

    def lcm_encode(self) -> bytes:
        payload = {"ts": self.ts, "contacts": [[c.part, c.kind] for c in self.contacts]}
        return LCMString(data=json.dumps(payload)).lcm_encode()  # type: ignore[no-any-return]

    @classmethod
    def lcm_decode(cls, data: bytes) -> Contacts:
        payload = json.loads(LCMString.lcm_decode(data).data)
        return cls([Contact(part, kind) for part, kind in payload["contacts"]], payload["ts"])
