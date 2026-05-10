# Copyright 2025-2026 Dimensional Inc.
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

"""Spec for G1 high-level control: WebRTC, native SDK, etc. all implement this protocol."""

from typing import Any, Protocol

from dimos.core.stream import In
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.spec.utils import Spec


class HighLevelG1Spec(Spec, Protocol):
    """Common high-level control interface for the Unitree G1."""

    cmd_vel: In[Twist]

    def move(self, twist: Twist, duration: float = 0.0) -> bool: ...

    def get_state(self) -> str: ...

    def publish_request(self, topic: str, data: dict[str, Any]) -> dict[str, Any]: ...

    def stand_up(self) -> bool: ...

    def lie_down(self) -> bool: ...


__all__ = ["HighLevelG1Spec"]
