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

from dataclasses import dataclass, field
import time


@dataclass(frozen=True)
class TeleopCommand:
    """Command envelope containing one typed payload or an explicit stop."""

    payload: object | None = None
    timestamp: float = field(default_factory=time.monotonic)
    stop: bool = False

    def __post_init__(self) -> None:
        if self.stop:
            if self.payload is not None:
                raise ValueError("TeleopCommand stop envelope must not contain a payload")
            return
        if self.payload is None:
            raise ValueError("TeleopCommand must contain a payload unless stop=True")
