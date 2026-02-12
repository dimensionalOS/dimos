#!/usr/bin/env python3
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

# Copyright 2025-2026 Dimensional Inc.

"""Int32 message type."""

from typing import ClassVar

from dimos_lcm.std_msgs import Int8 as LCMInt8


class Int8(LCMInt8):  # type: ignore[misc]
    """Int8 message."""

    msg_name: ClassVar[str] = "std_msgs.Int8"

    def __init__(self, data: int = 0) -> None:
        """Initialize Int8 with data value."""
        self.data = data
