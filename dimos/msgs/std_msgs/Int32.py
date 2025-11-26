#!/usr/bin/env python3
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

# Copyright 2025 Dimensional Inc.

"""Int32 message type."""

from typing import ClassVar

from dimos_lcm.std_msgs import Int32 as LCMInt32  # type: ignore[import-untyped]


class Int32(LCMInt32):  # type: ignore[misc]
    """ROS-compatible Int32 message."""

    msg_name: ClassVar[str] = "std_msgs.Int32"

    def __init__(self, data: int = 0) -> None:
        """Initialize Int32 with data value."""
        self.data = data
