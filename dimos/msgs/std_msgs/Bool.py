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

"""Bool message type."""

from dimos_lcm.std_msgs import Bool as LCMBool


class Bool(LCMBool):  # type: ignore[misc]
    """Bool message."""

    msg_name = "std_msgs.Bool"

    def __init__(self, data: bool = False) -> None:
        """Initialize Bool with data value."""
        self.data = data
