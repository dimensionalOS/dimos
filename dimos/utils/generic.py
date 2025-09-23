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

import os
from typing import Any, Optional


def truncate_display_string(arg: Any, max: Optional[int] = None) -> str:
    """
    If we print strings that are too long that potentially obscures more important logs.

    Use this function to truncate it to a reasonable length (configurable from the env).
    """
    string = str(arg)

    if max is not None:
        max_chars = max
    else:
        max_chars = int(os.getenv("TRUNCATE_MAX", "2000"))

    if max_chars == 0 or len(string) <= max_chars:
        return string

    return string[:max_chars] + "...(truncated)..."
