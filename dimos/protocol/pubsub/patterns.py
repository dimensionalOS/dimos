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

import re
from typing import TypeAlias


class Glob:
    def __init__(self, glob: str) -> None:
        self.glob = glob
        self.pattern = (
            re.escape(glob).replace(r"\*\*", ".*").replace(r"\*", "[^/]*").replace(r"\?", ".")
        )

    def __repr__(self) -> str:
        return f"Glob({self.glob!r})"


Pattern: TypeAlias = str | re.Pattern[str] | Glob


def pattern_matches(pattern: Pattern, value: str) -> bool:
    if isinstance(pattern, str):
        return pattern == value
    if isinstance(pattern, re.Pattern):
        return pattern.fullmatch(value) is not None
    return re.fullmatch(pattern.pattern, value) is not None
