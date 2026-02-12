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

from __future__ import annotations

import re
from typing import TypeVar

TopicT = TypeVar("TopicT")
MsgT = TypeVar("MsgT")


class Glob:
    """Glob pattern that compiles to regex

    Supports:
        * - matches any characters except /
        ** - matches any characters including /
        ? - matches single character

    Example:
        Topic(topic=Glob("/sensor/*"))  # matches /sensor/temp, /sensor/humidity
        Topic(topic=Glob("/robot/**"))  # matches /robot/arm/joint1, /robot/leg/motor
    """

    def __init__(self, pattern: str) -> None:
        self._glob = pattern
        self._regex = self._compile(pattern)

    @staticmethod
    def _compile(pattern: str) -> str:
        """Convert glob pattern to regex."""
        result = []
        i = 0
        while i < len(pattern):
            c = pattern[i]
            if c == "*":
                if i + 1 < len(pattern) and pattern[i + 1] == "*":
                    result.append(".*")
                    i += 2
                else:
                    result.append("[^/]*")
                    i += 1
            elif c == "?":
                result.append(".")
                i += 1
            elif c in r"\^$.|+[]{}()":
                result.append("\\" + c)
                i += 1
            else:
                result.append(c)
                i += 1
        return "".join(result)

    @property
    def pattern(self) -> str:
        """Return the regex pattern string."""
        return self._regex

    @property
    def glob(self) -> str:
        """Return the original glob pattern."""
        return self._glob

    def __repr__(self) -> str:
        return f"Glob({self._glob!r})"


Pattern = str | re.Pattern[str] | Glob


def pattern_matches(pattern: Pattern, topic_str: str) -> bool:
    """Check if a topic string matches a pattern.

    Args:
        pattern: A string (exact match), compiled regex, or Glob pattern.
        topic_str: The topic string to match against.

    Returns:
        True if the topic matches the pattern.
    """
    if isinstance(pattern, str):
        return pattern == topic_str
    elif isinstance(pattern, Glob):
        return bool(re.fullmatch(pattern.pattern, topic_str))
    else:
        return bool(pattern.fullmatch(topic_str))
