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

import time
from typing import Generator, Optional

from dimos.protocol.skill.skill import SkillContainer, skill
from dimos.protocol.skill.type import Reducer, Return, Stream


class TestContainer(SkillContainer):
    @skill()
    def add(self, x: int, y: int) -> int:
        return x + y

    @skill()
    def delayadd(self, x: int, y: int) -> int:
        time.sleep(0.3)
        return x + y

    @skill(stream=Stream.call_agent)
    def counter(self, count_to: int, delay: Optional[float] = 0.1) -> Generator[int, None, None]:
        """Counts from 1 to count_to, with an optional delay between counts."""
        for i in range(1, count_to + 1):
            if delay > 0:
                time.sleep(delay)
            yield i
