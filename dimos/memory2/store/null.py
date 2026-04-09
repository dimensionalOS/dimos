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

from typing import Any

from dimos.memory2.store.memory import MemoryStore


class NullStore(MemoryStore):
    """Live-only store — O(1) memory, no history/replay.

    Shorthand for ``MemoryStore(max_size=0)``.
    Observations get IDs (for live dedup) but are immediately discarded.
    """

    def __init__(self, **kwargs: Any) -> None:
        kwargs.setdefault("max_size", 0)
        super().__init__(**kwargs)
