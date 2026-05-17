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

from typing import Any, Protocol

from dimos.spec.utils import Spec


class TemporalMemorySpec(Spec, Protocol):
    def query(self, question: str) -> str: ...
    def get_state(self) -> dict[str, Any]: ...
    def get_entity_roster(self) -> list[dict[str, Any]]: ...
    def get_rolling_summary(self) -> str: ...
    def get_graph_db_stats(self) -> dict[str, Any]: ...


__all__ = ["TemporalMemorySpec"]
