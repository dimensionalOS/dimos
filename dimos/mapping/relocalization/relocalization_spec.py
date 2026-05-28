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

from typing import Protocol

from dimos.spec.utils import Spec


# Module-ref spec for runtime control of relocalization. Resolves (structurally)
# to RelocalizationModule, letting another module (e.g. RoomManager) load a named
# premap, score candidate maps, and query the current map without owning the
# mapping pipeline.
class RelocalizationSpec(Spec, Protocol):
    def load_map(self, name: str) -> bool: ...

    def score_map(self, name: str) -> float: ...

    def current_map(self) -> str | None: ...
