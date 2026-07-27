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

"""Read side of relocalization: is the ``world -> map`` estimate trustworthy?

This cannot be answered from tf. ``RelocalizationModule`` republishes its last
good transform on a timer, restamped to now, so a registration that has been
failing for minutes still presents a perfectly fresh-looking tf edge. Anything
that anchors or resolves against ``map`` has to ask here instead.
"""

from __future__ import annotations

from typing import Protocol

from dimos.memory2.locations import RelocStatus
from dimos.spec.utils import Spec


class RelocalizationSpec(Spec, Protocol):
    def reloc_status(self) -> RelocStatus: ...
