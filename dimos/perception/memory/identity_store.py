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

"""Persistent identity groups across :func:`localize` calls.

Every persistence read and write in ``localize`` goes through this store and
nothing else. A store owned by a long-lived caller makes verification
evidence cumulative over everything seen since the store was created; the
query window then bounds only new detection work, and frames a label has
already ingested are never re-segmented or re-lifted.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import TYPE_CHECKING

from dimos.perception.detection.identity import Identity

if TYPE_CHECKING:
    from collections.abc import Callable

    from dimos.perception.detection.type.detection3d.pointcloud import Detection3DPC


@dataclass
class LabelIdentity:
    identity: Identity  # groups + merged, cumulative
    ingested: set[float] = field(default_factory=set)  # frame ts already segmented+lifted+added
    ungrounded: tuple[float, float] | None = None  # best (score, ts) with no depth


@dataclass
class IdentityStore:
    labels: dict[str, LabelIdentity] = field(default_factory=dict)

    def get_or_create(
        self, label: str, is_same: Callable[[Detection3DPC, Detection3DPC], bool]
    ) -> LabelIdentity:
        entry = self.labels.get(label)
        if entry is None:
            entry = LabelIdentity(identity=Identity(is_same=is_same))
            self.labels[label] = entry
        return entry
