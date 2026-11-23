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

"""Short-term object IDs: OmDet boxes, BoT-SORT, optional crop embedding ReID."""

from dimos.perception.identity.dump import (
    TRACKS_FILE,
    dump_track_jsonl,
    sidecar_tracks_jsonl,
)
from dimos.perception.identity.mot import BotSortMot, assign_track_ids
from dimos.perception.identity.reid import AppearanceReId
from dimos.perception.identity.tracker import (
    DEFAULT_QUERIES,
    IdentityTracker,
    build_identity_tracker,
)

__all__ = [
    "DEFAULT_QUERIES",
    "TRACKS_FILE",
    "AppearanceReId",
    "BotSortMot",
    "IdentityTracker",
    "assign_track_ids",
    "build_identity_tracker",
    "dump_track_jsonl",
    "sidecar_tracks_jsonl",
]
