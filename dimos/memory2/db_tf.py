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

"""
The read-side transform-lookup interface for memory2, plus the graph-stream payload.

Kept dependency-light (no sqlite/numpy) so importing :class:`DbTf` or :class:`TfGraph`
costs nothing. The implementations live in sibling modules:
  * :class:`dimos.memory2.db_tf_live.DbTfLive` — in-RAM buffer; default for any Store.
  * :class:`dimos.memory2.db_tf_sql.DbTfSql` — sqlite graph-stream; sqlite store only.
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Any, Protocol

if TYPE_CHECKING:
    from dimos.msgs.geometry_msgs.Transform import Transform


class DbTf(Protocol):
    """Transform-lookup interface over a recording's transforms. Implemented by
    :class:`DbTfLive` (default) and :class:`DbTfSql` (sqlite)."""

    def get(
        self,
        target_frame: str,
        source_frame: str,
        time_point: float | None = None,
        time_tolerance: float | None = None,
    ) -> Transform | None: ...

    def has_transforms(self) -> bool: ...


class TfGraph:
    """A tf topology snapshot, recorded one per structure change.

    ``structure`` maps each child frame to ``{"parent": str, "static": bool}`` —
    the full tf tree as of this message's timestamp. The stream of these snapshots
    (the ``tf_graph`` stream) is the topology change-log that transform lookups
    walk to resolve a source->target chain at any past time. Defined here (not under
    ``dimos/msgs``) because it is a recording-internal payload, not a wire message;
    it is stored via the pickle codec."""

    structure: dict[str, dict[str, Any]]
    msg_name = "tf2_msgs.TfGraph"

    def __init__(self, structure: dict[str, dict[str, Any]]) -> None:
        # copy so later mutations of the writer's running structure don't alter an
        # already-recorded snapshot
        self.structure = {child: dict(entry) for child, entry in structure.items()}

    def __repr__(self) -> str:
        return f"TfGraph({len(self.structure)} frames)"
