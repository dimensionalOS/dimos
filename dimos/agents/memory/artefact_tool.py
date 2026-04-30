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

"""The ``get_artefact`` tool — the LLM's way to rehydrate evidence pages.

When an EVIDENCE page has been degraded to STRUCTURED or POINTER, the LLM
sees one of two bracket forms in its context: ``[artefact uuid=<uuid>]``
(POINTER) or ``[image artefact uuid=<uuid> src=... dims=... size=...]``
(STRUCTURED). Both forms carry the page's ``artefact_uuid``, not its
internal ``page.id``. If the LLM needs the full image, it calls this
tool with that UUID; the next assembled prompt will then include the
page at FULL fidelity.

The tool is self-describing: its ``description`` and ``args_schema`` are
the *only* way the LLM discovers it. The plan explicitly keeps the
system prompt unchanged.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

from pydantic import BaseModel, Field

if TYPE_CHECKING:
    from langchain_core.tools import StructuredTool

    from dimos.agents.memory.engine import MemoryEngine


__all__ = [
    "GET_ARTEFACT_TOOL_NAME",
    "build_get_artefact_tool",
]


GET_ARTEFACT_TOOL_NAME = "get_artefact"
"""Canonical name for the tool; used by tests and by the engine."""

_TOOL_DESCRIPTION = (
    "Rehydrate a degraded artefact back to full fidelity. Call this tool "
    "whenever you see one of these references in your context and need "
    "the original image, sensor snapshot or tool output:\n"
    "  - `[artefact uuid=<UUID>]` (POINTER fidelity)\n"
    "  - `[image artefact uuid=<UUID> src=... dims=... size=...]` "
    "(STRUCTURED fidelity)\n"
    "The next assistant turn will include the full artefact content. "
    "Pass the exact UUID from the `uuid=` field of the reference (the "
    "value immediately following `uuid=` and before the next space or "
    "closing bracket)."
)


class _GetArtefactArgs(BaseModel):
    """Arguments for ``get_artefact``."""

    uuid: str = Field(
        description="The artefact UUID extracted from the degraded reference "
        "(e.g. the part after 'uuid=' in '[artefact uuid=abc-123]')."
    )


def build_get_artefact_tool(engine: "MemoryEngine") -> "StructuredTool":
    """Build a :class:`StructuredTool` bound to *engine*.

    The returned tool is side-effectful: calling it mutates the engine's
    page table so the next :meth:`MemoryEngine.assemble` call renders the
    requested page at FULL. It returns a short acknowledgement string
    (never the full artefact body — that arrives via the next prompt).
    """
    # Deferred import: ``langchain_core`` is an optional extra, and the
    # memory package imports cleanly even in environments that have not
    # installed it. Importing at module scope would break those users.
    from langchain_core.tools import StructuredTool

    def _invoke(uuid: str) -> str:
        ok = engine.request_full(uuid)
        if not ok:
            # A page evicted from an *assembled prompt* still lives in
            # the ``PageTable`` and remains resolvable via
            # ``get_by_artefact``. The only way a UUID lookup truly
            # misses is if the page was never ingested, the UUID string
            # is malformed, or the engine was ``clear()``ed (e.g.
            # between sessions).
            return (
                f"No artefact found with UUID '{uuid}'. The UUID may be "
                "malformed, or the page table may have been cleared (e.g. "
                "after a new session). Double-check the `uuid=` value from "
                "the artefact reference and try again."
            )
        return (
            f"Artefact '{uuid}' scheduled for full-fidelity rehydration on the "
            "next turn. Continue your reasoning and reference the artefact on "
            "the next message."
        )

    return StructuredTool.from_function(
        func=_invoke,
        name=GET_ARTEFACT_TOOL_NAME,
        description=_TOOL_DESCRIPTION,
        args_schema=_GetArtefactArgs,
    )
