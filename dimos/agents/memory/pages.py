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

"""Core data types for the paged, multi-fidelity memory layer.

Each unit of conversation state is a :class:`Page`. A page stores all four
fidelity levels eagerly so the selector can pick any one on demand without
re-running the ingestor (which may involve image resizing or tokenization).

Invariants enforced by :meth:`Page.__post_init__`:

* ``representations`` contains at least ``POINTER``, ``STRUCTURED`` and one of
  ``COMPRESSED``/``FULL``.
* ``min_fidelity >= POINTER``.
* ``pinned_at_full`` implies both ``pinned`` and ``min_fidelity == FULL``.
* All representations are non-empty and their token estimates are
  monotonically non-decreasing in fidelity level.

Implementation notes:

* ``Page.ai_tool_calls: list[dict] | None`` — stores the OpenAI
  tool-call payload so ``Page.as_message`` can reattach it at every
  fidelity level, preserving LangChain ``AIMessage``/``ToolMessage``
  pairing across selector downgrades.
* ``Page.rep_at(level)`` has a third fallback rung ("fall up" to the
  lowest available representation when no rung ``<= level`` exists).
  Documented in ``rep_at``'s own docstring as defensive; unreachable in
  practice given the POINTER invariant but kept for robustness.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum, IntEnum
from typing import Any, Literal

from langchain_core.messages import AIMessage, HumanMessage, SystemMessage, ToolMessage
from langchain_core.messages.base import BaseMessage

__all__ = [
    "FidelityLevel",
    "MessageRole",
    "Page",
    "PageType",
    "Representation",
]

MessageRole = Literal["system", "human", "ai", "tool"]

# Runtime-checkable counterpart of the :data:`MessageRole` ``Literal`` alias.
# ``Literal`` is only meaningful to static type checkers, so we keep a
# frozenset around for ``Page.__post_init__`` to enforce at runtime.
_VALID_ROLES: frozenset[str] = frozenset({"system", "human", "ai", "tool"})


class FidelityLevel(IntEnum):
    """Fidelity ladder for a page.

    ``POINTER < STRUCTURED < COMPRESSED < FULL`` — higher is more expensive
    to include in a prompt but also more informative. Integer values are
    chosen so ordinary numeric comparisons work (``level >= min_fidelity``).
    """

    POINTER = 0
    STRUCTURED = 1
    COMPRESSED = 2
    FULL = 3


class PageType(Enum):
    """Why this page exists in the agent's memory."""

    BOOTSTRAP = "bootstrap"
    """System prompts, tool schemas — always pinned at FULL."""

    CONSTRAINT = "constraint"
    """Hard rules or invariants the agent must obey."""

    PLAN = "plan"
    """Current plan or open goals."""

    PREFERENCE = "preference"
    """User preferences carried across turns."""

    EVIDENCE = "evidence"
    """Images, sensor snapshots, or tool outputs with artefacts."""

    CONVERSATION = "conversation"
    """Plain user/AI/tool message turns."""


@dataclass
class Representation:
    """A single fidelity rendering of a page.

    Attributes:
        level: Which rung of the fidelity ladder this representation sits on.
        content: Either a plain string (text rendering) or a list of
            LangChain multimodal content parts
            (``[{"type": "text", ...}, {"type": "image_url", ...}]``).
        token_estimate: Cached token count for this representation; set by
            the ingestor so the selector never has to re-tokenize.
    """

    level: FidelityLevel
    content: str | list[dict[str, Any]]
    token_estimate: int

    def __post_init__(self) -> None:
        if not isinstance(self.level, FidelityLevel):
            raise TypeError(
                "Representation.level must be a FidelityLevel, got "
                f"{type(self.level).__name__}"
            )
        if self.token_estimate < 0:
            raise ValueError(
                f"Representation.token_estimate must be non-negative, got {self.token_estimate}"
            )
        if isinstance(self.content, str):
            if not self.content:
                raise ValueError("Representation text content must be non-empty")
        elif isinstance(self.content, list):
            if not self.content:
                raise ValueError("Representation multimodal content must be non-empty")
        else:
            raise TypeError(
                f"Representation.content must be str or list[dict], got {type(self.content).__name__}"
            )


@dataclass
class Page:
    """One addressable unit of conversation state.

    A page owns four representations (ingested eagerly). The selector picks
    the lowest representation that still fits the token budget after
    satisfying pins and minimum-fidelity invariants.

    Attributes:
        id: Stable internal identifier, unique per page within a session.
        type: What kind of state this page carries (see :class:`PageType`).
        provenance: Human-readable origin string, e.g. ``"user_input"``,
            ``"tool:camera.capture"``, ``"system"``.
        turn_seq: Monotonic per-agent turn counter assigned at ingestion.
        ts: ``time.time()`` snapshot at ingestion.
        role: LangChain role — ``system``, ``human``, ``ai`` or ``tool``.
        representations: Precomputed renderings keyed by fidelity.
        min_fidelity: Lower bound the selector may not drop below. For
            tool-call pairing and BOOTSTRAP pages we force ``FULL``.
        pinned: If True this page can never be evicted (it always occupies
            at least ``min_fidelity`` tokens in the assembled prompt).
        pinned_at_full: Stricter variant: the page must render at FULL.
            Used for the last ``pin_recent_evidence`` images.
        artefact_uuid: UUID embedded in tool-artefact messages; populated on
            EVIDENCE pages so the LLM can refer to them via ``get_artefact``.
        tool_call_id: If this is a ``ToolMessage``, the tool-call ID it is
            paired with. The selector keeps such pairs at the same fidelity
            bucket (LangChain rejects orphan tool messages at the API layer).
    """

    id: str
    type: PageType
    provenance: str
    turn_seq: int
    ts: float
    role: MessageRole
    representations: dict[FidelityLevel, Representation]
    min_fidelity: FidelityLevel = FidelityLevel.POINTER
    pinned: bool = False
    pinned_at_full: bool = False
    artefact_uuid: str | None = None
    tool_call_id: str | None = None
    # AI tool_calls payload preserved verbatim so we can rebuild pairing on egress.
    ai_tool_calls: list[dict[str, Any]] | None = field(default=None)

    def __post_init__(self) -> None:
        if not self.id:
            raise ValueError("Page.id must be a non-empty string")
        if not isinstance(self.type, PageType):
            raise TypeError(
                f"Page.type must be a PageType, got {type(self.type).__name__}"
            )
        if not isinstance(self.min_fidelity, FidelityLevel):
            raise TypeError(
                "Page.min_fidelity must be a FidelityLevel, got "
                f"{type(self.min_fidelity).__name__}"
            )
        if self.role not in _VALID_ROLES:
            raise ValueError(
                f"Page.role must be one of {sorted(_VALID_ROLES)}, got {self.role!r}"
            )
        if self.turn_seq < 0:
            raise ValueError(f"Page.turn_seq must be non-negative, got {self.turn_seq}")
        if self.ts < 0:
            raise ValueError(f"Page.ts must be non-negative, got {self.ts}")

        # Representation-set invariants: must cover POINTER + STRUCTURED +
        # at least one of COMPRESSED/FULL. Each representation's declared
        # level must match its dict key.
        if FidelityLevel.POINTER not in self.representations:
            raise ValueError(f"Page {self.id}: POINTER representation is required")
        if FidelityLevel.STRUCTURED not in self.representations:
            raise ValueError(f"Page {self.id}: STRUCTURED representation is required")
        has_full = FidelityLevel.FULL in self.representations
        has_compressed = FidelityLevel.COMPRESSED in self.representations
        if not (has_full or has_compressed):
            raise ValueError(
                f"Page {self.id}: at least one of COMPRESSED/FULL representation required"
            )

        for level, rep in self.representations.items():
            if rep.level != level:
                raise ValueError(
                    f"Page {self.id}: representations[{level.name}].level "
                    f"is {rep.level.name}; they must match"
                )

        # Token estimates must be monotonic across fidelity. A degenerate
        # equal chain is OK (e.g. empty text) but higher levels must never
        # be *cheaper*. This is the sanity knob the selector relies on.
        ladder = sorted(self.representations.items(), key=lambda kv: kv[0])
        prev_tokens = -1
        for _level, rep in ladder:
            if rep.token_estimate < prev_tokens:
                raise ValueError(
                    f"Page {self.id}: token estimates must be monotonic across fidelity"
                    f" (got {[r.token_estimate for _, r in ladder]})"
                )
            prev_tokens = rep.token_estimate

        if self.min_fidelity < FidelityLevel.POINTER:
            raise ValueError("Page.min_fidelity must be >= POINTER")

        if self.pinned_at_full:
            # The three pin-related flags must be set consistently by
            # the caller. Silent auto-mutation ("upgrade companion
            # flags") breaks round-trip construction — callers who pass
            # ``pinned_at_full=True, pinned=False`` would read back
            # ``pinned=True`` because the constructor rewrote the input.
            # Every production caller (ingestion.py BOOTSTRAP path,
            # page_table.py auto-rebalance, page_table.mark_pinned_full)
            # already sets all three explicitly, so rejection is safe.
            if not self.pinned:
                raise ValueError(
                    f"Page {self.id}: pinned_at_full=True requires pinned=True; "
                    f"got pinned=False. The three pin-related flags must be set "
                    f"consistently by the caller."
                )
            if self.min_fidelity is not FidelityLevel.FULL:
                raise ValueError(
                    f"Page {self.id}: pinned_at_full=True requires "
                    f"min_fidelity=FidelityLevel.FULL; got min_fidelity="
                    f"{self.min_fidelity.name}. The three pin-related flags "
                    f"must be set consistently by the caller."
                )

        # Tool messages must always carry a tool_call_id so the selector can
        # keep pair integrity. The inverse also holds: only tool messages may
        # carry a ``tool_call_id`` — setting it on any other role is a bug in
        # the caller and would silently misroute pairing downstream.
        if self.role == "tool" and not self.tool_call_id:
            raise ValueError(f"Page {self.id} (role=tool) requires tool_call_id")
        if self.role != "tool" and self.tool_call_id is not None:
            raise ValueError(
                f"Page {self.id} (role={self.role!r}) must not carry "
                "tool_call_id; only role='tool' pages may."
            )

    def max_available(self) -> FidelityLevel:
        """Highest fidelity representation actually present in this page."""
        return max(self.representations.keys())

    def rep_at(self, level: FidelityLevel) -> Representation:
        """Return representation at *level*, with a lenient fallback.

        Resolution order:

        1. Exact match at *level* if present.
        2. Otherwise the highest representation whose level is strictly
           ``<= level`` (degrade gracefully).
        3. Otherwise the lowest representation that exists (fall *up* to
           something rather than raise).

        **Contract change vs. the step-2 draft.** An earlier iteration
        raised :class:`ValueError` when no rung ``<= level`` existed; this
        implementation never raises. The relaxation is safe because
        :meth:`__post_init__` enforces that every page carries a
        ``POINTER`` representation — and ``POINTER`` is ``0``, the minimum
        of :class:`FidelityLevel`. Therefore for *any* requested
        ``level``, branch (2) already finds ``POINTER`` and branch (3) is
        a defensive no-op that only fires if a future refactor violates
        that invariant.

        The selector is expected to call :meth:`max_available` before
        :meth:`rep_at` when it needs "highest-feasible-under-budget"
        semantics; this helper simply refuses to crash.
        """
        if level in self.representations:
            return self.representations[level]

        lower = [lvl for lvl in self.representations if lvl <= level]
        if lower:
            return self.representations[max(lower)]
        return self.representations[min(self.representations.keys())]

    def as_message(self, level: FidelityLevel) -> BaseMessage:
        """Render this page as a LangChain :class:`BaseMessage` at *level*.

        The returned message preserves the page's role and, for tool
        messages, its ``tool_call_id``; AI tool-call payloads are also
        reattached so LangChain's graph keeps its bookkeeping intact.
        """
        rep = self.rep_at(level)
        content: Any = rep.content

        if self.role == "system":
            return SystemMessage(content=content)
        if self.role == "human":
            return HumanMessage(content=content)
        if self.role == "ai":
            # AIMessage accepts extra kwargs (tool_calls) which are required
            # by the LangGraph executor when the message is replayed from
            # history.
            if self.ai_tool_calls:
                return AIMessage(content=content, tool_calls=self.ai_tool_calls)
            return AIMessage(content=content)
        if self.role == "tool":
            if self.tool_call_id is None:  # pragma: no cover - guarded in __post_init__
                raise ValueError(f"Page {self.id}: tool role requires tool_call_id")
            return ToolMessage(content=content, tool_call_id=self.tool_call_id)

        raise ValueError(f"Page {self.id}: unknown role {self.role!r}")
