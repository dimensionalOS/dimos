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

"""Turn a LangChain :class:`BaseMessage` into one or more :class:`Page` objects.

Ingestion runs once per message; all four fidelity representations are
computed up-front and cached on the resulting page(s). Doing the expensive
work here means the selector loop can stay fast (it just picks which rung
of the ladder to render).

The hot path is the image-artefact case: we decode the base64 JPEG, resize
to 96x96, re-encode at quality 30, and store the thumbnail as a
``image_url`` part ready to be handed back to the LLM at COMPRESSED
fidelity.
"""

from __future__ import annotations

import base64
import json
import re
import time
from typing import Any
import uuid as uuid_module

import cv2
from langchain_core.messages import AIMessage, HumanMessage, SystemMessage, ToolMessage
from langchain_core.messages.base import BaseMessage
import numpy as np

from dimos.agents.memory.pages import (
    FidelityLevel,
    MessageRole,
    Page,
    PageType,
    Representation,
)
from dimos.agents.memory.tokens import (
    IMAGE_FULL_COST,
    IMAGE_THUMB_COST,
    TokenCounter,
)

__all__ = [
    "THUMBNAIL_JPEG_QUALITY",
    "THUMBNAIL_SIZE",
    "ingest_message",
]


THUMBNAIL_SIZE = 96
"""Width/height of the COMPRESSED image representation, in pixels."""

THUMBNAIL_JPEG_QUALITY = 30
"""JPEG quality used for the COMPRESSED image representation."""


# ---------------------------------------------------------------------------
# Helpers — role / page-type / artefact extraction
# ---------------------------------------------------------------------------


def _role_of(msg: BaseMessage) -> MessageRole:
    if isinstance(msg, SystemMessage):
        return "system"
    if isinstance(msg, HumanMessage):
        return "human"
    if isinstance(msg, AIMessage):
        return "ai"
    if isinstance(msg, ToolMessage):
        return "tool"
    # Fall-through: LangChain has more message subclasses (FunctionMessage
    # etc.) but the concrete ones in play for us are the four above.
    raise TypeError(f"Unsupported BaseMessage subclass: {type(msg).__name__}")


def _default_page_type(msg: BaseMessage, *, has_images: bool) -> PageType:
    if isinstance(msg, SystemMessage):
        return PageType.BOOTSTRAP
    if has_images:
        return PageType.EVIDENCE
    return PageType.CONVERSATION


def _default_provenance(msg: BaseMessage) -> str:
    if isinstance(msg, SystemMessage):
        return "system"
    if isinstance(msg, HumanMessage):
        return "user_input"
    if isinstance(msg, AIMessage):
        return "llm_response"
    if isinstance(msg, ToolMessage):
        return "tool_response"
    return "unknown"


# Pattern used both for artefact id detection (on ingest) and lookup hints
# (so tests can round-trip). Lifted from the existing McpClient convention:
#     "Tool call started with UUID: <uuid>"
# and
#     "This is the artefact for the '<tool>' tool with UUID:=<uuid>."
_UUID_RE = re.compile(r"UUID\s*[:=]\s*=?\s*([0-9a-fA-F-]{8,})")


def _extract_artefact_uuid(text: str) -> str | None:
    m = _UUID_RE.search(text)
    if m:
        return m.group(1)
    return None


def _split_content_parts(
    content: Any,
) -> tuple[list[str], list[dict[str, Any]]]:
    """Split a multimodal content field into ordered text and image parts.

    Returns ``(text_parts, image_parts)`` where text_parts is the list of
    text strings in original order (empty strings filtered out) and
    image_parts is the list of ``image_url``/``image`` dicts in original
    order. Plain-string content becomes a single-element text list.
    Unknown LangChain part types are JSON-stringified onto the text side
    so nothing is silently lost.

    The split rules in ``ingest_message`` rely on ``len(text_parts)`` and
    ``len(image_parts)`` separately, so we return the raw lists rather
    than a pre-joined string.
    """
    if isinstance(content, str):
        return ([content] if content else [], [])
    if not isinstance(content, list):
        return ([str(content)], [])

    texts: list[str] = []
    images: list[dict[str, Any]] = []
    for part in content:
        if not isinstance(part, dict):
            texts.append(str(part))
            continue
        ptype = part.get("type")
        if ptype == "text":
            t = str(part.get("text", ""))
            if t:
                texts.append(t)
        elif ptype in ("image_url", "image"):
            images.append(part)
        else:
            texts.append(json.dumps(part, default=str))
    return texts, images


# ---------------------------------------------------------------------------
# Image thumbnailing
# ---------------------------------------------------------------------------


def _decode_image_url(image_part: dict[str, Any]) -> np.ndarray | None:
    """Decode a LangChain ``image_url`` content part into a BGR ndarray.

    Supports only ``data:image/...;base64,...`` URLs; external HTTP URLs
    are returned as ``None`` because we can't reliably fetch them during
    ingestion.
    """
    url = image_part.get("image_url", {}).get("url") if image_part.get("type") == "image_url" \
        else image_part.get("source", {}).get("data")
    if not isinstance(url, str):
        return None

    prefix = "base64,"
    idx = url.find(prefix)
    if idx < 0:
        return None
    try:
        raw = base64.b64decode(url[idx + len(prefix):], validate=True)
    except Exception:
        return None
    arr = np.frombuffer(raw, dtype=np.uint8)
    if arr.size == 0:
        return None
    img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
    return img


def _image_src_tag(image_part: dict[str, Any]) -> str:
    """Short, grep-friendly source tag for the STRUCTURED bracketed format.

    The ``src`` field of the STRUCTURED rendering is a short provenance
    tag — never the full base64 payload, which would defeat the purpose
    of the STRUCTURED tier. For ``data:`` URIs we return the mime prefix
    (``data:image/jpeg``), for HTTP(S) URLs we return the URL, and for
    anything we cannot interpret we return ``unknown``. Spaces are
    escaped to ``%20`` so the whole bracketed artefact stays
    single-tokenable and the format stays grep-friendly.
    """
    url: Any = ""
    if image_part.get("type") == "image_url":
        url = image_part.get("image_url", {}).get("url", "")
    elif image_part.get("type") == "image":
        url = image_part.get("source", {}).get("data", "")
    if not isinstance(url, str) or not url:
        return "unknown"
    if url.startswith("data:"):
        # ``data:image/jpeg;base64,AAAA...`` -> ``data:image/jpeg``.
        head = url.split(";", 1)[0] if ";" in url else url.split(",", 1)[0]
        return head.replace(" ", "%20")
    return url.replace(" ", "%20")


def _encode_thumbnail(bgr: np.ndarray) -> str | None:
    """Resize *bgr* to 96x96 and JPEG-encode it at quality 30."""
    thumb = cv2.resize(
        bgr,
        (THUMBNAIL_SIZE, THUMBNAIL_SIZE),
        interpolation=cv2.INTER_AREA,
    )
    ok, buf = cv2.imencode(
        ".jpg",
        thumb,
        [int(cv2.IMWRITE_JPEG_QUALITY), THUMBNAIL_JPEG_QUALITY],
    )
    if not ok:
        return None
    return base64.b64encode(buf.tobytes()).decode("ascii")


# ---------------------------------------------------------------------------
# Representation builders
# ---------------------------------------------------------------------------


_TEXT_TRUNCATION_SENTENCES = 2
_STRUCTURED_SUMMARY_MAX_CHARS = 80


def _sentence_truncate(text: str, max_sentences: int = _TEXT_TRUNCATION_SENTENCES) -> str:
    """Return the first *max_sentences* sentences of *text*, with a marker.

    Sentence boundaries are approximated with a regex (``. ! ?`` followed
    by whitespace). Good enough for logs; the plan never claims this is
    linguistically correct.
    """
    if not text:
        return text
    # Find sentence terminators followed by whitespace or end-of-string.
    boundaries = [m.end() for m in re.finditer(r"[.!?]+(?:\s+|$)", text)]
    if len(boundaries) >= max_sentences:
        cutoff = boundaries[max_sentences - 1]
        head = text[:cutoff].rstrip()
        tail = text[cutoff:].strip()
        if tail:
            return head + " ... (truncated)"
        return head
    return text


def _short_summary(role: MessageRole, text: str, tokens: int) -> str:
    snippet = text.strip().replace("\n", " ")
    if len(snippet) > _STRUCTURED_SUMMARY_MAX_CHARS:
        snippet = snippet[: _STRUCTURED_SUMMARY_MAX_CHARS - 1] + "…"
    return json.dumps(
        {"role": role, "summary": snippet, "tokens": tokens},
        ensure_ascii=False,
    )


def _build_text_representations(
    role: MessageRole,
    text: str,
    page_id: str,
    token_counter: TokenCounter,
) -> dict[FidelityLevel, Representation]:
    """Build POINTER/STRUCTURED/COMPRESSED/FULL for a text message."""
    # Guard against fully-empty text — we still need a non-empty FULL rep
    # because ``Representation`` enforces that. Substitute a single-space
    # placeholder; this is only reached for deliberately-empty messages.
    full_text = text if text else " "
    compressed_text = _sentence_truncate(full_text) or full_text
    full_tokens = token_counter.count_text(full_text)
    compressed_tokens = token_counter.count_text(compressed_text)
    structured_text = _short_summary(role, full_text, full_tokens)
    structured_tokens = token_counter.count_text(structured_text)
    pointer_text = f"[page {page_id}]"
    pointer_tokens = token_counter.count_text(pointer_text)

    # Force monotonicity even if the heuristic disagrees (e.g. structured
    # summary ends up longer than the compressed truncation). The selector
    # relies on monotonic costs; we pick min(upper, lower+1)-style caps.
    structured_tokens = min(structured_tokens, max(pointer_tokens, structured_tokens))
    compressed_tokens = max(compressed_tokens, structured_tokens)
    full_tokens = max(full_tokens, compressed_tokens)

    return {
        FidelityLevel.POINTER: Representation(FidelityLevel.POINTER, pointer_text, pointer_tokens),
        FidelityLevel.STRUCTURED: Representation(
            FidelityLevel.STRUCTURED, structured_text, structured_tokens
        ),
        FidelityLevel.COMPRESSED: Representation(
            FidelityLevel.COMPRESSED, compressed_text, compressed_tokens
        ),
        FidelityLevel.FULL: Representation(FidelityLevel.FULL, full_text, full_tokens),
    }


def _build_image_representations(
    image_part: dict[str, Any],
    *,
    page_id: str,
    provenance: str,
    artefact_uuid: str,
    token_counter: TokenCounter,
    preamble_text: str = "",
) -> dict[FidelityLevel, Representation]:
    """Build all four representations for a message whose payload is one image.

    Structure of the four rungs:

    * FULL:        original image part (plus optional text preamble).
    * COMPRESSED:  96x96 q30 thumbnail as an ``image_url`` content part.
    * STRUCTURED:  one-line bracketed summary
                   ``"[image artefact uuid=<artefact_uuid> src=... dims=... size=...]"``.
                   The UUID here MUST be the ``artefact_uuid`` (the key
                   :class:`PageTable._by_artefact` indexes on and that
                   :meth:`MemoryEngine.request_full` looks up against) —
                   NOT the ``page.id``. An LLM that extracts the UUID
                   from this bracket and hands it to ``get_artefact``
                   relies on both representations agreeing.
    * POINTER:     ``"[artefact uuid=<artefact_uuid>]"``.

    If the image can't be decoded (external URL, corrupt base64), we fall
    back to a structured-only rung and a text FULL rung that just points
    the LLM at the artefact UUID. This keeps the memory layer resilient to
    half-formed messages from misbehaving tools.
    """
    bgr = _decode_image_url(image_part)

    # POINTER
    pointer_text = f"[artefact uuid={artefact_uuid}]"
    pointer_tokens = token_counter.count_text(pointer_text)

    # STRUCTURED — single-line bracketed text artefact. Do NOT swap it
    # for a JSON blob; a JSON rep would double-stringify inside prompts.
    if bgr is not None:
        height, width = int(bgr.shape[0]), int(bgr.shape[1])
        dims_str = f"{width}x{height}"
        # Rough size of the original base64 payload in bytes (~4/3 expansion).
        url_str = image_part.get("image_url", {}).get("url", "")
        est_size_kb = max(1, (len(url_str) * 3 // 4) // 1024)
        size_str = f"{est_size_kb}kB"
    else:
        # Unknown dims/size survive as ``unknown`` rather than crashing or
        # emitting ``None``; the format must never break the bracket.
        dims_str = "unknown"
        size_str = "unknown"
    src_tag = _image_src_tag(image_part)
    # The UUID inside the STRUCTURED bracket MUST be the ``artefact_uuid``
    # (the key ``PageTable._by_artefact`` is indexed on, which
    # ``MemoryEngine.request_full`` / ``PageTable.get_by_artefact``
    # resolve against) — NOT the ``page.id`` (which carries a ``"page-"``
    # prefix and is a different string entirely). If an LLM extracted the
    # UUID from a STRUCTURED rep and called ``get_artefact`` with a
    # ``page.id`` here, it would hit the "not found" path even though the
    # page was live in
    # the table. The POINTER at line 341 was always correct; this line
    # brings STRUCTURED into agreement.
    structured_text = (
        f"[image artefact uuid={artefact_uuid} "
        f"src={src_tag} dims={dims_str} size={size_str}]"
    )
    structured_tokens = token_counter.count_text(structured_text)

    # COMPRESSED — thumbnail (or text fallback when we can't decode).
    if bgr is not None:
        thumb_b64 = _encode_thumbnail(bgr)
    else:
        thumb_b64 = None

    if thumb_b64 is not None:
        thumb_url = f"data:image/jpeg;base64,{thumb_b64}"
        compressed_content: str | list[dict[str, Any]] = [
            {
                "type": "image_url",
                "image_url": {"url": thumb_url, "detail": "low"},
            },
        ]
        if preamble_text:
            compressed_content = [
                {"type": "text", "text": preamble_text},
                *compressed_content,  # type: ignore[misc]
            ]
        compressed_tokens = IMAGE_THUMB_COST + (
            token_counter.count_text(preamble_text) if preamble_text else 0
        )
    else:
        # Fall back to the structured text — same cost, same content.
        compressed_content = structured_text
        compressed_tokens = structured_tokens

    # FULL — the original part (optionally with its preamble).
    if preamble_text:
        full_content: list[dict[str, Any]] = [
            {"type": "text", "text": preamble_text},
            image_part,
        ]
    else:
        full_content = [image_part]
    full_tokens = IMAGE_FULL_COST + (
        token_counter.count_text(preamble_text) if preamble_text else 0
    )

    # Enforce POINTER <= STRUCTURED <= COMPRESSED <= FULL monotonicity.
    # The new bracketed STRUCTURED format is always longer than POINTER
    # in practice, but we clamp defensively in case a future counter
    # tokenises them unexpectedly.
    structured_tokens = max(structured_tokens, pointer_tokens)
    compressed_tokens = max(compressed_tokens, structured_tokens)
    full_tokens = max(full_tokens, compressed_tokens)

    return {
        FidelityLevel.POINTER: Representation(
            FidelityLevel.POINTER, pointer_text, pointer_tokens
        ),
        FidelityLevel.STRUCTURED: Representation(
            FidelityLevel.STRUCTURED, structured_text, structured_tokens
        ),
        FidelityLevel.COMPRESSED: Representation(
            FidelityLevel.COMPRESSED, compressed_content, compressed_tokens
        ),
        FidelityLevel.FULL: Representation(
            FidelityLevel.FULL, full_content, full_tokens
        ),
    }


# ---------------------------------------------------------------------------
# Public entry point
# ---------------------------------------------------------------------------


def ingest_message(
    msg: BaseMessage,
    *,
    turn_seq: int,
    token_counter: TokenCounter,
    ts: float | None = None,
    page_id: str | None = None,
    page_type_hint: PageType | None = None,
) -> list[Page]:
    """Turn *msg* into one or more :class:`Page` objects.

    A multimodal ``HumanMessage`` splits into one EVIDENCE page per
    image part, with any accompanying text kept on a separate
    CONVERSATION page. Non-multimodal messages always yield exactly one
    page, wrapped in a list. Every page produced from a single call
    shares the same ``turn_seq`` and ``ts``.

    Split rules (exhaustive):

    * ``SystemMessage`` → one BOOTSTRAP page, pinned at FULL.
    * ``AIMessage`` → one CONVERSATION page. ``ai_tool_calls`` is
      preserved verbatim, and its serialized-JSON token cost is added
      to every representation's ``token_estimate`` (see the
      "AI tool_calls token accounting" note below).
    * ``ToolMessage`` → one CONVERSATION page carrying ``tool_call_id``.
    * ``HumanMessage`` with string content → one CONVERSATION page.
    * ``HumanMessage`` with list content: classify each part by type.
      Let T = text parts, I = image parts (``image_url`` or ``image``).

      - ``len(I) == 0``: one CONVERSATION page from the joined text parts.
      - ``len(T) == 0`` and ``len(I) == 1``: one EVIDENCE page from the
        single image (no preamble).
      - ``len(T) == 0`` and ``len(I) >= 2``: one EVIDENCE page per image
        in original content-part order.
      - ``len(T) >= 1`` and ``len(I) >= 1``: one CONVERSATION page from
        the joined text plus one EVIDENCE page per image, ordered
        ``[CONVERSATION, EVIDENCE_1, ..., EVIDENCE_n]``. The CONVERSATION
        page's FULL text appends ``"\\n\\nAttached artefacts: [uuid1]
        [uuid2] ..."`` so the LLM can reference the pages it may want
        to rehydrate via the ``get_artefact`` tool.

    Unsupported message types still raise :class:`TypeError`.

    Args:
        msg: The LangChain message to ingest.
        turn_seq: Monotonic per-agent turn counter. Every page produced
            from this call shares this value.
        token_counter: Backend used to size each representation.
        ts: ``float | None = None`` — wall-clock timestamp (seconds
            since epoch) to attach to every produced Page's provenance.
            Defaults to ``time.time()`` when omitted (one ``time.time()``
            call per ``ingest_message`` invocation, not per page).
        page_id: Stable id. **Only honored when the call produces
            exactly one page.** In multi-page results (multimodal split)
            every page is assigned a fresh synthesized UUID and the
            caller-supplied ``page_id`` is silently ignored — see the
            "multi-page id contract" note below.
        page_type_hint: If given, overrides the auto-detected
            :class:`PageType` on the **single-page CONVERSATION** path
            (e.g. force BOOTSTRAP, CONSTRAINT, PLAN, PREFERENCE). The
            hint is IGNORED for single-EVIDENCE pages (EVIDENCE is
            structural) and for any multi-page result (no mixed
            semantics). See the "page_type_hint" note below.

    Signature notes:

    * ``ts`` is passed in from the engine so ingestion does not read the
      clock itself by default — this keeps the module side-effect free
      and lets tests inject deterministic timestamps. When ``ts`` is
      omitted, ``ingest_message`` falls back to ``time.time()`` so
      ergonomic callers can skip it.
    * ``page_id`` is optional because the common path should synthesise
      ids, but deterministic tests and the ``get_artefact`` correlation
      path both benefit from being able to pin one.

    Multi-page id contract: when a multimodal ``HumanMessage`` produces
    more than one page, the explicit ``page_id`` kwarg is silently
    ignored and every page is assigned a fresh synthesized UUID. This
    differs from the single-page path,
    where ``page_id`` was always honored. We chose silent-ignore over
    ``ValueError`` because callers who want pinned ids for the
    artefact-rehydration path only ever pass single-image messages;
    the multi-image case is a sink for whichever tool-result gets
    batched, and there is no sensible disambiguation scheme across
    N synthesized pages.

    ``page_type_hint`` semantics:

    * Single CONVERSATION page (message is System/AI/Tool/Human-text):
      hint is honored and replaces the auto-detected :class:`PageType`.
    * Single EVIDENCE page (one image, no text): hint is IGNORED —
      EVIDENCE is structural.
    * Multi-page result (cases e.3/e.4): hint is IGNORED entirely —
      mixing CONVERSATION + EVIDENCE pages under a single caller-
      supplied type would produce incoherent semantics.

    Returns:
        A non-empty ``list[Page]`` in ingest order (CONVERSATION first
        when present, then EVIDENCE pages in original content-part
        order). The list is handed to :class:`PageTable` as a batch by
        :class:`MemoryEngine.ingest`.

    AI ``tool_calls`` token accounting:
        When the input is an :class:`AIMessage` with ``tool_calls``,
        the serialized JSON cost of the ``tool_calls`` payload is
        added to every representation's ``token_estimate``. This is
        necessary because :meth:`Page.as_message` reattaches
        ``tool_calls`` at every fidelity level to preserve
        LangChain's AIMessage/ToolMessage pairing — the payload is in
        the wire-format message regardless of fidelity.

        The serialization is ``json.dumps(ai_tool_calls, sort_keys=True,
        default=str)`` so equal payloads produce equal token counts
        (deterministic for tests) and non-JSON-serializable args
        (datetimes, paths, etc.) fall back to ``repr`` rather than
        crashing. Counting is done via :meth:`TokenCounter.count_text`
        (the content-only counter). AIMessages WITHOUT ``tool_calls``
        are a zero-overhead path.
    """
    if ts is None:
        ts = time.time()

    role = _role_of(msg)
    text_parts, image_parts = _split_content_parts(msg.content)
    joined_text = " ".join(t for t in text_parts if t)
    has_text = bool(joined_text.strip())
    n_images = len(image_parts)

    # Cases (a)-(d), (e.1): no images → exactly one CONVERSATION/BOOTSTRAP page.
    if n_images == 0:
        return [
            _build_single_text_page(
                msg,
                role=role,
                text=joined_text,
                turn_seq=turn_seq,
                ts=ts,
                token_counter=token_counter,
                page_id=page_id,
                page_type_hint=page_type_hint,
            )
        ]

    # Case (e.2): exactly one image and no text → exactly one EVIDENCE page.
    # ``page_id`` is honored; ``page_type_hint`` is IGNORED (EVIDENCE is
    # structural).
    if n_images == 1 and not has_text:
        return [
            _build_single_evidence_page(
                msg,
                role=role,
                image_part=image_parts[0],
                preamble_text="",
                turn_seq=turn_seq,
                ts=ts,
                token_counter=token_counter,
                page_id=page_id,
                artefact_uuid=None,
            )
        ]

    # Cases (e.3) and (e.4): multi-page result. ``page_id`` is silently
    # ignored (see "multi-page id contract" note); ``page_type_hint`` is
    # ignored entirely.
    extracted_uuid = _extract_artefact_uuid(joined_text) if has_text else None
    evidence_pages: list[Page] = []
    evidence_uuids: list[str] = []
    for i, image_part in enumerate(image_parts):
        # The MCP convention embeds an artefact UUID in the accompanying
        # text; we honor it only for the first image, since a single
        # preamble cannot disambiguate across N images.
        if i == 0 and extracted_uuid:
            artefact_uuid = extracted_uuid
        else:
            artefact_uuid = _new_uuid()
        evidence_uuids.append(artefact_uuid)
        evidence_pages.append(
            _build_single_evidence_page(
                msg,
                role=role,
                image_part=image_part,
                preamble_text="",
                turn_seq=turn_seq,
                ts=ts,
                token_counter=token_counter,
                page_id=None,
                artefact_uuid=artefact_uuid,
            )
        )

    # Case (e.3): pure images — no CONVERSATION page.
    if not has_text:
        return evidence_pages

    # Case (e.4): CONVERSATION preamble + EVIDENCE pages. The trailing
    # ``Attached artefacts: [uuid1] [uuid2] ...`` line gives the LLM
    # something to quote when it decides whether to call ``get_artefact``.
    attached_line = "Attached artefacts: " + " ".join(
        f"[{u}]" for u in evidence_uuids
    )
    conv_text = f"{joined_text}\n\n{attached_line}"
    conv_page = _build_single_text_page(
        msg,
        role=role,
        text=conv_text,
        turn_seq=turn_seq,
        ts=ts,
        token_counter=token_counter,
        page_id=None,
        page_type_hint=None,
        force_type=PageType.CONVERSATION,
    )
    return [conv_page, *evidence_pages]


def _build_single_text_page(
    msg: BaseMessage,
    *,
    role: MessageRole,
    text: str,
    turn_seq: int,
    ts: float,
    token_counter: TokenCounter,
    page_id: str | None,
    page_type_hint: PageType | None,
    force_type: PageType | None = None,
) -> Page:
    """Build one text-only :class:`Page` (cases a-d and e.1, plus the
    CONVERSATION page in case e.4)."""
    pid = page_id or _new_page_id()
    reps = _build_text_representations(role, text, pid, token_counter)

    # Account for the serialized ``tool_calls`` payload on AI pages.
    # :meth:`Page.as_message` reattaches ``tool_calls`` at EVERY fidelity
    # level to preserve LangChain's AIMessage/ToolMessage pairing, so the
    # payload is in the outgoing wire-format message regardless of which
    # rung the selector picks. The token cost must therefore be added
    # uniformly to every representation's ``token_estimate``. Adding the
    # same constant to all four rungs preserves the
    # POINTER <= STRUCTURED <= COMPRESSED <= FULL monotonicity invariant
    # that the selector relies on.
    tool_calls = _clone_tool_calls(msg)
    if tool_calls:
        tool_calls_json = json.dumps(tool_calls, sort_keys=True, default=str)
        tool_calls_tokens = token_counter.count_text(tool_calls_json)
        if tool_calls_tokens:
            reps = {
                level: Representation(
                    level, rep.content, rep.token_estimate + tool_calls_tokens
                )
                for level, rep in reps.items()
            }

    page_type = (
        force_type
        if force_type is not None
        else (page_type_hint or _default_page_type(msg, has_images=False))
    )
    page = Page(
        id=pid,
        type=page_type,
        provenance=_default_provenance(msg),
        turn_seq=turn_seq,
        ts=ts,
        role=role,
        representations=reps,
        tool_call_id=getattr(msg, "tool_call_id", None),
        ai_tool_calls=tool_calls,
    )
    if page.type is PageType.BOOTSTRAP:
        page.pinned_at_full = True
        page.pinned = True
        page.min_fidelity = FidelityLevel.FULL
    return page


def _build_single_evidence_page(
    msg: BaseMessage,
    *,
    role: MessageRole,
    image_part: dict[str, Any],
    preamble_text: str,
    turn_seq: int,
    ts: float,
    token_counter: TokenCounter,
    page_id: str | None,
    artefact_uuid: str | None,
) -> Page:
    """Build one EVIDENCE :class:`Page` wrapping a single image part.

    ``artefact_uuid`` is caller-supplied when the split logic has
    already allocated uuids for a multi-page result; if ``None`` we
    extract from ``preamble_text`` (MCP convention) or synthesize.
    """
    pid = page_id or _new_page_id()
    if artefact_uuid is None:
        artefact_uuid = (
            _extract_artefact_uuid(preamble_text) if preamble_text else None
        ) or _new_uuid()
    reps = _build_image_representations(
        image_part,
        page_id=pid,
        provenance=_default_provenance(msg),
        artefact_uuid=artefact_uuid,
        token_counter=token_counter,
        preamble_text=preamble_text,
    )
    return Page(
        id=pid,
        type=PageType.EVIDENCE,
        provenance=_default_provenance(msg),
        turn_seq=turn_seq,
        ts=ts,
        role=role,
        representations=reps,
        artefact_uuid=artefact_uuid,
        tool_call_id=getattr(msg, "tool_call_id", None),
        ai_tool_calls=_clone_tool_calls(msg),
    )


def _new_page_id() -> str:
    return f"page-{uuid_module.uuid4()}"


def _new_uuid() -> str:
    return str(uuid_module.uuid4())


def _clone_tool_calls(msg: BaseMessage) -> list[dict[str, Any]] | None:
    """Extract ``tool_calls`` from an :class:`AIMessage`, if any.

    We take a shallow copy so downstream mutations (e.g. the selector
    producing a new :class:`AIMessage`) cannot bleed back into the stored
    page.
    """
    calls = getattr(msg, "tool_calls", None)
    if not calls:
        return None
    return [dict(c) for c in calls]
