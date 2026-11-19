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

"""Wire-exact capture of every LLM request and response.

``tracing_http_client(dir)`` is an ``httpx.Client`` whose event hooks write
the literal bytes sent to and received from the provider as
``<seq>-request.json`` / ``<seq>-response.json`` (auth header dropped). Give
it to any OpenAI-backed chat model as ``http_client=``; every call becomes
one pair of files, whole, not reconstructed from normalized messages.

``write_normalized(dir, messages, result)`` is the fallback for providers
that offer no HTTP hook: the same file pair built from LangChain's view of
the call, marked ``"normalized": true`` so a reader knows it is not wire-exact.

Both also save each image a request carries, the first time any request
carries it, as ``seen/r<seq>-<n>.<ext>`` beside *dir*: what the model looked
at, in one folder, as it looks.
"""

from __future__ import annotations

import base64
import binascii
from collections.abc import Iterator
import hashlib
import json
from pathlib import Path
import re
import threading
import time
from typing import Any

import httpx

from dimos.utils.logging_config import setup_logger

logger = setup_logger()

REQUEST_SUFFIX = "-request.json"
RESPONSE_SUFFIX = "-response.json"
_DROPPED_HEADERS = {"authorization", "x-api-key", "x-goog-api-key", "openai-organization", "cookie"}
SEEN_DIR = "seen"
_DATA_URL = re.compile(r"data:image/([a-z0-9.+-]+);base64,([A-Za-z0-9+/=]+)")
# Images already saved, per trace dir, by digest: a later request resends the whole history.
_saved: dict[Path, set[str]] = {}
_saved_lock = threading.Lock()


def _next_seq(trace_dir: Path) -> int:
    return len(list(trace_dir.glob(f"*{REQUEST_SUFFIX}")))


def _write(path: Path, record: dict[str, Any]) -> None:
    path.write_text(json.dumps(record, indent=2, default=str))


def _headers(headers: httpx.Headers) -> dict[str, str]:
    return {k: v for k, v in headers.items() if k.lower() not in _DROPPED_HEADERS}


def _body(content: bytes) -> Any:
    try:
        return json.loads(content)
    except ValueError:
        return content.decode("utf-8", errors="replace")


def _images(value: Any) -> Iterator[tuple[str, str]]:
    """(subtype, base64) of each image in a request body: data URLs (OpenAI) and
    ``media_type`` / ``mime_type`` + ``data`` blocks (Anthropic, Gemini)."""
    if isinstance(value, str):
        yield from _DATA_URL.findall(value)
    elif isinstance(value, dict):
        media = value.get("media_type") or value.get("mime_type") or value.get("mimeType")
        if (
            isinstance(media, str)
            and media.startswith("image/")
            and isinstance(value.get("data"), str)
        ):
            yield media.removeprefix("image/"), value["data"]
            return
        for item in value.values():
            yield from _images(item)
    elif isinstance(value, list):
        for item in value:
            yield from _images(item)


def seen_dir(trace_dir: Path) -> Path:
    return trace_dir.parent / SEEN_DIR


def _save_images(trace_dir: Path, seq: int, body: Any) -> None:
    """Never raises into the model call: text that only looks like an image (a data URL
    quoted in a prompt) is skipped, and any other failure is logged."""
    try:
        with _saved_lock:
            saved = _saved.setdefault(trace_dir.resolve(), set())
            index = 0
            for subtype, data in _images(body):
                digest = hashlib.sha256(data.encode()).hexdigest()
                if digest in saved:
                    continue
                try:
                    image = base64.b64decode(data, validate=True)
                except (binascii.Error, ValueError):
                    continue
                saved.add(digest)
                folder = seen_dir(trace_dir)
                folder.mkdir(exist_ok=True)
                ext = (
                    "jpg"
                    if subtype in ("jpeg", "jpg")
                    else re.sub(r"[^a-z0-9]", "", subtype) or "img"
                )
                (folder / f"r{seq:03d}-{index}.{ext}").write_bytes(image)
                index += 1
    except Exception:
        logger.exception("llm trace: images not saved", dir=str(trace_dir))


def save_request_images(trace_dir: Path, messages: list[Any]) -> None:
    """Save the images of a call about to go out that no HTTP hook may see (``write_normalized``
    records it afterwards), so ``seen/`` has them while the model works, or if it fails. The
    hook saving the same images too keeps one copy."""
    trace_dir.mkdir(parents=True, exist_ok=True)
    _save_images(trace_dir, _next_seq(trace_dir), [m.model_dump() for m in messages])


def request_path(trace_dir: Path, seq: int) -> Path:
    return trace_dir / f"{seq:03d}{REQUEST_SUFFIX}"


def response_path(trace_dir: Path, seq: int) -> Path:
    return trace_dir / f"{seq:03d}{RESPONSE_SUFFIX}"


def tracing_http_client(trace_dir: Path, **kwargs: Any) -> httpx.Client:
    """An httpx client that records each request/response pair under *trace_dir*."""
    trace_dir.mkdir(parents=True, exist_ok=True)
    lock = threading.Lock()

    def on_request(request: httpx.Request) -> None:
        # A hook that raises is retried by the SDK like a connection failure
        # and the gap is silent, so never let one out.
        try:
            with lock:
                seq = _next_seq(trace_dir)
                request.extensions["llm_trace"] = (seq, time.monotonic())
                body = _body(request.content)
                _write(
                    request_path(trace_dir, seq),
                    {
                        "started_at": time.time(),
                        "method": request.method,
                        "url": str(request.url),
                        "headers": _headers(request.headers),
                        "body": body,
                    },
                )
                _save_images(trace_dir, seq, body)
        except Exception:
            logger.exception("llm trace: request not recorded", dir=str(trace_dir))

    def on_response(response: httpx.Response) -> None:
        try:
            response.read()  # buffers the body; fine for the non-streaming calls this serves
            seq, t0 = response.request.extensions["llm_trace"]
            with lock:
                _write(
                    response_path(trace_dir, seq),
                    {
                        "status": response.status_code,
                        "latency_s": time.monotonic() - t0,
                        "headers": _headers(response.headers),
                        "body": _body(response.content),
                    },
                )
        except Exception:
            logger.exception("llm trace: response not recorded", dir=str(trace_dir))

    hooks = kwargs.pop("event_hooks", {})
    hooks = {
        "request": [*hooks.get("request", []), on_request],
        "response": [*hooks.get("response", []), on_response],
    }
    return httpx.Client(event_hooks=hooks, **kwargs)


def write_normalized(trace_dir: Path, messages: list[Any], result: Any) -> tuple[int, Path, Path]:
    """Record a call from LangChain's normalized view (no HTTP hook available)."""
    trace_dir.mkdir(parents=True, exist_ok=True)
    seq = _next_seq(trace_dir)
    req, resp = request_path(trace_dir, seq), response_path(trace_dir, seq)
    dumped = [m.model_dump() for m in messages]
    _write(req, {"normalized": True, "messages": dumped})
    _save_images(trace_dir, seq, dumped)
    _write(resp, {"normalized": True, "result": result.model_dump()})
    return seq, req, resp


def list_llm_trace_pairs(trace_dir: Path) -> list[tuple[int, Path, Path]]:
    """Completed request/response files, ordered by their numeric sequence."""
    pairs: list[tuple[int, Path, Path]] = []
    for request in trace_dir.glob(f"*{REQUEST_SUFFIX}"):
        seq = request.name.removesuffix(REQUEST_SUFFIX)
        response = request.with_name(f"{seq}{RESPONSE_SUFFIX}")
        if seq.isdigit() and response.is_file():
            pairs.append((int(seq), request, response))
    return sorted(pairs)


def last_request_unanswered(trace_dir: Path) -> bool:
    """Whether the newest request has no response: the call failed before one arrived."""
    seqs = [
        int(seq)
        for request in trace_dir.glob(f"*{REQUEST_SUFFIX}")
        if (seq := request.name.removesuffix(REQUEST_SUFFIX)).isdigit()
    ]
    return bool(seqs) and not response_path(trace_dir, max(seqs)).is_file()


def latest_pair(trace_dir: Path, after: int) -> tuple[int, Path, Path] | None:
    """The newest complete request/response pair with seq >= *after*, if any.
    A retried call leaves several pairs; the newest is the one that answered."""
    pairs = [pair for pair in list_llm_trace_pairs(trace_dir) if pair[0] >= after]
    return pairs[-1] if pairs else None
