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
"""

from __future__ import annotations

import json
from pathlib import Path
import threading
import time
from typing import Any

import httpx

from dimos.utils.logging_config import setup_logger

logger = setup_logger()

REQUEST_SUFFIX = "-request.json"
RESPONSE_SUFFIX = "-response.json"
_DROPPED_HEADERS = {"authorization", "x-api-key", "openai-organization", "cookie"}


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
                _write(
                    request_path(trace_dir, seq),
                    {
                        "method": request.method,
                        "url": str(request.url),
                        "headers": _headers(request.headers),
                        "body": _body(request.content),
                    },
                )
        except Exception:
            logger.exception("llm trace: request not recorded", dir=str(trace_dir))

    def on_response(response: httpx.Response) -> None:
        try:
            response.read()  # buffers the body; fine for the non-streaming calls this serves
            seq, t0 = response.request.extensions.get("llm_trace", (_next_seq(trace_dir) - 1, 0.0))
            with lock:
                _write(
                    response_path(trace_dir, seq),
                    {
                        "status": response.status_code,
                        "latency_s": time.monotonic() - t0 if t0 else None,
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
    _write(req, {"normalized": True, "messages": [m.model_dump() for m in messages]})
    _write(resp, {"normalized": True, "result": result.model_dump()})
    return seq, req, resp


def latest_pair(trace_dir: Path, after: int) -> tuple[int, Path, Path] | None:
    """The newest complete request/response pair with seq >= *after*, if any.
    A retried call leaves several pairs; the newest is the one that answered."""
    if not trace_dir.is_dir():
        return None
    seqs = sorted(
        int(p.name[: -len(REQUEST_SUFFIX)])
        for p in trace_dir.glob(f"*{REQUEST_SUFFIX}")
        if p.name[: -len(REQUEST_SUFFIX)].isdigit()
    )
    seqs = [s for s in seqs if s >= after and response_path(trace_dir, s).exists()]
    if not seqs:
        return None
    return seqs[-1], request_path(trace_dir, seqs[-1]), response_path(trace_dir, seqs[-1])
