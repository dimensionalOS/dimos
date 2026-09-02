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

"""Optional tracing, off unless someone is watching.

Wraps Langfuse behind an interface that costs nothing when it is absent. Three
constraints shape this, and each one has bitten a robot codebase before:

**A missing dependency must never break a run.** Tracing is a development
convenience. ``@skill`` sits in the hot path of every skill including
perception at 15 Hz, and an ImportError there would take the robot down for the
sake of a dashboard.

**Nothing is emitted unless a trace is open.** Background perception calls the
same wrapped code as an agent does. Emitting on every one of them would drown
the useful traces and bill for the privilege, so spans are recorded only inside
an :func:`agent_trace` block -- which only an agent-initiated task opens.

**No pixels, no keys.** Evidence travels as ``(stream, observation_id)``
references. An image belongs in the store, not in a trace, and a trace is a
place secrets leak to.
"""

from __future__ import annotations

import contextlib
import os
import sys
import threading
from typing import TYPE_CHECKING, Any

from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from collections.abc import Iterator, Mapping

logger = setup_logger()

#: Set when a trace is open on this thread. Read by :func:`span` to decide
#: whether anything is worth emitting.
_ACTIVE = threading.local()

_CLIENT: Any = None
_CLIENT_TRIED = False


def _client() -> Any:
    """The Langfuse client, or None. Constructed once, never raises."""
    global _CLIENT, _CLIENT_TRIED
    if _CLIENT_TRIED:
        return _CLIENT
    _CLIENT_TRIED = True
    if not (os.getenv("LANGFUSE_PUBLIC_KEY") and os.getenv("LANGFUSE_SECRET_KEY")):
        return None
    # The rest of dimos names this endpoint LANGFUSE_BASE_URL -- it is what
    # `.env` and `default.env` carry -- while the Langfuse SDK reads
    # LANGFUSE_HOST. Without this bridge a process started from `.env` gets a
    # working client pointed at the SDK's default host, so traces are not lost
    # loudly but silently written somewhere else.
    if os.getenv("LANGFUSE_BASE_URL") and not os.getenv("LANGFUSE_HOST"):
        os.environ["LANGFUSE_HOST"] = os.environ["LANGFUSE_BASE_URL"]
    try:
        from langfuse import get_client

        _CLIENT = get_client()
        logger.info("tracing enabled host=%s", os.getenv("LANGFUSE_HOST", "default"))
    except Exception as exc:
        logger.warning("tracing unavailable (%s); continuing untraced", exc)
        _CLIENT = None
    return _CLIENT


def enabled() -> bool:
    """Whether anything would be recorded right now."""
    return bool(getattr(_ACTIVE, "on", False) and _client() is not None)


@contextlib.contextmanager
def agent_trace(name: str, **metadata: Any) -> Iterator[Any]:
    """Open a trace around one agent-initiated task.

    Outside this block nothing is emitted, which is what keeps 15 Hz perception
    out of the dashboard.
    """
    client = _client()
    if client is None:
        yield None
        return

    previous = getattr(_ACTIVE, "on", False)
    _ACTIVE.on = True
    manager: Any = None
    root: Any = None
    try:
        try:
            manager = client.start_as_current_observation(name=name, as_type="agent")
            root = manager.__enter__()
        except Exception as exc:
            logger.warning("tracing failed to start (%s); the run itself is unaffected", exc)
            yield None
            return

        with contextlib.suppress(Exception):
            root.update(metadata=_clean(metadata))
        try:
            yield root
        finally:
            # Never let tracing teardown mask a business exception -- an
            # exception from the caller's body has to reach the eval runner, or
            # an infrastructure failure scores as a valid refusal -- and never
            # suppress it by returning the manager's boolean.
            with contextlib.suppress(Exception):
                manager.__exit__(*sys.exc_info())
    finally:
        _ACTIVE.on = previous
        with contextlib.suppress(Exception):
            client.flush()


@contextlib.contextmanager
def span(name: str, *, as_type: str = "span", **attributes: Any) -> Iterator[Any]:
    """Record one span, if a trace is open. A no-op otherwise.

    Attributes may be supplied late via the yielded handle's ``set`` -- most
    of what is worth recording about a skill is only known once it has run.

    ``as_type="tool"`` renders a skill call as a tool in the UI rather than a
    generic span, which is what it is.
    """
    if not enabled():
        yield _Recorder()
        return
    client = _client()
    manager: Any = None
    try:
        manager = client.start_as_current_observation(name=name, as_type=as_type)
        handle = manager.__enter__()
    except Exception as exc:
        logger.debug("span %s failed (%s)", name, exc)
        yield _Recorder()
        return

    recorder = _Recorder(handle)
    recorder.set(**attributes)
    try:
        yield recorder
    finally:
        # The observation manager must see the body exception for accurate
        # tracing, but its return value or teardown failure must never suppress
        # or replace that business exception.
        with contextlib.suppress(Exception):
            manager.__exit__(*sys.exc_info())


class _Recorder:
    """Attaches attributes to a span, or to nothing when none is open.

    A null handle rather than a second class, so callers need no branches.
    """

    def __init__(self, handle: Any = None) -> None:
        self._handle = handle

    def set(self, **attributes: Any) -> None:
        cleaned = _clean(attributes)
        if self._handle is None or not cleaned:
            return
        with contextlib.suppress(Exception):
            self._handle.update(metadata=cleaned)


def _clean(attributes: Mapping[str, Any]) -> dict[str, Any]:
    """Drop empties, and refuse anything that looks like a secret or an image.

    A denylist is weaker than a schema, but the alternative here is trusting
    every future call site to remember -- and the cost of one leaked key is
    much higher than the cost of an over-eager filter.
    """
    out: dict[str, Any] = {}
    for key, value in attributes.items():
        if value is None or value == "":
            continue
        lowered = key.lower()
        if any(word in lowered for word in ("key", "token", "secret", "password", "authorization")):
            continue
        if isinstance(value, (bytes, bytearray, memoryview)):
            continue
        out[key] = value
    return out


def langchain_handler() -> Any:
    """A LangChain callback that reports to the same trace, or None.

    Prompts, completions, token counts and cost come free from this; only the
    belief-specific attributes have to be added by hand.
    """
    if _client() is None:
        return None
    try:
        from langfuse.langchain import CallbackHandler

        return CallbackHandler()
    except Exception as exc:
        logger.debug("langchain tracing unavailable (%s)", exc)
        return None
