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

"""The gateway's events on zenoh, at `<ns>/dimos/events/<type>` (Desktop's docs/events.md), one JSON object a sample.

`<ns>` is Desktop's namespace. The gateway learns it from, first to last: `--zenoh-namespace`, `DIMOS_ZENOH_NAMESPACE`,
`DIMOS_APP`'s `zenohNamespace`, Desktop's config.yaml `desktop.namespace`, and else Desktop's own default,
`dimos-desktop/<host>-<desktop.port>`. The endpoint to dial: `--zenoh-connect`, `DIMOS_APP`'s `zenohConnect`, then
dimos's GlobalConfig `zenoh_connect` (which reads `ZENOH_CONNECT`; empty = a peer on the local network).
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
import json
import os
import re
import socket
from typing import TYPE_CHECKING, Any

from dimos.gateway import config

if TYPE_CHECKING:
    from dimos.gateway.topic_rates import TopicWatch
    from dimos.protocol.service.zenohservice import ZenohSessionPool

NAMESPACE_ENV = "DIMOS_ZENOH_NAMESPACE"
# Desktop's own default port (its config.rs DEFAULT_PORT)
DESKTOP_DEFAULT_PORT = 5555


def dimos_app() -> dict[str, Any]:
    """The `DIMOS_APP` JSON object Desktop passes its servers, or {}."""
    try:
        value = json.loads(os.environ.get("DIMOS_APP") or "{}")
    except json.JSONDecodeError:
        return {}
    return value if isinstance(value, dict) else {}


def host_chunk(hostname: str) -> str:
    """Lowercased, with anything but a-z 0-9 - turned into - (`Jeffs-MacBook.local` -> `jeffs-macbook-local`)."""
    return re.sub(r"[^a-z0-9-]", "-", hostname.lower())


def check_namespace(namespace: str) -> str:
    """A key expression with no wildcards, `?`, `#` or empty chunks; raises ValueError otherwise."""
    if not namespace or any(chunk == "" for chunk in namespace.split("/")):
        raise ValueError(f"zenoh namespace {namespace!r} is empty or has an empty chunk")
    if re.search(r"[*?#$]", namespace):
        raise ValueError(f"zenoh namespace {namespace!r} has a wildcard, `?`, `#` or `$`")
    return namespace


def desktop_namespace() -> str:
    """config.yaml's `desktop.namespace`, else Desktop's default `dimos-desktop/<host>-<port>`."""
    desktop = config.load_desktop_config().get("desktop")
    desktop = desktop if isinstance(desktop, dict) else {}
    configured = str(desktop.get("namespace") or "").strip()
    if configured:
        return configured
    return f"dimos-desktop/{host_chunk(socket.gethostname())}-{desktop.get('port') or DESKTOP_DEFAULT_PORT}"


def resolve_namespace(given: str | None = None) -> str:
    namespace = (
        given
        or os.environ.get(NAMESPACE_ENV)
        or dimos_app().get("zenohNamespace")
        or desktop_namespace()
    )
    return check_namespace(str(namespace))


def resolve_connect(given: str | None = None) -> list[str]:
    from dimos.core.global_config import global_config

    value = (
        given
        if given is not None
        else dimos_app().get("zenohConnect") or global_config.zenoh_connect
    )
    return [item.strip() for item in str(value or "").split(",") if item.strip()]


def event_key(namespace: str, event: dict[str, Any]) -> str:
    return f"{namespace}/dimos/events/{event['type']}"


@dataclass
class Publisher:
    """Puts each event on `<ns>/dimos/events/<type>`; `put(key, payload)` is the session's (a fake in tests)."""

    namespace: str
    put: Callable[[str, bytes], None]

    def __call__(self, event: dict[str, Any]) -> None:
        self.put(event_key(self.namespace, event), json.dumps(event).encode())

    def under(self, key: str, payload: dict[str, Any]) -> None:
        """`payload` on `<ns>/dimos/<key>` (a job's lines: `jobs/<job>`)."""
        self.put(f"{self.namespace}/dimos/{key}", json.dumps(payload).encode())


def open_publisher(
    namespace: str, connect: list[str], pool: ZenohSessionPool | None = None
) -> Publisher:
    """A publisher on a session from dimos's zenoh pool, configured like every other dimos zenoh session."""
    import zenoh

    from dimos.protocol.service.zenohservice import ZenohConfig, default_session_pool

    session = (pool or default_session_pool).acquire(ZenohConfig(connect=connect))

    def put(key: str, payload: bytes) -> None:
        session.put(key, payload, encoding=zenoh.Encoding.APPLICATION_JSON)

    return Publisher(namespace, put)


def topic_watch(connect: list[str], pool: ZenohSessionPool | None = None) -> TopicWatch:
    """A TopicWatch on a session from dimos's zenoh pool (the publisher's, when it has one)."""
    from dimos.gateway.topic_rates import TopicWatch
    from dimos.protocol.service.zenohservice import ZenohConfig, default_session_pool

    return TopicWatch(lambda: (pool or default_session_pool).acquire(ZenohConfig(connect=connect)))
