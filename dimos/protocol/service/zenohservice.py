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

from __future__ import annotations

import json
import platform
import socket
import threading
import time
from typing import TYPE_CHECKING, Any

from pydantic import Field
import zenoh

if TYPE_CHECKING:
    from collections.abc import Sequence

from dimos.protocol.service.spec import BaseConfig, Service
from dimos.utils.logging_config import setup_logger

zenoh.init_log_from_env_or("warn")

logger = setup_logger()

# Robot-side bridges (e.g. go2web) listen here so a remote dimos can dial in
# when multicast discovery fails. Zenoh's own default port.
ROBOT_ZENOH_PORT = 7447

# Poll interval while waiting for connect endpoints to link.
_CONNECT_POLL_INTERVAL = 0.05

# Interface scouting falls back to when network discovery is off. Zenoh takes an
# interface name, and Darwin spells loopback differently.
LOOPBACK_INTERFACE = "lo0" if platform.system() == "Darwin" else "lo"

# Explicit loopback endpoints wiring the coordinator and its workers together.
# Sibling discovery cannot rely on multicast scouting: macOS never delivers
# multicast pinned to lo0, so with scouting off the processes would never find
# each other. Instead each worker listens on a coordinator-allocated port and
# dials the workers spawned before it, and coordinator-side sessions dial every
# live worker. Process-wide; consumed by the ZenohConfig default factories.
_mesh_listen: str | None = None
_mesh_connect: tuple[str, ...] = ()


def allocate_mesh_endpoint() -> str:
    """Reserve a free loopback port and return it as a ``tcp/`` locator.

    The probe socket closes before zenoh binds the port at session open; a
    collision in that window fails the session open loudly rather than
    silently dropping traffic.
    """
    with socket.socket() as sock:
        sock.bind(("127.0.0.1", 0))
        port: int = sock.getsockname()[1]
    return f"tcp/127.0.0.1:{port}"


def configure_zenoh_mesh(listen: str | None, connect: Sequence[str]) -> None:
    """Set the loopback endpoints this process's future zenoh sessions mesh over.

    Workers pass their own ``listen`` endpoint plus the endpoints of the
    workers spawned before them; the coordinator passes ``None`` and dials all
    live workers. Already-open sessions are not reconfigured: the mesh stays
    complete because every session dials whatever existed when it opened and
    is dialled by everything newer.
    """
    global _mesh_listen, _mesh_connect
    _mesh_listen = listen
    _mesh_connect = tuple(connect)


def _default_connect_endpoints() -> list[str]:
    """Dial known robots and mesh siblings instead of trusting multicast scouting.

    Many APs filter multicast between WiFi clients, so a robot that is
    perfectly reachable over TCP never answers a scout. When the session is
    zenoh-transported and a robot IP is configured, it becomes an explicit
    endpoint; scouting stays on for everything else. An IP carrying its own
    ``:port`` is used as given.
    """
    from dimos.core.global_config import global_config

    out: list[str] = []
    if global_config.transport == "zenoh":
        ips = [global_config.robot_ip or "", *(global_config.robot_ips or "").split(",")]
        for ip in (x.strip() for x in ips):
            if not ip:
                continue
            endpoint = f"tcp/{ip}" if ":" in ip else f"tcp/{ip}:{ROBOT_ZENOH_PORT}"
            if endpoint not in out:
                out.append(endpoint)
    out.extend(endpoint for endpoint in _mesh_connect if endpoint not in out)
    return out


def _default_listen_endpoints() -> list[str]:
    return [_mesh_listen] if _mesh_listen else []


def _default_scouting() -> bool:
    from dimos.core.global_config import global_config

    return global_config.zenoh_scouting


def _default_connect_timeout() -> float:
    from dimos.core.global_config import global_config

    return global_config.zenoh_connect_timeout


def endpoint_addresses(endpoint: str) -> set[str]:
    """Resolve a locator to the ``host:port`` forms a live link may report.

    ``tcp/go2:7447`` is dialled by name but the established link reports the
    resolved address, so the name alone never matches.
    """
    _, _, address = endpoint.rpartition("/")
    host, _, port = address.rpartition(":")
    if not host:
        return {address}
    out = {f"{host}:{port}"}
    try:
        for info in socket.getaddrinfo(host, None):
            out.add(f"{info[4][0]}:{port}")
    except OSError:
        pass
    return out


class ZenohConfig(BaseConfig):
    mode: str = "peer"
    connect: list[str] = Field(default_factory=_default_connect_endpoints)
    listen: list[str] = Field(default_factory=_default_listen_endpoints)
    # Discover peers across the network. Off keeps discovery on loopback.
    scouting: bool = Field(default_factory=_default_scouting)
    # Seconds to block in start() waiting for `connect` endpoints to link.
    connect_timeout: float = Field(default_factory=_default_connect_timeout)

    @property
    def session_key(self) -> str:
        return (
            f"{self.mode}|{json.dumps(sorted(self.connect))}"
            f"|{json.dumps(sorted(self.listen))}|{self.scouting}"
        )


class ZenohSessionPool:
    def __init__(self) -> None:
        self._sessions: dict[str, zenoh.Session] = {}
        self._lock = threading.Lock()

    def acquire(self, config: ZenohConfig) -> zenoh.Session:
        """Open a session for this config, or return the existing shared one."""
        key = config.session_key
        with self._lock:
            if key not in self._sessions:
                zconfig = zenoh.Config()
                zconfig.insert_json5("mode", json.dumps(config.mode))
                if config.connect:
                    zconfig.insert_json5("connect/endpoints", json.dumps(config.connect))
                    # A dial can race a mesh sibling's listener still coming
                    # up; zenoh's default 1s initial retry would hold back the
                    # first messages. Retry fast, keep the default backoff cap.
                    # (Whole object: zenoh rejects inserts at the leaf keys.)
                    zconfig.insert_json5(
                        "connect/retry",
                        json.dumps(
                            {
                                "period_init_ms": 100,
                                "period_max_ms": 4000,
                                "period_increase_factor": 2,
                            }
                        ),
                    )
                if config.listen:
                    zconfig.insert_json5("listen/endpoints", json.dumps(config.listen))
                if not config.scouting:
                    # The coordinator and its workers reach each other over the
                    # explicit mesh endpoints above. Loopback multicast stays on
                    # for other same-host processes (CLIs attaching to a
                    # daemon); note macOS never delivers it on lo0, so those
                    # need network scouting or explicit endpoints there.
                    zconfig.insert_json5(
                        "scouting/multicast/interface", json.dumps(LOOPBACK_INTERFACE)
                    )
                    zconfig.insert_json5("scouting/gossip/enabled", "false")
                self._sessions[key] = zenoh.open(zconfig)
                logger.debug(f"Zenoh session opened in {config.mode} mode")
            return self._sessions[key]

    def close_all(self) -> None:
        """Close every pooled session and empty the pool."""
        with self._lock:
            for session in self._sessions.values():
                session.close()
            self._sessions.clear()


# Process-default pool used by production code. Constructing it opens no sessions.
default_session_pool = ZenohSessionPool()


class ZenohService(Service):
    config: ZenohConfig

    def __init__(self, *, session_pool: ZenohSessionPool | None = None, **kwargs: Any) -> None:
        # session_pool is keyword-only so it never reaches the pydantic config
        # (which is extra="forbid"). It rides the same **kwargs path as mode/connect/listen.
        super().__init__(**kwargs)
        self._session_pool = session_pool or default_session_pool
        self._session: zenoh.Session | None = None

    def start(self) -> None:
        self._session = self._session_pool.acquire(self.config)
        self._await_connect(self._session)
        super().start()

    def _await_connect(self, session: zenoh.Session) -> None:
        """Block until every configured connect endpoint has an established link.

        Opening a zenoh session returns before its endpoints are dialled, so
        without this a blueprint starts publishing into a session that has
        nowhere to send yet and the first messages are simply lost. LCM blocks
        in start() for the same reason.

        Unreachable endpoints are a warning, not an error: one robot being down
        should not stop the rest of the graph from coming up.
        """
        # Mesh endpoints are excluded: a sibling's listener only comes up once
        # its own first module deploys, zenoh keeps dialling in the background,
        # and the RPC retry loop rides out the gap. Blocking every session
        # start on them would stall deploys instead.
        pending = {
            ep: endpoint_addresses(ep) for ep in self.config.connect if ep not in _mesh_connect
        }
        if not pending or self.config.connect_timeout <= 0:
            return
        deadline = time.monotonic() + self.config.connect_timeout
        while pending:
            linked = {str(link.dst).rpartition("/")[2] for link in session.info.links()}
            for endpoint in [e for e, addrs in pending.items() if addrs & linked]:
                logger.debug(f"Zenoh linked {endpoint}")
                del pending[endpoint]
            if not pending:
                return
            if time.monotonic() >= deadline:
                logger.warning(
                    f"Zenoh endpoints not linked after {self.config.connect_timeout}s: "
                    f"{sorted(pending)} - continuing, published messages may be dropped"
                )
                return
            time.sleep(_CONNECT_POLL_INTERVAL)

    @property
    def session(self) -> zenoh.Session:
        if self._session is None:
            raise RuntimeError("Zenoh session not initialized. Call start() first.")
        return self._session
