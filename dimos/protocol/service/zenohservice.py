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
import os
import platform
import socket
import threading
import time
from typing import Any

from pydantic import Field
import zenoh

from dimos.protocol.service.spec import BaseConfig, Service
from dimos.utils.logging_config import setup_logger

zenoh.init_log_from_env_or("warn")

logger = setup_logger()

ZENOH_ROUTER_ENDPOINT_ENV = "DIMOS_ZENOH_ROUTER_ENDPOINT"
ZENOH_LOCAL_ROUTER_ENDPOINT = "tcp/127.0.0.1:7447"
ZENOH_LOCAL_ROUTER_LISTEN = "tcp/[::]:7447"

# Robot-side bridges (e.g. go2web) listen here so a remote dimos can dial in
# when multicast discovery fails. Zenoh's own default port.
ROBOT_ZENOH_PORT = 7447

# Poll interval while waiting for connect endpoints to link.
_CONNECT_POLL_INTERVAL = 0.05

# Interface scouting falls back to when network discovery is off. Zenoh takes an
# interface name, and Darwin spells loopback differently.
LOOPBACK_INTERFACE = "lo0" if platform.system() == "Darwin" else "lo"


def _robot_connect_endpoints() -> list[str]:
    """Return explicit robot endpoints when multicast scouting is insufficient.

    Many APs filter multicast between WiFi clients, so a robot that is
    reachable over TCP may never answer a scout. An IP carrying its own port is
    used as given.
    """
    from dimos.core.global_config import global_config

    if global_config.transport != "zenoh":
        return []
    ips = [global_config.robot_ip or "", *(global_config.robot_ips or "").split(",")]
    out: list[str] = []
    for ip in (x.strip() for x in ips):
        if not ip:
            continue
        endpoint = f"tcp/{ip}" if ":" in ip else f"tcp/{ip}:{ROBOT_ZENOH_PORT}"
        if endpoint not in out:
            out.append(endpoint)
    return out


def _default_mode() -> str:
    return "client" if os.getenv(ZENOH_ROUTER_ENDPOINT_ENV) else "peer"


def _default_connect_endpoints() -> list[str]:
    router_endpoint = os.getenv(ZENOH_ROUTER_ENDPOINT_ENV)
    if router_endpoint:
        return [router_endpoint]
    return _robot_connect_endpoints()


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


def _await_connect_endpoints(
    session: zenoh.Session,
    endpoints: list[str],
    timeout: float,
) -> None:
    pending = {endpoint: endpoint_addresses(endpoint) for endpoint in endpoints}
    if not pending or timeout <= 0:
        return

    deadline = time.monotonic() + timeout
    while pending:
        linked = {str(link.dst).rpartition("/")[2] for link in session.info.links()}
        for endpoint in [item for item, addresses in pending.items() if addresses & linked]:
            logger.debug("Zenoh linked", endpoint=endpoint)
            del pending[endpoint]
        if not pending:
            return
        if time.monotonic() >= deadline:
            logger.warning(
                "Zenoh endpoints not linked; continuing",
                timeout=timeout,
                endpoints=sorted(pending),
            )
            return
        time.sleep(_CONNECT_POLL_INTERVAL)


class ZenohConfig(BaseConfig):
    mode: str = Field(default_factory=_default_mode)
    connect: list[str] = Field(default_factory=_default_connect_endpoints)
    listen: list[str] = Field(default_factory=list)
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
                if config.listen:
                    zconfig.insert_json5("listen/endpoints", json.dumps(config.listen))
                if not config.scouting:
                    # Loopback multicast stays on so sibling worker processes on
                    # this host still discover each other -- cutting scouting
                    # outright leaves them unable to reach one another at all,
                    # since peers don't route each other's traffic.
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


class ZenohRouter:
    def __init__(
        self,
        listen: str = ZENOH_LOCAL_ROUTER_LISTEN,
        connect: list[str] | None = None,
    ) -> None:
        self._listen = listen
        self._connect = _robot_connect_endpoints() if connect is None else list(connect)
        self._session: zenoh.Session | None = None

    def start(self) -> None:
        config = zenoh.Config()
        config.insert_json5("mode", '"router"')
        config.insert_json5("listen/endpoints", json.dumps([self._listen]))
        if self._connect:
            config.insert_json5("connect/endpoints", json.dumps(self._connect))
        if not _default_scouting():
            config.insert_json5("scouting/multicast/interface", json.dumps(LOOPBACK_INTERFACE))
            config.insert_json5("scouting/gossip/enabled", "false")
        try:
            self._session = zenoh.open(config)
            _await_connect_endpoints(
                self._session,
                self._connect,
                _default_connect_timeout(),
            )
            logger.info("Local Zenoh router started", endpoint=self._listen)
        except zenoh.ZError as exc:
            if "Address already in use" not in str(exc):
                raise
            logger.info("Using existing local Zenoh router", endpoint=self._listen)

    def stop(self) -> None:
        if self._session is not None:
            self._session.close()
            self._session = None


class ZenohService(Service):
    config: ZenohConfig

    def __init__(self, *, session_pool: ZenohSessionPool | None = None, **kwargs: Any) -> None:
        # session_pool is keyword-only so it never reaches the pydantic config
        # (which is extra="forbid"). It rides the same **kwargs path as mode/connect/listen.
        super().__init__(**kwargs)
        self._session_pool = session_pool or default_session_pool
        self._session: zenoh.Session | None = None

    def start(self) -> None:
        endpoint = os.getenv(ZENOH_ROUTER_ENDPOINT_ENV)
        if endpoint and not self.config.model_fields_set:
            self.config = ZenohConfig(mode="client", connect=[endpoint])
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
        _await_connect_endpoints(
            session,
            self.config.connect,
            self.config.connect_timeout,
        )

    @property
    def session(self) -> zenoh.Session:
        if self._session is None:
            raise RuntimeError("Zenoh session not initialized. Call start() first.")
        return self._session
