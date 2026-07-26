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
import threading
from typing import Any

from pydantic import Field
import zenoh

from dimos.protocol.service.spec import BaseConfig, Service
from dimos.utils.logging_config import setup_logger

zenoh.init_log_from_env_or("warn")

logger = setup_logger()

# Robot-side bridges (e.g. go2web) listen here so a remote dimos can dial in
# when multicast discovery fails. Zenoh's own default port.
ROBOT_ZENOH_PORT = 7447


def _default_connect_endpoints() -> list[str]:
    """Dial known robots directly instead of trusting multicast scouting.

    Many APs filter multicast between WiFi clients, so a robot that is
    perfectly reachable over TCP never answers a scout. When the session is
    zenoh-transported and a robot IP is configured, it becomes an explicit
    endpoint; scouting stays on for everything else. An IP carrying its own
    ``:port`` is used as given.
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


class ZenohConfig(BaseConfig):
    mode: str = "peer"
    connect: list[str] = Field(default_factory=_default_connect_endpoints)
    listen: list[str] = []

    @property
    def session_key(self) -> str:
        return f"{self.mode}|{json.dumps(sorted(self.connect))}|{json.dumps(sorted(self.listen))}"


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
        super().start()

    @property
    def session(self) -> zenoh.Session:
        if self._session is None:
            raise RuntimeError("Zenoh session not initialized. Call start() first.")
        return self._session
