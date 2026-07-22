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
import sys
import threading
from typing import Any

import zenoh

from dimos.protocol.service.spec import BaseConfig, Service
from dimos.utils.logging_config import setup_logger

zenoh.init_log_from_env_or("warn")

logger = setup_logger()

LOOPBACK_INTERFACE = "lo0" if sys.platform == "darwin" else "lo"
LOCALHOST_LISTEN_ENDPOINT = "tcp/127.0.0.1:0"


class ZenohConfig(BaseConfig):
    mode: str = "peer"
    connect: list[str] = []
    listen: list[str] = []
    local_only: bool = True

    @property
    def session_key(self) -> str:
        return f"{self.mode}|{self.local_only}|{json.dumps(sorted(self.connect))}|{json.dumps(sorted(self.listen))}"


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
                if config.local_only:
                    zconfig.insert_json5(
                        "scouting/multicast/interface", json.dumps(LOOPBACK_INTERFACE)
                    )
                listen = config.listen or ([LOCALHOST_LISTEN_ENDPOINT] if config.local_only else [])
                if config.connect:
                    zconfig.insert_json5("connect/endpoints", json.dumps(config.connect))
                if listen:
                    zconfig.insert_json5("listen/endpoints", json.dumps(listen))
                self._sessions[key] = zenoh.open(zconfig)
                logger.debug(
                    f"Zenoh session opened in {config.mode} mode (local_only={config.local_only})"
                )
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
