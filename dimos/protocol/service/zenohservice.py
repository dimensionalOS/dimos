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

import hashlib
import json
import threading
from typing import Any

from pydantic import Field
import zenoh

from dimos.constants import CACHE_DIR
from dimos.core.global_config import global_config
from dimos.protocol.service.spec import BaseConfig, Service
from dimos.utils.logging_config import setup_logger

zenoh.init_log_from_env_or("warn")

logger = setup_logger()


class ZenohConfig(BaseConfig):
    mode: str = "peer"
    user: str = "dimos"
    password: str = Field(default_factory=lambda: global_config.zenoh_password)
    connect: list[str] = []
    listen: list[str] = []

    @property
    def session_key(self) -> str:
        return f"{self.mode}|{self.user}|{self.password}|{json.dumps(sorted(self.connect))}|{json.dumps(sorted(self.listen))}"


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
                if config.password:
                    zconfig.insert_json5("transport/auth/usrpwd/user", json.dumps(config.user))
                    zconfig.insert_json5(
                        "transport/auth/usrpwd/password", json.dumps(config.password)
                    )
                    zconfig.insert_json5(
                        "transport/auth/usrpwd/dictionary_file",
                        json.dumps(_password_dictionary_file(config.user, config.password)),
                    )
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


# zenoh requires a file for auth
def _password_dictionary_file(user: str, password: str) -> str:
    directory = CACHE_DIR / "zenoh_auth"
    directory.mkdir(parents=True, exist_ok=True)
    digest = hashlib.sha256(f"{user}:{password}".encode()).hexdigest()[:16]
    path = directory / f"{digest}.txt"
    path.write_text(f"{user}:{password}\n")
    path.chmod(0o600)
    return str(path)
