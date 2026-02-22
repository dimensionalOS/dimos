# Copyright 2025-2026 Dimensional Inc.
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

from dataclasses import dataclass, field
import json
import threading
from typing import Any

import zenoh

from dimos.protocol.service.spec import Service
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

_sessions: dict[str, zenoh.Session] = {}
_sessions_lock = threading.Lock()


@dataclass
class ZenohConfig:
    """Configuration for Zenoh service."""

    mode: str = "peer"
    connect: list[str] = field(default_factory=list)
    listen: list[str] = field(default_factory=list)

    @property
    def session_key(self) -> str:
        """Produce a hashable key for singleton session lookup."""
        return f"{self.mode}|{json.dumps(sorted(self.connect))}|{json.dumps(sorted(self.listen))}"


class ZenohService(Service[ZenohConfig]):
    default_config = ZenohConfig

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)

    def start(self) -> None:
        """Start the Zenoh service."""
        key = self.config.session_key
        with _sessions_lock:
            if key not in _sessions:
                config = zenoh.Config()
                config.insert_json5("mode", json.dumps(self.config.mode))
                if self.config.connect:
                    config.insert_json5("connect/endpoints", json.dumps(self.config.connect))
                if self.config.listen:
                    config.insert_json5("listen/endpoints", json.dumps(self.config.listen))
                _sessions[key] = zenoh.open(config)
                logger.info(f"Zenoh service started in {self.config.mode} mode")
        super().start()

    def stop(self) -> None:
        """Stop the Zenoh service."""
        super().stop()

    @property
    def session(self) -> zenoh.Session:
        """Get the Zenoh Session instance for this service's config."""
        key = self.config.session_key
        if key not in _sessions:
            raise RuntimeError("Zenoh session not initialized")
        return _sessions[key]


__all__ = [
    "ZenohConfig",
    "ZenohService",
]
