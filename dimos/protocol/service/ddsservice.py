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

from dataclasses import dataclass
import threading
from typing import TYPE_CHECKING, Any

from cyclonedds.domain import DomainParticipant

from dimos.protocol.service.spec import Service
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from cyclonedds.qos import Qos

logger = setup_logger()

_participant: DomainParticipant | None = None
_participant_lock = threading.Lock()


@dataclass
class DDSConfig:
    """Configuration for DDS service."""

    domain_id: int = 0
    qos: Qos | None = None


class DDSService(Service[DDSConfig]):
    default_config = DDSConfig

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)

    def start(self) -> None:
        """Start the DDS service."""
        global _participant
        with _participant_lock:
            if _participant is None:
                _participant = DomainParticipant(self.config.domain_id)
                logger.info(f"DDS service started with Cyclone DDS domain {self.config.domain_id}")
        super().start()

    def stop(self) -> None:
        """Stop the DDS service."""
        global _participant
        with _participant_lock:
            if _participant is not None:
                _participant = None
                logger.info("DDS service stopped")
        super().stop()

    @property
    def participant(self) -> DomainParticipant:
        """Get the DomainParticipant instance."""
        if _participant is None:
            raise RuntimeError("DomainParticipant not initialized")
        return _participant


__all__ = [
    "DDSConfig",
    "DDSService",
]
