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

try:
    from cyclonedds.domain import DomainParticipant

    DDS_AVAILABLE = True
except ImportError:
    DDS_AVAILABLE = False
    DomainParticipant = None  # type: ignore[assignment, misc]

from dimos.protocol.service.spec import Service
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from cyclonedds.qos import Qos

logger = setup_logger()

_participants: dict[int, DomainParticipant] = {}
_participants_lock = threading.Lock()


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
        domain_id = self.config.domain_id
        with _participants_lock:
            if domain_id not in _participants:
                _participants[domain_id] = DomainParticipant(domain_id)
                logger.info(f"DDS service started with Cyclone DDS domain {domain_id}")
        super().start()

    def stop(self) -> None:
        """Stop the DDS service."""
        super().stop()

    @property
    def participant(self) -> DomainParticipant:
        """Get the DomainParticipant instance for this service's domain."""
        domain_id = self.config.domain_id
        if domain_id not in _participants:
            raise RuntimeError(f"DomainParticipant not initialized for domain {domain_id}")
        return _participants[domain_id]


__all__ = [
    "DDSConfig",
    "DDSService",
]
