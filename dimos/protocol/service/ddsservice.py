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
from typing import Any

from cyclonedds.domain import DomainParticipant

from dimos.protocol.service.spec import Service
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


@dataclass
class DDSConfig:
    """Configuration for DDS service."""

    domain_id: int = 0
    autoconf: bool = True
    participant: DomainParticipant | None = None


class DDSService(Service[DDSConfig]):
    default_config = DDSConfig

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._participant_lock = threading.Lock()
        self._started = False
        # Support passing an existing DomainParticipant
        self.participant: DomainParticipant | None = self.config.participant

    def __getstate__(self) -> dict[str, Any]:
        """Exclude unpicklable runtime attributes when serializing."""
        state = self.__dict__.copy()
        # Remove unpicklable attributes
        state.pop("participant", None)
        state.pop("_participant_lock", None)
        return state

    def __setstate__(self, state: dict[str, Any]) -> None:
        """Restore object from pickled state."""
        self.__dict__.update(state)
        # Reinitialize runtime attributes
        self.participant = None
        self._participant_lock = threading.Lock()
        self._started = False

    def start(self) -> None:
        """Start the DDS service."""
        if self._started:
            return

        # Use provided participant or create new one
        with self._participant_lock:
            if self.participant is None:
                self.participant = self.config.participant or DomainParticipant(
                    self.config.domain_id
                )
                logger.info(f"DDS service started with Cyclone DDS domain {self.config.domain_id}")

        self._started = True

    def stop(self) -> None:
        """Stop the DDS service."""
        if not self._started:
            return

        with self._participant_lock:
            # Clean up participant if we created it
            if self.participant is not None and not self.config.participant:
                try:
                    self.participant.close()
                    logger.info("DDS participant closed")
                except Exception as e:
                    logger.warning(f"Error closing DDS participant: {e}")
                finally:
                    self.participant = None

        self._started = False


__all__ = ["DDSConfig", "DDSService"]
