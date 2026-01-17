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

from dimos.protocol.service.spec import Service


@dataclass
class DDSConfig:
    """Configuration for DDS service."""

    autoconf: bool = True


class DDSService(Service[DDSConfig]):
    """DDS service for in-memory distributed data sharing."""

    default_config = DDSConfig

    def __init__(self, **kwargs) -> None:
        super().__init__(**kwargs)

    def start(self) -> None:
        """Start the DDS service (no-op for in-memory implementation)."""
        pass

    def stop(self) -> None:
        """Stop the DDS service (no-op for in-memory implementation)."""
        pass


def autoconf() -> None:
    """Auto-configure system for DDS."""
    pass


__all__ = [
    "DDSConfig",
    "DDSService",
    "autoconf",
]
