# Copyright 2025 Dimensional Inc.
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

from abc import ABC, abstractmethod


class SimulatorBase(ABC):
    """Base class for simulators."""

    @abstractmethod
    def __init__(
        self,
        headless: bool = True,
        open_usd: str | None = None,  # Keep for Isaac compatibility
        entities: list[dict[str, str | dict]] | None = None,  # type: ignore[type-arg]  # Add for Genesis
    ) -> None:
        """Initialize the simulator.

        Args:
            headless: Whether to run without visualization
            open_usd: Path to USD file (for Isaac)
            entities: List of entity configurations (for Genesis)
        """
        self.headless = headless
        self.open_usd = open_usd
        self.stage = None

    @abstractmethod
    def get_stage(self):  # type: ignore[no-untyped-def]
        """Get the current stage/scene."""
        pass

    @abstractmethod
    def close(self):  # type: ignore[no-untyped-def]
        """Close the simulation."""
        pass
