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

"""Canonical scalar joint-coordinate semantics for manipulation planning."""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass
from enum import StrEnum
from itertools import pairwise
import math
from typing import Annotated, Literal

import numpy as np
from numpy.typing import NDArray
from pydantic import ConfigDict, Field, model_validator
from pydantic.dataclasses import dataclass as pydantic_dataclass
from typing_extensions import Self

from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.utils.trigonometry import angle_diff


class CoordinateTopology(StrEnum):
    """Topology of one public scalar joint coordinate."""

    INTERVAL = "interval"
    LINE = "line"
    CIRCLE = "circle"


_JOINT_COORDINATE_CONFIG = ConfigDict(extra="forbid", validate_default=True)
_NonEmptyString = Annotated[str, Field(min_length=1)]
_FiniteFloat = Annotated[float, Field(allow_inf_nan=False)]
_PositiveFiniteFloat = Annotated[float, Field(gt=0.0, allow_inf_nan=False)]
JointMechanismType = Literal["continuous", "prismatic", "revolute"]


@pydantic_dataclass(frozen=True, config=_JOINT_COORDINATE_CONFIG)
class JointCoordinate:
    """Compiled semantics and motion limits for one scalar URDF joint."""

    name: _NonEmptyString
    mechanism_type: JointMechanismType
    topology: CoordinateTopology
    lower: _FiniteFloat | None
    upper: _FiniteFloat | None
    max_velocity: _PositiveFiniteFloat
    max_acceleration: _PositiveFiniteFloat

    @model_validator(mode="after")
    def _validate_topology_bounds(self) -> Self:
        if self.topology is CoordinateTopology.INTERVAL:
            if self.lower is None or self.upper is None:
                raise ValueError(f"Interval joint '{self.name}' requires two position limits")
            if self.lower > self.upper:
                raise ValueError(f"Interval joint '{self.name}' has inverted position limits")
        elif self.lower is not None or self.upper is not None:
            raise ValueError(f"{self.topology.value} joint '{self.name}' cannot have bounds")
        return self


@dataclass(frozen=True)
class JointSpace:
    """Ordered joint metadata and topology-aware numerical operations.

    Normalize external inputs before numerical operations. Internal arrays use
    ``names`` order and may carry lifted (unwrapped) circular positions.
    """

    coordinates: tuple[JointCoordinate, ...]

    def __post_init__(self) -> None:
        names = self.names
        if len(names) != len(set(names)):
            raise ValueError("Joint space contains duplicate coordinate names")

    @property
    def names(self) -> tuple[str, ...]:
        return tuple(coordinate.name for coordinate in self.coordinates)

    @property
    def velocity_limits(self) -> tuple[float, ...]:
        return tuple(coordinate.max_velocity for coordinate in self.coordinates)

    @property
    def acceleration_limits(self) -> tuple[float, ...]:
        return tuple(coordinate.max_acceleration for coordinate in self.coordinates)

    @property
    def is_interval_only(self) -> bool:
        return all(
            coordinate.topology is CoordinateTopology.INTERVAL for coordinate in self.coordinates
        )

    def coordinate(self, name: str) -> JointCoordinate:
        try:
            return self.coordinates[self.names.index(name)]
        except ValueError as exc:
            raise KeyError(f"Unknown joint coordinate: {name}") from exc

    def select(self, names: tuple[str, ...] | list[str]) -> JointSpace:
        return JointSpace(tuple(self.coordinate(name) for name in names))

    def normalize_positions(
        self, positions: tuple[float, ...] | list[float] | NDArray[np.float64]
    ) -> NDArray[np.float64]:
        """Validate an ordered input vector and return a normalized copy."""
        values = np.asarray(positions, dtype=np.float64)
        self._validate_vector(values, "configuration")
        normalized = values.copy()
        for index, coordinate in enumerate(self.coordinates):
            value = normalized[index]
            if coordinate.topology is CoordinateTopology.CIRCLE:
                normalized[index] = angle_diff(float(value), 0.0)
            elif coordinate.topology is CoordinateTopology.INTERVAL:
                assert coordinate.lower is not None and coordinate.upper is not None
                if value < coordinate.lower or value > coordinate.upper:
                    raise ValueError(
                        f"Joint '{coordinate.name}' position {value} is outside "
                        f"[{coordinate.lower}, {coordinate.upper}]"
                    )
        return normalized

    def from_joint_state(self, state: JointState) -> NDArray[np.float64]:
        """Extract and validate this space's coordinates from a named state."""
        if not state.name:
            return self.normalize_positions(state.position)
        if len(state.name) != len(set(state.name)):
            raise ValueError("Joint state contains duplicate coordinate names")
        positions = dict(zip(state.name, state.position, strict=True))
        missing = [name for name in self.names if name not in positions]
        if missing:
            raise ValueError(f"Joint state is missing coordinates: {missing}")
        return self.normalize_positions([positions[name] for name in self.names])

    def delta(self, start: NDArray[np.float64], end: NDArray[np.float64]) -> NDArray[np.float64]:
        """Return shortest circular displacements in this space's order."""
        values = end - start
        for index, coordinate in enumerate(self.coordinates):
            if coordinate.topology is CoordinateTopology.CIRCLE:
                values[index] = angle_diff(float(values[index]), 0.0)
        return values

    def interpolate(
        self, start: NDArray[np.float64], end: NDArray[np.float64], fraction: float
    ) -> NDArray[np.float64]:
        """Interpolate continuously from start without wrapping the result."""
        if not math.isfinite(fraction):
            raise ValueError("Interpolation fraction must be finite")
        return start + fraction * self.delta(start, end)

    def distance(self, start: NDArray[np.float64], end: NDArray[np.float64]) -> float:
        """Return velocity-normalized L2 distance."""
        delta = self.delta(start, end)
        return float(np.linalg.norm(delta / np.asarray(self.velocity_limits)))

    def finite_sampling_domain(
        self,
        start: NDArray[np.float64],
        goal: NDArray[np.float64],
        margin: float,
    ) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
        if not math.isfinite(margin) or margin <= 0.0:
            raise ValueError("Planning-domain margin must be positive and finite")
        lower = np.empty(len(self.coordinates), dtype=np.float64)
        upper = np.empty(len(self.coordinates), dtype=np.float64)
        for index, coordinate in enumerate(self.coordinates):
            if coordinate.topology is CoordinateTopology.INTERVAL:
                assert coordinate.lower is not None and coordinate.upper is not None
                lower[index], upper[index] = coordinate.lower, coordinate.upper
            elif coordinate.topology is CoordinateTopology.LINE:
                lower[index] = min(start[index], goal[index]) - margin
                upper[index] = max(start[index], goal[index]) + margin
            else:
                lower[index], upper[index] = -math.pi, math.pi
        return lower, upper

    def lifted_positions(
        self, configurations: Sequence[NDArray[np.float64]]
    ) -> list[NDArray[np.float64]]:
        if not configurations:
            return []
        lifted = [configurations[0].copy()]
        for configuration in configurations[1:]:
            previous = lifted[-1]
            current = configuration.copy()
            delta = self.delta(previous, current)
            for index, coordinate in enumerate(self.coordinates):
                if coordinate.topology is CoordinateTopology.CIRCLE:
                    current[index] = previous[index] + delta[index]
            lifted.append(current)
        return lifted

    def path_length(self, configurations: Sequence[NDArray[np.float64]]) -> float:
        return sum(self.distance(start, end) for start, end in pairwise(configurations))

    def position_limits(self) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
        lower = np.asarray(
            [
                coordinate.lower if coordinate.topology is CoordinateTopology.INTERVAL else -np.inf
                for coordinate in self.coordinates
            ],
            dtype=np.float64,
        )
        upper = np.asarray(
            [
                coordinate.upper if coordinate.topology is CoordinateTopology.INTERVAL else np.inf
                for coordinate in self.coordinates
            ],
            dtype=np.float64,
        )
        return lower, upper

    def _validate_vector(self, values: NDArray[np.float64], label: str) -> None:
        if values.shape != (len(self.coordinates),):
            raise ValueError(
                f"Joint {label} has shape {values.shape}, expected {(len(self.coordinates),)}"
            )
        if not np.isfinite(values).all():
            raise ValueError(f"Joint {label} must contain only finite values")
