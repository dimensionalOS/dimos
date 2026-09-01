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

from dataclasses import dataclass
from enum import StrEnum
from itertools import pairwise
import math

import numpy as np
from numpy.typing import NDArray

from dimos.msgs.sensor_msgs.JointState import JointState


class CoordinateTopology(StrEnum):
    """Topology of one public scalar joint coordinate."""

    INTERVAL = "interval"
    LINE = "line"
    CIRCLE = "circle"


@dataclass(frozen=True)
class JointCoordinate:
    """Compiled semantics and motion limits for one scalar URDF joint."""

    name: str
    mechanism_type: str
    topology: CoordinateTopology
    lower: float | None
    upper: float | None
    max_velocity: float
    max_acceleration: float

    def __post_init__(self) -> None:
        if not self.name:
            raise ValueError("Joint coordinate name must be non-empty")
        if not math.isfinite(self.max_velocity) or self.max_velocity <= 0.0:
            raise ValueError(f"Joint '{self.name}' velocity limit must be positive and finite")
        if not math.isfinite(self.max_acceleration) or self.max_acceleration <= 0.0:
            raise ValueError(f"Joint '{self.name}' acceleration limit must be positive and finite")
        if self.topology is CoordinateTopology.INTERVAL:
            if self.lower is None or self.upper is None:
                raise ValueError(f"Interval joint '{self.name}' requires two position limits")
            if not math.isfinite(self.lower) or not math.isfinite(self.upper):
                raise ValueError(f"Interval joint '{self.name}' limits must be finite")
            if self.lower > self.upper:
                raise ValueError(f"Interval joint '{self.name}' has inverted position limits")
        elif self.lower is not None or self.upper is not None:
            raise ValueError(f"{self.topology.value} joint '{self.name}' cannot have bounds")


@dataclass(frozen=True)
class JointConfiguration:
    """Immutable normalized point in one named joint space."""

    names: tuple[str, ...]
    positions: tuple[float, ...]

    def __post_init__(self) -> None:
        if len(self.names) != len(self.positions):
            raise ValueError("Joint configuration names and positions must have equal length")

    def as_array(self) -> NDArray[np.float64]:
        return np.asarray(self.positions, dtype=np.float64)


@dataclass(frozen=True)
class JointTangent:
    """Immutable tangent vector in one named joint space."""

    names: tuple[str, ...]
    values: tuple[float, ...]

    def __post_init__(self) -> None:
        if len(self.names) != len(self.values):
            raise ValueError("Joint tangent names and values must have equal length")

    def as_array(self) -> NDArray[np.float64]:
        return np.asarray(self.values, dtype=np.float64)


@dataclass(frozen=True)
class JointSpace:
    """Ordered product of canonical scalar joint coordinates."""

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

    def configuration(
        self, positions: tuple[float, ...] | list[float] | NDArray[np.float64]
    ) -> JointConfiguration:
        values = np.asarray(positions, dtype=np.float64)
        self._validate_vector(values, "configuration")
        normalized = values.copy()
        for index, coordinate in enumerate(self.coordinates):
            value = normalized[index]
            if coordinate.topology is CoordinateTopology.CIRCLE:
                normalized[index] = _wrap_angle(value)
            elif coordinate.topology is CoordinateTopology.INTERVAL:
                assert coordinate.lower is not None and coordinate.upper is not None
                if value < coordinate.lower or value > coordinate.upper:
                    raise ValueError(
                        f"Joint '{coordinate.name}' position {value} is outside "
                        f"[{coordinate.lower}, {coordinate.upper}]"
                    )
        return JointConfiguration(self.names, tuple(float(value) for value in normalized))

    def from_joint_state(self, state: JointState) -> JointConfiguration:
        if not state.name:
            return self.configuration(state.position)
        positions = dict(zip(state.name, state.position, strict=True))
        missing = [name for name in self.names if name not in positions]
        if missing:
            raise ValueError(f"Joint state is missing coordinates: {missing}")
        return self.configuration([positions[name] for name in self.names])

    def to_joint_state(self, configuration: JointConfiguration) -> JointState:
        self._validate_names(configuration.names)
        return JointState(name=list(self.names), position=list(configuration.positions))

    def delta(self, start: JointConfiguration, end: JointConfiguration) -> JointTangent:
        self._validate_names(start.names)
        self._validate_names(end.names)
        values = end.as_array() - start.as_array()
        for index, coordinate in enumerate(self.coordinates):
            if coordinate.topology is CoordinateTopology.CIRCLE:
                values[index] = _wrap_angle(values[index])
        return JointTangent(self.names, tuple(float(value) for value in values))

    def integrate(self, start: JointConfiguration, tangent: JointTangent) -> JointConfiguration:
        self._validate_names(start.names)
        self._validate_names(tangent.names)
        return self.configuration(start.as_array() + tangent.as_array())

    def interpolate(
        self, start: JointConfiguration, end: JointConfiguration, fraction: float
    ) -> JointConfiguration:
        if not math.isfinite(fraction):
            raise ValueError("Interpolation fraction must be finite")
        tangent = self.delta(start, end)
        return self.integrate(
            start,
            JointTangent(self.names, tuple(fraction * value for value in tangent.values)),
        )

    def distance(self, start: JointConfiguration, end: JointConfiguration) -> float:
        delta = self.delta(start, end).as_array()
        return float(np.linalg.norm(delta / np.asarray(self.velocity_limits)))

    def finite_sampling_domain(
        self,
        start: JointConfiguration,
        goal: JointConfiguration,
        margin: float,
    ) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
        if not math.isfinite(margin) or margin <= 0.0:
            raise ValueError("Planning-domain margin must be positive and finite")
        self._validate_names(start.names)
        self._validate_names(goal.names)
        lower = np.empty(len(self.coordinates), dtype=np.float64)
        upper = np.empty(len(self.coordinates), dtype=np.float64)
        for index, coordinate in enumerate(self.coordinates):
            if coordinate.topology is CoordinateTopology.INTERVAL:
                assert coordinate.lower is not None and coordinate.upper is not None
                lower[index], upper[index] = coordinate.lower, coordinate.upper
            elif coordinate.topology is CoordinateTopology.LINE:
                lower[index] = min(start.positions[index], goal.positions[index]) - margin
                upper[index] = max(start.positions[index], goal.positions[index]) + margin
            else:
                lower[index], upper[index] = -math.pi, math.pi
        return lower, upper

    def lifted_positions(
        self, configurations: list[JointConfiguration] | tuple[JointConfiguration, ...]
    ) -> list[tuple[float, ...]]:
        if not configurations:
            return []
        for configuration in configurations:
            self._validate_names(configuration.names)
        lifted = [configurations[0].positions]
        previous = np.asarray(lifted[0], dtype=np.float64)
        circle_indices = [
            index
            for index, coordinate in enumerate(self.coordinates)
            if coordinate.topology is CoordinateTopology.CIRCLE
        ]
        for configuration in configurations[1:]:
            current = np.asarray(configuration.positions, dtype=np.float64).copy()
            for index in circle_indices:
                current[index] = previous[index] + _wrap_angle(current[index] - previous[index])
            lifted.append(tuple(float(value) for value in current))
            previous = current
        return lifted

    def path_length(self, configurations: list[JointConfiguration]) -> float:
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

    def _validate_names(self, names: tuple[str, ...]) -> None:
        if names != self.names:
            raise ValueError(f"Expected joint coordinates {self.names}, got {names}")

    def _validate_vector(self, values: NDArray[np.float64], label: str) -> None:
        if values.shape != (len(self.coordinates),):
            raise ValueError(
                f"Joint {label} has shape {values.shape}, expected {(len(self.coordinates),)}"
            )
        if not np.isfinite(values).all():
            raise ValueError(f"Joint {label} must contain only finite values")


def _wrap_angle(value: float) -> float:
    return (float(value) + math.pi) % (2.0 * math.pi) - math.pi
