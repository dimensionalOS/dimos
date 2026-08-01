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

"""Frozen provisional configuration for v1 spatial benchmark generation.

This configuration is provisional before the first corpus release.  Any change
to its semantics after release must use a new major configuration version.
"""

from __future__ import annotations

import hashlib
import math
from typing import Literal

from pydantic import Field, NonNegativeFloat, PositiveFloat, model_validator

from dimos.benchmark.spatial.models import MapVariant, SpatialModel


class SquareFootprintConfig(SpatialModel):
    """Square robot footprint and conservative collision clearance in metres."""

    side_length_m: PositiveFloat
    safety_margin_m: NonNegativeFloat


class WorldFrameConfig(SpatialModel):
    """Metric world-frame convention used by geometry, trajectories, and maps."""

    space: Literal["metric"] = "metric"
    units: Literal["m"] = "m"
    handedness: Literal["right-handed"] = "right-handed"
    gravity_axis: Literal["+z"] = "+z"
    x_axis: Literal["forward"] = "forward"
    y_axis: Literal["left"] = "left"
    yaw_direction: Literal["counterclockwise"] = "counterclockwise"
    yaw_units: Literal["rad"] = "rad"


class VoxelMapperConfig(SpatialModel):
    """Pinned portable settings passed to :class:`VoxelGridMapper`."""

    voxel_size: PositiveFloat
    block_count: int = Field(gt=0)
    device: Literal["CPU:0"] = "CPU:0"
    carve_columns: Literal[True] = True
    frame_id: Literal["world"] = "world"
    emit_every: int = Field(gt=0)


class LidarNoiseProfile(SpatialModel):
    """Deterministic pre-mapping lidar and planar pose perturbation settings."""

    name: MapVariant
    max_range_m: PositiveFloat
    angular_resolution_deg: PositiveFloat
    range_sigma_m: NonNegativeFloat
    dropout_probability: float = Field(ge=0.0, le=1.0)
    coverage_probability: float = Field(gt=0.0, le=1.0)
    planar_drift_sigma_m: NonNegativeFloat
    yaw_drift_sigma_rad: NonNegativeFloat
    drift_correlation: float = Field(ge=0.0, lt=1.0)

    @model_validator(mode="after")
    def validate_clean_profile(self) -> LidarNoiseProfile:
        """Require the clean control to contain no stochastic perturbation."""

        if self.name is MapVariant.CLEAN and (
            self.range_sigma_m != 0.0
            or self.dropout_probability != 0.0
            or self.coverage_probability != 1.0
            or self.planar_drift_sigma_m != 0.0
            or self.yaw_drift_sigma_rad != 0.0
            or self.drift_correlation != 0.0
        ):
            raise ValueError(
                "the clean profile must not contain stochastic noise, dropout, or drift"
            )
        return self


class GeometryToleranceConfig(SpatialModel):
    """Reject ambiguous geometry and bound continuous sweep discretization error."""

    collision_uncertainty_margin_m: NonNegativeFloat
    opening_uncertainty_margin_m: NonNegativeFloat
    room_boundary_uncertainty_margin_m: NonNegativeFloat
    translation_sweep_step_m: PositiveFloat
    rotation_sweep_step_rad: PositiveFloat
    rotation_refinement_limit: int = Field(default=12, gt=0)


class SeedPolicyConfig(SpatialModel):
    """Stable seed derivation policy for stochastic map-generation profiles."""

    algorithm: Literal["sha256"] = "sha256"
    seed_bits: Literal[64] = 64
    inputs: tuple[Literal["profile-name", "opaque-id"], ...] = ("profile-name", "opaque-id")


class SpatialBenchmarkConfig(SpatialModel):
    """Complete frozen v1 generator policy for the static spatial benchmark."""

    config_version: Literal["1.0"] = "1.0"
    status: Literal["provisional-pre-release"] = "provisional-pre-release"
    footprint: SquareFootprintConfig
    world_frame: WorldFrameConfig
    mapper: VoxelMapperConfig
    lidar_profiles: tuple[LidarNoiseProfile, ...] = Field(min_length=3, max_length=3)
    geometry_tolerances: GeometryToleranceConfig
    seed_policy: SeedPolicyConfig

    @model_validator(mode="after")
    def validate_profiles(self) -> SpatialBenchmarkConfig:
        """Require exactly one explicitly named profile for every map variant."""

        expected = set(MapVariant)
        names = {profile.name for profile in self.lidar_profiles}
        if names != expected:
            raise ValueError(
                "lidar_profiles must contain clean, noisy-01, and noisy-02 exactly once"
            )
        if len(names) != len(self.lidar_profiles):
            raise ValueError("lidar_profiles must not contain duplicate variants")
        return self


def derive_v1_seed(profile_name: MapVariant, *opaque_inputs: str) -> int:
    """Derive a reproducible integer seed from a profile name and opaque stable IDs.

    Inputs are length-delimited before hashing so distinct input sequences cannot
    collide through concatenation.  This deliberately avoids Python's randomized
    ``hash()`` implementation.
    """

    if not opaque_inputs or any(not value for value in opaque_inputs):
        raise ValueError("at least one non-empty opaque input is required")
    digest = hashlib.sha256()
    for value in (profile_name.value, *opaque_inputs):
        encoded = value.encode("utf-8")
        digest.update(len(encoded).to_bytes(4, byteorder="big"))
        digest.update(encoded)
    return int.from_bytes(digest.digest()[:8], byteorder="big", signed=False)


SPATIAL_BENCHMARK_V1 = SpatialBenchmarkConfig(
    footprint=SquareFootprintConfig(side_length_m=0.60, safety_margin_m=0.05),
    world_frame=WorldFrameConfig(),
    mapper=VoxelMapperConfig(
        voxel_size=0.05,
        block_count=2_000_000,
        device="CPU:0",
        carve_columns=True,
        frame_id="world",
        emit_every=1,
    ),
    lidar_profiles=(
        LidarNoiseProfile(
            name=MapVariant.CLEAN,
            max_range_m=12.0,
            angular_resolution_deg=0.5,
            range_sigma_m=0.0,
            dropout_probability=0.0,
            coverage_probability=1.0,
            planar_drift_sigma_m=0.0,
            yaw_drift_sigma_rad=0.0,
            drift_correlation=0.0,
        ),
        LidarNoiseProfile(
            name=MapVariant.NOISY_01,
            max_range_m=12.0,
            angular_resolution_deg=0.5,
            range_sigma_m=0.02,
            dropout_probability=0.02,
            coverage_probability=0.95,
            planar_drift_sigma_m=0.005,
            yaw_drift_sigma_rad=math.radians(0.1),
            drift_correlation=0.90,
        ),
        LidarNoiseProfile(
            name=MapVariant.NOISY_02,
            max_range_m=12.0,
            angular_resolution_deg=0.5,
            range_sigma_m=0.05,
            dropout_probability=0.08,
            coverage_probability=0.80,
            planar_drift_sigma_m=0.015,
            yaw_drift_sigma_rad=math.radians(0.3),
            drift_correlation=0.95,
        ),
    ),
    geometry_tolerances=GeometryToleranceConfig(
        collision_uncertainty_margin_m=0.05,
        opening_uncertainty_margin_m=0.10,
        room_boundary_uncertainty_margin_m=0.10,
        translation_sweep_step_m=0.01,
        rotation_sweep_step_rad=math.radians(1.0),
    ),
    seed_policy=SeedPolicyConfig(),
)
