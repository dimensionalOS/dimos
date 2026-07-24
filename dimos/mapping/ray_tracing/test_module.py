# Copyright 2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.

import pytest

from dimos.mapping.ray_tracing.module import RayTracingVoxelMapConfig


def test_pose_match_tolerance_defaults_to_point_one_seconds() -> None:
    assert RayTracingVoxelMapConfig().pose_match_tolerance_s == 0.1


def test_pose_match_tolerance_accepts_non_default_value() -> None:
    config = RayTracingVoxelMapConfig(pose_match_tolerance_s=0.025)
    assert config.pose_match_tolerance_s == 0.025


@pytest.mark.parametrize("value", [0.0, -0.001])
def test_pose_match_tolerance_rejects_non_positive_values(value: float) -> None:
    with pytest.raises(ValueError):
        RayTracingVoxelMapConfig(pose_match_tolerance_s=value)
