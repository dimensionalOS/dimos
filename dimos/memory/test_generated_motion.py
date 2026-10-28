# Copyright 2026 Dimensional Inc.
# SPDX-License-Identifier: Apache-2.0
"""Motion filtering consumes stored poses as generated values."""

import math

from dimos.memory.transform import SpeedLimit, speed
from dimos.memory.type.observation import Observation


def observation(ts: float, x: float, y: float, yaw: float = 0.0) -> Observation[str]:
    return Observation(
        ts=ts, pose_tuple=(x, y, 0.0, 0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)), _data=str(ts)
    )


def test_speed_uses_generated_pose_positions() -> None:
    values = [observation(1.0, 0.0, 0.0), observation(3.0, 3.0, 4.0), observation(3.0, 6.0, 8.0)]
    result = list(speed()(iter(values)))
    assert [obs.data for obs in result] == [2.5, 0.0]
    assert [obs.ts for obs in result] == [3.0, 3.0]


def test_speed_limit_applies_linear_and_angular_thresholds() -> None:
    values = [
        observation(0.0, 0.0, 0.0),
        observation(1.0, 0.5, 0.0),
        observation(2.0, 1.0, 0.0, math.pi / 2),
        observation(3.0, 1.5, 0.0, math.pi / 2),
    ]
    result = list(SpeedLimit(max_mps=1.0, max_dps=45.0)(iter(values)))
    assert [obs.ts for obs in result] == [1.0, 3.0]


def test_speed_limit_retains_prior_quaternion_dot_semantics() -> None:
    # Stored values are not normalized by the historical filter.
    values = [
        Observation(ts=ts, pose_tuple=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0), _data=ts)
        for ts in (0.0, 1.0)
    ]
    assert list(SpeedLimit(max_mps=1.0, max_dps=179.0)(iter(values))) == []
