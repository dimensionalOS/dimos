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

from __future__ import annotations

import numpy as np
from numpy.typing import NDArray
import pytest

from dimos.simulation.sensors.airy.lidar import LENS_HEIGHT, sim_airy
from dimos.simulation.sensors.airy.pattern import BEAMS, FRAME_RATE, POINT_RATE, AiryPattern


class _Wall:
    """Every ray hits something 2 m away, head on."""

    def __init__(self) -> None:
        self.origin = np.zeros(3)

    def cast(
        self, origin: NDArray[np.float64], directions: NDArray[np.float64], max_range: float
    ) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
        self.origin = origin
        return np.full(len(directions), 2.0), directions


def test_one_frame_fires_every_beam_at_each_azimuth_step_and_spins_clockwise() -> None:
    per_frame = round(POINT_RATE / FRAME_RATE)
    d = AiryPattern().directions(0, per_frame)
    assert np.allclose(np.linalg.norm(d, axis=1), 1.0)
    elevation = np.degrees(np.arcsin(d[:, 2]))
    assert len(np.unique(elevation.round(6))) == BEAMS
    assert elevation.min() == pytest.approx(-0.11)
    assert elevation.max() == pytest.approx(88.475)
    beam = d[::BEAMS]
    step = np.degrees(np.arctan2(beam[1, 1], beam[1, 0]) - np.arctan2(beam[0, 1], beam[0, 0]))
    assert step == pytest.approx(-0.4)
    assert np.allclose(AiryPattern().directions(per_frame, per_frame), d)


def test_rays_leave_the_lens_and_returns_report_in_the_sensor_frame() -> None:
    wall = _Wall()
    lidar = sim_airy(wall, seed=0)
    # 90 degrees about y, so the sensor's z axis points along world x
    rotation = np.array([[0.0, 0.0, 1.0], [0.0, 1.0, 0.0], [-1.0, 0.0, 0.0]])
    points = lidar.cast(np.array([1.0, 2.0, 3.0]), rotation, 960)
    assert wall.origin == pytest.approx([1.0 + LENS_HEIGHT, 2.0, 3.0])
    assert np.median(np.linalg.norm(points - [0.0, 0.0, LENS_HEIGHT], axis=1)) == pytest.approx(
        2.0, abs=0.01
    )
