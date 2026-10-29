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

from dimos.simulation.sensors.airy.pattern import BEAMS, FRAME_RATE, POINT_RATE, AiryPattern


def test_one_frame_covers_the_hemisphere_at_the_spec_resolution() -> None:
    per_frame = round(POINT_RATE / FRAME_RATE)
    d = AiryPattern().directions(0, per_frame)
    assert np.allclose(np.linalg.norm(d, axis=1), 1.0)
    elevation = np.degrees(np.arcsin(d[:, 2]))
    assert len(np.unique(elevation.round(6))) == BEAMS
    assert 0.0 < elevation.min() < 1.0
    assert 89.0 < elevation.max() < 90.0
    azimuth = np.unique(np.degrees(np.arctan2(d[::BEAMS, 1], d[::BEAMS, 0])) % 360.0)
    assert np.allclose(np.diff(azimuth), 0.4)
    assert np.allclose(AiryPattern().directions(per_frame, per_frame), d)
