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

"""The fusion arithmetic, on synthetic frames with a known scale."""

from __future__ import annotations

import numpy as np

from dimos.perception.depth2depth.fusion import FuseConfig, fit_affine, fuse

TRUTH = np.linspace(0.5, 5.0, 64 * 64, dtype=np.float32).reshape(64, 64)


def prediction_of(truth: np.ndarray, scale: float = 2.0, offset: float = 0.3) -> np.ndarray:
    """What a model that reads `scale` times too far would predict."""
    return ((truth - offset) / scale).astype(np.float32)


def test_fit_recovers_the_scale_and_offset() -> None:
    prediction = prediction_of(TRUTH)
    valid = np.ones_like(TRUTH, dtype=bool)
    fit = fit_affine(prediction, TRUTH, valid, FuseConfig())
    assert fit is not None
    assert abs(fit[0] - 2.0) < 1e-3, fit
    assert abs(fit[1] - 0.3) < 1e-3, fit


def test_fit_ignores_outliers() -> None:
    prediction = prediction_of(TRUTH)
    raw = TRUTH.copy()
    raw[:8] = 5.0  # a torn block the sensor put meters away from the truth
    fit = fit_affine(prediction, raw, np.ones_like(raw, dtype=bool), FuseConfig())
    assert fit is not None
    assert abs(fit[0] - 2.0) < 0.05, fit


def test_fit_needs_enough_pixels() -> None:
    valid = np.zeros_like(TRUTH, dtype=bool)
    valid.flat[:100] = True
    assert fit_affine(prediction_of(TRUTH), TRUTH, valid, FuseConfig()) is None


def test_holes_are_filled_and_sensor_readings_survive() -> None:
    raw = TRUTH.copy()
    raw[16:32] = 0.0  # a hole: no texture there
    fusion = fuse(prediction_of(TRUTH), raw, FuseConfig())
    assert not fusion.kept_raw[16:32].any(), "a hole is not a sensor reading"
    assert fusion.kept_raw[:16].all(), "agreeing sensor pixels are kept byte for byte"
    np.testing.assert_array_equal(fusion.fused[:16], raw[:16])
    # The hole is filled with the aligned prediction, which matches the truth.
    assert np.abs(fusion.fused[16:32] - TRUTH[16:32]).max() < 1e-2


def test_out_of_range_depth_is_a_hole_not_evidence() -> None:
    raw = TRUTH.copy()
    raw[40:] = 9.0  # past far_m: the sensor is guessing
    fusion = fuse(prediction_of(TRUTH), raw, FuseConfig())
    assert not fusion.kept_raw[40:].any()
    assert abs(fusion.a - 2.0) < 0.05, fusion.a


def test_a_disagreeing_sensor_pixel_is_replaced() -> None:
    raw = TRUTH.copy()
    raw[32, 32] = TRUTH[32, 32] + 2.0  # a flyer well past max(0.3, 10%)
    fusion = fuse(prediction_of(TRUTH), raw, FuseConfig())
    assert not fusion.kept_raw[32, 32]
    assert abs(fusion.fused[32, 32] - TRUTH[32, 32]) < 1e-2


def test_the_fit_is_smoothed_across_frames() -> None:
    config = FuseConfig()
    # A frame whose true scale is 4 after one whose scale was 2 moves the EMA
    # by ema_new_weight, not the whole way.
    fusion = fuse(prediction_of(TRUTH, scale=4.0, offset=0.0), TRUTH, config, previous=(2.0, 0.0))
    assert abs(fusion.a - (0.7 * 2.0 + 0.3 * 4.0)) < 1e-3, fusion.a


def test_an_unfittable_frame_keeps_the_previous_scale() -> None:
    blank = np.zeros_like(TRUTH)  # every pixel a hole: nothing to fit against
    fusion = fuse(prediction_of(TRUTH), blank, FuseConfig(), previous=(2.0, 0.3))
    assert (fusion.a, fusion.b) == (2.0, 0.3)
    assert not fusion.kept_raw.any()
