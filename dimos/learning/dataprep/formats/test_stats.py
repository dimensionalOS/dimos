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

"""Unit tests for the streaming feature-stats aggregator (`_stats.py`).

Pure numpy — no I/O, no optional deps. Verifies the Welford mean/std/min/max,
the low-dim quantile reservoir, and the image per-channel reduction +
subsampling.
"""

from __future__ import annotations

import numpy as np

from dimos.learning.dataprep.formats._stats import StreamingStats


def test_scalar_mean_std_minmax_count() -> None:
    s = StreamingStats()
    for v in ([1.0, 10.0], [2.0, 20.0], [3.0, 30.0]):
        s.update("state", np.array(v))
    out = s.finalize()["state"]
    # population variance (m2 / n): [1,2,3] → 2/3 ; [10,20,30] → 200/3
    np.testing.assert_allclose(out["mean"], [2.0, 20.0])
    np.testing.assert_allclose(out["std"], [np.sqrt(2 / 3), np.sqrt(200 / 3)])
    assert out["min"] == [1.0, 10.0]
    assert out["max"] == [3.0, 30.0]
    assert out["count"] == 3


def test_single_sample_has_zero_std() -> None:
    s = StreamingStats()
    s.update("x", np.array([5.0, 7.0]))
    out = s.finalize()["x"]
    assert out["std"] == [0.0, 0.0]
    assert out["count"] == 1


def test_lowdim_quantiles_present_and_bounded() -> None:
    s = StreamingStats()
    for i in range(100):
        s.update("x", np.array([float(i)]))
    out = s.finalize()["x"]
    assert "q01" in out and "q99" in out
    assert out["min"][0] <= out["q01"][0] <= out["q99"][0] <= out["max"][0]


def test_image_reduced_to_per_channel_no_quantiles() -> None:
    # image_subsample=1 → every frame counts. Constant per-channel values.
    s = StreamingStats(image_subsample=1)
    img = np.zeros((4, 4, 3), dtype=np.uint8)
    img[..., 0], img[..., 1], img[..., 2] = 10, 20, 30
    for _ in range(5):
        s.update("cam", img)
    out = s.finalize()["cam"]
    np.testing.assert_allclose(out["mean"], [10.0, 20.0, 30.0])
    np.testing.assert_allclose(out["min"], [10.0, 20.0, 30.0])
    np.testing.assert_allclose(out["max"], [10.0, 20.0, 30.0])
    assert out["count"] == 5
    # Images skip the quantile reservoir (per-pixel stats would blow up memory).
    assert "q01" not in out and "q99" not in out


def test_image_subsampling_counts_every_nth_frame() -> None:
    s = StreamingStats(image_subsample=10)
    img = np.zeros((2, 2, 3), dtype=np.uint8)
    for _ in range(25):
        s.update("cam", img)
    # frames 0, 10, 20 sampled → count 3
    assert s.finalize()["cam"]["count"] == 3


def test_empty_aggregator_finalizes_empty() -> None:
    assert StreamingStats().finalize() == {}
