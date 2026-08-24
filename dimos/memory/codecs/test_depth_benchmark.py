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

import pickle

import cv2
import numpy as np
import pytest

from dimos.memory.codecs.tool_depth_benchmark import (
    benchmark,
    fidelity,
    load_frames,
    synthetic_frames,
)
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat


def test_fidelity_reports_metric_error_and_invalid_mismatch() -> None:
    source = Image(
        data=np.array([[1.0, 2.0], [np.nan, 4.0]], dtype=np.float32),
        format=ImageFormat.DEPTH,
    )
    decoded = Image(
        data=np.array([[1.001, 1.997], [3.0, 4.0]], dtype=np.float32),
        format=ImageFormat.DEPTH,
    )

    result = fidelity(source, decoded)

    assert result.max_error_mm == pytest.approx(3.0, abs=0.001)
    assert result.mean_error_mm == pytest.approx(4 / 3, abs=0.001)
    assert result.rmse_mm == pytest.approx(np.sqrt(10 / 3), abs=0.001)
    assert result.invalid_mismatch_pct == 25.0


@pytest.mark.parametrize("dtype", ["uint16", "float32"])
def test_benchmark_compares_lerc_without_timing_thresholds(dtype: str) -> None:
    rows = benchmark(synthetic_frames(dtype, count=1), repeats=1)
    by_name = {row.codec: row for row in rows}

    assert by_name["raw-lcm"].compression_ratio == 1.0
    assert by_name["raw-lcm"].max_error_mm == 0.0
    assert by_name["lerc-5mm"].max_error_mm <= 5.0
    assert by_name["lerc-5mm"].invalid_mismatch_pct == 0.0
    assert by_name["lerc-5mm"].bytes_per_frame < by_name["raw-lcm"].bytes_per_frame


def test_uint16_only_candidates_are_skipped_for_float_depth() -> None:
    rows = benchmark(synthetic_frames("float32", count=1), repeats=1)
    names = {row.codec for row in rows}

    assert "png3" not in names
    assert "jpegxl-lossless" not in names
    assert "zfp-reversible" in names


def test_load_frames_reads_uint16_png_directory(tmp_path) -> None:
    depth_dir = tmp_path / "depth"
    depth_dir.mkdir()
    pixels = np.array([[0, 1000], [2500, 5000]], dtype=np.uint16)
    assert cv2.imwrite(str(depth_dir / "00000.png"), pixels)

    frames = load_frames(tmp_path, stream_name=None, count=1)

    assert len(frames) == 1
    assert frames[0].format is ImageFormat.DEPTH16
    assert np.array_equal(frames[0].data, pixels)


def test_load_frames_reads_timestamped_float_pickle_directory(tmp_path) -> None:
    depth_dir = tmp_path / "depth"
    depth_dir.mkdir()
    pixels = np.array([[np.nan, 1.25], [2.5, 4.0]], dtype=np.float32)
    (depth_dir / "000.pickle").write_bytes(pickle.dumps((123.5, pixels)))

    frames = load_frames(tmp_path, stream_name=None, count=1)

    assert len(frames) == 1
    assert frames[0].format is ImageFormat.DEPTH
    assert frames[0].ts == 123.5
    assert np.array_equal(frames[0].data, pixels, equal_nan=True)
