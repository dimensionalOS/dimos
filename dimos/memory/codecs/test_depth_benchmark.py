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

from datetime import datetime, timezone
import json
import pickle

import cv2
import numpy as np
import pytest

from dimos.memory.codecs.base import codec_id
import dimos.memory.codecs.tool_depth_benchmark as depth_benchmark
from dimos.memory.codecs.tool_depth_benchmark import (
    BenchmarkRow,
    Candidate,
    StreamBenchmark,
    benchmark,
    candidates,
    discover_depth_streams,
    fidelity,
    synthetic_frames,
    write_results,
)
from dimos.memory.store.sqlite import SqliteStore
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat


class _KnownErrorCodec:
    def encode(self, image: Image) -> bytes:
        return image.lcm_encode()

    def decode(self, payload: bytes) -> Image:
        image = Image.lcm_decode(payload)
        error_m = 0.001 if image.ts == 1.0 else 0.003
        return Image(
            data=image.data + error_m,
            format=image.format,
            frame_id=image.frame_id,
            ts=image.ts,
        )


def _depth(
    pixels: list[list[int]],
    *,
    frame_id: str = "depth",
    ts: float = 1.0,
) -> Image:
    return Image(
        data=np.array(pixels, dtype=np.uint16),
        format=ImageFormat.DEPTH16,
        frame_id=frame_id,
        ts=ts,
    )


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
def test_benchmark_checks_all_compatible_codecs(dtype: str) -> None:
    rows = benchmark(synthetic_frames(dtype, count=1))
    by_name = {row.codec: row for row in rows}

    raw = by_name["lcm"]
    lerc = by_name["lerc"]
    assert isinstance(raw, BenchmarkRow)
    assert isinstance(lerc, BenchmarkRow)
    assert raw.compression_ratio == 1.0
    assert raw.max_error_mm == 0.0
    assert lerc.max_error_mm <= 5.0
    assert lerc.invalid_mismatch_pct == 0.0
    assert lerc.encoded_bytes < lerc.raw_bytes


def test_candidates_are_supported_storage_codecs() -> None:
    configured = candidates()

    names = [candidate.name for candidate in configured]
    assert names == ["lcm", "lz4+lcm", "jpeg", "lerc"]
    assert [codec_id(candidate.codec) for candidate in configured] == names


def test_benchmark_aggregates_fidelity_across_all_pixels(monkeypatch) -> None:
    frames = [
        Image(
            data=np.array([[1.0, np.nan, np.nan]], dtype=np.float32),
            format=ImageFormat.DEPTH,
            frame_id="depth",
            ts=1.0,
        ),
        Image(
            data=np.ones((1, 3), dtype=np.float32),
            format=ImageFormat.DEPTH,
            frame_id="depth",
            ts=2.0,
        ),
    ]

    candidate = Candidate("known-error", _KnownErrorCodec(), 3.0)
    monkeypatch.setattr(depth_benchmark, "candidates", lambda: [candidate])

    rows = benchmark(frames)

    row = rows[0]
    assert isinstance(row, BenchmarkRow)
    assert row.frames == 2
    assert row.mean_error_mm == pytest.approx(2.5, abs=0.001)
    assert row.rmse_mm == pytest.approx(np.sqrt(7), abs=0.001)
    assert row.max_error_mm == pytest.approx(3.0, abs=0.001)


def test_benchmark_rejects_inconsistent_stream_shape() -> None:
    frames = [_depth([[1000]], ts=1.0), _depth([[1000, 2000]], ts=2.0)]

    with pytest.raises(ValueError, match="changed format, dtype, or shape"):
        benchmark(frames)


def test_discover_depth_streams_reads_complete_png_directory(tmp_path) -> None:
    depth_dir = tmp_path / "depth"
    depth_dir.mkdir()
    first = np.array([[0, 1000], [2500, 5000]], dtype=np.uint16)
    second = np.array([[10, 1010], [2510, 5010]], dtype=np.uint16)
    assert cv2.imwrite(str(depth_dir / "00000.png"), first)
    assert cv2.imwrite(str(depth_dir / "00001.png"), second)

    sources = discover_depth_streams(tmp_path)
    frames = list(sources[0].frames())

    assert len(sources) == 1
    assert sources[0].stream == "depth"
    assert sources[0].frame_count == 2
    assert [frame.ts for frame in frames] == [1.0, 2.0]
    assert np.array_equal(frames[0].data, first)
    assert np.array_equal(frames[1].data, second)


def test_discover_depth_streams_reads_timestamped_float_pickle_directory(tmp_path) -> None:
    depth_dir = tmp_path / "depth"
    depth_dir.mkdir()
    pixels = np.array([[np.nan, 1.25], [2.5, 4.0]], dtype=np.float32)
    (depth_dir / "000.pickle").write_bytes(pickle.dumps((123.5, pixels)))

    source = discover_depth_streams(tmp_path)[0]
    frames = list(source.frames())

    assert source.frame_count == 1
    assert frames[0].format is ImageFormat.DEPTH
    assert frames[0].ts == 123.5
    assert np.array_equal(frames[0].data, pixels, equal_nan=True)


def test_discover_depth_streams_finds_and_filters_sqlite_streams(tmp_path) -> None:
    database = tmp_path / "recording.db"
    with SqliteStore(path=str(database)) as store:
        store.stream("depth_left", Image, codec="lcm", eager_blobs=True).append(
            _depth([[1000]], frame_id="left"), ts=1.0
        )
        store.stream("depth_right", Image, codec="lcm", eager_blobs=True).append(
            _depth([[2000]], frame_id="right"), ts=1.0
        )
        store.stream("notes", str, eager_blobs=True).append("not depth", ts=1.0)

    all_depth = discover_depth_streams(database)
    selected = discover_depth_streams(database, ["depth_right"])

    assert [source.stream for source in all_depth] == ["depth_left", "depth_right"]
    assert [source.stream for source in selected] == ["depth_right"]
    assert next(iter(selected[0].frames())).frame_id == "right"


def test_write_results_creates_json_and_markdown_artifacts(tmp_path) -> None:
    frames = synthetic_frames("uint16", count=1)
    result = StreamBenchmark(
        source="synthetic-uint16",
        stream="depth",
        frames=1,
        shape=frames[0].shape,
        dtype=str(frames[0].dtype),
        image_format=frames[0].format.value,
        codecs=benchmark(frames),
    )
    output = tmp_path / "results"

    write_results(
        output,
        [result],
        ["synthetic-uint16"],
        datetime(2026, 8, 24, tzinfo=timezone.utc),
    )

    document = json.loads((output / "results.json").read_text())
    markdown = (output / "results.md").read_text()
    assert document["schema_version"] == 1
    assert document["results"][0]["frames"] == 1
    assert document["results"][0]["codecs"][0]["encode_wall_p95_ms"] >= 0
    assert "# Depth codec benchmark" in markdown
    assert "Encode wall" in markdown


def test_write_results_refuses_nonempty_output_directory(tmp_path) -> None:
    output = tmp_path / "results"
    output.mkdir()
    (output / "existing.txt").write_text("keep")

    with pytest.raises(ValueError, match="not empty"):
        write_results(
            output,
            [],
            [],
            datetime(2026, 8, 24, tzinfo=timezone.utc),
        )
