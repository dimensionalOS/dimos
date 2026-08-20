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

from pathlib import Path

import numpy as np
import pytest

from dimos.memory.recorder_fidelity import (
    FidelityReport,
    PersistedSample,
    PublishedSample,
    SourceConformance,
    StreamFidelity,
    StreamProfile,
    WorkloadProfile,
    build_bandwidth_metrics,
    check_source_conformance,
    compare_stream,
    counter_delta,
    default_profile,
    payload_digest,
    read_process_io,
    render_report,
    shared_loss_windows,
)
from dimos.memory.tool_recorder_fidelity import run_harness, run_storage_control
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2


def _published(*timestamps_ns: int) -> list[PublishedSample]:
    return [
        PublishedSample(sequence=index, source_ts_ns=timestamp, published_monotonic_ns=timestamp)
        for index, timestamp in enumerate(timestamps_ns)
    ]


def test_profile_round_trips_json(tmp_path: Path) -> None:
    profile = default_profile()
    path = tmp_path / "profile.json"

    profile.write(path)

    assert WorkloadProfile.read(path) == profile
    assert [stream.kind for stream in profile.streams] == [
        "rgb",
        "grayscale",
        "grayscale",
        "depth",
        "imu",
        "pointcloud",
        "odometry",
        "tf",
    ]


def test_source_conformance_rejects_under_rate_and_long_gap() -> None:
    stream = StreamProfile(
        name="imu",
        kind="imu",
        payload_type="dimos.msgs.sensor_msgs.Imu.Imu",
        rate_hz=400.0,
        transport="lcm",
        codec="lcm",
    )
    samples = _published(0, 2_500_000, 205_000_000)

    result = check_source_conformance(stream, samples, scheduled_count=4)

    assert result.valid is False
    assert result.count_ratio == 0.75
    assert result.max_interval_s == pytest.approx(0.2025)
    assert result.violations == (
        "published 75.0% of scheduled observations",
        "p99 interval 0.202500s exceeds 0.005000s",
        "maximum interval 0.202500s exceeds 0.100000s",
    )


def test_compare_stream_localizes_transport_and_persistence_loss() -> None:
    source = _published(1_000_000_000, 2_000_000_000, 3_000_000_000, 4_000_000_000)
    received = [1_000_000_000, 3_000_000_000, 4_000_000_000]
    persisted = [
        PersistedSample(1_000_000_000, "a"),
        PersistedSample(4_000_000_000, "wrong"),
    ]

    result = compare_stream(
        "depth",
        source,
        persisted,
        received_ts_ns=received,
        expected_digests={0: "a", 3: "d"},
    )

    assert result.missing_before_receive == (1,)
    assert result.missing_before_persist == (1, 2)
    assert result.corrupt_sequences == (3,)
    assert result.max_persisted_gap_s == 3.0
    assert result.faithful is False


def test_compare_stream_detects_duplicates_and_reordering() -> None:
    source = _published(1_000_000_000, 2_000_000_000)
    persisted = [
        PersistedSample(2_000_000_000),
        PersistedSample(1_000_000_000),
        PersistedSample(1_000_000_000),
    ]

    result = compare_stream("odometry", source, persisted)

    assert result.duplicate_persisted_timestamps == (1_000_000_000,)
    assert result.reordered is True
    assert result.faithful is False


def test_shared_loss_window_requires_every_data_stream() -> None:
    source = {
        "rgb": _published(1_000_000_000, 2_000_000_000, 3_000_000_000),
        "imu": _published(1_000_000_000, 2_050_000_000, 3_000_000_000),
        "tf": _published(1_000_000_000, 2_000_000_000, 3_000_000_000),
    }
    reports = {
        "rgb": StreamFidelity(
            stream="rgb",
            source_count=3,
            received_count=3,
            persisted_count=2,
            missing_before_persist=(1,),
        ),
        "imu": StreamFidelity(
            stream="imu",
            source_count=3,
            received_count=3,
            persisted_count=2,
            missing_before_persist=(1,),
        ),
        "tf": StreamFidelity(stream="tf", source_count=3, received_count=None, persisted_count=3),
    }

    result = shared_loss_windows(source, reports)

    assert result == ((2.0, 2.0),)


def test_image_payload_digest_covers_pixels_and_format() -> None:
    pixels = np.arange(12, dtype=np.uint8).reshape(2, 2, 3)
    rgb = Image(data=pixels, format=ImageFormat.RGB, frame_id="camera", ts=1.0)
    bgr = Image(data=pixels, format=ImageFormat.BGR, frame_id="camera", ts=1.0)

    assert payload_digest(rgb) != payload_digest(bgr)
    assert payload_digest(rgb) == payload_digest(
        Image(data=pixels.copy(), format=ImageFormat.RGB, frame_id="camera", ts=2.0)
    )


def test_pointcloud_payload_digest_uses_observation_time_separately() -> None:
    points = np.arange(12, dtype=np.float32).reshape(4, 3)
    first = PointCloud2.from_numpy(points, frame_id="lidar", timestamp=1.0)
    later = PointCloud2.from_numpy(points.copy(), frame_id="lidar", timestamp=2.0)
    changed = PointCloud2.from_numpy(points + 1, frame_id="lidar", timestamp=1.0)

    assert payload_digest(first) == payload_digest(later)
    assert payload_digest(first) != payload_digest(changed)


def test_report_renders_actionable_counts() -> None:
    report = FidelityReport(
        profile="test",
        mode="baseline",
        duration_s=30.0,
        source_valid=True,
        faithful=False,
        source={
            "depth": SourceConformance(
                valid=True,
                scheduled=3,
                published=3,
                count_ratio=1.0,
                p99_interval_s=0.03,
                max_interval_s=0.03,
            )
        },
        streams={
            "depth": StreamFidelity(
                stream="depth",
                source_count=3,
                received_count=3,
                persisted_count=2,
                missing_before_persist=(1,),
                max_persisted_gap_s=1.0,
            )
        },
        shared_loss_windows=((2.0, 3.0),),
    )

    rendered = render_report(report)

    assert "Recorder fidelity: FAIL" in rendered
    assert "depth" in rendered
    assert "shared loss windows: 2.000..3.000" in rendered


def test_bandwidth_metrics_distinguish_payload_database_and_device_bytes() -> None:
    profile = WorkloadProfile(
        streams=(
            StreamProfile(
                name="imu",
                kind="imu",
                payload_type="dimos.msgs.sensor_msgs.Imu.Imu",
                rate_hz=5.0,
                transport="lcm",
                codec="lcm",
                raw_bytes=100,
                encoded_bytes=50,
            ),
        )
    )

    result = build_bandwidth_metrics(
        profile,
        duration_s=2.0,
        measurement_elapsed_s=4.0,
        published_counts={"imu": 10},
        persisted_counts={"imu": 8},
        persisted_bytes={"imu": 400},
        active_files={"database": 500, "wal": 200},
        final_files={"database": 600},
        process_io={"write_bytes": 1_600, "wchar": 900, "syscw": 12},
    )

    assert result.offered_raw_bytes == 1_000
    assert result.persisted_payload_bytes == 400
    assert result.sqlite_active_bytes == 700
    assert result.sqlite_final_bytes == 600
    assert result.process_reported_block_write_bytes == 1_600
    assert result.process_block_write_bytes == 1_600
    assert result.process_block_write_amplification == 4.0
    assert result.process_character_write_amplification == 2.25
    assert result.sqlite_size_amplification == 1.5
    assert result.streams["imu"].codec_compression_ratio == 2.0


def test_bandwidth_metrics_reject_incomplete_kernel_block_counter() -> None:
    profile = WorkloadProfile(
        streams=(
            StreamProfile(
                name="imu",
                kind="imu",
                payload_type="dimos.msgs.sensor_msgs.Imu.Imu",
                rate_hz=1.0,
                transport="lcm",
                codec="lcm",
                raw_bytes=100,
            ),
        )
    )

    result = build_bandwidth_metrics(
        profile,
        duration_s=1.0,
        measurement_elapsed_s=1.0,
        published_counts={"imu": 1},
        persisted_counts={"imu": 1},
        persisted_bytes={"imu": 100},
        active_files={"database": 1_000_000},
        final_files={"database": 1_000_000},
        process_io={"write_bytes": 4_096, "wchar": 2_000},
    )

    assert result.process_reported_block_write_bytes == 4_096
    assert result.process_block_write_bytes is None
    assert result.process_block_write_mib_s is None


def test_process_io_parser_and_delta_are_portable(tmp_path: Path) -> None:
    counters_path = tmp_path / "io"
    counters_path.write_text("wchar: 900\nsyscw: 12\nwrite_bytes: 1600\ninvalid: value\n")

    counters = read_process_io(counters_path)

    assert counters == {"wchar": 900, "syscw": 12, "write_bytes": 1_600}
    assert counter_delta(
        {"wchar": 800, "write_bytes": 2_000},
        {"wchar": 900, "write_bytes": 1_600},
    ) == {"wchar": 100, "write_bytes": 0}


def test_storage_batch_control_reduces_small_message_transactions(tmp_path: Path) -> None:
    profile = default_profile().model_copy(
        update={
            "streams": tuple(
                stream.model_copy(update={"rate_hz": 400.0})
                if stream.kind in {"imu", "odometry"}
                else stream.model_copy(update={"rate_hz": 1.0})
                for stream in default_profile().streams
            )
        }
    )

    result = run_storage_control(
        profile,
        duration_s=0.1,
        output_dir=tmp_path,
        mode="batch-small-control",
    )

    assert result["rows"]["imu"] == 40
    assert result["rows"]["odometry"] == 40
    assert result["transactions"]["imu"] < result["rows"]["imu"]
    assert result["transactions"]["odometry"] < result["rows"]["odometry"]


@pytest.mark.self_hosted
def test_recorder_preserves_full_stream_mix_during_controlled_encoder_stall(
    tmp_path: Path,
) -> None:
    report = run_harness(
        default_profile(),
        duration_s=30.0,
        output_dir=tmp_path,
        mode="encoder-stall",
        stall_duration_s=1.0,
    )

    if not report.source_valid:
        raise RuntimeError(f"invalid source workload: {report.source}")
    assert report.faithful, report.streams
