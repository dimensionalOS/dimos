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

"""Recorder fidelity workload profiles, manifests, and report analysis.

This module contains deterministic, side-effect-free analysis used by the
on-demand recorder stress tool.  The actual multi-process harness lives in
``tool_recorder_fidelity.py`` so normal pytest collection stays cheap.
"""

from __future__ import annotations

from collections import Counter
from collections.abc import Mapping, Sequence
from dataclasses import dataclass
import hashlib
from itertools import pairwise
import json
import math
import os
from pathlib import Path
import platform
import statistics
import sys
from typing import Any, Literal

import numpy as np
from pydantic import BaseModel, Field, field_validator, model_validator

from dimos.memory.codecs.base import codec_from_id
from dimos.memory.store.sqlite import SqliteStore
from dimos.memory.stream import Stream
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.tf2_msgs.TFMessage import TFMessage

StreamKind = Literal["rgb", "grayscale", "depth", "imu", "pointcloud", "odometry", "tf"]
TransportKind = Literal["lcm", "shm", "zenoh"]


class StreamProfile(BaseModel):
    """One stream in a calibrated recorder workload."""

    name: str
    kind: StreamKind
    payload_type: str
    rate_hz: float = Field(gt=0)
    transport: TransportKind
    codec: str
    frame_id: str = "sensor_link"
    shape: tuple[int, ...] | None = None
    dtype: str | None = None
    point_count: int | None = Field(default=None, ge=0)
    raw_bytes: int = Field(default=0, ge=0)
    encoded_bytes: int = Field(default=0, ge=0)

    @model_validator(mode="after")
    def _validate_payload_shape(self) -> StreamProfile:
        if self.kind in {"rgb", "grayscale", "depth"}:
            if self.shape is None or self.dtype is None:
                raise ValueError(f"{self.kind} stream {self.name!r} needs shape and dtype")
        if self.kind == "pointcloud" and self.point_count is None:
            raise ValueError(f"pointcloud stream {self.name!r} needs point_count")
        return self


class WorkloadProfile(BaseModel):
    """A portable, non-sensitive description of a recorder workload."""

    name: str = "recorder-production"
    seed: int = 7
    streams: tuple[StreamProfile, ...]
    calibration_source: str | None = None

    @field_validator("streams")
    @classmethod
    def _unique_stream_names(cls, value: tuple[StreamProfile, ...]) -> tuple[StreamProfile, ...]:
        names = [stream.name for stream in value]
        duplicates = sorted(name for name, count in Counter(names).items() if count > 1)
        if duplicates:
            raise ValueError(f"duplicate stream names: {duplicates}")
        if not value:
            raise ValueError("a workload profile needs at least one stream")
        return value

    def stream(self, name: str) -> StreamProfile:
        for stream in self.streams:
            if stream.name == name:
                return stream
        raise KeyError(name)

    def with_transport(self, transport: TransportKind) -> WorkloadProfile:
        return self.model_copy(
            update={
                "streams": tuple(
                    stream.model_copy(update={"transport": transport}) for stream in self.streams
                )
            }
        )

    def with_rate_scale(self, scale: float) -> WorkloadProfile:
        """Return a copy with every stream rate multiplied by ``scale``."""
        if not math.isfinite(scale) or scale <= 0:
            raise ValueError("rate scale must be finite and greater than zero")
        return self.model_copy(
            update={
                "streams": tuple(
                    stream.model_copy(update={"rate_hz": stream.rate_hz * scale})
                    for stream in self.streams
                )
            }
        )

    def write(self, path: Path) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(self.model_dump_json(indent=2) + "\n")

    @classmethod
    def read(cls, path: Path) -> WorkloadProfile:
        return cls.model_validate_json(path.read_text())


@dataclass(frozen=True, slots=True)
class PublishedSample:
    """One source observation that completed its transport publish call."""

    sequence: int
    source_ts_ns: int
    published_monotonic_ns: int


@dataclass(frozen=True, slots=True)
class PersistedSample:
    """One observation found in the completed recording."""

    source_ts_ns: int
    payload_digest: str | None = None


class SourceRun(BaseModel):
    """Publisher output used to prove that the requested load was offered."""

    duration_s: float = Field(gt=0)
    samples: dict[str, list[tuple[int, int, int]]]
    scheduled_counts: dict[str, int]

    def published(self, stream: str) -> list[PublishedSample]:
        return [PublishedSample(*row) for row in self.samples.get(stream, [])]


class SourceConformance(BaseModel):
    valid: bool
    scheduled: int
    published: int
    count_ratio: float
    p99_interval_s: float | None
    max_interval_s: float | None
    violations: tuple[str, ...] = ()


class StreamFidelity(BaseModel):
    stream: str
    source_count: int
    received_count: int | None
    persisted_count: int
    missing_before_receive: tuple[int, ...] = ()
    missing_before_persist: tuple[int, ...] = ()
    duplicate_persisted_timestamps: tuple[int, ...] = ()
    reordered: bool = False
    corrupt_sequences: tuple[int, ...] = ()
    max_persisted_gap_s: float | None = None

    @property
    def faithful(self) -> bool:
        return not (
            self.missing_before_receive
            or self.missing_before_persist
            or self.duplicate_persisted_timestamps
            or self.reordered
            or self.corrupt_sequences
            or self.source_count != self.persisted_count
        )


class StreamBandwidth(BaseModel):
    published_messages: int
    persisted_messages: int
    raw_bytes_per_message: int
    persisted_payload_bytes: int
    offered_raw_mib_s: float
    persisted_payload_mib_s: float
    codec_compression_ratio: float | None


class BandwidthMetrics(BaseModel):
    measurement_elapsed_s: float
    drain_elapsed_s: float = 0.0
    offered_raw_bytes: int
    persisted_payload_bytes: int
    sqlite_active_bytes: int
    sqlite_final_bytes: int
    process_reported_block_write_bytes: int | None = None
    process_block_write_bytes: int | None = None
    process_write_characters: int | None = None
    process_write_syscalls: int | None = None
    offered_raw_mib_s: float
    persisted_payload_mib_s: float
    sqlite_final_mib: float
    sqlite_growth_mib_s: float
    process_block_write_mib_s: float | None = None
    process_character_write_mib_s: float | None = None
    process_block_write_amplification: float | None = None
    process_character_write_amplification: float | None = None
    sqlite_size_amplification: float | None = None
    active_files: dict[str, int] = Field(default_factory=dict)
    final_files: dict[str, int] = Field(default_factory=dict)
    streams: dict[str, StreamBandwidth] = Field(default_factory=dict)


class RealtimeSample(BaseModel):
    elapsed_s: float
    source_active: bool
    queues: dict[str, dict[str, int | float | str | None]]
    writer: dict[str, int | float | str | None]
    sqlite_files: dict[str, int]
    process_io: dict[str, int]
    process_cpu_s: float
    device_io: dict[str, int | str] = Field(default_factory=dict)
    io_pressure: dict[str, float | int] = Field(default_factory=dict)


class RealtimeMetrics(BaseModel):
    source_active_s: float
    drain_elapsed_s: float
    receive_to_commit_p99_ms: float
    receive_to_commit_max_ms: float
    initial_backlog_median_ms: float
    final_backlog_median_ms: float
    final_backlog_age_ms: float
    recovery_s: float | None = None
    passed: bool
    violations: tuple[str, ...] = ()


class DeviceWriteMetrics(BaseModel):
    available: bool
    device: str | None = None
    write_bytes: int | None = None
    write_mib_s: float | None = None
    write_iops: float | None = None
    average_write_time_ms: float | None = None
    utilization: float | None = None
    io_pressure_stall_s: float | None = None
    unavailable_reason: str | None = None


class StorageCapacityReport(BaseModel):
    profile: str
    duration_s: float
    rate_scale: float
    source_valid: bool
    submitted_messages: int
    committed_messages: int
    committed_payload_bytes: int
    offered_mib_s: float
    effective_committed_mib_s: float
    drain_elapsed_s: float
    receive_to_commit_p99_ms: float
    receive_to_commit_max_ms: float
    passed: bool
    violations: tuple[str, ...] = ()
    writer: dict[str, Any] = Field(default_factory=dict)


class FidelityReport(BaseModel):
    profile: str
    mode: str
    duration_s: float
    source_valid: bool
    faithful: bool
    source: dict[str, SourceConformance]
    streams: dict[str, StreamFidelity]
    shared_loss_windows: tuple[tuple[float, float], ...] = ()
    timings: dict[str, dict[str, float | int]] = Field(default_factory=dict)
    bandwidth: BandwidthMetrics | None = None
    realtime: RealtimeMetrics | None = None
    device: DeviceWriteMetrics | None = None
    samples: tuple[RealtimeSample, ...] = ()
    environment: dict[str, str | int] = Field(default_factory=dict)
    recorder: dict[str, Any] = Field(default_factory=dict)

    @property
    def successful(self) -> bool:
        return (
            self.source_valid and self.faithful and (self.realtime is None or self.realtime.passed)
        )

    def write(self, path: Path) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(self.model_dump_json(indent=2) + "\n")


def default_profile() -> WorkloadProfile:
    """Return a usable profile until a real recording is calibrated."""

    image_type = "dimos.msgs.sensor_msgs.Image.Image"
    return WorkloadProfile(
        calibration_source="synthetic defaults; replace with `calibrate` output",
        streams=(
            StreamProfile(
                name="rgb",
                kind="rgb",
                payload_type=image_type,
                rate_hz=30.0,
                transport="shm",
                codec="jpeg",
                frame_id="rgb_link",
                shape=(480, 640, 3),
                dtype="uint8",
                raw_bytes=480 * 640 * 3,
                encoded_bytes=13_459,
            ),
            *(
                StreamProfile(
                    name=name,
                    kind="grayscale",
                    payload_type=image_type,
                    rate_hz=30.0,
                    transport="shm",
                    codec="lz4+lcm",
                    frame_id=f"{name}_link",
                    shape=(480, 640),
                    dtype="uint8",
                    raw_bytes=480 * 640,
                    encoded_bytes=307_310 if name == "grayscale_left" else 307_311,
                )
                for name in ("grayscale_left", "grayscale_right")
            ),
            StreamProfile(
                name="depth",
                kind="depth",
                payload_type=image_type,
                rate_hz=30.0,
                transport="shm",
                codec="lz4+lcm",
                frame_id="depth_link",
                shape=(480, 640),
                dtype="uint16",
                raw_bytes=480 * 640 * 2,
                encoded_bytes=614_521,
            ),
            StreamProfile(
                name="imu",
                kind="imu",
                payload_type="dimos.msgs.sensor_msgs.Imu.Imu",
                rate_hz=400.0,
                transport="lcm",
                codec="lcm",
                frame_id="imu_link",
                raw_bytes=329,
                encoded_bytes=329,
            ),
            StreamProfile(
                name="pointlio",
                kind="pointcloud",
                payload_type="dimos.msgs.sensor_msgs.PointCloud2.PointCloud2",
                rate_hz=10.0,
                transport="shm",
                codec="lcm",
                frame_id="lidar_link",
                point_count=20_000,
                raw_bytes=320_129,
                encoded_bytes=320_129,
            ),
            StreamProfile(
                name="odometry",
                kind="odometry",
                payload_type="dimos.msgs.nav_msgs.Odometry.Odometry",
                rate_hz=200.0,
                transport="lcm",
                codec="lcm",
                frame_id="world",
                raw_bytes=724,
                encoded_bytes=724,
            ),
            StreamProfile(
                name="tf",
                kind="tf",
                payload_type="dimos.msgs.tf2_msgs.TFMessage.TFMessage",
                rate_hz=20.0,
                transport="lcm",
                codec="lcm",
                frame_id="world",
                raw_bytes=103,
                encoded_bytes=107,
            ),
        ),
    )


def percentile(values: Sequence[float], quantile: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    index = max(0, min(len(ordered) - 1, math.ceil(quantile * len(ordered)) - 1))
    return ordered[index]


def check_source_conformance(
    stream: StreamProfile,
    samples: Sequence[PublishedSample],
    scheduled_count: int,
) -> SourceConformance:
    """Validate count and scheduling independently from recorder fidelity."""

    intervals = [
        (right.published_monotonic_ns - left.published_monotonic_ns) / 1e9
        for left, right in pairwise(samples)
    ]
    ratio = len(samples) / scheduled_count if scheduled_count else 1.0
    p99 = percentile(intervals, 0.99)
    maximum = max(intervals, default=None)
    period = 1.0 / stream.rate_hz
    violations: list[str] = []
    if ratio < 0.99:
        violations.append(f"published {ratio:.1%} of scheduled observations")
    if p99 is not None and p99 > period * 2:
        violations.append(f"p99 interval {p99:.6f}s exceeds {period * 2:.6f}s")
    max_allowed = max(period * 2, 0.1)
    if maximum is not None and maximum > max_allowed:
        violations.append(f"maximum interval {maximum:.6f}s exceeds {max_allowed:.6f}s")
    return SourceConformance(
        valid=not violations,
        scheduled=scheduled_count,
        published=len(samples),
        count_ratio=ratio,
        p99_interval_s=p99,
        max_interval_s=maximum,
        violations=tuple(violations),
    )


_TS_TOLERANCE_NS = 2_000


def _match_timestamps(
    expected: Sequence[int], actual: Sequence[int]
) -> tuple[list[int], list[int]]:
    """Return missing expected indices and actual-to-expected sequence numbers."""

    missing: list[int] = []
    matched_sequences: list[int] = []
    cursor = 0
    for sequence, expected_ts in enumerate(expected):
        while cursor < len(actual) and actual[cursor] < expected_ts - _TS_TOLERANCE_NS:
            cursor += 1
        if cursor >= len(actual) or abs(actual[cursor] - expected_ts) > _TS_TOLERANCE_NS:
            missing.append(sequence)
            continue
        matched_sequences.append(sequence)
        cursor += 1
    return missing, matched_sequences


def compare_stream(
    stream: str,
    source: Sequence[PublishedSample],
    persisted: Sequence[PersistedSample],
    *,
    received_ts_ns: Sequence[int] | None = None,
    expected_digests: Mapping[int, str] | None = None,
) -> StreamFidelity:
    expected_ts = [sample.source_ts_ns for sample in source]
    persisted_ts = [sample.source_ts_ns for sample in persisted]
    missing_persisted_positions, matched_positions = _match_timestamps(expected_ts, persisted_ts)
    missing_persisted = [source[position].sequence for position in missing_persisted_positions]
    missing_received: list[int] = []
    if received_ts_ns is not None:
        missing_received_positions, _ = _match_timestamps(expected_ts, sorted(received_ts_ns))
        missing_received = [source[position].sequence for position in missing_received_positions]

    duplicates = tuple(sorted(ts for ts, count in Counter(persisted_ts).items() if count > 1))
    reordered = persisted_ts != sorted(persisted_ts)
    corrupt: list[int] = []
    if expected_digests:
        for persisted_sample, position in zip(persisted, matched_positions, strict=False):
            sequence = source[position].sequence
            expected_digest = expected_digests.get(sequence)
            if expected_digest is not None and persisted_sample.payload_digest != expected_digest:
                corrupt.append(sequence)

    gaps = [(right - left) / 1e9 for left, right in pairwise(persisted_ts)]
    return StreamFidelity(
        stream=stream,
        source_count=len(source),
        received_count=None if received_ts_ns is None else len(received_ts_ns),
        persisted_count=len(persisted),
        missing_before_receive=tuple(missing_received),
        missing_before_persist=tuple(missing_persisted),
        duplicate_persisted_timestamps=duplicates,
        reordered=reordered,
        corrupt_sequences=tuple(corrupt),
        max_persisted_gap_s=max(gaps, default=None),
    )


def shared_loss_windows(
    source: Mapping[str, Sequence[PublishedSample]],
    stream_reports: Mapping[str, StreamFidelity],
) -> tuple[tuple[float, float], ...]:
    """Find source-time windows in which every data stream lost observations."""

    missing_by_stream: list[list[float]] = []
    for name, report in stream_reports.items():
        if name == "tf" or not report.missing_before_persist:
            continue
        samples = source[name]
        by_sequence = {sample.sequence: sample for sample in samples}
        missing_by_stream.append(
            [
                by_sequence[sequence].source_ts_ns / 1e9
                for sequence in report.missing_before_persist
                if sequence in by_sequence
            ]
        )
    data_stream_count = sum(1 for name in stream_reports if name != "tf")
    if len(missing_by_stream) != data_stream_count or not missing_by_stream:
        return ()

    common: list[float] = []
    anchor = missing_by_stream[0]
    for ts in anchor:
        if all(any(abs(other - ts) <= 0.1 for other in stream) for stream in missing_by_stream[1:]):
            common.append(ts)
    if not common:
        return ()

    windows: list[tuple[float, float]] = []
    start = previous = common[0]
    for ts in common[1:]:
        if ts - previous > 0.2:
            windows.append((start, previous))
            start = ts
        previous = ts
    windows.append((start, previous))
    return tuple(windows)


def payload_digest(value: Any) -> str:
    """Return a stable digest of decoded message content."""

    digest = hashlib.sha256()
    if isinstance(value, Image):
        digest.update(value.format.value.encode())
        digest.update(value.frame_id.encode())
        digest.update(str(value.data.shape).encode())
        digest.update(str(value.data.dtype).encode())
        digest.update(np.ascontiguousarray(value.data).tobytes())
    elif isinstance(value, PointCloud2):
        points, colors = value.as_numpy()
        digest.update(value.frame_id.encode())
        digest.update(str(points.shape).encode())
        digest.update(str(points.dtype).encode())
        digest.update(np.ascontiguousarray(points).tobytes())
        if colors is not None:
            digest.update(str(colors.shape).encode())
            digest.update(str(colors.dtype).encode())
            digest.update(np.ascontiguousarray(colors).tobytes())
    elif hasattr(value, "lcm_encode"):
        digest.update(value.lcm_encode())
    else:
        digest.update(repr(value).encode())
    return digest.hexdigest()


def codec_roundtrip(value: Any, stream: StreamProfile) -> Any:
    """Apply the configured storage codec before comparing lossy payloads."""

    codec = codec_from_id(stream.codec, stream.payload_type)
    return codec.decode(codec.encode(value))


def persisted_samples(path: Path, profile: WorkloadProfile) -> dict[str, list[PersistedSample]]:
    """Read and digest every supported stream from a completed recording."""

    store = SqliteStore(path=str(path), must_exist=True)
    try:
        result: dict[str, list[PersistedSample]] = {}
        available = set(store.list_streams())
        for stream_profile in profile.streams:
            if stream_profile.name not in available:
                result[stream_profile.name] = []
                continue
            samples: list[PersistedSample] = []
            recorded_stream: Stream[Any] = store.stream(stream_profile.name)
            for observation in recorded_stream:
                samples.append(
                    PersistedSample(
                        source_ts_ns=round(observation.ts * 1e9),
                        payload_digest=payload_digest(observation.data),
                    )
                )
            result[stream_profile.name] = samples
        return result
    finally:
        store.stop()


def persisted_payload_sizes(path: Path, profile: WorkloadProfile) -> dict[str, int]:
    """Return codec payload bytes stored for each stream, excluding SQLite metadata."""

    store = SqliteStore(path=str(path), must_exist=True)
    try:
        available = set(store.list_streams())
        return {
            stream.name: int(store.stream(stream.name).size_bytes() or 0)
            if stream.name in available
            else 0
            for stream in profile.streams
        }
    finally:
        store.stop()


def read_process_io(path: Path = Path("/proc/self/io")) -> dict[str, int]:
    """Read Linux per-process I/O counters, or return no counters elsewhere."""

    try:
        lines = path.read_text().splitlines()
    except (FileNotFoundError, PermissionError, OSError):
        return {}
    counters: dict[str, int] = {}
    for line in lines:
        name, separator, value = line.partition(":")
        if separator and value.strip().isdigit():
            counters[name] = int(value)
    return counters


def read_device_io(path: Path) -> dict[str, int | str]:
    """Read Linux counters for the block device backing *path*."""

    try:
        device = path.stat().st_dev
        major, minor = os.major(device), os.minor(device)
        lines = Path("/proc/diskstats").read_text().splitlines()
    except (FileNotFoundError, PermissionError, OSError):
        return {}
    for line in lines:
        fields = line.split()
        if len(fields) < 14 or int(fields[0]) != major or int(fields[1]) != minor:
            continue
        return {
            "device": fields[2],
            "writes_completed": int(fields[7]),
            "sectors_written": int(fields[9]),
            "write_time_ms": int(fields[10]),
            "io_time_ms": int(fields[12]),
        }
    return {}


def read_io_pressure(path: Path = Path("/proc/pressure/io")) -> dict[str, float | int]:
    """Read system-wide Linux I/O pressure counters when available."""

    try:
        lines = path.read_text().splitlines()
    except (FileNotFoundError, PermissionError, OSError):
        return {}
    result: dict[str, float | int] = {}
    for line in lines:
        fields = line.split()
        if not fields:
            continue
        prefix = fields[0]
        for field in fields[1:]:
            name, separator, value = field.partition("=")
            if not separator:
                continue
            key = f"{prefix}_{name}"
            result[key] = int(value) if name == "total" else float(value)
    return result


def counter_delta(before: Mapping[str, int], after: Mapping[str, int]) -> dict[str, int]:
    """Calculate monotonic counter growth for keys present in both snapshots."""

    return {name: max(0, after[name] - value) for name, value in before.items() if name in after}


def sqlite_file_sizes(path: Path) -> dict[str, int]:
    """Measure the main database and transient WAL/SHM files that currently exist."""

    candidates = {
        "database": path,
        "wal": Path(f"{path}-wal"),
        "shm": Path(f"{path}-shm"),
    }
    return {
        name: candidate.stat().st_size
        for name, candidate in candidates.items()
        if candidate.exists()
    }


def build_bandwidth_metrics(
    profile: WorkloadProfile,
    *,
    duration_s: float,
    measurement_elapsed_s: float,
    drain_elapsed_s: float = 0.0,
    published_counts: Mapping[str, int],
    persisted_counts: Mapping[str, int],
    persisted_bytes: Mapping[str, int],
    active_files: Mapping[str, int],
    final_files: Mapping[str, int],
    process_io: Mapping[str, int],
) -> BandwidthMetrics:
    """Build logical and physical write metrics with explicit byte boundaries."""

    mib = 1024 * 1024
    streams: dict[str, StreamBandwidth] = {}
    offered_raw_bytes = 0
    persisted_payload_bytes = 0
    for stream in profile.streams:
        published = published_counts.get(stream.name, 0)
        persisted = persisted_counts.get(stream.name, 0)
        payload_bytes = persisted_bytes.get(stream.name, 0)
        stream_raw_bytes = stream.raw_bytes * published
        encoded_raw_bytes = stream.raw_bytes * persisted
        offered_raw_bytes += stream_raw_bytes
        persisted_payload_bytes += payload_bytes
        streams[stream.name] = StreamBandwidth(
            published_messages=published,
            persisted_messages=persisted,
            raw_bytes_per_message=stream.raw_bytes,
            persisted_payload_bytes=payload_bytes,
            offered_raw_mib_s=stream_raw_bytes / duration_s / mib,
            persisted_payload_mib_s=payload_bytes / duration_s / mib,
            codec_compression_ratio=(encoded_raw_bytes / payload_bytes if payload_bytes else None),
        )

    sqlite_active_bytes = sum(active_files.values())
    sqlite_final_bytes = sum(final_files.values())
    reported_block_write_bytes = process_io.get("write_bytes")
    # Some container/overlay filesystems expose the counter but do not charge
    # SQLite writes to it. A value below the final file footprint cannot be a
    # complete physical-write measurement, so report it as unavailable.
    process_block_write_bytes = (
        reported_block_write_bytes
        if reported_block_write_bytes is not None
        and reported_block_write_bytes >= sqlite_final_bytes
        else None
    )
    process_wchar = process_io.get("wchar")
    process_syscw = process_io.get("syscw")
    return BandwidthMetrics(
        measurement_elapsed_s=measurement_elapsed_s,
        drain_elapsed_s=drain_elapsed_s,
        offered_raw_bytes=offered_raw_bytes,
        persisted_payload_bytes=persisted_payload_bytes,
        sqlite_active_bytes=sqlite_active_bytes,
        sqlite_final_bytes=sqlite_final_bytes,
        process_reported_block_write_bytes=reported_block_write_bytes,
        process_block_write_bytes=process_block_write_bytes,
        process_write_characters=process_wchar,
        process_write_syscalls=process_syscw,
        offered_raw_mib_s=offered_raw_bytes / duration_s / mib,
        persisted_payload_mib_s=persisted_payload_bytes / duration_s / mib,
        sqlite_final_mib=sqlite_final_bytes / mib,
        sqlite_growth_mib_s=sqlite_final_bytes / duration_s / mib,
        process_block_write_mib_s=(
            process_block_write_bytes / measurement_elapsed_s / mib
            if process_block_write_bytes is not None
            else None
        ),
        process_character_write_mib_s=(
            process_wchar / measurement_elapsed_s / mib if process_wchar is not None else None
        ),
        process_block_write_amplification=(
            process_block_write_bytes / persisted_payload_bytes
            if process_block_write_bytes is not None and persisted_payload_bytes
            else None
        ),
        process_character_write_amplification=(
            process_wchar / persisted_payload_bytes
            if process_wchar is not None and persisted_payload_bytes
            else None
        ),
        sqlite_size_amplification=(
            sqlite_final_bytes / persisted_payload_bytes if persisted_payload_bytes else None
        ),
        active_files=dict(active_files),
        final_files=dict(final_files),
        streams=streams,
    )


def _sample_backlog_age_s(sample: RealtimeSample) -> float:
    queue_age = max(
        (float(status.get("oldest_age_s") or 0.0) for status in sample.queues.values()),
        default=0.0,
    )
    return max(queue_age, float(sample.writer.get("oldest_age_s") or 0.0))


def build_realtime_metrics(
    *,
    mode: str,
    source_active_s: float,
    drain_elapsed_s: float,
    writer: Mapping[str, Any],
    samples: Sequence[RealtimeSample],
    stall_end_elapsed_s: float | None,
) -> RealtimeMetrics:
    """Apply the recorder's bounded-latency and stall-recovery contract."""

    active = [sample for sample in samples if sample.source_active]
    initial = [_sample_backlog_age_s(sample) for sample in active if sample.elapsed_s <= 5.0]
    final = [
        _sample_backlog_age_s(sample)
        for sample in active
        if sample.elapsed_s >= max(0.0, source_active_s - 5.0)
    ]
    initial_median = statistics.median(initial) if initial else 0.0
    final_median = statistics.median(final) if final else 0.0
    final_age = _sample_backlog_age_s(active[-1]) if active else 0.0
    p99_ms = float(writer.get("receive_to_commit_p99_ms") or 0.0)
    max_ms = float(writer.get("receive_to_commit_max_ms") or 0.0)
    violations: list[str] = []
    recovery_s: float | None = None

    if mode in {"baseline", "wal-control"}:
        if p99_ms >= 100.0:
            violations.append(f"receive-to-commit p99 {p99_ms:.3f}ms is not below 100ms")
        if final_age >= 0.1:
            violations.append(f"final backlog age {final_age:.3f}s is not below 0.100s")
        if final_median - initial_median > 0.010:
            violations.append(
                f"backlog median grew by {final_median - initial_median:.3f}s; limit is 0.010s"
            )
    elif stall_end_elapsed_s is not None:
        below = 0
        for sample in samples:
            if sample.elapsed_s < stall_end_elapsed_s:
                continue
            if _sample_backlog_age_s(sample) < 0.1:
                below += 1
                if below == 3:
                    recovery_s = sample.elapsed_s - stall_end_elapsed_s
                    break
            else:
                below = 0
        if recovery_s is None or recovery_s > 2.0:
            rendered = "not observed" if recovery_s is None else f"{recovery_s:.3f}s"
            violations.append(f"stall recovery was {rendered}; limit is 2.000s")

    return RealtimeMetrics(
        source_active_s=source_active_s,
        drain_elapsed_s=drain_elapsed_s,
        receive_to_commit_p99_ms=p99_ms,
        receive_to_commit_max_ms=max_ms,
        initial_backlog_median_ms=initial_median * 1e3,
        final_backlog_median_ms=final_median * 1e3,
        final_backlog_age_ms=final_age * 1e3,
        recovery_s=recovery_s,
        passed=not violations,
        violations=tuple(violations),
    )


def build_device_write_metrics(
    samples: Sequence[RealtimeSample], source_active_s: float
) -> DeviceWriteMetrics:
    """Summarize device-wide deltas over the source-active window."""

    active = [sample for sample in samples if sample.source_active and sample.device_io]
    if len(active) < 2 or source_active_s <= 0:
        return DeviceWriteMetrics(
            available=False,
            unavailable_reason="backing block device counters unavailable",
        )
    first, last = active[0], active[-1]
    if first.device_io.get("device") != last.device_io.get("device"):
        return DeviceWriteMetrics(available=False, unavailable_reason="backing device changed")
    elapsed = max(last.elapsed_s - first.elapsed_s, 1e-9)
    writes = max(
        0,
        int(last.device_io.get("writes_completed", 0))
        - int(first.device_io.get("writes_completed", 0)),
    )
    sectors = max(
        0,
        int(last.device_io.get("sectors_written", 0))
        - int(first.device_io.get("sectors_written", 0)),
    )
    write_time_ms = max(
        0,
        int(last.device_io.get("write_time_ms", 0)) - int(first.device_io.get("write_time_ms", 0)),
    )
    io_time_ms = max(
        0,
        int(last.device_io.get("io_time_ms", 0)) - int(first.device_io.get("io_time_ms", 0)),
    )
    first_pressure = int(first.io_pressure.get("some_total", 0))
    last_pressure = int(last.io_pressure.get("some_total", 0))
    return DeviceWriteMetrics(
        available=True,
        device=str(first.device_io["device"]),
        write_bytes=sectors * 512,
        write_mib_s=sectors * 512 / elapsed / (1024 * 1024),
        write_iops=writes / elapsed,
        average_write_time_ms=write_time_ms / writes if writes else 0.0,
        utilization=io_time_ms / (elapsed * 1_000),
        io_pressure_stall_s=max(0, last_pressure - first_pressure) / 1e6,
    )


def environment_metadata() -> dict[str, str | int]:
    return {
        "platform": platform.platform(),
        "machine": platform.machine(),
        "python": sys.version.split()[0],
        "cpu_count": 0 if (count := os.cpu_count()) is None else count,
    }


def summarize_timings(values: Mapping[str, Sequence[float]]) -> dict[str, dict[str, float | int]]:
    result: dict[str, dict[str, float | int]] = {}
    for name, durations in values.items():
        if not durations:
            continue
        result[name] = {
            "count": len(durations),
            "mean_ms": statistics.fmean(durations) * 1_000,
            "p50_ms": (percentile(durations, 0.50) or 0.0) * 1_000,
            "p95_ms": (percentile(durations, 0.95) or 0.0) * 1_000,
            "p99_ms": (percentile(durations, 0.99) or 0.0) * 1_000,
            "max_ms": max(durations) * 1_000,
        }
    return result


def render_report(report: FidelityReport) -> str:
    """Render a compact operator-facing summary."""

    lines = [
        f"Recorder fidelity: {'PASS' if report.successful else 'FAIL'}",
        f"profile={report.profile} mode={report.mode} duration={report.duration_s:.1f}s",
        "",
        "stream                 source   receive   stored   missing   corrupt   max-gap",
    ]
    for name, stream in report.streams.items():
        receive = "-" if stream.received_count is None else str(stream.received_count)
        max_gap = (
            "-" if stream.max_persisted_gap_s is None else f"{stream.max_persisted_gap_s:.3f}s"
        )
        lines.append(
            f"{name:<22} {stream.source_count:>7} {receive:>9} {stream.persisted_count:>8} "
            f"{len(stream.missing_before_persist):>9} {len(stream.corrupt_sequences):>9} {max_gap:>10}"
        )
    if report.shared_loss_windows:
        lines.append("")
        lines.append(
            "shared loss windows: "
            + ", ".join(f"{start:.3f}..{end:.3f}" for start, end in report.shared_loss_windows)
        )
    if report.bandwidth is not None:
        bandwidth = report.bandwidth
        block_rate = (
            "unavailable"
            if bandwidth.process_block_write_mib_s is None
            else f"{bandwidth.process_block_write_mib_s:.2f} MiB/s"
        )
        character_rate = (
            "unavailable"
            if bandwidth.process_character_write_mib_s is None
            else f"{bandwidth.process_character_write_mib_s:.2f} MiB/s"
        )
        lines.extend(
            (
                "",
                f"offered raw: {bandwidth.offered_raw_mib_s:.2f} MiB/s",
                f"persisted codec payload: {bandwidth.persisted_payload_mib_s:.2f} MiB/s",
                f"SQLite growth: {bandwidth.sqlite_growth_mib_s:.2f} MiB/s "
                f"({bandwidth.sqlite_final_mib:.2f} MiB final)",
                f"recorder write-syscall bytes: {character_rate}",
                f"recorder process block writes: {block_rate}",
            )
        )
    if report.realtime is not None:
        realtime = report.realtime
        lines.extend(
            (
                "",
                f"receive-to-commit p99: {realtime.receive_to_commit_p99_ms:.2f} ms",
                f"receive-to-commit max: {realtime.receive_to_commit_max_ms:.2f} ms",
                f"source active / drain: {realtime.source_active_s:.3f}s / "
                f"{realtime.drain_elapsed_s:.3f}s",
                f"realtime SLA: {'PASS' if realtime.passed else 'FAIL'}",
            )
        )
        lines.extend(f"  - {violation}" for violation in realtime.violations)
    if report.device is not None:
        if report.device.available:
            lines.append(
                f"device {report.device.device}: {report.device.write_mib_s:.2f} MiB/s, "
                f"{report.device.average_write_time_ms:.2f} ms/write, "
                f"{report.device.utilization:.1%} utilized"
            )
        else:
            lines.append(f"device writes: unavailable ({report.device.unavailable_reason})")
    return "\n".join(lines)


def _stream_kind(value: Any) -> StreamKind | None:
    if isinstance(value, Image):
        if value.format in {ImageFormat.RGB, ImageFormat.BGR, ImageFormat.RGBA, ImageFormat.BGRA}:
            return "rgb"
        if value.format in {ImageFormat.DEPTH, ImageFormat.DEPTH16}:
            return "depth"
        return "grayscale"
    if isinstance(value, Imu):
        return "imu"
    if isinstance(value, PointCloud2):
        return "pointcloud"
    if isinstance(value, Odometry):
        return "odometry"
    if isinstance(value, TFMessage):
        return "tf"
    return None


def calibrate_recording(path: Path, *, sample_limit: int = 64) -> WorkloadProfile:
    """Extract rates, payload shapes, codecs, and byte sizes from a mem2 DB."""

    store = SqliteStore(path=str(path), must_exist=True)
    streams: list[StreamProfile] = []
    try:
        for name in store.list_streams():
            stream: Stream[Any] = store.stream(name)
            observations = list(stream.limit(sample_limit))
            if not observations:
                continue
            first = observations[0].data
            kind = _stream_kind(first)
            if kind is None:
                continue
            timestamps = [obs.ts for obs in observations]
            intervals = [right - left for left, right in pairwise(timestamps) if right > left]
            rate_hz = 1.0 / statistics.median(intervals) if intervals else 1.0
            stored = store._registry.get(name)  # type: ignore[attr-defined]
            codec = "lcm" if stored is None else str(stored["codec_id"])
            size = stream.size_bytes() or 0
            count = max(stream.count(), 1)
            raw = len(first.lcm_encode()) if hasattr(first, "lcm_encode") else 0
            kwargs: dict[str, Any] = {}
            if isinstance(first, Image):
                kwargs.update(shape=tuple(first.data.shape), dtype=str(first.data.dtype))
                raw = int(first.data.nbytes)
            elif kind == "pointcloud":
                tensor = first.pointcloud_tensor
                kwargs["point_count"] = (
                    int(tensor.point["positions"].shape[0]) if "positions" in tensor.point else 0
                )
            streams.append(
                StreamProfile(
                    name=name,
                    kind=kind,
                    payload_type=f"{type(first).__module__}.{type(first).__qualname__}",
                    rate_hz=rate_hz,
                    transport="shm"
                    if kind in {"rgb", "grayscale", "depth", "pointcloud"}
                    else "lcm",
                    codec=codec,
                    frame_id=getattr(first, "frame_id", "world"),
                    raw_bytes=raw,
                    encoded_bytes=round(size / count),
                    **kwargs,
                )
            )
    finally:
        store.stop()
    if not streams:
        raise ValueError(f"no supported recorder streams found in {path}")
    return WorkloadProfile(
        name=f"calibrated-{path.stem}",
        streams=tuple(streams),
        calibration_source=path.name,
    )


def load_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text())  # type: ignore[no-any-return]
