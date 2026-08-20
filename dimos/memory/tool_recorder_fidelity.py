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

"""Generate, record, and verify a production-shaped mem2 workload.

Examples::

    pytest -s dimos/memory/tool_recorder_fidelity.py -- --help
    python -m dimos.memory.tool_recorder_fidelity write-default profile.json
    python -m dimos.memory.tool_recorder_fidelity calibrate recording.db profile.json
    python -m dimos.memory.tool_recorder_fidelity run profile.json --duration 300

The harness subclasses Recorder only to observe stage boundaries and inject
controlled stalls.  It does not change the production Recorder implementation.
"""

from __future__ import annotations

import argparse
from collections import defaultdict
from collections.abc import Callable, Sequence
import json
from pathlib import Path
import sqlite3
import threading
import time
from typing import Any, Literal, cast

import numpy as np
from reactivex.disposable import Disposable

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.core.core import rpc
from dimos.core.global_config import GlobalConfig
from dimos.core.module import Module
from dimos.core.stream import In, Out
from dimos.core.transport import LCMTransport, pSHMQueueTransport
from dimos.core.transport_factory import make_transport
from dimos.memory.backend import Backend
from dimos.memory.codecs.base import Codec
from dimos.memory.module import Recorder, RecorderConfig
from dimos.memory.recorder_fidelity import (
    FidelityReport,
    SourceRun,
    StreamProfile,
    WorkloadProfile,
    build_bandwidth_metrics,
    calibrate_recording,
    check_source_conformance,
    codec_roundtrip,
    compare_stream,
    counter_delta,
    default_profile,
    environment_metadata,
    payload_digest,
    persisted_payload_sizes,
    persisted_samples,
    read_process_io,
    render_report,
    shared_loss_windows,
    sqlite_file_sizes,
    summarize_timings,
)
from dimos.memory.stream import Stream
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.tf2_msgs.TFMessage import TFMessage

Mode = Literal["baseline", "encoder-stall", "sqlite-lock"]
StorageControlMode = Literal["encoder-control", "split-db-control", "batch-small-control"]


class SyntheticSource(Module):
    """Publish the seven canonical streams from independent scheduler threads."""

    rgb: Out[Image]
    grayscale_left: Out[Image]
    grayscale_right: Out[Image]
    depth: Out[Image]
    imu: Out[Imu]
    pointlio: Out[PointCloud2]
    odometry: Out[Odometry]
    tf: Out[TFMessage]

    dedicated_worker = True

    @rpc
    def run_profile(self, profile_data: dict[str, Any], duration_s: float) -> dict[str, Any]:
        profile = WorkloadProfile.model_validate(profile_data)
        # Import heavy message dependencies and allocate reusable payloads before
        # the source clock starts. First-use cost is not part of sensor cadence.
        for stream in profile.streams:
            make_message(stream, 0, 1_000_000_000, profile.seed)
        base_ts_ns = (time.time_ns() // 1_000_000_000 + 1) * 1_000_000_000
        start_monotonic_ns = time.monotonic_ns() + 250_000_000
        samples: dict[str, list[tuple[int, int, int]]] = defaultdict(list)
        scheduled_counts: dict[str, int] = {}
        failures: list[BaseException] = []
        lock = threading.Lock()

        def publish_stream(stream: StreamProfile) -> None:
            try:
                count = max(1, round(duration_s * stream.rate_hz))
                scheduled_counts[stream.name] = count
                period_ns = round(1e9 / stream.rate_hz)
                sequence = 0
                while sequence < count:
                    deadline = start_monotonic_ns + sequence * period_ns
                    now = time.monotonic_ns()
                    if now < deadline:
                        time.sleep((deadline - now) / 1e9)
                        now = time.monotonic_ns()
                    elif now - deadline >= period_ns:
                        # Skip missed deadlines instead of publishing a catch-up burst.
                        sequence += (now - deadline) // period_ns
                        if sequence >= count:
                            break
                        deadline = start_monotonic_ns + sequence * period_ns
                    source_ts_ns = base_ts_ns + sequence * period_ns
                    message = make_message(stream, sequence, source_ts_ns, profile.seed)
                    cast("Out[Any]", getattr(self, stream.name)).publish(message)
                    published_ns = time.monotonic_ns()
                    with lock:
                        samples[stream.name].append((sequence, source_ts_ns, published_ns))
                    sequence += 1
            except BaseException as error:
                with lock:
                    failures.append(error)

        threads = [
            threading.Thread(target=publish_stream, args=(stream,), name=f"source-{stream.name}")
            for stream in profile.streams
        ]
        for thread in threads:
            thread.start()
        for thread in threads:
            thread.join()
        if failures:
            raise RuntimeError(f"source publisher failed: {failures[0]}") from failures[0]
        return SourceRun(
            duration_s=duration_s,
            samples=dict(samples),
            scheduled_counts=scheduled_counts,
        ).model_dump()


class FidelityRecorderConfig(RecorderConfig):
    stall_stream: str | None = None
    stall_after_messages: int = 10
    stall_duration_s: float = 1.0


class _TimingCodec:
    def __init__(
        self,
        inner: Codec[Any],
        record: Callable[[float, int], None],
        maybe_stall: Callable[[], None],
    ) -> None:
        self._inner = inner
        self._record = record
        self._maybe_stall = maybe_stall

    def encode(self, value: Any) -> bytes:
        self._maybe_stall()
        started = time.perf_counter()
        encoded = self._inner.encode(value)
        self._record(time.perf_counter() - started, len(encoded))
        return encoded

    def decode(self, data: bytes) -> Any:
        return self._inner.decode(data)


class _TimingBackend:
    def __init__(self, inner: Backend[Any], record: Callable[[float], None]) -> None:
        self._inner = inner
        self._record = record

    def append(self, observation: Any) -> Any:
        started = time.perf_counter()
        try:
            return self._inner.append(observation)
        finally:
            self._record(time.perf_counter() - started)

    def __getattr__(self, name: str) -> Any:
        return getattr(self._inner, name)


class FidelityRecorder(Recorder):
    """The real Recorder write path with test-only observation and timing taps."""

    config: FidelityRecorderConfig

    rgb: In[Image]
    grayscale_left: In[Image]
    grayscale_right: In[Image]
    depth: In[Image]
    imu: In[Imu]
    pointlio: In[PointCloud2]
    odometry: In[Odometry]

    dedicated_worker = True

    @rpc
    def start(self) -> None:
        self._measurement_started_ns = time.monotonic_ns()
        self._process_io_start = read_process_io()
        self._fidelity_lock = threading.Lock()
        self._received: dict[str, list[int]] = defaultdict(list)
        self._received_monotonic: dict[str, list[int]] = defaultdict(list)
        self._codec_s: dict[str, list[float]] = defaultdict(list)
        self._append_s: dict[str, list[float]] = defaultdict(list)
        self._encoded_bytes: dict[str, int] = defaultdict(int)
        self._stall_fired = False
        super().start()

    def _port_to_stream(self, name: str, input_topic: In[Any], stream: Stream[Any]) -> None:
        """Install taps around the same callback stages used by Recorder."""

        def on_received(message: Any) -> None:
            ts = getattr(message, "ts", None)
            if ts is None:
                return
            with self._fidelity_lock:
                self._received[name].append(round(float(ts) * 1e9))
                self._received_monotonic[name].append(time.monotonic_ns())

        self.register_disposable(Disposable(input_topic.subscribe(on_received)))

        backend = cast("Backend[Any]", stream._source)  # type: ignore[attr-defined]
        original_codec = backend.codec

        def maybe_stall() -> None:
            if self.config.stall_stream != name or self._stall_fired:
                return
            with self._fidelity_lock:
                count = len(self._received[name])
                if count < self.config.stall_after_messages or self._stall_fired:
                    return
                self._stall_fired = True
            time.sleep(self.config.stall_duration_s)

        def record_codec(duration: float, size: int) -> None:
            with self._fidelity_lock:
                self._codec_s[name].append(duration)
                self._encoded_bytes[name] += size

        backend.codec = _TimingCodec(original_codec, record_codec, maybe_stall)
        timed_backend = _TimingBackend(
            backend,
            lambda duration: self._record_append(name, duration),
        )
        stream._source = timed_backend  # type: ignore[assignment]
        super()._port_to_stream(name, input_topic, stream)

    def _record_append(self, name: str, duration: float) -> None:
        with self._fidelity_lock:
            self._append_s[name].append(duration)

    @rpc
    def fidelity_snapshot(self) -> dict[str, Any]:
        process_io = counter_delta(self._process_io_start, read_process_io())
        active_files = sqlite_file_sizes(Path(self.config.db_path))
        measurement_elapsed_s = (time.monotonic_ns() - self._measurement_started_ns) / 1e9
        with self._fidelity_lock:
            return {
                "received": {name: list(values) for name, values in self._received.items()},
                "received_monotonic": {
                    name: list(values) for name, values in self._received_monotonic.items()
                },
                "codec_s": {name: list(values) for name, values in self._codec_s.items()},
                "append_s": {name: list(values) for name, values in self._append_s.items()},
                "encoded_bytes": dict(self._encoded_bytes),
                "stall_fired": self._stall_fired,
                "measurement_elapsed_s": measurement_elapsed_s,
                "process_io": process_io,
                "sqlite_active_files": active_files,
                "recorder": super().recording_status(),
            }


def _image_template(stream: StreamProfile, seed: int) -> np.ndarray[Any, np.dtype[Any]]:
    assert stream.shape is not None and stream.dtype is not None
    rng = np.random.default_rng(seed + sum(stream.name.encode()))
    dtype = np.dtype(stream.dtype)
    if np.issubdtype(dtype, np.integer):
        maximum = np.iinfo(dtype).max
        # A gradient plus moderate noise resembles compressible sensor data better
        # than all-zero or full-entropy arrays.
        x = np.linspace(0, maximum, stream.shape[1], dtype=np.float64)
        base = np.broadcast_to(x, stream.shape[:2])
        noise = rng.normal(0, maximum / 64, stream.shape[:2])
        array = np.clip(base + noise, 0, maximum).astype(dtype)
    else:
        array = rng.random(stream.shape[:2], dtype=np.float32).astype(dtype)
    if len(stream.shape) == 3:
        array = np.repeat(array[..., None], stream.shape[2], axis=2)
    return np.ascontiguousarray(array)


_MESSAGE_TEMPLATES: dict[tuple[str, int, tuple[int, ...] | None, str | None, int | None], Any] = {}


def make_message(stream: StreamProfile, sequence: int, ts_ns: int, seed: int) -> Any:
    """Build deterministic content; only timestamp and motion fields vary."""

    ts = ts_ns / 1e9
    key = (stream.name, seed, stream.shape, stream.dtype, stream.point_count)
    if stream.kind in {"rgb", "grayscale", "depth"}:
        array = _MESSAGE_TEMPLATES.setdefault(key, _image_template(stream, seed))
        formats = {
            "rgb": ImageFormat.RGB,
            "grayscale": ImageFormat.GRAY16
            if np.dtype(stream.dtype or "uint8").itemsize == 2
            else ImageFormat.GRAY,
            "depth": ImageFormat.DEPTH16
            if np.dtype(stream.dtype or "uint16").itemsize == 2
            else ImageFormat.DEPTH,
        }
        return Image(data=array, format=formats[stream.kind], frame_id=stream.frame_id, ts=ts)
    if stream.kind == "imu":
        phase = sequence / max(stream.rate_hz, 1.0)
        return Imu(
            angular_velocity=Vector3(phase, phase * 0.5, -phase),
            linear_acceleration=Vector3(0.0, 0.0, 9.81),
            frame_id=stream.frame_id,
            ts=ts,
        )
    if stream.kind == "odometry":
        return Odometry(
            ts=ts,
            frame_id=stream.frame_id,
            child_frame_id="base_link",
            pose=Pose(sequence / stream.rate_hz, 0.0, 0.0),
        )
    if stream.kind == "pointcloud":
        template = _MESSAGE_TEMPLATES.get(key)
        if template is None:
            rng = np.random.default_rng(seed + 101)
            points = rng.normal(size=(stream.point_count or 0, 3)).astype(np.float32)
            template = PointCloud2.from_numpy(points, frame_id=stream.frame_id, timestamp=ts)
            _MESSAGE_TEMPLATES[key] = template
        # The payload tensor is immutable for this workload, but the timestamp
        # is per observation. Return a distinct message wrapper so an accepted
        # publication can never be changed by the source's next tick while a
        # deliberately stalled Recorder still holds it.
        return PointCloud2(template._pcd_tensor, frame_id=stream.frame_id, ts=ts)
    if stream.kind == "tf":
        data_frames = (
            "rgb_link",
            "grayscale_left_link",
            "grayscale_right_link",
            "depth_link",
            "imu_link",
            "lidar_link",
            "base_link",
        )
        child = data_frames[sequence % len(data_frames)]
        return TFMessage(Transform(frame_id="world", child_frame_id=child, ts=ts))
    raise ValueError(f"unsupported stream kind: {stream.kind}")


def _message_type(stream: StreamProfile) -> type[Any]:
    return {
        "rgb": Image,
        "grayscale": Image,
        "depth": Image,
        "imu": Imu,
        "pointcloud": PointCloud2,
        "odometry": Odometry,
        "tf": TFMessage,
    }[stream.kind]


def _transport(stream: StreamProfile) -> Any:
    topic = f"/recorder-fidelity/{stream.name}"
    message_type = _message_type(stream)
    if stream.transport == "shm":
        capacity = max(3_686_400, stream.raw_bytes * 2, stream.encoded_bytes * 2)
        slots = max(8, round(stream.rate_hz * 2.5))
        return pSHMQueueTransport(topic, default_capacity=capacity, slots=slots)
    if stream.transport == "zenoh":
        return make_transport(topic, message_type, g=GlobalConfig(transport="zenoh"))
    return LCMTransport(topic, message_type)


def _canonical_profile(profile: WorkloadProfile) -> WorkloadProfile:
    """Map calibrated names onto the fixed source/recorder port vocabulary."""

    expected = {
        "rgb": ["rgb"],
        "grayscale": ["grayscale_left", "grayscale_right"],
        "depth": ["depth"],
        "imu": ["imu"],
        "pointcloud": ["pointlio"],
        "odometry": ["odometry"],
        "tf": ["tf"],
    }
    seen: dict[str, int] = defaultdict(int)
    streams: list[StreamProfile] = []
    for stream in profile.streams:
        choices = expected[stream.kind]
        index = seen[stream.kind]
        if index >= len(choices):
            continue
        seen[stream.kind] += 1
        streams.append(stream.model_copy(update={"name": choices[index]}))
    missing = [
        name
        for choices in expected.values()
        for name in choices
        if name not in {s.name for s in streams}
    ]
    if missing:
        raise ValueError(f"profile does not define the required streams: {missing}")
    return profile.model_copy(update={"streams": tuple(streams)})


def _wait_for_quiet(recorder: Any, *, timeout_s: float = 10.0) -> dict[str, Any]:
    deadline = time.monotonic() + timeout_s
    previous = -1
    unchanged_since = time.monotonic()
    snapshot: dict[str, Any] = {}
    while time.monotonic() < deadline:
        snapshot = recorder.fidelity_snapshot()
        count = sum(len(values) for values in snapshot["append_s"].values())
        if count != previous:
            previous = count
            unchanged_since = time.monotonic()
        elif time.monotonic() - unchanged_since >= 0.5:
            return snapshot
        time.sleep(0.05)
    return snapshot


def _expected_digests(
    profile: WorkloadProfile,
    source: SourceRun,
) -> dict[str, dict[int, str]]:
    result: dict[str, dict[int, str]] = {}
    for stream in profile.streams:
        digests: dict[int, str] = {}
        for sample in source.published(stream.name):
            message = make_message(stream, sample.sequence, sample.source_ts_ns, profile.seed)
            expected = codec_roundtrip(message, stream)
            digests[sample.sequence] = payload_digest(expected)
        result[stream.name] = digests
    return result


def run_harness(
    profile: WorkloadProfile,
    *,
    duration_s: float,
    output_dir: Path,
    mode: Mode = "baseline",
    stall_duration_s: float = 1.0,
) -> FidelityReport:
    """Run the source and Recorder in separate workers and verify the DB."""

    profile = _canonical_profile(profile)
    output_dir.mkdir(parents=True, exist_ok=True)
    db_path = output_dir / "recording.db"
    transport_map = {
        (stream.name, _message_type(stream)): _transport(stream) for stream in profile.streams
    }
    codecs = {stream.name: stream.codec for stream in profile.streams if stream.kind != "tf"}
    stall_stream = "depth" if mode == "encoder-stall" else None
    blueprint = (
        autoconnect(
            SyntheticSource.blueprint(),
            FidelityRecorder.blueprint(
                db_path=str(db_path),
                record_tf=True,
                stream_codecs=codecs,
                stall_stream=stall_stream,
                stall_duration_s=stall_duration_s,
                default_frame_id="base_link",
                tf_tolerance=1.0,
            ),
        )
        .transports(transport_map)
        .global_config(n_workers=2, viewer="none")
    )
    coordinator = ModuleCoordinator.build(blueprint)
    lock_thread: threading.Thread | None = None
    try:
        if mode == "sqlite-lock":
            lock_thread = threading.Thread(
                target=_hold_writer_lock,
                args=(db_path, 0.75, stall_duration_s),
                daemon=True,
            )
            lock_thread.start()
        source_proxy = coordinator.get_instance(SyntheticSource)
        recorder_proxy = coordinator.get_instance(FidelityRecorder)
        source_run = SourceRun.model_validate(
            source_proxy.run_profile(profile.model_dump(), duration_s)
        )
        snapshot = _wait_for_quiet(recorder_proxy)
    finally:
        if lock_thread is not None:
            lock_thread.join(timeout=stall_duration_s + 2)
        coordinator.stop()

    persisted = persisted_samples(db_path, profile)
    payload_sizes = persisted_payload_sizes(db_path, profile)
    expected_digests = _expected_digests(profile, source_run)
    source_reports = {
        stream.name: check_source_conformance(
            stream,
            source_run.published(stream.name),
            source_run.scheduled_counts[stream.name],
        )
        for stream in profile.streams
    }
    stream_reports = {
        stream.name: compare_stream(
            stream.name,
            source_run.published(stream.name),
            persisted[stream.name],
            received_ts_ns=snapshot.get("received", {}).get(stream.name),
            expected_digests=expected_digests[stream.name],
        )
        for stream in profile.streams
    }
    source_samples = {stream.name: source_run.published(stream.name) for stream in profile.streams}
    published_counts = {
        stream.name: len(source_run.published(stream.name)) for stream in profile.streams
    }
    persisted_counts = {name: len(samples) for name, samples in persisted.items()}
    bandwidth = build_bandwidth_metrics(
        profile,
        duration_s=duration_s,
        measurement_elapsed_s=float(snapshot.get("measurement_elapsed_s", duration_s)),
        published_counts=published_counts,
        persisted_counts=persisted_counts,
        persisted_bytes=payload_sizes,
        active_files=snapshot.get("sqlite_active_files", {}),
        final_files=sqlite_file_sizes(db_path),
        process_io=snapshot.get("process_io", {}),
    )
    report = FidelityReport(
        profile=profile.name,
        mode=mode,
        duration_s=duration_s,
        source_valid=all(item.valid for item in source_reports.values()),
        faithful=all(item.faithful for item in stream_reports.values()),
        source=source_reports,
        streams=stream_reports,
        shared_loss_windows=shared_loss_windows(source_samples, stream_reports),
        timings={
            **{
                f"codec/{name}": values
                for name, values in summarize_timings(snapshot.get("codec_s", {})).items()
            },
            **{
                f"append/{name}": values
                for name, values in summarize_timings(snapshot.get("append_s", {})).items()
            },
        },
        bandwidth=bandwidth,
        environment=environment_metadata(),
        recorder=snapshot.get("recorder", {}),
    )
    report.write(output_dir / "report.json")
    (output_dir / "source_manifest.json").write_text(source_run.model_dump_json(indent=2) + "\n")
    return report


def _hold_writer_lock(path: Path, delay_s: float, duration_s: float) -> None:
    time.sleep(delay_s)
    connection = sqlite3.connect(path, timeout=10.0)
    try:
        connection.execute("BEGIN IMMEDIATE")
        time.sleep(duration_s)
        connection.rollback()
    finally:
        connection.close()


def run_storage_control(
    profile: WorkloadProfile,
    *,
    duration_s: float,
    output_dir: Path,
    mode: StorageControlMode,
) -> dict[str, Any]:
    """Write baseline-sized blobs with one storage factor changed.

    These controls do not pretend to be Recorder implementations. They answer
    narrower questions: whether compression-sized bytes fit through SQLite,
    whether the shared database is the source of writer stalls, and whether
    high-rate telemetry transactions dominate commit overhead.
    """

    profile = _canonical_profile(profile)
    output_dir.mkdir(parents=True, exist_ok=True)
    shared_path = output_dir / f"{mode}.db"
    paths = {
        stream.name: output_dir / f"{mode}-{stream.name}.db"
        if mode == "split-db-control"
        else shared_path
        for stream in profile.streams
    }
    for path in set(paths.values()):
        connection = sqlite3.connect(path)
        try:
            connection.execute("PRAGMA journal_mode=WAL")
            connection.execute("PRAGMA synchronous=NORMAL")
            for stream in profile.streams:
                if paths[stream.name] == path:
                    connection.execute(
                        f'CREATE TABLE IF NOT EXISTS "{stream.name}" '
                        "(sequence INTEGER PRIMARY KEY, ts_ns INTEGER NOT NULL, data BLOB NOT NULL)"
                    )
            connection.commit()
        finally:
            connection.close()

    started_ns = time.monotonic_ns() + 100_000_000
    lock = threading.Lock()
    commit_s: dict[str, list[float]] = defaultdict(list)
    rows: dict[str, int] = defaultdict(int)
    failures: list[BaseException] = []

    def write_stream(stream: StreamProfile) -> None:
        connection = sqlite3.connect(paths[stream.name], timeout=30.0)
        connection.execute("PRAGMA synchronous=NORMAL")
        payload = bytes(max(stream.encoded_bytes, 1))
        total = max(1, round(duration_s * stream.rate_hz))
        period_ns = round(1e9 / stream.rate_hz)
        batch_size = (
            min(64, max(1, round(stream.rate_hz * 0.025)))
            if mode == "batch-small-control" and stream.kind in {"imu", "odometry"}
            else 1
        )
        pending: list[tuple[int, int, bytes]] = []
        try:
            for sequence in range(total):
                deadline = started_ns + sequence * period_ns
                now = time.monotonic_ns()
                if now < deadline:
                    time.sleep((deadline - now) / 1e9)
                pending.append((sequence, sequence * period_ns, payload))
                if len(pending) < batch_size and sequence + 1 < total:
                    continue
                commit_started = time.perf_counter()
                connection.executemany(
                    f'INSERT INTO "{stream.name}" (sequence, ts_ns, data) VALUES (?, ?, ?)',
                    pending,
                )
                connection.commit()
                duration = time.perf_counter() - commit_started
                with lock:
                    commit_s[stream.name].append(duration)
                    rows[stream.name] += len(pending)
                pending.clear()
        except BaseException as error:
            with lock:
                failures.append(error)
        finally:
            connection.close()

    threads = [
        threading.Thread(target=write_stream, args=(stream,), name=f"control-{stream.name}")
        for stream in profile.streams
    ]
    wall_started = time.perf_counter()
    for thread in threads:
        thread.start()
    for thread in threads:
        thread.join()
    wall_s = time.perf_counter() - wall_started
    if failures:
        raise RuntimeError(f"storage control failed: {failures[0]}") from failures[0]

    result: dict[str, Any] = {
        "mode": mode,
        "requested_duration_s": duration_s,
        "wall_duration_s": wall_s,
        "rows": dict(rows),
        "transactions": {name: len(values) for name, values in commit_s.items()},
        "commit_timings": summarize_timings(commit_s),
        "database_mib": sum(path.stat().st_size for path in set(paths.values())) / (1024 * 1024),
        "encoded_mib_s": sum(stream.encoded_bytes * rows[stream.name] for stream in profile.streams)
        / wall_s
        / (1024 * 1024),
    }
    (output_dir / f"{mode}.json").write_text(json.dumps(result, indent=2) + "\n")
    return result


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    write_default = commands.add_parser("write-default", help="write the built-in profile")
    write_default.add_argument("output", type=Path)
    calibrate = commands.add_parser("calibrate", help="derive a profile from a mem2 DB")
    calibrate.add_argument("recording", type=Path)
    calibrate.add_argument("output", type=Path)
    run = commands.add_parser("run", help="run one end-to-end workload")
    run.add_argument("profile", type=Path)
    run.add_argument("--duration", type=float, default=300.0)
    run.add_argument("--output", type=Path, default=Path("recorder-fidelity-results"))
    run.add_argument(
        "--mode", choices=("baseline", "encoder-stall", "sqlite-lock"), default="baseline"
    )
    run.add_argument("--transport", choices=("lcm", "shm", "zenoh"))
    run.add_argument("--stall-duration", type=float, default=1.0)
    control = commands.add_parser("control", help="run one storage-only hypothesis control")
    control.add_argument("profile", type=Path)
    control.add_argument(
        "--mode",
        choices=("encoder-control", "split-db-control", "batch-small-control"),
        required=True,
    )
    control.add_argument("--duration", type=float, default=60.0)
    control.add_argument("--output", type=Path, default=Path("recorder-fidelity-results"))
    matrix = commands.add_parser("matrix", help="run baseline plus the three targeted controls")
    matrix.add_argument("profile", type=Path)
    matrix.add_argument("--duration", type=float, default=60.0)
    matrix.add_argument("--output", type=Path, default=Path("recorder-fidelity-results"))
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    if args.command == "write-default":
        default_profile().write(args.output)
        return 0
    if args.command == "calibrate":
        calibrate_recording(args.recording).write(args.output)
        return 0
    profile = WorkloadProfile.read(args.profile)
    if args.command == "control":
        result = run_storage_control(
            profile,
            duration_s=args.duration,
            output_dir=args.output,
            mode=cast("StorageControlMode", args.mode),
        )
        print(json.dumps(result, indent=2))
        return 0
    if args.command == "matrix":
        baseline = run_harness(
            profile,
            duration_s=args.duration,
            output_dir=args.output / "baseline",
        )
        storage_modes: tuple[StorageControlMode, ...] = (
            "encoder-control",
            "split-db-control",
            "batch-small-control",
        )
        controls = {
            mode: run_storage_control(
                profile,
                duration_s=args.duration,
                output_dir=args.output / mode,
                mode=mode,
            )
            for mode in storage_modes
        }
        (args.output / "matrix.json").write_text(
            json.dumps(
                {"baseline": baseline.model_dump(mode="json"), "controls": controls},
                indent=2,
            )
            + "\n"
        )
        print(render_report(baseline))
        return 0 if baseline.source_valid and baseline.faithful else 1
    if args.transport:
        profile = profile.with_transport(args.transport)
    report = run_harness(
        profile,
        duration_s=args.duration,
        output_dir=args.output,
        mode=args.mode,
        stall_duration_s=args.stall_duration,
    )
    print(render_report(report))
    return 0 if report.source_valid and report.faithful else 1


if __name__ == "__main__":
    raise SystemExit(main())
