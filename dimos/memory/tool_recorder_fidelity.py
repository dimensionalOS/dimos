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
from dataclasses import asdict
import json
from pathlib import Path
import sqlite3
import threading
import time
from typing import Any, Literal, cast
import uuid

import numpy as np

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.core.core import rpc
from dimos.core.global_config import GlobalConfig
from dimos.core.module import Module
from dimos.core.stream import In, Out
from dimos.core.transport import LCMTransport, pSHMQueueTransport
from dimos.core.transport_factory import make_transport
from dimos.memory.backend import Backend, PreparedAppend
from dimos.memory.codecs.base import Codec, resolve_payload_type
from dimos.memory.module import Recorder, RecorderConfig
from dimos.memory.record_writer import RecordWriter
from dimos.memory.recorder_fidelity import (
    FidelityReport,
    RealtimeSample,
    SourceRun,
    StorageCapacityReport,
    StreamProfile,
    WorkloadProfile,
    build_bandwidth_metrics,
    build_device_write_metrics,
    build_realtime_metrics,
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
    read_device_io,
    read_io_pressure,
    read_process_io,
    render_report,
    shared_loss_windows,
    sqlite_file_sizes,
    summarize_timings,
)
from dimos.memory.recorder_queue import RecorderFailedError
from dimos.memory.store.sqlite import SqliteStore
from dimos.memory.stream import Stream
from dimos.memory.type.observation import Observation
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.tf2_msgs.TFMessage import TFMessage

Mode = Literal["baseline", "encoder-stall", "sqlite-lock", "bounded-preparation"]
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
    sample_interval_s: float = 0.1


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
        self._measurement_started_ns: int | None = None
        self._source_started_ns: int | None = None
        self._source_ended_ns: int | None = None
        self._process_io_start: dict[str, int] = {}
        self._fidelity_lock = threading.Lock()
        self._received: dict[str, list[int]] = defaultdict(list)
        self._received_monotonic: dict[str, list[int]] = defaultdict(list)
        self._codec_s: dict[str, list[float]] = defaultdict(list)
        self._append_s: dict[str, list[float]] = defaultdict(list)
        self._stage_s: dict[str, list[float]] = defaultdict(list)
        self._encoded_bytes: dict[str, int] = defaultdict(int)
        self._stall_fired = False
        self._stall_started_ns: int | None = None
        self._stall_ended_ns: int | None = None
        self._samples: list[dict[str, Any]] = []
        self._sampling_stop = threading.Event()
        self._sampling_thread: threading.Thread | None = None
        super().start()

    @rpc
    def begin_measurement(self) -> None:
        """Start low-overhead recorder and storage sampling."""
        self._measurement_started_ns = time.monotonic_ns()
        self._process_io_start = read_process_io()
        self._sampling_stop.clear()
        self._sampling_thread = threading.Thread(
            target=self._sample_loop,
            name="recorder-fidelity-sampler",
            daemon=True,
        )
        self._sampling_thread.start()

    @rpc
    def end_measurement(self, source_started_ns: int, source_ended_ns: int) -> None:
        """Record the exact source-active bounds in the shared monotonic clock."""
        self._source_started_ns = source_started_ns
        self._source_ended_ns = source_ended_ns

    def _sample_loop(self) -> None:
        while not self._sampling_stop.is_set():
            self._take_sample()
            self._sampling_stop.wait(self.config.sample_interval_s)

    def _take_sample(self) -> None:
        now_ns = time.monotonic_ns()
        queues = {
            name: recording_queue.status().__dict__
            for name, recording_queue in self._recording_queues.items()
        }
        sample = {
            "monotonic_ns": now_ns,
            "queues": queues,
            "writer": self._record_writer.live_status(),
            "sqlite_files": sqlite_file_sizes(Path(self.config.db_path)),
            "process_io": counter_delta(self._process_io_start, read_process_io()),
            "process_cpu_s": time.process_time(),
            "device_io": read_device_io(Path(self.config.db_path).parent),
            "io_pressure": read_io_pressure(),
        }
        with self._fidelity_lock:
            self._samples.append(sample)

    def _port_to_stream(self, name: str, input_topic: In[Any], stream: Stream[Any]) -> None:
        """Install taps around the same callback stages used by Recorder."""
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
                self._stall_started_ns = time.monotonic_ns()
            time.sleep(self.config.stall_duration_s)
            with self._fidelity_lock:
                self._stall_ended_ns = time.monotonic_ns()

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

    def _on_recording_received(self, name: str, message: Any) -> None:
        ts = getattr(message, "ts", None)
        if ts is None:
            return
        with self._fidelity_lock:
            self._received[name].append(round(float(ts) * 1e9))
            self._received_monotonic[name].append(time.monotonic_ns())

    def _record_append(self, name: str, duration: float) -> None:
        with self._fidelity_lock:
            self._append_s[name].append(duration)

    def _on_recording_stage(self, name: str, stage: str, duration_s: float) -> None:
        with self._fidelity_lock:
            self._stage_s[f"{stage}/{name}"].append(duration_s)

    @rpc
    def fidelity_snapshot(self) -> dict[str, Any]:
        if self._measurement_started_ns is None:
            raise RuntimeError("measurement has not started")
        self._take_sample()
        process_io = counter_delta(self._process_io_start, read_process_io())
        active_files = sqlite_file_sizes(Path(self.config.db_path))
        now_ns = time.monotonic_ns()
        source_started_ns = self._source_started_ns or self._measurement_started_ns
        source_ended_ns = self._source_ended_ns or now_ns
        source_active_s = max(0.0, (source_ended_ns - source_started_ns) / 1e9)
        drain_elapsed_s = (
            max(0.0, (now_ns - source_ended_ns) / 1e9) if self._source_ended_ns is not None else 0.0
        )
        with self._fidelity_lock:
            samples = [
                {
                    "elapsed_s": (sample["monotonic_ns"] - source_started_ns) / 1e9,
                    "source_active": source_started_ns <= sample["monotonic_ns"] <= source_ended_ns,
                    **{key: value for key, value in sample.items() if key != "monotonic_ns"},
                }
                for sample in self._samples
            ]
            return {
                "received": {name: list(values) for name, values in self._received.items()},
                "received_monotonic": {
                    name: list(values) for name, values in self._received_monotonic.items()
                },
                "codec_s": {name: list(values) for name, values in self._codec_s.items()},
                "append_s": {name: list(values) for name, values in self._append_s.items()},
                "stage_s": {name: list(values) for name, values in self._stage_s.items()},
                "encoded_bytes": dict(self._encoded_bytes),
                "stall_fired": self._stall_fired,
                "stall_end_elapsed_s": (
                    (self._stall_ended_ns - source_started_ns) / 1e9
                    if self._stall_ended_ns is not None
                    else None
                ),
                "source_active_s": source_active_s,
                "drain_elapsed_s": drain_elapsed_s,
                "process_io": process_io,
                "sqlite_active_files": active_files,
                "recorder": super().recording_status(),
                "samples": samples,
            }

    @rpc
    def stop(self) -> None:
        self._sampling_stop.set()
        if self._sampling_thread is not None:
            self._sampling_thread.join(timeout=1.0)
        super().stop()


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


def _transport(stream: StreamProfile, run_id: str) -> Any:
    topic = f"/recorder-fidelity/{run_id}/{stream.name}"
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


def _wait_for_quiet(
    recorder: Any, *, expected_committed: int, timeout_s: float = 10.0
) -> dict[str, Any]:
    deadline = time.monotonic() + timeout_s
    snapshot: dict[str, Any] = {}
    while time.monotonic() < deadline:
        snapshot = recorder.fidelity_snapshot()
        status = snapshot["recorder"]
        queues = status["queues"].values()
        writer = status["writer"]
        if (
            all(queue_status["pending"] == 0 for queue_status in queues)
            and writer["pending"] == 0
            and writer["committed"] >= expected_committed
        ):
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
    # Keep typed LCM channels below its fixed channel-name limit after the
    # ``#module.MessageType`` suffix is appended by Topic.__str__.
    run_id = uuid.uuid4().hex[:8]
    transport_map = {
        (stream.name, _message_type(stream)): _transport(stream, run_id)
        for stream in profile.streams
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
                preparation_concurrency=2 if mode == "bounded-preparation" else None,
            ),
        )
        .transports(transport_map)
        .global_config(n_workers=2, viewer="none")
    )
    coordinator = ModuleCoordinator.build(blueprint)
    lock_thread: threading.Thread | None = None
    lock_window_ns: list[int] = []
    try:
        source_proxy = coordinator.get_instance(SyntheticSource)
        recorder_proxy = coordinator.get_instance(FidelityRecorder)
        recorder_proxy.begin_measurement()
        if mode == "sqlite-lock":
            lock_thread = threading.Thread(
                target=_hold_writer_lock,
                args=(db_path, 0.75, stall_duration_s, lock_window_ns),
                daemon=True,
            )
            lock_thread.start()
        source_run = SourceRun.model_validate(
            source_proxy.run_profile(profile.model_dump(), duration_s)
        )
        published = [
            sample for stream in profile.streams for sample in source_run.published(stream.name)
        ]
        source_started_ns = min(sample.published_monotonic_ns for sample in published)
        source_ended_ns = max(sample.published_monotonic_ns for sample in published)
        recorder_proxy.end_measurement(source_started_ns, source_ended_ns)
        snapshot = _wait_for_quiet(
            recorder_proxy,
            expected_committed=len(published),
        )
        if len(lock_window_ns) == 2:
            snapshot["stall_end_elapsed_s"] = (lock_window_ns[1] - source_started_ns) / 1e9
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
    source_active_s = float(snapshot.get("source_active_s", duration_s))
    drain_elapsed_s = float(snapshot.get("drain_elapsed_s", 0.0))
    samples = tuple(RealtimeSample.model_validate(sample) for sample in snapshot.get("samples", []))
    writer_status = snapshot.get("recorder", {}).get("writer", {})
    bandwidth = build_bandwidth_metrics(
        profile,
        duration_s=source_active_s,
        measurement_elapsed_s=source_active_s,
        drain_elapsed_s=drain_elapsed_s,
        published_counts=published_counts,
        persisted_counts=persisted_counts,
        persisted_bytes=payload_sizes,
        active_files=snapshot.get("sqlite_active_files", {}),
        final_files=sqlite_file_sizes(db_path),
        process_io=snapshot.get("process_io", {}),
    )
    realtime = build_realtime_metrics(
        mode=mode,
        source_active_s=source_active_s,
        drain_elapsed_s=drain_elapsed_s,
        writer=writer_status,
        samples=samples,
        stall_end_elapsed_s=snapshot.get("stall_end_elapsed_s"),
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
            **summarize_timings(snapshot.get("stage_s", {})),
        },
        bandwidth=bandwidth,
        realtime=realtime,
        device=build_device_write_metrics(samples, source_active_s),
        samples=samples,
        environment=environment_metadata(),
        recorder=snapshot.get("recorder", {}),
    )
    report.write(output_dir / "report.json")
    (output_dir / "source_manifest.json").write_text(source_run.model_dump_json(indent=2) + "\n")
    return report


def run_capacity_search(
    profile: WorkloadProfile,
    *,
    trial_duration_s: float,
    confirm_duration_s: float,
    max_scale: float,
    resolution: float,
    output_dir: Path,
) -> dict[str, Any]:
    """Find the highest source-valid, faithful workload rate multiplier."""
    if trial_duration_s <= 0 or confirm_duration_s <= 0:
        raise ValueError("capacity durations must be greater than zero")
    if max_scale < 1:
        raise ValueError("max_scale must be at least 1")
    if resolution <= 0 or resolution > max_scale:
        raise ValueError("resolution must be greater than zero and no larger than max_scale")

    output_dir.mkdir(parents=True, exist_ok=True)
    trials: list[dict[str, Any]] = []
    reports: dict[float, FidelityReport] = {}

    def measure(scale: float, duration_s: float, phase: str) -> FidelityReport:
        scale = round(scale, 6)
        label = f"{phase}-{scale:.3f}x".replace(".", "_")
        trial_dir = output_dir / label
        try:
            report = run_harness(
                profile.with_rate_scale(scale),
                duration_s=duration_s,
                output_dir=trial_dir,
            )
        finally:
            for database_file in trial_dir.glob("recording.db*"):
                database_file.unlink(missing_ok=True)
        trials.append(
            {
                "phase": phase,
                "scale": scale,
                "duration_s": duration_s,
                "source_valid": report.source_valid,
                "faithful": report.faithful,
                "report": f"{label}/report.json",
            }
        )
        reports[scale] = report
        return report

    best = 0.0
    upper: float | None = None
    limiting_stage = "max_scale"
    scale = 1.0
    while scale <= max_scale:
        report = measure(scale, trial_duration_s, "trial")
        if report.source_valid and report.faithful:
            best = scale
            if scale == max_scale:
                break
            scale = min(max_scale, scale * 2)
            continue
        upper = scale
        limiting_stage = "source" if not report.source_valid else "recorder"
        break

    if upper is not None and best > 0:
        while upper - best > resolution:
            midpoint = round(((best + upper) / 2) / resolution) * resolution
            midpoint = max(best + resolution, min(upper - resolution, midpoint))
            if midpoint <= best or midpoint >= upper:
                break
            report = measure(midpoint, trial_duration_s, "trial")
            if report.source_valid and report.faithful:
                best = midpoint
            else:
                upper = midpoint
                limiting_stage = "source" if not report.source_valid else "recorder"

    confirmed = False
    confirmed_scale = best
    while confirmed_scale > 0:
        report = measure(confirmed_scale, confirm_duration_s, "confirm")
        if report.source_valid and report.faithful:
            confirmed = True
            break
        limiting_stage = "source" if not report.source_valid else "recorder"
        confirmed_scale = max(
            (candidate for candidate in reports if candidate < confirmed_scale),
            default=0.0,
        )

    result = {
        "profile": profile.name,
        "max_faithful_scale": confirmed_scale if confirmed else 0.0,
        "limiting_stage": limiting_stage,
        "trial_duration_s": trial_duration_s,
        "confirm_duration_s": confirm_duration_s,
        "max_scale": max_scale,
        "resolution": resolution,
        "trials": trials,
    }
    (output_dir / "capacity.json").write_text(json.dumps(result, indent=2) + "\n")
    return result


def _capacity_payloads(
    profile: WorkloadProfile,
    recording: Path | None,
    *,
    duration_s: float,
) -> dict[str, list[bytes]]:
    if recording is None:
        return {stream.name: [bytes(max(1, stream.encoded_bytes))] for stream in profile.streams}

    connection = sqlite3.connect(recording)
    try:
        result: dict[str, list[bytes]] = {}
        for stream in profile.streams:
            first_row = connection.execute(f'SELECT MIN(ts) FROM "{stream.name}"').fetchone()
            if first_row is None or first_row[0] is None:
                raise ValueError(f"recording stream {stream.name!r} is empty")
            first_ts = float(first_row[0])
            rows = connection.execute(
                f'SELECT blob.data FROM "{stream.name}" AS metadata '
                f'JOIN "{stream.name}_blob" AS blob ON blob.id = metadata.id '
                "WHERE metadata.ts >= ? AND metadata.ts < ? ORDER BY metadata.ts",
                (first_ts, first_ts + duration_s),
            ).fetchall()
            if not rows:
                raise ValueError(f"recording stream {stream.name!r} has no encoded payloads")
            result[stream.name] = [bytes(row[0]) for row in rows]
        return result
    finally:
        connection.close()


def run_writer_capacity(
    profile: WorkloadProfile,
    *,
    duration_s: float,
    rate_scale: float,
    output_dir: Path,
    recording: Path | None = None,
) -> StorageCapacityReport:
    """Measure the real RecordWriter and SQLite path without transport or encoding."""

    if duration_s <= 0:
        raise ValueError("duration must be greater than zero")
    if rate_scale <= 0:
        raise ValueError("rate scale must be greater than zero")
    profile = _canonical_profile(profile)
    payloads = _capacity_payloads(profile, recording, duration_s=duration_s)
    output_dir.mkdir(parents=True, exist_ok=True)
    db_path = output_dir / "writer-capacity.db"
    if db_path.exists():
        raise FileExistsError(db_path)

    store = SqliteStore(path=str(db_path))
    writer = RecordWriter()
    backends: dict[str, Backend[Any]] = {}
    expected_counts: dict[str, int] = {}
    schedule: list[tuple[float, str, int]] = []
    for stream in profile.streams:
        payload_type = resolve_payload_type(stream.payload_type)
        target = store.stream(stream.name, payload_type, codec=stream.codec)
        backends[stream.name] = cast("Backend[Any]", target._source)  # type: ignore[attr-defined]
        count = max(1, round(duration_s * stream.rate_hz * rate_scale))
        expected_counts[stream.name] = count
        schedule.extend(
            (sequence / (stream.rate_hz * rate_scale), stream.name, sequence)
            for sequence in range(count)
        )
    schedule.sort(key=lambda item: item[0])

    start = time.monotonic() + 0.1
    last_submit = start
    try:
        for relative_s, stream_name, sequence in schedule:
            deadline = start + relative_s
            now = time.monotonic()
            if now < deadline:
                time.sleep(deadline - now)
            accepted = time.monotonic()
            last_submit = accepted
            stream_payloads = payloads[stream_name]
            encoded = stream_payloads[sequence % len(stream_payloads)]
            prepared = PreparedAppend(
                observation=Observation(id=-1, ts=relative_s, pose=None, _data=None),
                encoded=encoded,
            )
            writer.submit(
                backends[stream_name],
                prepared,
                accepted_monotonic=accepted,
                stream_name=stream_name,
            )
        source_deadline = start + duration_s
        writer.close(timeout_s=max(30.0, duration_s))
        finished = time.monotonic()
        status = writer.status()
        persisted_counts = {name: int(store.stream(name).count()) for name in expected_counts}
        persisted_bytes = {
            name: int(store.stream(name).size_bytes() or 0) for name in expected_counts
        }
    finally:
        if writer.status().failed is None:
            try:
                writer.close(timeout_s=1.0)
            except RecorderFailedError:
                pass
        store.stop()

    submitted = sum(expected_counts.values())
    committed = sum(persisted_counts.values())
    committed_bytes = sum(persisted_bytes.values())
    active_elapsed_s = max(duration_s, last_submit - start)
    drain_elapsed_s = max(0.0, finished - source_deadline)
    violations: list[str] = []
    source_valid = last_submit <= source_deadline + 0.1
    if not source_valid:
        violations.append("writer load generator fell more than 100ms behind schedule")
    if committed != submitted:
        violations.append(f"committed {committed} of {submitted} submitted rows")
    if status.receive_to_commit_p99_ms >= 100.0:
        violations.append(
            f"receive-to-commit p99 {status.receive_to_commit_p99_ms:.3f}ms is not below 100ms"
        )
    if drain_elapsed_s >= 0.1:
        violations.append(f"drain time {drain_elapsed_s:.3f}s is not below 0.100s")
    report = StorageCapacityReport(
        profile=profile.name,
        duration_s=duration_s,
        rate_scale=rate_scale,
        source_valid=source_valid,
        submitted_messages=submitted,
        committed_messages=committed,
        committed_payload_bytes=committed_bytes,
        offered_mib_s=status.submitted_payload_bytes / duration_s / (1024 * 1024),
        effective_committed_mib_s=committed_bytes
        / (active_elapsed_s + drain_elapsed_s)
        / (1024 * 1024),
        drain_elapsed_s=drain_elapsed_s,
        receive_to_commit_p99_ms=status.receive_to_commit_p99_ms,
        receive_to_commit_max_ms=status.receive_to_commit_max_ms,
        passed=not violations,
        violations=tuple(violations),
        writer=asdict(status),
    )
    (output_dir / "writer-capacity.json").write_text(report.model_dump_json(indent=2) + "\n")
    return report


def _hold_writer_lock(
    path: Path,
    delay_s: float,
    duration_s: float,
    window_ns: list[int] | None = None,
) -> None:
    time.sleep(delay_s)
    connection = sqlite3.connect(path, timeout=10.0)
    try:
        connection.execute("BEGIN IMMEDIATE")
        if window_ns is not None:
            window_ns.append(time.monotonic_ns())
        time.sleep(duration_s)
        connection.rollback()
        if window_ns is not None:
            window_ns.append(time.monotonic_ns())
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
        "--mode",
        choices=("baseline", "encoder-stall", "sqlite-lock", "bounded-preparation"),
        default="baseline",
    )
    run.add_argument("--transport", choices=("lcm", "shm", "zenoh"))
    run.add_argument("--stall-duration", type=float, default=1.0)
    capacity = commands.add_parser("capacity", help="find maximum faithful rate scale")
    capacity.add_argument("profile", type=Path)
    capacity.add_argument("--trial-duration", type=float, default=15.0)
    capacity.add_argument("--confirm-duration", type=float, default=30.0)
    capacity.add_argument("--max-scale", type=float, default=4.0)
    capacity.add_argument("--resolution", type=float, default=0.125)
    capacity.add_argument("--output", type=Path, default=Path("recorder-capacity-results"))
    writer_capacity = commands.add_parser(
        "writer-capacity", help="measure isolated RecordWriter and SQLite headroom"
    )
    writer_capacity.add_argument("profile", type=Path)
    writer_capacity.add_argument("--duration", type=float, default=30.0)
    writer_capacity.add_argument("--rate-scale", type=float, default=1.5)
    writer_capacity.add_argument("--recording", type=Path)
    writer_capacity.add_argument(
        "--output", type=Path, default=Path("recorder-writer-capacity-results")
    )
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
    if args.command == "writer-capacity":
        capacity_report = run_writer_capacity(
            profile,
            duration_s=args.duration,
            rate_scale=args.rate_scale,
            output_dir=args.output,
            recording=args.recording,
        )
        print(capacity_report.model_dump_json(indent=2))
        return 0 if capacity_report.passed else 1
    if args.command == "capacity":
        result = run_capacity_search(
            profile,
            trial_duration_s=args.trial_duration,
            confirm_duration_s=args.confirm_duration,
            max_scale=args.max_scale,
            resolution=args.resolution,
            output_dir=args.output,
        )
        print(json.dumps(result, indent=2))
        return 0 if result["max_faithful_scale"] >= 1.0 else 1
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
        return 0 if baseline.successful else 1
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
    return 0 if report.successful else 1


if __name__ == "__main__":
    raise SystemExit(main())
