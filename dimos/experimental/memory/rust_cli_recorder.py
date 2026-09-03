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

"""Managed standalone Rust recorder used by the experimental CLI engine."""

from __future__ import annotations

from dataclasses import dataclass
import json
import os
from pathlib import Path
import signal
import subprocess
import threading
import time
from typing import IO, Any

from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.core.global_config import global_config
from dimos.core.transport import LCMTransport, ZenohTransport
from dimos.core.transport_factory import session_config
from dimos.experimental.memory.rust_recorder import RustStreamSpec
from dimos.memory.store.sqlite import SqliteStore
from dimos.memory.tap import matching, recording_dir
from dimos.msgs.sensor_msgs.Image import Image
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

_RUST_DIR = Path(__file__).resolve().parent / "rust"
_EXECUTABLE = _RUST_DIR / "result/bin/dimos-memory-recorder-cli"
_BUILD_COMMAND = ("nix", "build", "-L", ".#dimos-memory-recorder")
_READY_TIMEOUT = 10.0


@dataclass(frozen=True)
class RustRecordingPlan:
    """Validated topics, schemas, and artifact for one native recording."""

    backend: str
    topics: dict[str, str]
    streams: list[RustStreamSpec]
    payload_types: dict[str, type[Any]]
    path: Path


def prepare_rust_recorder() -> None:
    """Build the CLI recorder before module startup when it is unavailable or requested."""
    if global_config.record_engine != "rust" or not global_config.record or global_config.replay:
        return
    if _EXECUTABLE.exists() and not global_config.build_native:
        return

    logger.info("Building experimental Rust recorder", executable=str(_EXECUTABLE))
    started = time.perf_counter()
    process = subprocess.Popen(
        _BUILD_COMMAND,
        cwd=_RUST_DIR,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
    )
    assert process.stdout is not None
    for raw in process.stdout:
        line = raw.decode("utf-8", errors="replace").rstrip()
        if line:
            logger.info(line, module="rust-recorder-build")
    returncode = process.wait()
    if returncode != 0:
        raise RuntimeError(
            "Rust recorder build failed "
            f"after {time.perf_counter() - started:.2f}s (exit {returncode})"
        )
    if not _EXECUTABLE.exists():
        raise FileNotFoundError(f"Rust recorder build did not produce {_EXECUTABLE}")


def make_plan(transports: dict[tuple[str, type], Any]) -> RustRecordingPlan:
    """Resolve and validate the stream plan the standalone recorder will receive."""
    selected = matching(global_config.record_topics, (name for name, _ in transports))
    unsupported_transports: list[str] = []
    streams: list[RustStreamSpec] = []
    topics: dict[str, str] = {}
    payload_types: dict[str, type[Any]] = {}
    backends: set[str] = set()

    for index, ((name, payload_type), transport) in enumerate(transports.items()):
        if name not in selected:
            continue
        if not hasattr(payload_type, "lcm_encode") or not hasattr(payload_type, "lcm_decode"):
            logger.info(
                "--record: stream is not a DimOS LCM message type; skipped",
                stream=name,
                payload_type=str(payload_type),
            )
            continue
        if type(transport) is LCMTransport:
            backend = "lcm"
        elif type(transport) is ZenohTransport:
            backend = "zenoh"
        else:
            unsupported_transports.append(f"{name} ({type(transport).__name__})")
            continue

        backends.add(backend)
        port = f"stream_{index}"
        streams.append(
            RustStreamSpec(
                port=port,
                name=name,
                payload_type=f"{payload_type.__module__}.{payload_type.__qualname__}",
                codec="jpeg" if issubclass(payload_type, Image) else "lcm",
            )
        )
        topics[port] = transport.channel
        payload_types[name] = payload_type

    if unsupported_transports:
        details = ", ".join(sorted(unsupported_transports))
        raise ValueError(
            "The Rust recorder supports only exact LCMTransport and ZenohTransport streams; "
            f"unsupported selections: {details}. Narrow --record-topics or use "
            "--record-engine python."
        )
    if not streams:
        raise ValueError(
            f"--record-topics {global_config.record_topics!r} selected no Rust-recordable streams"
        )
    names = [stream.name for stream in streams]
    duplicates = sorted({name for name in names if names.count(name) > 1})
    if duplicates:
        raise ValueError(f"Duplicate recorded stream names: {duplicates}")
    if len(backends) != 1:
        raise ValueError(
            "The Rust recorder cannot open mixed LCM and Zenoh streams in one artifact; "
            "narrow --record-topics or use --record-engine python."
        )

    suffix = "db" if global_config.record == "sqlite" else "mcap"
    return RustRecordingPlan(
        backend=backends.pop(),
        topics=topics,
        streams=streams,
        payload_types=payload_types,
        path=recording_dir() / f"memory.{suffix}",
    )


def _prepare_artifact(plan: RustRecordingPlan) -> None:
    plan.path.parent.mkdir(parents=True, exist_ok=True)
    if plan.path.exists():
        plan.path.unlink()
    if global_config.record == "mcap":
        return
    with SqliteStore(path=str(plan.path)) as store:
        for stream in plan.streams:
            store.stream(stream.name, plan.payload_types[stream.name], codec=stream.codec)


class RustRecordingSession:
    """Own the recorder subprocess from explicit readiness through final flush."""

    def __init__(self, plan: RustRecordingPlan) -> None:
        self.plan = plan
        self.failure_event = threading.Event()
        self._ready = threading.Event()
        self._stopping = threading.Event()
        self._failure: str | None = None
        self._process: subprocess.Popen[bytes] | None = None
        self._threads: list[threading.Thread] = []

    def start(self) -> None:
        _prepare_artifact(self.plan)
        store = {"kind": global_config.record, "path": str(self.plan.path)}
        launch = {
            "topics": self.plan.topics,
            "config": {
                "store": store,
                "encoding_threads": global_config.record_encoding_threads,
                "streams": [stream.model_dump() for stream in self.plan.streams],
            },
            "session": session_config().to_wire() if self.plan.backend == "zenoh" else {},
        }
        env = {
            **os.environ,
            "DIMOS_TRANSPORT": self.plan.backend,
            "RUST_LOG": os.environ.get("DIMOS_LOG_LEVEL", "info").lower(),
        }
        self._process = subprocess.Popen(
            [_EXECUTABLE],
            cwd=_RUST_DIR,
            env=env,
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            start_new_session=True,
        )
        assert self._process.stdin is not None
        self._process.stdin.write(json.dumps(launch).encode() + b"\n")
        self._process.stdin.close()

        self._threads = [
            self._start_thread(self._read_logs, self._process.stdout, "info"),
            self._start_thread(self._read_logs, self._process.stderr, "warning"),
            self._start_thread(self._watch_process),
        ]
        ready = self._ready.wait(_READY_TIMEOUT)
        if not ready or self._failure is not None or self._process.poll() is not None:
            message = self._failure or f"did not report ready within {_READY_TIMEOUT:g}s"
            self.stop()
            raise RuntimeError(f"Rust recorder startup failed: {message}")
        logger.info(
            "Experimental Rust recorder ready",
            artifact_path=str(self.plan.path),
            streams=len(self.plan.streams),
            pid=self._process.pid,
        )

    def _start_thread(self, target: Any, *args: Any) -> threading.Thread:
        thread = threading.Thread(target=target, args=args, daemon=True, name="rust-recorder")
        thread.start()
        return thread

    def _read_logs(self, stream: IO[bytes] | None, fallback_level: str) -> None:
        if stream is None:
            return
        for raw in stream:
            line = raw.decode("utf-8", errors="replace").rstrip()
            if not line:
                continue
            try:
                data = json.loads(line)
                fields = data.pop("fields", {})
                message = fields.pop("message", line)
                event = fields.pop("event", None)
                if event is not None:
                    fields["native_event"] = event
                level = data.pop("level", fallback_level).lower()
                log = logger.warning if level in {"warn", "warning"} else getattr(logger, level)
                log(message, module="rust-recorder", **fields, **data)
                if event == "recorder_ready":
                    self._ready.set()
            except (json.JSONDecodeError, TypeError, AttributeError):
                getattr(logger, fallback_level)(line, module="rust-recorder")
        stream.close()

    def _watch_process(self) -> None:
        assert self._process is not None
        returncode = self._process.wait()
        if not self._stopping.is_set() or returncode != 0:
            self._failure = f"process exited with status {returncode}"
            self.failure_event.set()
            self._ready.set()

    def stop(self) -> None:
        process = self._process
        if process is None:
            return
        self._stopping.set()
        if process.poll() is None:
            process.send_signal(signal.SIGTERM)
            try:
                process.wait(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)
            except subprocess.TimeoutExpired:
                logger.warning("Rust recorder did not flush in time; sending SIGKILL")
                process.kill()
                process.wait(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)
                self._failure = "process did not flush before the shutdown timeout"
                self.failure_event.set()
        for thread in self._threads:
            if thread is not threading.current_thread():
                thread.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)
        if process.returncode != 0 and self._failure is None:
            self._failure = f"process exited with status {process.returncode}"
            self.failure_event.set()
        self._process = None

    def raise_if_failed(self) -> None:
        if self._failure is not None:
            raise RuntimeError(f"Rust recorder failed: {self._failure}")
