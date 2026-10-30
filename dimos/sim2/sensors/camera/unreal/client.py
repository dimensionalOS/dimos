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

"""A bounded RGB-D protocol and an owned native Unreal renderer process.

Physics never waits on this connection: a separate camera worker sends complete
snapshots. Unreal echoes their identity before returning aligned RGBA/depth.
"""

from __future__ import annotations

from collections.abc import Iterator
from contextlib import contextmanager, suppress
from dataclasses import dataclass
import json
import math
from pathlib import Path
import re
import socket
import struct
import subprocess
import threading
import time
from typing import Any, BinaryIO

import numpy as np
from numpy.typing import NDArray

PROTOCOL_VERSION = 1
MAX_HEADER_BYTES = 1 << 20
MAX_IMAGE_DIMENSION = 4096
IDENTITY_FIELDS = ("episode", "sequence", "physics_tick", "sim_time", "timestamp")
LOOPBACK_HOST = "127.0.0.1"
IO_POLL_SECONDS = 0.2


def level_package(level: str) -> str:
    """Accept one content map path, never an Unreal travel URL or command line."""
    match = re.fullmatch(
        r"(/(?:Game|Engine)/(?:[A-Za-z0-9_]+/)*([A-Za-z0-9_]+))(?:\.([A-Za-z0-9_]+))?", level
    )
    if match is None or (match[3] is not None and match[3] != match[2]):
        raise ValueError(
            "Unreal level must be a /Game or /Engine map package or matching object path"
        )
    return match[1]


@dataclass(frozen=True)
class RenderedFrame:
    header: dict[str, Any]
    rgba: NDArray[np.uint8]
    depth: NDArray[np.float32]
    roundtrip_seconds: float


class RendererClient:
    """One serial request stream; close() also interrupts a pending receive."""

    def __init__(self, connection: socket.socket) -> None:
        self.connection = connection
        self._request_lock = threading.RLock()
        self._closed = threading.Event()

    @contextmanager
    def _transaction(self, timeout: float | None = None) -> Iterator[float | None]:
        with self._request_lock:
            configured_timeout = self.connection.gettimeout()
            if timeout is None:
                timeout = configured_timeout
            deadline = None if timeout is None else time.monotonic() + timeout
            try:
                yield deadline
            finally:
                with suppress(OSError):
                    self.connection.settimeout(configured_timeout)

    def _remaining(self, deadline: float | None) -> None:
        if self._closed.is_set():
            raise ConnectionError("Unreal connection closed")
        remaining = IO_POLL_SECONDS
        if deadline is not None:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError("Unreal response exceeded its deadline")
        # A close from another thread does not reliably wake every platform's
        # socket timeout wait. Polling bounds cancellation without extending the
        # transaction's overall deadline.
        self.connection.settimeout(min(IO_POLL_SECONDS, remaining))

    def _send(self, payload: bytes, deadline: float | None) -> None:
        view = memoryview(payload)
        offset = 0
        while offset < len(view):
            self._remaining(deadline)
            try:
                sent = self.connection.send(view[offset:])
            except TimeoutError:
                continue
            if sent == 0:
                raise ConnectionError("Unreal disconnected before receiving its command")
            offset += sent

    def _receive(self, size: int, deadline: float | None) -> bytes:
        chunks = bytearray(size)
        view = memoryview(chunks)
        offset = 0
        while offset < size:
            self._remaining(deadline)
            try:
                received = self.connection.recv_into(view[offset:])
            except TimeoutError:
                continue
            if received == 0:
                raise ConnectionError("Unreal disconnected before completing its response")
            offset += received
        return bytes(chunks)

    def _receive_header(self, deadline: float | None) -> dict[str, Any]:
        size = struct.unpack("<I", self._receive(4, deadline))[0]
        if not 0 < size <= MAX_HEADER_BYTES:
            raise ValueError(f"invalid Unreal response header length: {size}")
        value = json.loads(self._receive(size, deadline))
        if not isinstance(value, dict):
            raise ValueError("Unreal response must be a JSON object")
        if value.get("type") == "error":
            raise RuntimeError(f"Unreal renderer: {value.get('message', 'unspecified error')}")
        return value

    def receive_header(self) -> dict[str, Any]:
        with self._transaction() as deadline:
            return self._receive_header(deadline)

    def _request(
        self, command: dict[str, Any], expected: str, deadline: float | None
    ) -> dict[str, Any]:
        payload = json.dumps(command, allow_nan=False, separators=(",", ":")).encode()
        if len(payload) > MAX_HEADER_BYTES:
            raise ValueError("Unreal command exceeds the protocol header limit")
        self._send(struct.pack("<I", len(payload)) + payload, deadline)
        response = self._receive_header(deadline)
        if response.get("type") != expected:
            raise ValueError(f"expected Unreal {expected!r} response, received {response!r}")
        return response

    def request(self, command: dict[str, Any], expected: str) -> dict[str, Any]:
        with self._transaction() as deadline:
            return self._request(command, expected, deadline)

    def load(self, manifest: Path, *, timeout: float | None = None) -> None:
        """Allow cold asset setup its own budget, retaining the capture timeout."""
        if timeout is not None and (not math.isfinite(timeout) or timeout <= 0):
            raise ValueError("Unreal load timeout must be positive and finite")
        with self._transaction(timeout) as deadline:
            self._request({"type": "load", "manifest": str(manifest.resolve())}, "loaded", deadline)

    def reset(self, episode: int) -> None:
        response = self.request({"type": "reset", "episode": episode}, "reset")
        if type(response.get("episode")) is not int or response["episode"] != episode:
            raise ValueError("Unreal reset acknowledged the wrong episode")

    def capture(self, command: dict[str, Any]) -> RenderedFrame:
        before = time.monotonic()
        width, height = command["camera"]["width"], command["camera"]["height"]
        if not all(type(v) is int and 0 < v <= MAX_IMAGE_DIMENSION for v in (width, height)):
            raise ValueError("camera dimensions must be positive integers no larger than 4096")
        with self._transaction() as deadline:
            response = self._request({**command, "type": "capture"}, "frame", deadline)
            if any(
                isinstance(response.get(key), bool) or response.get(key) != command[key]
                for key in IDENTITY_FIELDS
            ) or any(type(response.get(key)) is not int for key in IDENTITY_FIELDS[:3]):
                raise ValueError("Unreal returned an image for a different physics snapshot")
            if any(
                type(response.get(key)) is not int or response[key] != expected
                for key, expected in (("width", width), ("height", height))
            ):
                raise ValueError("Unreal returned the wrong image dimensions")
            size = width * height * 4
            if any(
                type(response.get(key)) is not int or response[key] != size
                for key in ("rgba_bytes", "depth_bytes")
            ):
                raise ValueError("Unreal RGB-D payload sizes do not match the requested dimensions")
            if response.get("depth_unit") != "m" or response.get("depth_kind") != "axial":
                raise ValueError("Unreal must return axial camera depth in meters")
            rgba = np.frombuffer(self._receive(size, deadline), dtype=np.uint8).reshape(
                height, width, 4
            )
            depth = np.frombuffer(self._receive(size, deadline), dtype="<f4").reshape(height, width)
        if np.isnan(depth).any() or np.any(depth < 0):
            raise ValueError("Unreal depth contains NaN or negative distances")
        return RenderedFrame(response, rgba, depth, time.monotonic() - before)

    def close(self, *, graceful: bool = False) -> None:
        # Never wait for an in-flight capture: socket shutdown must interrupt it.
        if graceful and self._request_lock.acquire(blocking=False):
            try:
                with suppress(OSError, ValueError, RuntimeError):
                    self.connection.settimeout(2.0)
                    self.request({"type": "shutdown"}, "shutdown")
            finally:
                self._request_lock.release()
        self._closed.set()
        with suppress(OSError):
            self.connection.shutdown(socket.SHUT_RDWR)
        self.connection.close()


class RendererProcess:
    """Launch one renderer, using a reserved ephemeral loopback socket.

    A packaged executable needs only ``executable``. ``project`` supports native
    development with UnrealEditor -game; neither mode uses editor Python.
    ``level`` selects a static authored background with its native materials.
    Its collision geometry must already be present in the physics scene.
    """

    def __init__(
        self,
        executable: Path,
        log_path: Path,
        *,
        project: Path | None = None,
        level: str | None = None,
        startup_timeout: float = 180.0,
        capture_timeout: float = 15.0,
        offscreen: bool = True,
    ) -> None:
        if not all(
            math.isfinite(value) and value > 0 for value in (startup_timeout, capture_timeout)
        ):
            raise ValueError("Unreal startup and capture timeouts must be positive and finite")
        self.executable = executable
        self.project = project
        self.level = level_package(level) if level is not None else None
        self.log_path = log_path
        self.startup_timeout = startup_timeout
        self.capture_timeout = capture_timeout
        self.offscreen = offscreen
        self.client: RendererClient | None = None
        self.process: subprocess.Popen[bytes] | None = None
        self._log: BinaryIO | None = None
        self._cancelled = threading.Event()
        self._state_lock = threading.Lock()
        self._lifecycle_lock = threading.Lock()
        self._starting = False
        self._started = False
        self._closed = False

    def start(self) -> RendererClient:
        with self._lifecycle_lock:
            with self._state_lock:
                if self._closed or self._cancelled.is_set():
                    raise RuntimeError("Unreal startup cancelled")
                if self._started:
                    raise RuntimeError("Unreal renderer has already started")
                self._started = self._starting = True
            try:
                return self._start()
            except BaseException:
                self._dispose(graceful=False)
                raise
            finally:
                with self._state_lock:
                    self._starting = False

    def _start(self) -> RendererClient:
        if not self.executable.is_file():
            raise FileNotFoundError(self.executable)
        if self.project is not None and not self.project.is_file():
            raise FileNotFoundError(self.project)
        self.log_path.parent.mkdir(parents=True, exist_ok=True)
        log = self.log_path.open("wb")
        with self._state_lock:
            self._log = log
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as listener:
            listener.bind((LOOPBACK_HOST, 0))
            listener.listen(1)
            args = [str(self.executable.resolve())]
            if self.project is not None:
                args.append(str(self.project.resolve()))
            if self.level is not None:
                args += [
                    f"{self.level}?game=/Script/DimosRenderer.DimosRendererGameMode",
                    f"-DimosLevel={self.level}",
                ]
            if self.project is not None:
                args.append("-game")
            args += [
                "-unattended",
                "-nosound",
                "-nosplash",
                "-NoVSync",
                f"-DimosPort={listener.getsockname()[1]}",
                f"-abslog={self.log_path.with_suffix('.engine.log').resolve()}",
            ]
            if self.offscreen:
                args.append("-RenderOffscreen")
            process = subprocess.Popen(args, stdout=log, stderr=subprocess.STDOUT)
            with self._state_lock:
                self.process = process
            deadline = time.monotonic() + self.startup_timeout
            while True:
                if self._cancelled.is_set():
                    raise RuntimeError("Unreal startup cancelled")
                if process.poll() is not None:
                    raise RuntimeError(f"Unreal exited during startup; see {self.log_path}")
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise TimeoutError(f"Unreal did not connect; see {self.log_path}")
                listener.settimeout(min(0.2, remaining))
                try:
                    connection, _ = listener.accept()
                    break
                except TimeoutError:
                    continue
            client = RendererClient(connection)
            with self._state_lock:
                self.client = client
            if self._cancelled.is_set():
                raise RuntimeError("Unreal startup cancelled")
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError(
                    "Unreal did not complete its handshake before the startup deadline"
                )
            connection.settimeout(remaining)
            ready = client.receive_header()
            if (
                ready.get("type") != "ready"
                or type(ready.get("version")) is not int
                or ready["version"] != PROTOCOL_VERSION
            ):
                raise ValueError(f"unsupported Unreal renderer handshake: {ready!r}")
            connection.settimeout(self.capture_timeout)
            if self._cancelled.is_set():
                raise RuntimeError("Unreal startup cancelled")
            return client

    def cancel(self) -> None:
        """Interrupt a blocked request; terminate only this owned subprocess."""
        self._cancelled.set()
        with self._state_lock:
            client, process = self.client, self.process
        if client is not None:
            client.close()
        if process is not None and process.poll() is None:
            with suppress(ProcessLookupError):
                process.terminate()

    def close(self) -> None:
        with self._state_lock:
            self._closed = True
            starting = self._starting
        if starting:
            self.cancel()
        with self._lifecycle_lock:
            self._dispose(graceful=not self._cancelled.is_set())

    def _dispose(self, *, graceful: bool) -> None:
        with self._state_lock:
            client, process, log = self.client, self.process, self._log
        try:
            if client is not None:
                client.close(graceful=graceful)
            if process is None:
                return
            if (not graceful or self._cancelled.is_set()) and process.poll() is None:
                with suppress(ProcessLookupError):
                    process.terminate()
            try:
                process.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                if graceful and not self._cancelled.is_set():
                    with suppress(ProcessLookupError):
                        process.terminate()
                    try:
                        process.wait(timeout=5.0)
                        return
                    except subprocess.TimeoutExpired:
                        pass
                with suppress(ProcessLookupError):
                    process.kill()
                process.wait(timeout=5.0)
        finally:
            if log is not None:
                log.close()
            with self._state_lock:
                self.client = self.process = self._log = None

    def __enter__(self) -> RendererClient:
        return self.start()

    def __exit__(self, *_: Any) -> None:
        self.close()
