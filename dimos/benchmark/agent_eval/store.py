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

"""Exclusive, append-only, non-overwriting storage for one local attempt."""

from __future__ import annotations

from datetime import UTC, datetime
import fcntl
import hashlib
import os
from pathlib import Path
import time
from typing import Any
from uuid import uuid4

from pydantic import BaseModel, JsonValue

from dimos.benchmark.agent_eval.models import (
    ArtifactReference,
    AttemptId,
    LifecycleEvent,
    NormalizedOutcome,
    OperationId,
)
from dimos.benchmark.spatial.utilities import canonical_json


class AttemptAlreadyActiveError(RuntimeError):
    pass


class AttemptStore:
    """Own a target-wide lock and one fresh immutable attempt directory."""

    def __init__(self, output_root: Path, attempt_id: str | None = None) -> None:
        self.output_root = output_root.resolve()
        self.attempt_id: AttemptId = attempt_id or f"attempt_{uuid4().hex}"
        self.path = self.output_root / self.attempt_id
        self._lock_fd = -1
        self._events_fd = -1
        self._started_monotonic = time.monotonic()
        self._event_sequence = 0
        self._last_event_offset = 0.0
        self._closed = False
        self._reserve()

    def _reserve(self) -> None:
        self.output_root.mkdir(parents=True, exist_ok=True)
        lock_path = self.output_root / ".attached-target.lock"
        self._lock_fd = os.open(lock_path, os.O_RDWR | os.O_CREAT | os.O_CLOEXEC, 0o600)
        try:
            fcntl.flock(self._lock_fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            os.close(self._lock_fd)
            self._lock_fd = -1
            raise AttemptAlreadyActiveError(
                f"another attempt is active for {self.output_root}"
            ) from exc
        try:
            self.path.mkdir(mode=0o700)
        except BaseException:
            self.close()
            raise
        self._events_fd = os.open(
            self.path / "events.jsonl",
            os.O_WRONLY | os.O_APPEND | os.O_CREAT | os.O_EXCL | os.O_CLOEXEC,
            0o600,
        )

    def append_event(
        self,
        kind: str,
        *,
        operation_id: str | None = None,
        payload: dict[str, JsonValue] | None = None,
    ) -> LifecycleEvent:
        self._require_open()
        offset = max(
            self._last_event_offset,
            time.monotonic() - self._started_monotonic,
        )
        self._event_sequence += 1
        event = LifecycleEvent(
            sequence=self._event_sequence,
            attempt_id=self.attempt_id,
            operation_id=operation_id,
            occurred_at=datetime.now(UTC),
            monotonic_offset_s=offset,
            kind=kind,
            payload=payload or {},
        )
        encoded = canonical_json(event.model_dump(mode="json")) + b"\n"
        _write_all(self._events_fd, encoded)
        os.fsync(self._events_fd)
        self._last_event_offset = offset
        return event

    def write_artifact(
        self, relative_path: str, value: BaseModel | JsonValue | bytes | str
    ) -> ArtifactReference:
        self._require_open()
        path = self._resolve_relative(relative_path)
        path.parent.mkdir(parents=True, exist_ok=True)
        if isinstance(value, BaseModel):
            data = canonical_json(value.model_dump(mode="json")) + b"\n"
        elif isinstance(value, bytes):
            data = value
        elif isinstance(value, str):
            data = value.encode("utf-8")
        else:
            data = canonical_json(value) + b"\n"
        descriptor = os.open(
            path,
            os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_CLOEXEC,
            0o600,
        )
        try:
            _write_all(descriptor, data)
            os.fsync(descriptor)
        except BaseException:
            os.close(descriptor)
            path.unlink(missing_ok=True)
            raise
        else:
            os.close(descriptor)
        _fsync_directory(path.parent)
        return ArtifactReference(
            path=relative_path,
            sha256=hashlib.sha256(data).hexdigest(),
            size_bytes=len(data),
        )

    def write_outcome(self, outcome: NormalizedOutcome) -> ArtifactReference:
        self._require_open()
        if outcome.attempt_id != self.attempt_id:
            raise ValueError("outcome attempt identity mismatch")
        relative_path = "outcome.v1.json"
        final_path = self.path / relative_path
        temp_path = self.path / f".outcome.v1.json.tmp-{uuid4().hex}"
        data = canonical_json(outcome.model_dump(mode="json")) + b"\n"
        descriptor = os.open(
            temp_path,
            os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_CLOEXEC,
            0o600,
        )
        try:
            _write_all(descriptor, data)
            os.fsync(descriptor)
            os.close(descriptor)
            descriptor = -1
            os.link(temp_path, final_path)
            _fsync_directory(self.path)
        finally:
            if descriptor >= 0:
                os.close(descriptor)
            temp_path.unlink(missing_ok=True)
        return ArtifactReference(
            path=relative_path,
            sha256=hashlib.sha256(data).hexdigest(),
            size_bytes=len(data),
        )

    def verify_artifacts(self, artifacts: tuple[ArtifactReference, ...]) -> bool:
        """Return whether every admitted reference still matches retained bytes."""
        for artifact in artifacts:
            try:
                path = self._resolve_relative(artifact.path)
                data = path.read_bytes()
            except (OSError, ValueError):
                return False
            if (
                len(data) != artifact.size_bytes
                or hashlib.sha256(data).hexdigest() != artifact.sha256
            ):
                return False
        return True

    def _resolve_relative(self, relative_path: str) -> Path:
        if not relative_path or relative_path.startswith("/") or ".." in relative_path.split("/"):
            raise ValueError("artifact path must be attempt-relative")
        result = self.path / relative_path
        if result.resolve().parent != self.path and self.path not in result.resolve().parents:
            raise ValueError("artifact path escapes attempt directory")
        return result

    def _require_open(self) -> None:
        if self._closed:
            raise RuntimeError("attempt store is closed")

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        if self._events_fd >= 0:
            os.close(self._events_fd)
            self._events_fd = -1
        if self._lock_fd >= 0:
            fcntl.flock(self._lock_fd, fcntl.LOCK_UN)
            os.close(self._lock_fd)
            self._lock_fd = -1

    def __enter__(self) -> AttemptStore:
        return self

    def __exit__(self, *_args: Any) -> None:
        self.close()


def new_operation_id() -> OperationId:
    return f"operation_{uuid4().hex}"


def _write_all(descriptor: int, data: bytes) -> None:
    view = memoryview(data)
    while view:
        view = view[os.write(descriptor, view) :]


def _fsync_directory(path: Path) -> None:
    descriptor = os.open(path, os.O_RDONLY | os.O_DIRECTORY | os.O_CLOEXEC)
    try:
        os.fsync(descriptor)
    finally:
        os.close(descriptor)
