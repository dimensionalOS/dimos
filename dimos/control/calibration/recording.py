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

"""Where a calibration procedure reads back what it recorded.

A procedure marks poses and steps as it moves the arm; the run's recording
keeps those markers next to the connection's ``joint_state`` stream, and the
offline analyses (``gravity_fit``, ``step_report``) read both back. A
:class:`RecordingSource` gives them one time window of it.

:class:`SqliteRecording` is the ``--record sqlite`` tap of a ``dimos run``:
``recordings/<run-id>/memory.db`` (``dimos.memory.tap.recording_dir``), found
from the ``DIMOS_RUN_ID`` every module worker inherits. The tap writes from a
queue, so a reader first waits until the joint samples past the window have
landed (:meth:`RecordingSource.wait_for`). Before the arm moves, the check step
publishes a probe on ``calibration_events`` and waits for it to land
(:meth:`RecordingSource.wait_for_probe`), so a recorder that is not writing
stops the run before any motion, not after the poses.
"""

from __future__ import annotations

import json
import os
from pathlib import Path
import time
from typing import Any, Protocol

from dimos.constants import RECORDINGS_DIR
from dimos.core.coordination.process_lifecycle import DIMOS_RUN_ID_ENV
from dimos.memory.store.sqlite import SqliteStore
from dimos.robot.manipulators.calibration.gravity_fit import (
    GravityRecording,
    load_gravity_recording,
)
from dimos.robot.manipulators.calibration.step_report import (
    JOINT_STATE_STREAM,
    Recording,
    load_recording,
)

DATABASE_NAME = "memory.db"
# Joint samples this long after a window must be readable before it is read:
# the window's markers were published before them.
FLUSH_MARGIN_S = 0.3
FLUSH_TIMEOUT_S = 10.0
# A probe must land within this; the recorder writes within milliseconds.
PROBE_TIMEOUT_S = 2.0
CALIBRATION_EVENTS_STREAM = "calibration_events"
PROBE_KEY = "probe"
_POLL_S = 0.1


class RecordingUnavailableError(RuntimeError):
    """The procedure cannot read back what it recorded."""


class RecordingSource(Protocol):
    """The joint samples and markers of one run, by time window (Unix seconds)."""

    @property
    def directory(self) -> Path | None:
        """Where reports about the recording are written, or ``None`` for none."""
        ...

    def wait_for(self, until: float, timeout_s: float = FLUSH_TIMEOUT_S) -> None:
        """Return once everything recorded up to ``until`` can be read; raise if it never is."""
        ...

    def steps(self, start: float, end: float) -> Recording:
        """The joint traces and step markers between ``start`` and ``end``."""
        ...

    def gravity(self, start: float, end: float) -> GravityRecording:
        """The joint traces, step markers and pose markers between ``start`` and ``end``."""
        ...

    def wait_for_probe(self, token: str, since: float, timeout_s: float = PROBE_TIMEOUT_S) -> None:
        """Return once the probe ``token``, published after ``since``, is recorded; raise if not."""
        ...


def probe_payload(token: str) -> dict[str, Any]:
    """What a probe publishes on ``calibration_events``."""
    return {"stream": PROBE_KEY, PROBE_KEY: token}


class SqliteRecording:
    """A run's ``memory.db``, read while the recorder may still be writing it."""

    def __init__(self, directory: Path) -> None:
        self._directory = directory

    @classmethod
    def of_this_run(cls) -> SqliteRecording | None:
        """The recording of the ``dimos run`` this process belongs to, or ``None`` outside one."""
        run_id = os.environ.get(DIMOS_RUN_ID_ENV)
        return cls(RECORDINGS_DIR / run_id) if run_id else None

    @property
    def directory(self) -> Path:
        return self._directory

    @property
    def database(self) -> Path:
        return self._directory / DATABASE_NAME

    def unavailable(self) -> str | None:
        """Why nothing can be read back yet, or ``None`` once the database exists."""
        if not self.database.exists():
            return (
                f"nothing is being recorded: {self.database} does not exist; run the calibration "
                "blueprint with dimos run, which records to sqlite"
            )
        return None

    def wait_for(self, until: float, timeout_s: float = FLUSH_TIMEOUT_S) -> None:
        deadline = time.monotonic() + timeout_s
        while True:
            if self._has_samples_after(until + FLUSH_MARGIN_S):
                return
            if time.monotonic() >= deadline:
                raise RecordingUnavailableError(
                    f"{self.database} has no {JOINT_STATE_STREAM} samples after "
                    f"{until + FLUSH_MARGIN_S:.2f} after {timeout_s:g} s; is the recorder running?"
                )
            time.sleep(_POLL_S)

    def _has_samples_after(self, t: float) -> bool:
        if not self.database.exists():
            return False
        store = SqliteStore(path=str(self.database), must_exist=True)
        store.start()
        try:
            if JOINT_STATE_STREAM not in set(store.list_streams()):
                return False
            stream: Any = store.stream(JOINT_STATE_STREAM)
            return bool(stream.after(t).exists())
        finally:
            store.stop()

    def wait_for_probe(self, token: str, since: float, timeout_s: float = PROBE_TIMEOUT_S) -> None:
        deadline = time.monotonic() + timeout_s
        while not self._has_probe(token, since):
            if time.monotonic() >= deadline:
                raise RecordingUnavailableError(
                    f"the recorder is not writing: a marker sent on {CALIBRATION_EVENTS_STREAM} "
                    f"did not reach {self.database} within {timeout_s:g} s; run the calibration "
                    "blueprint with dimos run, which records it to sqlite"
                )
            time.sleep(_POLL_S)

    def _has_probe(self, token: str, since: float) -> bool:
        if not self.database.exists():
            return False
        store = SqliteStore(path=str(self.database), must_exist=True)
        store.start()
        try:
            if CALIBRATION_EVENTS_STREAM not in set(store.list_streams()):
                return False
            stream: Any = store.stream(CALIBRATION_EVENTS_STREAM)
            for obs in stream.after(since - 1.0):
                try:
                    payload = json.loads(obs.data.data)
                except (AttributeError, TypeError, ValueError):
                    continue
                if isinstance(payload, dict) and payload.get(PROBE_KEY) == token:
                    return True
            return False
        finally:
            store.stop()

    def steps(self, start: float, end: float) -> Recording:
        return load_recording(self._directory, (start, end))

    def gravity(self, start: float, end: float) -> GravityRecording:
        return load_gravity_recording(self._directory, (start, end))
