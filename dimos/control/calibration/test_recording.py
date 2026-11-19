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

"""Reading one step's window back from a run's ``--record sqlite`` recording."""

from __future__ import annotations

from collections.abc import Callable, Iterator
from contextlib import closing
from dataclasses import asdict
import json
import math
from pathlib import Path
from typing import Any

import pytest

from dimos.constants import RECORDINGS_DIR
from dimos.control.calibration.recording import (
    CALIBRATION_EVENTS_STREAM,
    RecordingUnavailableError,
    SqliteRecording,
    probe_payload,
)
from dimos.control.calibration.step_analysis import StepEvent
from dimos.control.calibration.step_report import extract_steps
from dimos.core.coordination.process_lifecycle import DIMOS_RUN_ID_ENV
from dimos.core.stream import Transport
from dimos.memory.store.sqlite import SqliteStore
from dimos.memory.tap import TransportRecorder
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.std_msgs.String import String

JOINTS = ["left_joint1", "left_joint2"]


class _Transport(Transport[Any]):
    def __init__(self) -> None:
        self.callbacks: list[Callable[[Any], None]] = []

    def subscribe(
        self, callback: Callable[[Any], None], selfstream: Any = None
    ) -> Callable[[], None]:
        self.callbacks.append(callback)
        return lambda: self.callbacks.remove(callback)

    def start(self) -> None:
        pass

    def stop(self) -> None:
        pass

    def broadcast(self, selfstream: Any, msg: Any) -> None:
        for callback in self.callbacks:
            callback(msg)


def _step(t0: float, delta: float, before: float) -> StepEvent:
    return StepEvent(
        joint="left_joint2",
        delta_deg=delta,
        kp=80.0,
        kd=5.0,
        t0=t0,
        hold_s=1.0,
        target_before_deg=before,
        target_after_deg=before + delta,
    )


@pytest.fixture
def recorded(tmp_path: Path) -> Iterator[Path]:
    """A run that recorded 4 s of joint states and commands as a calibration run does, and markers.

    A pose marker at 1 s, steps at 1.2 s and 2.4 s, the end marker at 3.6 s,
    and one step outside that window, at 0.1 s. Each step's target goes out on
    ``arm_position_command`` 3 ms after its marker, and the joint moves 10 ms after it.
    """
    with SqliteStore(path=str(tmp_path / "memory.db")) as store:
        with closing(TransportRecorder(store)) as recorder:
            names = ("joint_state", "arm_position_command", "step_events", "gravity_events")
            transports = {name: _Transport() for name in names}
            for name, transport in transports.items():
                payload = String if name.endswith("_events") else JointState
                assert recorder.tap(name, payload, transport) is not None
            for k in range(401):
                t = 100.0 + k / 100.0
                target = 10.0 + (1.0 if 101.195 <= t < 102.395 else 0.0)
                transports["arm_position_command"].publish(
                    JointState(
                        ts=t + 0.003,
                        name=[f"arm/{name}" for name in JOINTS],
                        position=[0.0, math.radians(target)],
                    )
                )
                # Joint 2 steps up 1 deg at 1.21 s and back at 2.41 s.
                q2 = 10.0 + (1.0 if 101.205 <= t < 102.405 else 0.0)
                transports["joint_state"].publish(
                    JointState(
                        ts=t,
                        name=[f"arm/{name}" for name in JOINTS],
                        position=[0.0, math.radians(q2)],
                        velocity=[0.0, 0.0],
                        effort=[0.0, 0.0],
                    )
                )
            for t0, delta, before in ((100.1, 1.0, 10.0), (101.2, 1.0, 10.0), (102.4, -1.0, 11.0)):
                transports["step_events"].publish(
                    String(json.dumps(asdict(_step(t0, delta, before))))
                )
            for label, t0 in (("pose_01", 101.0), ("end", 103.6)):
                marker = {"label": label, "t0": t0, "targets_deg": {}, "data": {}}
                transports["gravity_events"].publish(String(json.dumps(marker)))
    yield tmp_path


def test_a_window_holds_only_its_own_samples_and_markers(recorded: Path) -> None:
    recording = SqliteRecording(recorded)
    assert recording.unavailable() is None

    steps = recording.steps(101.0, 103.6)

    assert [event.t0 for event in steps.events] == [101.2, 102.4]
    trace = steps.traces["arm/left_joint2"]
    assert trace.t[0] == pytest.approx(101.0) and trace.t[-1] == pytest.approx(103.6)
    assert steps.t_end == pytest.approx(103.6)
    # The step analysis finds the joint behind the connection's "arm/" prefix, and
    # times each step from its first recorded command, 3 ms after the marker.
    records = extract_steps(steps)
    assert [r.event.delta_deg for r in records] == [1.0, -1.0]
    assert all(r.metrics.passed for r in records)
    assert set(steps.commands) == {f"arm/{name}" for name in JOINTS}
    assert [r.t[0] for r in records] == pytest.approx([0.007, 0.007])
    assert [r.metrics.first_motion_s for r in records] == pytest.approx([0.007, 0.007])
    gravity = recording.gravity(101.0, 103.6)
    assert [m.label for m in gravity.markers] == ["pose_01", "end"]
    assert [m.label for m in recording.gravity(100.0, 102.0).markers] == ["pose_01"]


def test_waiting_for_the_recorder_returns_once_later_samples_are_in(recorded: Path) -> None:
    recording = SqliteRecording(recorded)
    recording.wait_for(103.0, timeout_s=1.0)
    with pytest.raises(RecordingUnavailableError, match="no joint_state samples after"):
        recording.wait_for(104.0, timeout_s=0.2)


def test_a_run_without_a_recording_says_so(tmp_path: Path) -> None:
    reason = SqliteRecording(tmp_path / "missing").unavailable()
    assert reason is not None and "does not exist" in reason


def test_the_run_recording_is_found_from_the_run_id(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setenv(DIMOS_RUN_ID_ENV, "20261118-101500-abcd-dual-openyam-calibrate")
    recording = SqliteRecording.of_this_run()
    assert recording is not None
    assert recording.database == (
        RECORDINGS_DIR / "20261118-101500-abcd-dual-openyam-calibrate" / "memory.db"
    )
    monkeypatch.delenv(DIMOS_RUN_ID_ENV)
    assert SqliteRecording.of_this_run() is None


def test_a_probe_is_found_once_the_recorder_wrote_it(tmp_path: Path) -> None:
    with SqliteStore(path=str(tmp_path / "memory.db")) as store:
        with closing(TransportRecorder(store)) as recorder:
            events = _Transport()
            assert recorder.tap(CALIBRATION_EVENTS_STREAM, String, events) is not None
            events.publish(String(json.dumps({"stream": "gravity_events", "marker": {}})))
            events.publish(String(json.dumps(probe_payload("check-1"))))
    recording = SqliteRecording(tmp_path)

    recording.wait_for_probe("check-1", since=0.0, timeout_s=1.0)
    with pytest.raises(RecordingUnavailableError, match="the recorder is not writing"):
        recording.wait_for_probe("check-2", since=0.0, timeout_s=0.2)
