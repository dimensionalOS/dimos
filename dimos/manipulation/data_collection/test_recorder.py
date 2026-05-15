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

"""Tests for the standalone Rerun data recorder.

The recorder runs entirely on typed `In[T]` stream slots (image, joint_state,
desired_joint_action). Tests drive the handlers (`_on_image`, `_on_joint_state`,
`_on_desired_joint_action`) directly — no fake pubsub needed.
"""

from __future__ import annotations

from collections.abc import Callable
from pathlib import Path
import threading
import time

import numpy as np
import pytest
import rerun_bindings as rb

from dimos.manipulation.data_collection.recorder import RerunDataRecorder
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.msgs.sensor_msgs.JointState import JointState

# ── Helpers ──────────────────────────────────────────────────────────────────


def _make_image(value: int = 0) -> Image:
    arr = np.full((4, 4, 3), value % 256, dtype=np.uint8)
    return Image.from_numpy(arr, format=ImageFormat.RGB)


def _make_joint_state(value: float = 0.0) -> JointState:
    return JointState(
        name=["arm/joint1", "arm/gripper"],
        position=[value, value * 0.1],
    )


def _path_factory(directory: Path) -> Callable[[], Path]:
    """Sequential episode path factory rooted at ``directory``."""
    counter = {"n": 0}

    def factory() -> Path:
        counter["n"] += 1
        return directory / f"episode_{counter['n']:03d}.rrd"

    return factory


def _entity_paths(rec: rb.Recording) -> list[str]:
    return [str(col.entity_path) for col in rec.schema().component_columns()]


def _has_entity(rec: rb.Recording, path: str) -> bool:
    return any(p == path for p in _entity_paths(rec))


# ── Tests ────────────────────────────────────────────────────────────────────


def test_start_leaves_recorder_idle_until_first_toggle(tmp_path: Path) -> None:
    """Recorder boots IDLE — no `.rrd` exists until the operator presses
    the toggle button. Warmup motion / scene setup before the first demo
    never end up on disk."""
    recorder = RerunDataRecorder(
        record_path_factory=_path_factory(tmp_path),
        recording_id_factory=lambda p: f"sess/{p.stem}",
        episode_metadata=lambda n: {
            "episode_id": f"sess/episode_{n:03d}",
            "episode_index": str(n),
            "session_id": "sess",
        },
    )
    recorder.start()
    # Push a payload before any toggle — must be silently discarded.
    recorder._on_image(_make_image(value=0))
    assert not (tmp_path / "episode_001.rrd").exists()

    # First toggle opens episode_001.
    new_path = recorder.toggle_recording()
    assert new_path == tmp_path / "episode_001.rrd"
    recorder._on_image(_make_image(value=1))
    recorder.stop()

    expected = tmp_path / "episode_001.rrd"
    assert expected.exists(), list(tmp_path.iterdir())

    rec = rb.load_recording(str(expected))
    assert rec.recording_id() == "sess/episode_001"
    paths = _entity_paths(rec)
    assert "/meta/episode_id" in paths
    assert "/meta/episode_index" in paths
    assert "/meta/session_id" in paths
    # Operator absent because we didn't configure it.
    assert "/meta/operator" not in paths


def test_payload_messages_land_in_current_episode(tmp_path: Path) -> None:
    recorder = RerunDataRecorder(
        record_path_factory=_path_factory(tmp_path),
    )
    recorder.start()
    recorder.toggle_recording()  # IDLE → RECORDING
    n = 5
    for i in range(n):
        recorder._on_image(_make_image(value=i))
    recorder.stop()

    rec = rb.load_recording(str(tmp_path / "episode_001.rrd"))
    assert _has_entity(rec, "/observation/camera/usb")


def test_toggle_off_then_on_creates_two_episodes_with_idle_gap(tmp_path: Path) -> None:
    """Toggle has stop-and-wait semantics so the operator can reset the scene
    between demonstrations:

    1. RECORDING → press → IDLE (current file closed)
    2. Messages pushed while IDLE are silently discarded by the recorder
    3. IDLE → press → RECORDING (new episode opened)
    """
    recorder = RerunDataRecorder(
        record_path_factory=_path_factory(tmp_path),
    )
    recorder.start()
    # Operator presses to begin episode_001.
    assert recorder.toggle_recording() == tmp_path / "episode_001.rrd"
    for i in range(3):
        recorder._on_image(_make_image(value=i))

    # 1) Toggle off — return None signals "now IDLE".
    assert recorder.toggle_recording() is None

    # 2) Messages during the scene reset land in NO file.
    for i in range(5):
        recorder._on_image(_make_image(value=i + 500))

    # 3) Toggle on — returns the new episode path.
    new_path = recorder.toggle_recording()
    assert new_path == tmp_path / "episode_002.rrd"
    for i in range(2):
        recorder._on_image(_make_image(value=i + 100))
    recorder.stop()

    first = tmp_path / "episode_001.rrd"
    second = tmp_path / "episode_002.rrd"
    assert first.exists()
    assert second.exists()
    rec1 = rb.load_recording(str(first))
    rec2 = rb.load_recording(str(second))
    assert _has_entity(rec1, "/observation/camera/usb")
    assert _has_entity(rec2, "/observation/camera/usb")
    # Recording IDs differ so the catalog can address them as distinct rows.
    assert rec1.recording_id() != rec2.recording_id()


def test_toggle_off_with_empty_episode_deletes_file(tmp_path: Path) -> None:
    """Toggling off an episode that received no payload discards the empty
    file. A subsequent toggle-on opens the next slot."""
    recorder = RerunDataRecorder(
        record_path_factory=_path_factory(tmp_path),
    )
    recorder.start()
    # Open episode_001 with no payload logged.
    assert recorder.toggle_recording() == tmp_path / "episode_001.rrd"
    # Toggle off — empty episode should be discarded.
    assert recorder.toggle_recording() is None
    assert not (tmp_path / "episode_001.rrd").exists()

    # Push during the IDLE gap; nothing lands on disk.
    recorder._on_image(_make_image(value=99))

    # Toggle on — opens episode_002, slot 001 is gone (honest signal).
    assert recorder.toggle_recording() == tmp_path / "episode_002.rrd"
    recorder._on_image(_make_image(value=1))
    recorder.stop()

    assert not (tmp_path / "episode_001.rrd").exists()
    second = tmp_path / "episode_002.rrd"
    assert second.exists()
    rec = rb.load_recording(str(second))
    assert _has_entity(rec, "/observation/camera/usb")


def test_toggle_off_under_message_pressure_drops_nothing_to_disk(tmp_path: Path) -> None:
    """Concurrent pushes around a toggle-off must not leak into either the
    closing file (after we declared it closed) or any new file (we're IDLE).

    The recorder is IDLE between toggle-off and toggle-on; messages dispatched
    during that window are silently discarded."""
    recorder = RerunDataRecorder(
        record_path_factory=_path_factory(tmp_path),
    )
    recorder.start()
    recorder.toggle_recording()  # IDLE → RECORDING

    total = 200

    def pusher() -> None:
        for i in range(total):
            recorder._on_image(_make_image(value=i))
            time.sleep(0.0005)

    t = threading.Thread(target=pusher)
    t.start()
    time.sleep(0.02)
    assert recorder.toggle_recording() is None  # → IDLE
    t.join(timeout=5.0)
    assert not t.is_alive(), "pusher thread leaked"

    # No new file should have been opened — we are still IDLE.
    assert not (tmp_path / "episode_002.rrd").exists()

    # Now toggle back on and push one more before stopping.
    new_path = recorder.toggle_recording()
    assert new_path == tmp_path / "episode_002.rrd"
    recorder._on_image(_make_image(value=999))
    recorder.stop()

    rec2 = rb.load_recording(str(tmp_path / "episode_002.rrd"))
    assert _has_entity(rec2, "/observation/camera/usb")


def test_toggle_is_safe_when_factory_absent(tmp_path: Path) -> None:
    """When `record_path_factory` is unset, `start()` raises before the
    recorder ever enters RECORDING, and `toggle_recording()` is a logged-
    warning no-op returning None."""
    recorder = RerunDataRecorder()
    try:
        with pytest.raises(RuntimeError, match="record_path_factory"):
            recorder.start()
        # Toggling without a configured factory stays IDLE.
        assert recorder.toggle_recording() is None
    finally:
        recorder.stop()


def test_image_slot_logs_under_configured_camera_key(tmp_path: Path) -> None:
    """Frames received on the typed `image: In[Image]` slot are logged
    under `/observation/camera/{camera_key}`."""
    recorder = RerunDataRecorder(
        record_path_factory=_path_factory(tmp_path),
        camera_key="usb",
    )
    recorder.start()
    recorder.toggle_recording()
    for i in range(3):
        recorder._on_image(_make_image(value=i))
    recorder.stop()

    rec = rb.load_recording(str(tmp_path / "episode_001.rrd"))
    assert _has_entity(rec, "/observation/camera/usb")


def test_image_slot_respects_idle_state(tmp_path: Path) -> None:
    """Frames received on the typed slot while the recorder is IDLE are
    dropped on the floor; no file is opened, no log lands anywhere."""
    recorder = RerunDataRecorder(
        record_path_factory=_path_factory(tmp_path),
    )
    recorder.start()
    # Recorder is IDLE — push some frames anyway.
    for i in range(5):
        recorder._on_image(_make_image(value=i))
    recorder.stop()
    assert not (tmp_path / "episode_001.rrd").exists()


def test_camera_key_controls_entity_path(tmp_path: Path) -> None:
    """`camera_key` controls the leaf segment of the camera entity path."""
    recorder = RerunDataRecorder(
        record_path_factory=_path_factory(tmp_path),
        camera_key="wrist",
    )
    recorder.start()
    recorder.toggle_recording()
    recorder._on_image(_make_image(value=42))
    recorder.stop()

    rec = rb.load_recording(str(tmp_path / "episode_001.rrd"))
    assert _has_entity(rec, "/observation/camera/wrist")
    # The default key is NOT used.
    assert not _has_entity(rec, "/observation/camera/usb")


def test_joint_state_slot_logs_measured_scalars(tmp_path: Path) -> None:
    """Frames received on `joint_state: In[JointState]` are logged as
    per-joint scalars under `/observation/state/<joint>`."""
    recorder = RerunDataRecorder(
        record_path_factory=_path_factory(tmp_path),
    )
    recorder.start()
    recorder.toggle_recording()
    for i in range(3):
        recorder._on_joint_state(_make_joint_state(value=float(i)))
    recorder.stop()

    rec = rb.load_recording(str(tmp_path / "episode_001.rrd"))
    assert _has_entity(rec, "/observation/state/joint1")
    assert _has_entity(rec, "/observation/state/gripper")


def test_desired_joint_action_slot_logs_action_scalars(tmp_path: Path) -> None:
    """Frames received on `desired_joint_action: In[JointState]` are logged
    as per-joint scalars under `/action/<joint>`."""
    recorder = RerunDataRecorder(
        record_path_factory=_path_factory(tmp_path),
    )
    recorder.start()
    recorder.toggle_recording()
    for i in range(3):
        recorder._on_desired_joint_action(_make_joint_state(value=float(i) * 0.5))
    recorder.stop()

    rec = rb.load_recording(str(tmp_path / "episode_001.rrd"))
    assert _has_entity(rec, "/action/joint1")
    assert _has_entity(rec, "/action/gripper")


def test_joint_state_slots_respect_idle_state(tmp_path: Path) -> None:
    """Joint state and desired action received while IDLE are dropped."""
    recorder = RerunDataRecorder(
        record_path_factory=_path_factory(tmp_path),
    )
    recorder.start()
    for i in range(5):
        recorder._on_joint_state(_make_joint_state(value=float(i)))
        recorder._on_desired_joint_action(_make_joint_state(value=float(i)))
    recorder.stop()
    assert not (tmp_path / "episode_001.rrd").exists()


def test_recorder_is_independent_of_viewer(tmp_path: Path) -> None:
    """Recorder writes .rrd regardless of whether a bridge is running.

    Viewer and recorder are orthogonal sinks.
    """
    recorder = RerunDataRecorder(
        record_path_factory=_path_factory(tmp_path),
    )
    # No bridge instantiated — recorder must still record once toggled on.
    recorder.start()
    recorder.toggle_recording()
    recorder._on_image(_make_image(value=1))
    recorder.stop()

    rec = rb.load_recording(str(tmp_path / "episode_001.rrd"))
    assert _has_entity(rec, "/observation/camera/usb")
