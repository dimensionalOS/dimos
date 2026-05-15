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

Tasks 4.1-4.7 from openspec/changes/add-rerun-data-recorder/tasks.md.
"""

from __future__ import annotations

from collections.abc import Callable
from pathlib import Path
import threading
import time
from typing import Any

import numpy as np
import pytest
from rerun._baseclasses import Archetype
import rerun_bindings as rb

from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.manipulation.data_collection.recorder import RerunDataRecorder
from dimos.visualization.rerun.bridge import RerunBridgeModule

# ── Fakes ────────────────────────────────────────────────────────────────────


class FakePubSub:
    """Minimal subscribe_all-capable pubsub for recorder tests.

    `push(topic, msg)` invokes every registered callback synchronously. No real
    transport, no LCM, no threads.
    """

    def __init__(self) -> None:
        self._callbacks: list[Callable[[Any, Any], None]] = []
        self.started = False
        self.stopped = False

    def subscribe_all(self, callback: Callable[[Any, Any], None]) -> Callable[[], None]:
        self._callbacks.append(callback)

        def unsubscribe() -> None:
            if callback in self._callbacks:
                self._callbacks.remove(callback)

        return unsubscribe

    def start(self) -> None:
        self.started = True

    def stop(self) -> None:
        self.stopped = True

    def push(self, topic: Any, msg: Any) -> None:
        for cb in list(self._callbacks):
            cb(msg, topic)


# ── Helpers ──────────────────────────────────────────────────────────────────


class _Topic:
    """Topic-like object that exposes ``.name`` (mirroring `Topic.__str__`)."""

    def __init__(self, name: str) -> None:
        self.name = name

    def __str__(self) -> str:
        return self.name


def _make_image(value: int = 0) -> Image:
    arr = np.full((4, 4, 3), value % 256, dtype=np.uint8)
    return Image.from_numpy(arr, format=ImageFormat.RGB)


def _make_joint_state(measured: bool, value: float) -> JointState:
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


def test_compose_converter_mirrors_bridge() -> None:
    """Drift-detector: the recorder's `_compose_converter` must produce the
    same (entity_path, archetype) for the same input the bridge would produce.

    Per design.md Decision 5, the bridge and recorder duplicate the small
    composition logic. This test pins them structurally so a future change to
    one without the other fails CI.
    """

    def topic_to_entity(topic: Any) -> str:
        return f"/observation/{topic.name.lstrip('/')}"

    visual_override: dict[Any, Any] = {}

    bridge = RerunBridgeModule(
        pubsubs=[],
        visual_override=visual_override,
        topic_to_entity=topic_to_entity,
    )
    recorder = RerunDataRecorder(
        pubsubs=[],
        visual_override=visual_override,
        topic_to_entity=topic_to_entity,
        record_path_factory=lambda: Path("/tmp/__unused__.rrd"),
    )
    try:
        topic = _Topic("/camera/usb")
        msg = _make_image(value=7)

        bridge_path = bridge._get_entity_path(topic)
        recorder_path = recorder._get_entity_path(topic)
        assert bridge_path == recorder_path == "/observation/camera/usb"

        bridge_out = bridge._visual_override_for_entity_path(bridge_path)(msg)
        recorder_out = recorder._compose_converter(recorder_path)(msg)

        assert type(bridge_out) is type(recorder_out)
        assert isinstance(bridge_out, Archetype)
        assert isinstance(recorder_out, Archetype)
    finally:
        bridge.stop()
        recorder.stop()


def test_start_leaves_recorder_idle_until_first_toggle(tmp_path: Path) -> None:
    """Recorder boots IDLE — no `.rrd` exists until the operator presses
    the toggle button. Warmup motion / scene setup before the first demo
    never end up on disk."""
    pubsub = FakePubSub()
    recorder = RerunDataRecorder(
        pubsubs=[pubsub],
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
    pubsub.push(_Topic("/observation/camera/usb"), _make_image(value=0))
    assert not (tmp_path / "episode_001.rrd").exists()

    # First toggle opens episode_001.
    new_path = recorder.toggle_recording()
    assert new_path == tmp_path / "episode_001.rrd"
    pubsub.push(_Topic("/observation/camera/usb"), _make_image(value=1))
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
    pubsub = FakePubSub()
    recorder = RerunDataRecorder(
        pubsubs=[pubsub],
        record_path_factory=_path_factory(tmp_path),
        entity_prefix="",
    )
    recorder.start()
    recorder.toggle_recording()  # IDLE → RECORDING
    n = 5
    for i in range(n):
        pubsub.push(_Topic("/observation/camera/usb"), _make_image(value=i))
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
    pubsub = FakePubSub()
    recorder = RerunDataRecorder(
        pubsubs=[pubsub],
        record_path_factory=_path_factory(tmp_path),
        entity_prefix="",
    )
    recorder.start()
    # Operator presses to begin episode_001.
    assert recorder.toggle_recording() == tmp_path / "episode_001.rrd"
    for i in range(3):
        pubsub.push(_Topic("/observation/camera/usb"), _make_image(value=i))

    # 1) Toggle off — return None signals "now IDLE".
    assert recorder.toggle_recording() is None

    # 2) Messages during the scene reset land in NO file.
    for i in range(5):
        pubsub.push(_Topic("/observation/camera/usb"), _make_image(value=i + 500))

    # 3) Toggle on — returns the new episode path.
    new_path = recorder.toggle_recording()
    assert new_path == tmp_path / "episode_002.rrd"
    for i in range(2):
        pubsub.push(_Topic("/observation/camera/usb"), _make_image(value=i + 100))
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
    pubsub = FakePubSub()
    recorder = RerunDataRecorder(
        pubsubs=[pubsub],
        record_path_factory=_path_factory(tmp_path),
        entity_prefix="",
    )
    recorder.start()
    # Open episode_001 with no payload logged.
    assert recorder.toggle_recording() == tmp_path / "episode_001.rrd"
    # Toggle off — empty episode should be discarded.
    assert recorder.toggle_recording() is None
    assert not (tmp_path / "episode_001.rrd").exists()

    # Push during the IDLE gap; nothing lands on disk.
    pubsub.push(_Topic("/observation/camera/usb"), _make_image(value=99))

    # Toggle on — opens episode_002, slot 001 is gone (honest signal).
    assert recorder.toggle_recording() == tmp_path / "episode_002.rrd"
    pubsub.push(_Topic("/observation/camera/usb"), _make_image(value=1))
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
    pubsub = FakePubSub()
    recorder = RerunDataRecorder(
        pubsubs=[pubsub],
        record_path_factory=_path_factory(tmp_path),
        entity_prefix="",
    )
    recorder.start()
    recorder.toggle_recording()  # IDLE → RECORDING

    total = 200

    def pusher() -> None:
        for i in range(total):
            pubsub.push(_Topic("/observation/camera/usb"), _make_image(value=i))
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
    pubsub.push(_Topic("/observation/camera/usb"), _make_image(value=999))
    recorder.stop()

    rec2 = rb.load_recording(str(tmp_path / "episode_002.rrd"))
    assert _has_entity(rec2, "/observation/camera/usb")


def test_toggle_is_safe_when_factory_absent(tmp_path: Path) -> None:
    """When `record_path_factory` is unset, `start()` raises before the
    recorder ever enters RECORDING, and `toggle_recording()` is a logged-
    warning no-op returning None."""
    recorder = RerunDataRecorder(pubsubs=[FakePubSub()])
    try:
        with pytest.raises(RuntimeError, match="record_path_factory"):
            recorder.start()
        # Toggling without a configured factory stays IDLE.
        assert recorder.toggle_recording() is None
    finally:
        recorder.stop()


def test_recorder_isolated_from_viewer_failures(tmp_path: Path) -> None:
    """Recorder writes .rrd regardless of whether a bridge is running.

    Mirrors design.md Decision 10: viewer and recorder are orthogonal sinks.
    """
    pubsub = FakePubSub()
    recorder = RerunDataRecorder(
        pubsubs=[pubsub],
        record_path_factory=_path_factory(tmp_path),
        entity_prefix="",
    )
    # No bridge instantiated — recorder must still record once toggled on.
    recorder.start()
    recorder.toggle_recording()
    pubsub.push(_Topic("/observation/camera/usb"), _make_image(value=1))
    recorder.stop()

    rec = rb.load_recording(str(tmp_path / "episode_001.rrd"))
    assert _has_entity(rec, "/observation/camera/usb")
