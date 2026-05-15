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

"""End-to-end integration tests for the Piper data collection recorder.

These tests drive the same ``piper_data_collection_rerun_config()`` the
blueprint uses, calling the recorder's typed-slot handlers directly. They
cover the contract between the recorder and the data-collection-specific
wiring (entity-path schema, episode-rollover semantics, empty-discard), and
exercise the optional Rerun 0.32 catalog server when the SDK is new enough.
"""

from __future__ import annotations

from collections.abc import Callable
from pathlib import Path
from typing import Any

import numpy as np
import pytest
import rerun_bindings as rb

from dimos.manipulation.data_collection.piper_blueprint_config import (
    piper_data_collection_joint_short_names,
    piper_data_collection_rerun_config,
)
from dimos.manipulation.data_collection.recorder import RerunDataRecorder
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.msgs.sensor_msgs.JointState import JointState


def _make_image() -> Image:
    return Image.from_numpy(np.zeros((4, 4, 3), dtype=np.uint8), format=ImageFormat.RGB)


def _make_joint_state() -> JointState:
    names = [f"arm/{n}" for n in piper_data_collection_joint_short_names()]
    positions = [0.0] * len(names)
    return JointState(name=names, position=positions)


def _recorder_kwargs(
    cfg: dict[str, Any], *, record_path_factory: Callable[[], Path]
) -> dict[str, Any]:
    """Subset of the config the recorder accepts, with the path factory
    swapped to a test-local one."""
    return {
        "camera_key": cfg["camera_key"],
        "record_path_factory": record_path_factory,
        "recording_id_factory": cfg["recording_id_factory"],
        "episode_metadata": cfg["episode_metadata"],
    }


def _drive_session(
    tmp_path: Path,
    *,
    toggle_between_batches: bool,
    first_batch_size: int,
    second_batch_size: int,
) -> RerunDataRecorder:
    """Drive a session with at most two batches separated by an
    operator-style toggle-off / toggle-on cycle.

    Between toggle-off and toggle-on the recorder is IDLE — messages dispatched
    in that window are silently discarded. Tests that want to exercise that
    discard behavior should push during the idle gap themselves.
    """
    cfg = piper_data_collection_rerun_config(session_name="20260514T120000Z")

    counter = {"n": 0}

    def factory() -> Path:
        counter["n"] += 1
        return tmp_path / f"episode_{counter['n']:03d}.rrd"

    recorder = RerunDataRecorder(**_recorder_kwargs(cfg, record_path_factory=factory))
    recorder.start()
    # Operator presses the toggle to begin episode_001 (recorder defaults to IDLE).
    recorder.toggle_recording()
    for _ in range(first_batch_size):
        recorder._on_image(_make_image())
        recorder._on_joint_state(_make_joint_state())
        recorder._on_desired_joint_action(_make_joint_state())
    if toggle_between_batches:
        # Stop. The operator resets the scene here — modeled by the same
        # interface presses (no payload is required, but the recorder is IDLE).
        recorder.toggle_recording()
        # Resume.
        recorder.toggle_recording()
    for _ in range(second_batch_size):
        recorder._on_image(_make_image())
        recorder._on_joint_state(_make_joint_state())
        recorder._on_desired_joint_action(_make_joint_state())
    recorder.stop()
    return recorder


def _entity_paths(rec: rb.Recording) -> list[str]:
    return [str(col.entity_path) for col in rec.schema().component_columns()]


def test_session_with_toggle_writes_two_episodes_with_lerobot_paths(
    tmp_path: Path,
) -> None:
    """Toggle-off / scene-reset / toggle-on splits a session into two `.rrd`
    files; each contains the camera observation, per-joint state/action
    scalars, and the static metadata under `/meta/...`."""
    _drive_session(
        tmp_path,
        toggle_between_batches=True,
        first_batch_size=3,
        second_batch_size=2,
    )

    first = tmp_path / "episode_001.rrd"
    second = tmp_path / "episode_002.rrd"
    assert first.exists()
    assert second.exists()

    joints = piper_data_collection_joint_short_names()
    for rrd in (first, second):
        rec = rb.load_recording(str(rrd))
        paths = _entity_paths(rec)
        assert "/observation/camera/usb" in paths, rrd.name
        for j in joints:
            assert f"/observation/state/{j}" in paths, (rrd.name, j)
            assert f"/action/{j}" in paths, (rrd.name, j)
        # Static per-episode metadata.
        assert "/meta/episode_id" in paths
        assert "/meta/episode_index" in paths
        assert "/meta/session_id" in paths


def test_messages_during_idle_gap_are_not_persisted(tmp_path: Path) -> None:
    """Between toggle-off and toggle-on the recorder is IDLE — messages
    dispatched during this window must NOT appear in either the closing
    episode (already closed) or the next episode (not yet opened)."""
    cfg = piper_data_collection_rerun_config(session_name="20260514T120000Z")
    counter = {"n": 0}

    def factory() -> Path:
        counter["n"] += 1
        return tmp_path / f"episode_{counter['n']:03d}.rrd"

    recorder = RerunDataRecorder(**_recorder_kwargs(cfg, record_path_factory=factory))
    recorder.start()
    # Operator toggles on (recorder is IDLE by default).
    recorder.toggle_recording()
    recorder._on_image(_make_image())

    # Toggle off: the operator is resetting the scene.
    assert recorder.toggle_recording() is None

    # While IDLE, push messages on every recorded slot — none should land.
    for _ in range(10):
        recorder._on_image(_make_image())
        recorder._on_joint_state(_make_joint_state())
        recorder._on_desired_joint_action(_make_joint_state())

    # Verify no new file opened during the gap.
    assert not (tmp_path / "episode_002.rrd").exists()

    # Toggle on: resumes into episode_002.
    new_path = recorder.toggle_recording()
    assert new_path == tmp_path / "episode_002.rrd"
    recorder._on_image(_make_image())
    recorder.stop()

    # Both files exist; both contain the camera entity from their own batches,
    # nothing leaked across.
    rec1 = rb.load_recording(str(tmp_path / "episode_001.rrd"))
    rec2 = rb.load_recording(str(tmp_path / "episode_002.rrd"))
    assert "/observation/camera/usb" in _entity_paths(rec1)
    assert "/observation/camera/usb" in _entity_paths(rec2)


def test_empty_first_episode_is_discarded_on_toggle_off(tmp_path: Path) -> None:
    """If the operator toggles off before any payload has been logged, the
    empty file is removed; the next toggle-on opens the next slot."""
    _drive_session(
        tmp_path,
        toggle_between_batches=True,
        first_batch_size=0,
        second_batch_size=2,
    )
    assert not (tmp_path / "episode_001.rrd").exists()
    second = tmp_path / "episode_002.rrd"
    assert second.exists()
    rec = rb.load_recording(str(second))
    assert "/observation/camera/usb" in _entity_paths(rec)


def test_catalog_server_lists_each_episode_as_a_row(tmp_path: Path) -> None:
    """Catalog server smoke test — skipped on rerun-sdk < 0.32 where the
    dataset-listing API (`server.get_dataset(...).list_entries()`) is absent.

    On a sufficiently new SDK, two episode files in the same directory should
    surface as two rows in the corresponding dataset.
    """
    import rerun as rr

    server_factory = getattr(getattr(rr, "server", None), "Server", None)
    if server_factory is None:
        pytest.skip("rerun-sdk does not expose rr.server.Server (needs 0.32+)")

    _drive_session(
        tmp_path,
        toggle_between_batches=True,
        first_batch_size=1,
        second_batch_size=1,
    )
    server = server_factory(datasets={"piper": str(tmp_path)})
    try:
        if not hasattr(server, "get_dataset"):
            pytest.skip("rerun-sdk Server.get_dataset not available (needs 0.32+)")
        ds = server.get_dataset("piper")
        if not hasattr(ds, "list_entries"):
            pytest.skip("rerun-sdk dataset.list_entries not available (needs 0.32+)")
        rows = list(ds.list_entries())
        assert len(rows) == 2
    finally:
        close = getattr(server, "shutdown", None) or getattr(server, "close", None)
        if close is not None:
            close()
