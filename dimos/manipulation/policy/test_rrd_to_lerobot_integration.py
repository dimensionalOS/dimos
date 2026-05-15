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

"""End-to-end integration tests for ``scripts/datasets/rrd_to_lerobot.py``.

Drives `RerunDataRecorder` against an in-memory pubsub fake to write a real
multi-episode rrd session into ``tmp_path``, then runs the converter and
asserts the LeRobot v2 dataset directory has the expected layout.

Requires both ``lerobot`` and a rerun-sdk version that exposes
``rerun.dataframe.view().select()`` (>=0.30). Both are skipped automatically
when not installable.
"""

from __future__ import annotations

import importlib.util
from pathlib import Path
import sys
from typing import Any

import numpy as np
import pytest

# Skip the whole module if either optional dep is missing.
pytest.importorskip("lerobot", reason="datasets extra (lerobot) not installed")
rrx = pytest.importorskip(
    "rerun.experimental", reason="rerun-sdk >=0.32 (rerun.experimental.RrdReader) required"
)
if not hasattr(rrx, "RrdReader"):
    pytest.skip(
        "rerun.experimental.RrdReader not available (need rerun-sdk >= 0.32)",
        allow_module_level=True,
    )
# `data_collection_vis` (used below for the recorder fixture) transitively
# pulls in the Piper teleop blueprint and mujoco; skip the integration when
# mujoco isn't installed.
pytest.importorskip("mujoco", reason="recorder fixtures require the sim extra")


from dimos.manipulation.data_collection.piper_blueprint_config import (
    piper_data_collection_joint_short_names,
    piper_data_collection_rerun_config,
)
from dimos.manipulation.data_collection.recorder import RerunDataRecorder
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.msgs.sensor_msgs.JointState import JointState

_SCRIPT_PATH = Path(__file__).resolve().parents[3] / "scripts" / "datasets" / "rrd_to_lerobot.py"


def _load_script():
    spec = importlib.util.spec_from_file_location("rrd_to_lerobot", _SCRIPT_PATH)
    assert spec is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules["rrd_to_lerobot"] = module
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


@pytest.fixture(scope="module")
def script():
    return _load_script()


# ── Recorder driving (mirrors test_data_collection_integration.py) ────────


class _FakePubSub:
    def __init__(self) -> None:
        self._cbs: list[Any] = []

    def subscribe_all(self, cb):  # type: ignore[no-untyped-def]
        self._cbs.append(cb)
        return lambda: self._cbs.remove(cb) if cb in self._cbs else None

    def start(self) -> None:
        pass

    def stop(self) -> None:
        pass

    def push(self, topic: Any, msg: Any) -> None:
        for cb in list(self._cbs):
            cb(msg, topic)


class _Topic:
    def __init__(self, name: str) -> None:
        self.name = name

    def __str__(self) -> str:
        return self.name


def _make_image() -> Image:
    return Image.from_numpy(np.zeros((480, 640, 3), dtype=np.uint8), format=ImageFormat.RGB)


def _make_joint_state() -> JointState:
    names = [f"arm/{n}" for n in piper_data_collection_joint_short_names()]
    return JointState(name=names, position=[0.0] * len(names))


def _drive_session(tmp_path: Path, *, n_episodes: int, frames_per_episode: int) -> list[Path]:
    pubsub = _FakePubSub()
    cfg = piper_data_collection_rerun_config(session_name="20260514T120000Z")

    counter = {"n": 0}
    paths: list[Path] = []

    def factory() -> Path:
        counter["n"] += 1
        p = tmp_path / f"episode_{counter['n']:03d}.rrd"
        paths.append(p)
        return p

    recorder = RerunDataRecorder(
        pubsubs=[pubsub],
        visual_override=cfg["visual_override"],
        entity_prefix=cfg["entity_prefix"],
        topic_to_entity=cfg["topic_to_entity"],
        camera_entity_path=cfg["camera_entity_path"],
        record_path_factory=factory,
        recording_id_factory=cfg["recording_id_factory"],
        episode_metadata=cfg["episode_metadata"],
    )
    recorder.start()
    for _ in range(n_episodes):
        recorder.toggle_recording()
        for _ in range(frames_per_episode):
            recorder._on_color_image(_make_image())
            pubsub.push(_Topic("/coordinator/joint_state"), _make_joint_state())
            pubsub.push(_Topic("/coordinator/desired_joint_action"), _make_joint_state())
        recorder.toggle_recording()
    recorder.stop()
    return [p for p in paths if p.exists()]


# ── End-to-end conversion ─────────────────────────────────────────────────


def test_two_episode_session_produces_lerobot_v2_layout(tmp_path, script):
    rrds = _drive_session(tmp_path / "rrd", n_episodes=2, frames_per_episode=4)
    assert len(rrds) == 2

    output_dir = tmp_path / "dataset"
    rc = script.main(
        [
            "--input",
            str(rrds[0].parent),
            "--output",
            str(output_dir),
            "--task",
            "pick the red block",
        ]
    )
    assert rc == 0, "converter exited non-zero"

    # LeRobot v2 layout sanity.
    assert (output_dir / "info.json").exists()
    assert (output_dir / "meta" / "episodes.jsonl").exists()
    assert (output_dir / "tasks.jsonl").exists()
    eps = (output_dir / "meta" / "episodes.jsonl").read_text().splitlines()
    assert len([e for e in eps if e.strip()]) == 2

    parquets = sorted((output_dir / "data").rglob("episode_*.parquet"))
    assert len(parquets) == 2
    videos = sorted((output_dir / "videos").rglob("episode_*.mp4"))
    assert len(videos) == 2


def test_empty_rrd_in_session_is_skipped_with_warning(tmp_path, script, caplog):
    rrds = _drive_session(tmp_path / "rrd", n_episodes=1, frames_per_episode=3)
    # Inject a deliberately empty rrd between the valid one(s).
    empty = rrds[0].parent / "episode_002.rrd"
    empty.write_bytes(b"")

    output_dir = tmp_path / "dataset"
    with caplog.at_level("WARNING"):
        rc = script.main(
            [
                "--input",
                str(rrds[0].parent),
                "--output",
                str(output_dir),
                "--task",
                "pick",
            ]
        )
    assert rc == 0
    assert any(
        "episode_002.rrd" in rec.message
        and ("skip" in rec.message.lower() or "fail" in rec.message.lower())
        for rec in caplog.records
    ), [r.message for r in caplog.records]
    # The valid episode still landed.
    eps = (output_dir / "meta" / "episodes.jsonl").read_text().splitlines()
    assert len([e for e in eps if e.strip()]) == 1
