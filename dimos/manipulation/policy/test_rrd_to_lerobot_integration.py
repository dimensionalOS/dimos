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

Drives `RerunDataRecorder` through its typed-slot handlers to write a real
multi-episode rrd session into ``tmp_path``, then runs the converter and
asserts the LeRobot v3.0 dataset directory has the expected layout.

Requires both ``lerobot`` and a rerun-sdk version that exposes
``rerun.dataframe.view().select()`` (>=0.30). Both are skipped automatically
when not installable.
"""

from __future__ import annotations

import importlib.util
from pathlib import Path
import sys

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


# ── Recorder driving (mirrors test_piper_integration.py) ──────────────────


def _make_image() -> Image:
    return Image.from_numpy(np.zeros((480, 640, 3), dtype=np.uint8), format=ImageFormat.RGB)


def _make_joint_state() -> JointState:
    names = [f"arm/{n}" for n in piper_data_collection_joint_short_names()]
    return JointState(name=names, position=[0.0] * len(names))


def _drive_session(tmp_path: Path, *, n_episodes: int, frames_per_episode: int) -> list[Path]:
    cfg = piper_data_collection_rerun_config(session_name="20260514T120000Z")

    counter = {"n": 0}
    paths: list[Path] = []

    def factory() -> Path:
        counter["n"] += 1
        p = tmp_path / f"episode_{counter['n']:03d}.rrd"
        paths.append(p)
        return p

    recorder = RerunDataRecorder(
        camera_key=cfg["camera_key"],
        record_path_factory=factory,
        recording_id_factory=cfg["recording_id_factory"],
        episode_metadata=cfg["episode_metadata"],
    )
    recorder.start()
    for _ in range(n_episodes):
        recorder.toggle_recording()
        for _ in range(frames_per_episode):
            recorder._on_image(_make_image())
            recorder._on_joint_state(_make_joint_state())
            recorder._on_desired_joint_action(_make_joint_state())
        recorder.toggle_recording()
    recorder.stop()
    return [p for p in paths if p.exists()]


# ── End-to-end conversion ─────────────────────────────────────────────────


def test_two_episode_session_produces_lerobot_v3_layout(tmp_path, script):
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

    # LeRobot v3.0 layout: per-feature concatenated parquet/mp4 chunks under
    # data/ and videos/, with episode boundaries encoded via the
    # `episode_index` column rather than separate per-episode files.
    import json

    import pyarrow.parquet as pq

    info = json.loads((output_dir / "meta" / "info.json").read_text())
    assert info.get("codebase_version") == "v3.0", info.get("codebase_version")
    assert info.get("total_episodes") == 2, info.get("total_episodes")
    assert (output_dir / "meta" / "tasks.parquet").exists()

    parquets = sorted((output_dir / "data").rglob("file-*.parquet"))
    assert parquets, list((output_dir / "data").rglob("*"))
    table = pq.read_table(parquets[0])
    unique_episodes = set(table.column("episode_index").to_pylist())
    assert unique_episodes == {0, 1}, unique_episodes

    videos = sorted((output_dir / "videos").rglob("file-*.mp4"))
    assert videos, list((output_dir / "videos").rglob("*"))


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
    # The valid episode still landed (v3.0: episode count lives in meta/info.json).
    import json

    info = json.loads((output_dir / "meta" / "info.json").read_text())
    assert info.get("total_episodes") == 1, info.get("total_episodes")
