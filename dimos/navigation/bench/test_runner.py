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

from __future__ import annotations

import json
import os
from pathlib import Path
import subprocess
import sys

import pytest

from dimos.memory.store.sqlite import SqliteStore
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.navigation.bench.ground_truth import Difficulty
from dimos.navigation.bench.playback import Puppet, find_episode, frames
from dimos.navigation.bench.runner import (
    RECORDING_FILE,
    RESULTS_FILE,
    RUN_FILE,
    SCORE_FILE,
    Progress,
    RunConfig,
    rescore,
    run,
)
from dimos.navigation.bench.suite import Case, Manifest, Rules, Split
from dimos.simulation.go2_legged.policy import OnnxGo2Policy
from dimos.simulation.scenes.procedural import Params, office

T0 = 1000.0

FAKE_BLUEPRINT = """
import json, signal, sys, time
from pathlib import Path
from dimos.memory.store.sqlite import SqliteStore
from dimos.msgs.geometry_msgs.PointStamped import PointStamped
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.std_msgs.Bool import Bool

args = dict(a.split("=", 1) for a in sys.argv if a.startswith("--") and "=" in a)
out = Path(args["--episodedriver.out-dir"])
case_id = args["--episodedriver.case-id"]
manifest = json.loads(Path(args["--episodedriver.manifest"]).read_text())
case = next(c for c in manifest["cases"] if c["id"] == case_id)
arrive = case["tag"] != "stay_put"
recording = Path(args["--fake-recordings"]) / f"{case_id}-{time.time_ns()}" / "memory.db"
recording.parent.mkdir(parents=True)
store = SqliteStore(path=str(recording))
store.start()
t0 = time.time()
x, y, z = case["goal"]
poses = store.stream("ground_truth", PoseStamped)
for i in range(20):
    px = x if arrive else x - 3.0
    poses.append(PoseStamped(px, y, z + 0.3, 0, 0, 0, 1, ts=t0 + i, frame_id="odom"), ts=t0 + i)
store.stream("goal", PointStamped).append(PointStamped(x, y, z, ts=t0, frame_id="odom"), ts=t0)
if arrive:
    store.stream("goal_reached", Bool).append(Bool(True), ts=t0 + 5)
store.stop()
(recording.parent / "rerun.rrd").write_bytes(b"rrd")
print(f"Recording to {recording}", flush=True)
(out / "terminal.json").write_text(json.dumps({"reason": "arrived" if arrive else "timeout", "t0": t0}))
print("scene", args["--simgo2world.scene-params"], *[a for a in sys.argv if a.startswith("--set-")], flush=True)
signal.pause()
"""


@pytest.fixture(autouse=True)
def _private_state(monkeypatch: pytest.MonkeyPatch, tmp_path: Path) -> None:
    monkeypatch.setattr("dimos.navigation.bench.runner.SLOTS_DIR", tmp_path / "slots")
    monkeypatch.setattr("dimos.navigation.bench.runner.RUNS_INDEX", tmp_path / "runs")
    monkeypatch.setattr("dimos.navigation.bench.playback.RUNS_INDEX", tmp_path / "runs")


def _case(case_id: str, tag: str, split: Split, params: Params) -> Case:
    scene = office(1, **params)
    return Case(
        family="office",
        seed=1,
        params=params,
        start=(1.0, 1.0, 0.0),
        goal=(4.0, 1.0, float(scene.params["z0"])),
        tag=tag,
        id=case_id,
        split=split,
        difficulty=Difficulty(0.5, 0, 1.0, 0),
        route_length=3.0,
        scene_digest=scene.digest(),
    )


@pytest.fixture
def suite(tmp_path: Path) -> Path:
    cases = [
        _case("mined-s1-aaaaaa", "mined", "dev", {}),
        _case("narrow_door-s1-bbbbbb", "narrow_door", "dev", {"door_width": 0.5}),
        _case("stay_put-s1-cccccc", "stay_put", "dev", {}),
    ]
    path = tmp_path / "suite.json"
    Manifest("test", Rules(), 0, None, False, cases, []).save(path)
    return path


def test_run_scores_every_episode_in_parallel_and_rescores_the_same(
    suite: Path, tmp_path: Path
) -> None:
    (tmp_path / "fake.py").write_text(FAKE_BLUEPRINT)
    reports: list[Progress] = []
    run_dir = run(
        RunConfig(
            suite=suite,
            blueprint="fake",
            procs=3,
            out_dir=tmp_path / "run",
            overrides=(f"--fake-recordings={tmp_path / 'recordings'}", "--set-x=1"),
            command=(sys.executable, str(tmp_path / "fake.py")),
        ),
        reports.append,
    )
    results = (run_dir / RESULTS_FILE).read_text()
    rows = {r["case_id"]: r for r in map(json.loads, results.splitlines())}
    assert len(rows) == 3 and all(r["error"] is None for r in rows.values())
    assert rows["mined-s1-aaaaaa"]["outcome"] == "success"
    assert rows["stay_put-s1-cccccc"]["outcome"] == "timeout"
    assert reports[-1].done == reports[-1].total == 3 and not reports[-1].running
    assert max(len(r.running) for r in reports) <= 3
    assert ("stay_put-s1-cccccc", "timeout") in [r.finished for r in reports]
    episode = run_dir / "narrow_door-s1-bbbbbb"
    assert (episode / RECORDING_FILE).exists() and (episode / SCORE_FILE).exists()
    assert (episode / "rerun.rrd").read_bytes() == b"rrd"
    assert 'scene {"door_width": 0.5} --set-x=1' in (episode / "run.log").read_text()
    assert not any((tmp_path / "recordings").iterdir())
    assert json.loads((run_dir / RUN_FILE).read_text())["finished"] is True
    assert (tmp_path / "runs").read_text().strip() == str(run_dir.resolve())
    assert set(rescore(run_dir).read_text().splitlines()) == set(results.splitlines())
    assert find_episode("narrow_door") == episode


def test_playback_poses_the_puppet_from_a_recording(tmp_path: Path) -> None:
    joints = list(OnnxGo2Policy.joint_names)
    store = SqliteStore(path=str(tmp_path / "memory.db"))
    store.start()
    for k in range(3):
        t = T0 + 0.02 * k
        pose = PoseStamped(1.0 + k, 2.0, 0.3, 0.0, 0.0, 0.0, 1.0, ts=t, frame_id="odom")
        store.stream("ground_truth", PoseStamped).append(pose, ts=t)
        state = JointState(ts=t + 0.001, name=joints, position=[0.1 * k] * len(joints))
        store.stream("joint_state", JointState).append(state, ts=t + 0.001)
    store.stop()
    recorded = frames(tmp_path / "memory.db")
    assert [f.position[0] for f in recorded] == [1.0, 2.0, 3.0]
    puppet = Puppet(office(1))
    puppet.pose(recorded[-1])
    assert puppet.trunk_position() == pytest.approx([3.0, 2.0, 0.3])
    assert puppet.data.qpos[7:] == pytest.approx([0.2] * len(joints))


def test_the_dimos_entry_point_imports_without_mujoco(tmp_path: Path) -> None:
    (tmp_path / "mujoco").mkdir()
    (tmp_path / "mujoco" / "__init__.py").write_text('raise ImportError("no mujoco here")\n')
    env = {**os.environ, "PYTHONPATH": f"{tmp_path}{os.pathsep}{os.environ.get('PYTHONPATH', '')}"}
    probe = subprocess.run(
        [sys.executable, "-c", "import dimos.cli.dimos"], env=env, capture_output=True, text=True
    )
    assert probe.returncode == 0, probe.stderr[-2000:]
