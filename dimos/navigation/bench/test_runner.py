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

import _thread
from dataclasses import replace
import json
import os
from pathlib import Path
import subprocess
import sys
import time

import cv2
import numpy as np
from numpy.typing import NDArray
import pytest

from dimos.memory.store.sqlite import SqliteStore
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.navigation.bench.ground_truth import Difficulty
from dimos.navigation.bench.playback import Puppet, find_episode, frames
from dimos.navigation.bench.runner import (
    RECORDING_FILE,
    REPLAY_FILE,
    RESULTS_FILE,
    RUN_FILE,
    SCORE_FILE,
    SUITE_FILE,
    LowDiskError,
    Progress,
    RunConfig,
    _provenance,
    _require_whole,
    _snapshot,
    _until_verdict,
    rescore,
    run,
)
from dimos.navigation.bench.scorer import Commands, Poses, Recording, score
from dimos.navigation.bench.snapshot import BACKGROUND, SNAPSHOT_PX, snapshot_path
from dimos.navigation.bench.suite import Case, Level, Manifest, Rules, Split
from dimos.simulation.go2_legged.policy import OnnxGo2Policy
from dimos.simulation.scenes.procedural import Params, block, office

T0 = 1000.0

FAKE_BLUEPRINT = """
import json, signal, subprocess, sys, time
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
if case["tag"] == "hangs":
    signal.pause()
if case["tag"] == "no_terminal":
    child = subprocess.Popen(["sleep", "300"])
    (out / "child.pid").write_text(str(child.pid))
    sys.exit(1)
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
reason = case["tag"] if case["tag"] == "driver_error" else "arrived" if arrive else "timeout"
(out / "terminal.json").write_text(json.dumps({"reason": reason, "t0": t0}))
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
    Manifest("test", Rules(), 0, cases).save(path)
    return path


def _alive(pid: int) -> bool:
    state = subprocess.run(["ps", "-o", "stat=", "-p", str(pid)], capture_output=True, text=True)
    return bool(state.stdout.strip()) and not state.stdout.strip().startswith("Z")


def _git(root: Path, *args: str) -> str:
    identity = ("-c", "user.name=t", "-c", "user.email=t@t", "-c", "commit.gpgsign=false")
    done = subprocess.run(
        ["git", "-C", str(root), *identity, *args], capture_output=True, text=True, check=True
    )
    return done.stdout.strip()


@pytest.fixture
def checkout(monkeypatch: pytest.MonkeyPatch, tmp_path: Path) -> Path:
    root = tmp_path / "checkout"
    root.mkdir()
    _git(root, "init", "-q")
    _git(root, "commit", "-q", "--allow-empty", "--no-verify", "-m", "first")
    monkeypatch.setattr("dimos.navigation.bench.runner.DIMOS_PROJECT_ROOT", root)
    return root


def test_provenance_names_the_commit_and_notices_uncommitted_changes(
    checkout: Path, tmp_path: Path
) -> None:
    commit = _git(checkout, "rev-parse", "HEAD")
    assert _provenance(checkout) == {"commit": commit, "dirty": False}
    (checkout / "changed").write_text("x")
    assert _provenance(checkout) == {"commit": commit, "dirty": True}
    assert _provenance(tmp_path) == {"commit": None, "dirty": None}
    (tmp_path / ".git").mkdir()
    assert _provenance(tmp_path) == {"commit": None, "dirty": None}


def test_run_scores_every_episode_and_rescores_the_same(
    suite: Path, checkout: Path, tmp_path: Path
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
    assert ("stay_put-s1-cccccc", "timeout") in [r.finished for r in reports]
    episode = run_dir / "narrow_door-s1-bbbbbb"
    assert (episode / RECORDING_FILE).exists() and (episode / SCORE_FILE).exists()
    assert (episode / "rerun.rrd").read_bytes() == b"rrd"
    assert 'scene {"door_width": 0.5} --set-x=1' in (episode / "run.log").read_text()
    assert not any((tmp_path / "recordings").iterdir())
    meta = json.loads((run_dir / RUN_FILE).read_text())
    assert meta["finished"] is True
    assert (meta["commit"], meta["dirty"]) == (_git(checkout, "rev-parse", "HEAD"), False)
    assert (tmp_path / "runs").read_text().strip() == str(run_dir.resolve())
    suite.unlink()
    assert set(rescore(run_dir).read_text().splitlines()) == set(results.splitlines())
    assert find_episode("narrow_door") == episode


def test_keeping_failures_drops_the_recordings_of_successes_and_rescores_the_same(
    suite: Path, tmp_path: Path
) -> None:
    (tmp_path / "fake.py").write_text(FAKE_BLUEPRINT)
    run_dir = run(
        RunConfig(
            suite=suite,
            blueprint="fake",
            procs=3,
            out_dir=tmp_path / "run",
            overrides=(f"--fake-recordings={tmp_path / 'recordings'}",),
            keep="failures",
            command=(sys.executable, str(tmp_path / "fake.py")),
        )
    )
    results = (run_dir / RESULTS_FILE).read_text()
    rows = {r["case_id"]: r for r in map(json.loads, results.splitlines())}
    assert rows["mined-s1-aaaaaa"]["outcome"] == "success"
    assert rows["stay_put-s1-cccccc"]["outcome"] == "timeout"
    success, failure = run_dir / "mined-s1-aaaaaa", run_dir / "stay_put-s1-cccccc"
    assert {f.name for f in success.iterdir()} == {"terminal.json", SCORE_FILE, "run.log"}
    assert (failure / RECORDING_FILE).exists()
    assert (failure / REPLAY_FILE).read_bytes() == b"rrd"
    assert set(rescore(run_dir).read_text().splitlines()) == set(results.splitlines())
    (success / SCORE_FILE).unlink()
    rescored = {r["case_id"]: r for r in map(json.loads, rescore(run_dir).read_text().splitlines())}
    lost = rescored["mined-s1-aaaaaa"]
    assert (lost["outcome"], lost["error"]) == (None, "no recording")
    assert rescored["stay_put-s1-cccccc"] == rows["stay_put-s1-cccccc"]


def test_an_interrupted_run_keeps_finished_episodes_and_drops_the_rest(tmp_path: Path) -> None:
    cases = [
        _case("mined-s1-aaaaaa", "mined", "dev", {}),
        _case("hangs-s1-bbbbbb", "hangs", "dev", {}),
        _case("hangs-s1-cccccc", "hangs", "dev", {}),
    ]
    Manifest("test", Rules(), 0, cases).save(tmp_path / "suite.json")
    (tmp_path / "fake.py").write_text(FAKE_BLUEPRINT)
    interrupted: list[bool] = []

    def interrupt_once_both_hang(progress: Progress) -> None:
        if len(progress.running) == 2 and not interrupted:
            interrupted.append(True)
            _thread.interrupt_main()

    config = RunConfig(
        suite=tmp_path / "suite.json",
        blueprint="fake",
        procs=2,
        out_dir=tmp_path / "run",
        overrides=(f"--fake-recordings={tmp_path / 'recordings'}",),
        command=(sys.executable, str(tmp_path / "fake.py")),
    )
    with pytest.raises(KeyboardInterrupt):
        run(config, interrupt_once_both_hang)
    results = tmp_path / "run" / RESULTS_FILE
    written = results.read_text()
    assert [json.loads(row)["case_id"] for row in written.splitlines()] == ["mined-s1-aaaaaa"]
    assert {d.name for d in (tmp_path / "run").iterdir() if d.is_dir()} == {"mined-s1-aaaaaa"}
    assert rescore(tmp_path / "run").read_text() == written
    meta = json.loads((tmp_path / "run" / RUN_FILE).read_text())
    assert meta["finished"] is False and meta["time"] >= meta["started"]
    time.sleep(1.0)
    assert results.read_text() == written


def test_a_run_starts_no_episode_once_free_disk_is_under_the_floor(
    suite: Path, tmp_path: Path
) -> None:
    config = RunConfig(suite=suite, out_dir=tmp_path / "run", min_free_gb=1e12)
    with pytest.raises(LowDiskError):
        run(config)
    assert not [d for d in (tmp_path / "run").iterdir() if d.is_dir()]
    assert json.loads((tmp_path / "run" / RUN_FILE).read_text())["finished"] is False


def test_harness_failures_are_errors_that_survive_a_rescore(tmp_path: Path) -> None:
    cases = [
        _case("mined-s1-aaaaaa", "mined", "dev", {}),
        _case("driver_error-s1-bbbbbb", "driver_error", "dev", {}),
        _case("no_terminal-s1-cccccc", "no_terminal", "dev", {}),
    ]
    Manifest("test", Rules(), 0, cases).save(tmp_path / "suite.json")
    (tmp_path / "fake.py").write_text(FAKE_BLUEPRINT)
    run_dir = run(
        RunConfig(
            suite=tmp_path / "suite.json",
            blueprint="fake",
            procs=2,
            out_dir=tmp_path / "run",
            overrides=(f"--fake-recordings={tmp_path / 'recordings'}",),
            command=(sys.executable, str(tmp_path / "fake.py")),
        )
    )
    results = (run_dir / RESULTS_FILE).read_text()
    rows = {r["case_id"]: r for r in map(json.loads, results.splitlines())}
    assert rows["mined-s1-aaaaaa"]["outcome"] == "success"
    failed = rows["driver_error-s1-bbbbbb"]
    assert (failed["outcome"], failed["error"]) == (None, "driver_error")
    lost = rows["no_terminal-s1-cccccc"]
    assert (lost["outcome"], lost["error"]) == (None, "no terminal record")
    orphan = int((run_dir / "no_terminal-s1-cccccc" / "child.pid").read_text())
    deadline = time.monotonic() + 5.0
    while _alive(orphan) and time.monotonic() < deadline:
        time.sleep(0.05)
    assert not _alive(orphan)
    assert set(rescore(run_dir).read_text().splitlines()) == set(results.splitlines())


def test_a_stuck_verdict_is_scored_at_the_drivers_end_and_not_the_shutdown_tail() -> None:
    rules, goal, t_end = Rules(), (20.0, 0.0, 0.0), 30.0
    t = np.arange(0.0, t_end + 0.5 + 1e-9, 0.05)
    x = np.where(t <= 20.0, 0.5 * t, 10.0 + 0.029 * (t - 20.0)) + np.where(
        t > t_end + 0.2, 0.05, 0.0
    )
    xyz = np.column_stack([x, np.zeros_like(t), np.full_like(t, rules.stand_height_m)])
    recording = Recording(
        end=float(t[-1]),
        pose=Poses(t, xyz, np.zeros_like(xyz)),
        goals=[(0.0, goal)],
        commands=Commands(t, np.tile([0.5, 0.0, 0.0], (len(t), 1))),
        arrivals=[],
        contacts=[],
    )
    assert score(recording, rules, goal, 20.0).outcome == "timeout"
    cut = _until_verdict(recording, {"reason": "stuck", "t_end": t_end})
    assert score(cut, rules, goal, 20.0).outcome == "stuck"
    assert _until_verdict(recording, {"reason": "arrived", "t_end": t_end}) is recording


def test_a_recording_that_lost_ground_truth_is_refused() -> None:
    terminal: dict[str, object] = {"reason": "arrived", "t0": T0, "t_end": T0 + 10.0}
    t = T0 + np.arange(-2.0, 12.0, 0.02)

    def recording(keep: NDArray[np.bool_]) -> Recording:
        return Recording(
            end=float(t[keep][-1]),
            pose=Poses(t[keep], np.zeros((keep.sum(), 3)), np.zeros((keep.sum(), 3))),
        )

    _require_whole(recording(np.ones_like(t, dtype=bool)), terminal)
    with pytest.raises(ValueError, match=r"missing 2\.0 s"):
        _require_whole(recording((t < T0 + 4.0) | (t > T0 + 6.0)), terminal)
    with pytest.raises(ValueError, match=r"missing 3\.0 s"):
        _require_whole(recording(t < T0 + 7.0), terminal)
    _require_whole(recording(t < T0 + 7.0), {"reason": "arrived", "t0": T0})


@pytest.mark.self_hosted
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


def _block_run(tmp_path: Path) -> tuple[Path, Case]:
    case = replace(
        _case("block-s1-aaaaaa", "block", "dev", {}),
        family="block",
        params={"block_height": 0.2},
        level=Level("block_height", 0.2),
    )
    Manifest("test", Rules(), 0, [case]).save(tmp_path / SUITE_FILE)
    return tmp_path, case


def test_no_snapshot_is_saved_and_nothing_raised_when_no_backend_renders(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    run_dir, case = _block_run(tmp_path)
    monkeypatch.setattr("dimos.navigation.bench.runner.GL_BACKENDS", ("missing",))
    _snapshot(run_dir, [case])
    assert not snapshot_path(run_dir, case).exists()
    _snapshot(run_dir, [replace(case, level=None)])
    assert sorted(p.name for p in run_dir.iterdir()) == [SUITE_FILE]


@pytest.mark.self_hosted
def test_the_snapshot_shows_the_goal_and_the_feature_on_a_plain_background(tmp_path: Path) -> None:
    run_dir, case = _block_run(tmp_path)
    scene = block(1, block_height=0.2)
    case = replace(case, feature=(scene.params["block_x"], scene.params["block_y"]))
    Manifest("test", Rules(), 0, [case]).save(run_dir / SUITE_FILE)
    _snapshot(run_dir, [case])
    if not snapshot_path(run_dir, case).exists():
        pytest.skip("no GL backend renders here")
    image = cv2.imread(str(snapshot_path(run_dir, case))).astype(int)
    assert image.shape[:2] == SNAPSHOT_PX[::-1]
    blue, green, red = image[:, :, 0], image[:, :, 1], image[:, :, 2]
    assert (green > 2 * np.maximum(red, blue)).sum() > 20
    assert (red > 2 * np.maximum(green, blue)).sum() > 10
    assert (image[0, 0] == BACKGROUND).all() and (image[-1, -1] == BACKGROUND).all()
