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
from pathlib import Path
import sys
import threading
import time

import pytest

from dimos.navigation.bench.ground_truth import Difficulty
from dimos.navigation.bench.runner import (
    RECORDING_FILE,
    RESULTS_FILE,
    RUN_FILE,
    SCORE_FILE,
    Episode,
    Progress,
    RunConfig,
    _episode,
    _live,
    rescore,
    run,
    stop_all,
)
from dimos.navigation.bench.suite import Case, Manifest, Rules, Split
from dimos.simulation.scenes.procedural import Params, office

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
if case["tag"] == "hang":
    signal.pause()
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
print("scene", args["--simgo2world.seed"], args["--simgo2world.scene-params"], " ".join(a for a in sys.argv if a.startswith("--set-")), flush=True)
signal.pause()
"""


@pytest.fixture(autouse=True)
def _private_slots(monkeypatch: pytest.MonkeyPatch, tmp_path: Path) -> None:
    monkeypatch.setattr("dimos.navigation.bench.runner.SLOTS_DIR", tmp_path / "slots")
    monkeypatch.setattr("dimos.navigation.bench.runner.RUNS_INDEX", tmp_path / "runs")


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
        _case("mined-s1-dddddd", "mined", "held_out", {}),
    ]
    path = tmp_path / "suite.json"
    Manifest("test", Rules(), 0, None, False, cases, []).save(path)
    return path


@pytest.fixture
def fake_blueprint(tmp_path: Path) -> tuple[str, ...]:
    script = tmp_path / "fake_blueprint.py"
    script.write_text(FAKE_BLUEPRINT)
    return (sys.executable, str(script))


def test_run_scores_every_episode_of_the_split_in_parallel(
    suite: Path, fake_blueprint: tuple[str, ...], tmp_path: Path
) -> None:
    reports: list[Progress] = []
    run_dir = run(
        RunConfig(
            suite=suite,
            blueprint="fake",
            procs=3,
            out_dir=tmp_path / "run",
            overrides=(f"--fake-recordings={tmp_path / 'recordings'}", "--set-x=1"),
            command=fake_blueprint,
        ),
        reports.append,
    )
    rows = [json.loads(line) for line in (run_dir / RESULTS_FILE).read_text().splitlines()]
    assert len(rows) == 3
    assert reports[0].done == 0 and len(reports[0].running) == 1
    assert reports[-1].done == reports[-1].total == 3 and not reports[-1].running
    assert sum(reports[-1].outcomes.values()) == 3
    assert sorted(r.finished for r in reports if r.finished) == [
        ("mined-s1-aaaaaa", "success"),
        ("narrow_door-s1-bbbbbb", "success"),
        ("stay_put-s1-cccccc", "timeout"),
    ]
    assert max(len(r.running) for r in reports) <= 3
    assert {r["case_id"] for r in rows} == {
        "mined-s1-aaaaaa",
        "narrow_door-s1-bbbbbb",
        "stay_put-s1-cccccc",
    }
    by_case = {r["case_id"]: r for r in rows}
    assert by_case["mined-s1-aaaaaa"]["outcome"] == "success"
    assert by_case["mined-s1-aaaaaa"]["terminal"] == "arrived"
    assert by_case["stay_put-s1-cccccc"]["outcome"] == "timeout"
    assert all(r["error"] is None for r in rows)
    episode = run_dir / "narrow_door-s1-bbbbbb"
    assert (episode / RECORDING_FILE).exists()
    assert (episode / SCORE_FILE).exists()
    assert (episode / "rerun.rrd").read_bytes() == b"rrd"
    assert 'scene 1 {"door_width": 0.5} --set-x=1' in (episode / "run.log").read_text()
    assert not any((tmp_path / "recordings").iterdir())
    meta = json.loads((run_dir / RUN_FILE).read_text())
    assert meta["finished"] is True
    assert (tmp_path / "runs").read_text().strip() == str(run_dir.resolve())
    assert meta["episodes"] == 3
    assert meta["overrides"][-1] == "--set-x=1"


def test_rescore_reproduces_the_results(
    suite: Path, fake_blueprint: tuple[str, ...], tmp_path: Path
) -> None:
    run_dir = run(
        RunConfig(
            suite=suite,
            split=None,
            cases=("mined-s1-dd",),
            out_dir=tmp_path / "run",
            overrides=(f"--fake-recordings={tmp_path / 'recordings'}",),
            command=fake_blueprint,
        )
    )
    before = (run_dir / RESULTS_FILE).read_text()
    assert rescore(run_dir).read_text() == before
    assert json.loads(before)["case_id"] == "mined-s1-dddddd"


def test_stop_all_ends_an_episode_that_never_terminates(
    suite: Path, fake_blueprint: tuple[str, ...], tmp_path: Path
) -> None:
    manifest = Manifest.load(suite)
    case = _case("hang-s1-eeeeee", "hang", "dev", {})
    config = RunConfig(
        suite=suite,
        overrides=(f"--fake-recordings={tmp_path / 'recordings'}",),
        command=fake_blueprint,
    )
    episode = Episode(case, tmp_path / "hang")
    results: list = []
    worker = threading.Thread(target=lambda: results.append(_episode(config, manifest, episode, 0)))
    worker.start()
    deadline = time.time() + 10.0
    while not _live and time.time() < deadline:
        time.sleep(0.05)
    assert _live
    stop_all()
    worker.join(timeout=30.0)
    assert not worker.is_alive()
    assert results[0].error == "no terminal record"
    assert not _live
