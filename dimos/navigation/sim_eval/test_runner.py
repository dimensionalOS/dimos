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
import typer

from dimos.navigation.sim_eval.cli import override_flag, seeds_of
from dimos.navigation.sim_eval.ground_truth import Difficulty
from dimos.navigation.sim_eval.runner import (
    RECORDING_FILE,
    REPLAY_FILE,
    RESULTS_FILE,
    RUN_FILE,
    SCORE_FILE,
    Episode,
    RunConfig,
    _episode,
    _live,
    rescore,
    run,
    stop_all,
)
from dimos.navigation.sim_eval.suite import Case, Manifest, Rules
from dimos.simulation.scenes.procedural import office

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


def _case(case_id: str, tag: str, split: str, params: dict) -> Case:
    scene = office(1, **params)
    return Case(
        family="office",
        seed=1,
        params=params,
        start=(1.0, 1.0, 0.0),
        goal=(4.0, 1.0, float(scene.params["z0"])),
        tag=tag,
        id=case_id,
        split=split,  # type: ignore[arg-type]
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
    run_dir = run(
        RunConfig(
            suite=suite,
            blueprint="fake",
            repeats=2,
            procs=3,
            out_dir=tmp_path / "run",
            overrides=(f"--fake-recordings={tmp_path / 'recordings'}", "--set-x=1"),
            command=fake_blueprint,
        )
    )
    rows = [json.loads(line) for line in (run_dir / RESULTS_FILE).read_text().splitlines()]
    assert len(rows) == 6
    assert {(r["case_id"], r["repeat"]) for r in rows} == {
        (c, r)
        for c in ("mined-s1-aaaaaa", "narrow_door-s1-bbbbbb", "stay_put-s1-cccccc")
        for r in (0, 1)
    }
    by_case = {(r["case_id"], r["repeat"]): r for r in rows}
    assert by_case[("mined-s1-aaaaaa", 0)]["outcome"] == "success"
    assert by_case[("mined-s1-aaaaaa", 0)]["terminal"] == "arrived"
    assert by_case[("stay_put-s1-cccccc", 1)]["outcome"] == "timeout"
    assert all(r["error"] is None for r in rows)
    episode = run_dir / "episodes" / "narrow_door-s1-bbbbbb" / "r1"
    assert (episode / RECORDING_FILE).exists()
    assert (episode / SCORE_FILE).exists()
    assert 'scene 1 {"door_width": 0.5} --set-x=1' in (episode / "run.log").read_text()
    assert not any((tmp_path / "recordings").iterdir())
    assert (episode / REPLAY_FILE).read_bytes() == b"rrd"
    meta = json.loads((run_dir / RUN_FILE).read_text())
    assert meta["finished"] is True
    assert meta["episodes"] == 6
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
    episode = Episode(case, 0, tmp_path / "hang" / "r0")
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


def test_cli_parses_seed_lists_and_overrides() -> None:
    assert seeds_of("1,3,5-8") == (1, 3, 5, 6, 7, 8)
    assert override_flag("MLSPlannerNative.step_threshold_m=0.24") == (
        "--mlsplannernative.step-threshold-m=0.24"
    )
    with pytest.raises(typer.BadParameter):
        override_flag("MLSPlannerNative.step_threshold_m")
