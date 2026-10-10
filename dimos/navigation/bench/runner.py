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

"""Runs a suite: one blueprint process group per episode, several at a time, scored as they finish."""

from __future__ import annotations

from collections import Counter
from collections.abc import Callable, Iterator
from concurrent.futures import ThreadPoolExecutor
from contextlib import contextmanager
from dataclasses import asdict, dataclass, field, replace
import fcntl
import json
import os
from pathlib import Path
import re
import shutil
import signal
import subprocess
import threading
import time

import numpy as np

from dimos.constants import STATE_DIR
from dimos.navigation.bench.driver import HARNESS_REASONS, TERMINAL_FILE
from dimos.navigation.bench.scorer import Outcome, Recording, score
from dimos.navigation.bench.suite import Case, Manifest, Split

RUNS_DIR = STATE_DIR / "nav-bench"
SLOTS_DIR = RUNS_DIR / "slots"
RUNS_INDEX = RUNS_DIR / "runs"
MAX_SLOTS = 64
RESULTS_FILE = "results.jsonl"
RUN_FILE = "run.json"
SUITE_FILE = "suite.json"
SCORE_FILE = "score.json"
RECORDING_FILE = "memory.db"
REPLAY_FILE = "rerun.rrd"
RECORD_TOPICS = (
    "ground_truth,joint_state,odometry,tf,scene,contacts,goal,planner_path,path,cmd_vel,"
    "goal_reached,reference_path,phase,lidar,local_map,surface_map"
)
LCM_PORT_BASE = 7800
ZENOH_SCOUT_GROUP = "224.0.0.224"
LCM_GROUP = "239.255.76.67"
EPISODE_CAP_S = 900.0
ZENOH_PORT_BASE = 7500
RERUN_PORT_BASE = 9900
WEBSOCKET_PORT_BASE = 3100
STOP_GRACE_S = 20.0
POLL_S = 0.5
VERDICT_SLACK_S = 0.1
RECORDING_GAP_S = 1.0
RECORDING_LINE = re.compile(r"Recording to (\S+memory\.db)")

_live: set[subprocess.Popen[bytes]] = set()
_live_lock = threading.Lock()


@dataclass(frozen=True)
class RunConfig:
    suite: Path
    blueprint: str = "go2-sim-nav-episode"
    split: Split | None = "dev"
    cases: tuple[str, ...] = ()
    procs: int = 4
    out_dir: Path | None = None
    overrides: tuple[str, ...] = ()
    policy: Path | None = None
    viewer: bool = False
    command: tuple[str, ...] = ("dimos",)


@dataclass(frozen=True)
class Episode:
    case: Case
    dir: Path


@dataclass
class Result:
    case_id: str
    tag: str
    split: str
    episode: str
    terminal: str | None
    outcome: Outcome | None
    error: str | None = None
    metrics: dict[str, object] = field(default_factory=dict)


@dataclass(frozen=True)
class Progress:
    """Where a run stands."""

    done: int
    total: int
    running: tuple[str, ...]
    outcomes: Counter[str]
    finished: tuple[str, str] | None = None


Report = Callable[[Progress], None]


def selects(prefix: str, case_id: str) -> bool:
    """Whether a case id is the prefix or continues it at a dash, so s1 does not take s10."""
    return case_id == prefix or case_id.startswith(prefix + "-")


def run(config: RunConfig, report: Report | None = None) -> Path:
    """Run every selected case once and return the run directory."""
    manifest = Manifest.load(config.suite)
    manifest.check_drift()
    cases = [
        c
        for c in manifest.cases
        if (config.split is None or c.split == config.split)
        and (not config.cases or any(selects(prefix, c.id) for prefix in config.cases))
    ]
    if not cases:
        raise ValueError(
            f"no cases match split {config.split!r} and {config.cases} in {config.suite}"
        )
    run_dir = config.out_dir or RUNS_DIR / time.strftime(f"%Y%m%d-%H%M%S-{manifest.suite}")
    run_dir.mkdir(parents=True, exist_ok=False)
    RUNS_INDEX.parent.mkdir(parents=True, exist_ok=True)
    with RUNS_INDEX.open("a") as index:
        index.write(f"{run_dir.resolve()}\n")
    episodes = [Episode(case, run_dir / case.id) for case in cases]
    manifest.save(run_dir / SUITE_FILE)
    started = time.time()
    _write_run(run_dir, config, len(episodes), started, finished=False)
    results = run_dir / RESULTS_FILE
    lock = threading.Lock()
    running: list[str] = []
    outcomes: Counter[str] = Counter()

    def progress(finished: tuple[str, str] | None = None) -> None:
        if report is not None:
            report(
                Progress(
                    sum(outcomes.values()),
                    len(episodes),
                    tuple(running),
                    Counter(outcomes),
                    finished,
                )
            )

    def one(episode: Episode) -> Result:
        with lock:
            running.append(episode.dir.name)
            progress()
        with _slot() as slot:
            result = _episode(config, manifest, episode, slot)
        with lock, results.open("a") as out:
            out.write(json.dumps(asdict(result)) + "\n")
            running.remove(episode.dir.name)
            verdict = str(result.outcome or result.error)
            outcomes[verdict] += 1
            progress((episode.dir.name, verdict))
        return result

    pool = ThreadPoolExecutor(max_workers=config.procs)
    try:
        first = one(episodes[0])
        if first.terminal is None:
            raise RuntimeError(
                f"the first episode produced no terminal record, see {episodes[0].dir}"
            )
        for future in [pool.submit(one, episode) for episode in episodes[1:]]:
            future.result()
    except BaseException:
        pool.shutdown(wait=False, cancel_futures=True)
        stop_all()
        raise
    pool.shutdown()
    _write_run(run_dir, config, len(episodes), started, finished=True)
    return run_dir


@contextmanager
def _slot() -> Iterator[int]:
    """A port slot no other runner on this machine holds, locked for the episode's lifetime."""
    SLOTS_DIR.mkdir(parents=True, exist_ok=True)
    for slot in range(MAX_SLOTS):
        handle = (SLOTS_DIR / f"{slot}.lock").open("w")
        try:
            fcntl.flock(handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except OSError:
            handle.close()
            continue
        try:
            yield slot
        finally:
            fcntl.flock(handle, fcntl.LOCK_UN)
            handle.close()
        return
    raise RuntimeError(f"all {MAX_SLOTS} episode slots are held by running episodes")


def stop_all() -> None:
    """Stop every episode process this runner has going."""
    with _live_lock:
        live = list(_live)
    for process in live:
        _stop(process)


def rescore(run_dir: Path) -> Path:
    """Score every episode in the run again from its recording, by the suite the run saved."""
    manifest = Manifest.load(run_dir / SUITE_FILE)
    results = run_dir / RESULTS_FILE
    results.unlink(missing_ok=True)
    with results.open("a") as out:
        for episode_dir in sorted(d for d in run_dir.iterdir() if d.is_dir()):
            episode = Episode(manifest.case(episode_dir.name), episode_dir)
            terminal = _terminal(episode_dir)
            out.write(json.dumps(asdict(_score(manifest, episode, terminal))) + "\n")
    return results


def _write_run(
    run_dir: Path, config: RunConfig, episodes: int, started: float, finished: bool
) -> None:
    record = {
        "blueprint": config.blueprint,
        "split": config.split,
        "cases": list(config.cases),
        "overrides": list(config.overrides),
        "policy": str(config.policy) if config.policy else None,
        "episodes": episodes,
        "finished": finished,
        "started": started,
        "time": time.time(),
    }
    (run_dir / RUN_FILE).write_text(json.dumps(record, indent=2) + "\n")


def _episode(config: RunConfig, manifest: Manifest, episode: Episode, slot: int) -> Result:
    episode.dir.mkdir(parents=True, exist_ok=True)
    log = episode.dir / "run.log"
    with log.open("w") as out:
        process = subprocess.Popen(
            _command(config, episode, slot),
            stdout=out,
            stderr=subprocess.STDOUT,
            env=_environment(slot),
            start_new_session=True,
        )
        with _live_lock:
            _live.add(process)
        try:
            deadline = time.time() + EPISODE_CAP_S
            while time.time() < deadline and process.poll() is None:
                if (episode.dir / TERMINAL_FILE).exists():
                    break
                time.sleep(POLL_S)
        finally:
            _stop(process)
            with _live_lock:
                _live.discard(process)
    _collect_recording(log, episode)
    return _score(manifest, episode, _terminal(episode.dir))


def _command(config: RunConfig, episode: Episode, slot: int) -> list[str]:
    """The dimos run invocation, on a private zenoh scouting group and rerun port for its slot."""
    case = episode.case
    command = [
        *config.command,
        "--viewer",
        "rerun",
        "--rerun-save",
        *(() if config.viewer else ("--rerun-open", "none")),
        "--zenoh-scout-addr",
        f"{ZENOH_SCOUT_GROUP}:{ZENOH_PORT_BASE + slot}",
        "--rerun-websocket-server-port",
        str(WEBSOCKET_PORT_BASE + slot),
        "run",
        config.blueprint,
        f"--rerunbridgemodule.connect-url=rerun+http://127.0.0.1:{RERUN_PORT_BASE + slot}/proxy",
        "--record",
        "sqlite",
        "--record-topics",
        RECORD_TOPICS,
        f"--episodedriver.manifest={episode.dir.parent / SUITE_FILE}",
        f"--episodedriver.case-id={case.id}",
        f"--episodedriver.out-dir={episode.dir}",
        f"--simgo2world.family={case.family}",
        f"--simgo2world.seed={case.seed}",
        f"--simgo2world.scene-params={json.dumps(case.params)}",
    ]
    if config.policy is not None:
        command.append(f"--simgo2world.policy={config.policy}")
    return [*command, *config.overrides]


def _environment(slot: int) -> dict[str, str]:
    """A private LCM multicast port for the slot. LCM reads this variable itself."""
    env = dict(os.environ)
    env["LCM_DEFAULT_URL"] = f"udpm://{LCM_GROUP}:{LCM_PORT_BASE + slot}?ttl=0"
    return env


def _stop(process: subprocess.Popen[bytes]) -> None:
    """Stop the episode's whole session, whether or not its leader is still alive."""
    deadline = time.time() + STOP_GRACE_S
    _signal_session(process, signal.SIGTERM)
    while time.time() < deadline and _signal_session(process, 0):
        time.sleep(POLL_S)
    _signal_session(process, signal.SIGKILL)
    process.wait()


def _signal_session(process: subprocess.Popen[bytes], signum: int) -> bool:
    """Signal the session the leader started, and say whether anything is left in it."""
    process.poll()
    try:
        os.killpg(process.pid, signum)
    except ProcessLookupError:
        return False
    except PermissionError:
        # macOS answers this for a session whose leader is dead but not yet reaped
        return True
    return True


def _collect_recording(log: Path, episode: Episode) -> None:
    """Move the run's recording and rerun file into the episode, found from the log."""
    match = RECORDING_LINE.search(log.read_text(errors="replace"))
    if match is None:
        return
    source = Path(match.group(1))
    if not source.parent.exists():
        return
    for file in source.parent.iterdir():
        if file.name.startswith(RECORDING_FILE):
            shutil.move(str(file), episode.dir / file.name)
        elif file.name == REPLAY_FILE:
            shutil.move(str(file), episode.dir / REPLAY_FILE)
    if not any(source.parent.iterdir()):
        source.parent.rmdir()


def _terminal(episode_dir: Path) -> dict[str, object] | None:
    path = episode_dir / TERMINAL_FILE
    if not path.exists():
        return None
    record: dict[str, object] = json.loads(path.read_text())
    return record


def _score(manifest: Manifest, episode: Episode, terminal: dict[str, object] | None) -> Result:
    case = episode.case
    result = Result(
        case_id=case.id,
        tag=case.tag,
        split=case.split,
        episode=str(episode.dir),
        terminal=str(terminal["reason"]) if terminal else None,
        outcome=None,
    )
    recording = episode.dir / RECORDING_FILE
    if terminal is None:
        result.error = "no terminal record"
        return result
    if result.terminal in HARNESS_REASONS:
        result.error = result.terminal
        return result
    if not recording.exists():
        result.error = "no recording"
        return result
    try:
        recorded = Recording.from_store(recording)
        _require_whole(recorded, terminal)
        scored = score(
            _until_verdict(recorded, terminal),
            manifest.rules,
            case.goal,
            case.route_length,
        )
    except ValueError as error:
        result.error = str(error)
        return result
    result.outcome = scored.outcome
    result.metrics = asdict(scored)
    (episode.dir / SCORE_FILE).write_text(json.dumps(result.metrics, indent=2) + "\n")
    return result


def _require_whole(recording: Recording, terminal: dict[str, object]) -> None:
    """Refuse a recording that lost ground truth between the goal and the driver's verdict."""
    t0, t_end = terminal.get("t0"), terminal.get("t_end")
    if recording.pose is None or not isinstance(t0, float) or not isinstance(t_end, float):
        return
    t = recording.pose.t
    gap = float(np.diff([t0, *t[(t > t0) & (t < t_end)], t_end]).max())
    if gap > RECORDING_GAP_S:
        raise ValueError(f"recording is missing {gap:.1f} s of ground truth")


def _until_verdict(recording: Recording, terminal: dict[str, object]) -> Recording:
    """The recording cut just after the driver's verdict, so the shutdown tail is not scored.

    Recorded stamps trail the driver's clock a little, hence the slack. An arrival is left whole.
    """
    t_end = terminal.get("t_end")
    if terminal["reason"] == "arrived" or not isinstance(t_end, float):
        return recording
    return replace(recording, end=min(recording.end, t_end + VERDICT_SLACK_S))
