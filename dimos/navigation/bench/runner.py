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
from dataclasses import asdict, dataclass, field
import fcntl
import json
import os
from pathlib import Path
import re
import shutil
import signal
import socket
import subprocess
import threading
import time

from dimos.constants import STATE_DIR
from dimos.core.coordination.blueprint_config.sources import configuration_environment
from dimos.navigation.bench.driver import TERMINAL_FILE
from dimos.navigation.bench.scorer import Recording, score
from dimos.navigation.bench.suite import Case, Manifest, Split, git_state
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

RUNS_DIR = STATE_DIR / "nav-bench"
SLOTS_DIR = RUNS_DIR / "slots"
MAX_SLOTS = 64
RESULTS_FILE = "results.jsonl"
RUN_FILE = "run.json"
SCORE_FILE = "score.json"
RECORDING_FILE = "memory.db"
REPLAY_FILE = "rerun.rrd"
RECORD_TOPICS = (
    "ground_truth,joint_state,odometry,tf,scene,contacts,goal,planner_path,path,cmd_vel,"
    "goal_reached,lidar,local_map,surface_map"
)
LCM_PORT_BASE = 7800
ZENOH_SCOUT_GROUP = "224.0.0.224"
LCM_GROUP = "239.255.76.67"
EPISODE_CAP_S = 900.0
ZENOH_PORT_BASE = 7500
RERUN_PORT_BASE = 9900
WEBSOCKET_PORT_BASE = 3100
STOP_GRACE_S = 20.0
RECORDING_LINE = re.compile(r"Recording to (\S+memory\.db)")
POLICY_ENV = "SIMGO2WORLD__POLICY"

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
    viewer: str = "none"
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
    outcome: str | None
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


def run(config: RunConfig, report: Report | None = None) -> Path:
    """Run every selected case once and return the run directory."""
    manifest = Manifest.load(config.suite)
    manifest.check_drift()
    cases = [
        c
        for c in manifest.cases
        if (config.split is None or c.split == config.split)
        and (not config.cases or c.id.startswith(config.cases))
    ]
    if not cases:
        raise ValueError(
            f"no cases match split {config.split!r} and {config.cases} in {config.suite}"
        )
    run_dir = config.out_dir or RUNS_DIR / time.strftime(f"%Y%m%d-%H%M%S-{manifest.suite}")
    run_dir.mkdir(parents=True, exist_ok=False)
    episodes = [Episode(case, run_dir / case.id) for case in cases]
    _write_run(run_dir, config, manifest, len(episodes), finished=False)
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

    previous = signal.signal(signal.SIGTERM, _interrupt)
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
    finally:
        signal.signal(signal.SIGTERM, previous)
    pool.shutdown()
    _write_run(run_dir, config, manifest, len(episodes), finished=True)
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


def _interrupt(*_: object) -> None:
    raise KeyboardInterrupt


def rescore(run_dir: Path) -> Path:
    """Score every episode in the run again from its recording, by the suite the run names."""
    meta = json.loads((run_dir / RUN_FILE).read_text())
    manifest = Manifest.load(Path(meta["suite"]))
    cases = {c.id: c for c in manifest.cases}
    results = run_dir / RESULTS_FILE
    results.unlink(missing_ok=True)
    with results.open("a") as out:
        for episode_dir in sorted(_episode_dirs(run_dir)):
            episode = Episode(cases[episode_dir.name], episode_dir)
            terminal = _terminal(episode_dir)
            out.write(json.dumps(asdict(_score(manifest, episode, terminal))) + "\n")
    return results


def _episode_dirs(run_dir: Path) -> Iterator[Path]:
    return (d for d in run_dir.iterdir() if d.is_dir() and (d / TERMINAL_FILE).exists())


def _write_run(
    run_dir: Path, config: RunConfig, manifest: Manifest, episodes: int, finished: bool
) -> None:
    sha, dirty = git_state()
    record = {
        "suite": str(config.suite.resolve()),
        "suite_name": manifest.suite,
        "rules_version": manifest.rules.version,
        "blueprint": config.blueprint,
        "split": config.split,
        "cases": list(config.cases),
        "procs": config.procs,
        "overrides": list(config.overrides),
        "policy": str(config.policy) if config.policy else _environment_policy(),
        "record_topics": RECORD_TOPICS,
        "viewer": config.viewer,
        "git_sha": sha,
        "git_dirty": dirty,
        "host": socket.gethostname(),
        "episodes": episodes,
        "finished": finished,
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
                time.sleep(0.5)
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
        *(("--rerun-open", "none") if config.viewer == "none" else ()),
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
        f"--episodedriver.manifest={config.suite.resolve()}",
        f"--episodedriver.case-id={case.id}",
        f"--episodedriver.out-dir={episode.dir}",
        f"--simgo2world.family={case.family}",
        f"--simgo2world.seed={case.seed}",
        f"--simgo2world.scene-params={json.dumps(case.params)}",
    ]
    if config.policy is not None:
        command.append(f"--simgo2world.policy={config.policy}")
    return [*command, *config.overrides]


def _environment_policy() -> str | None:
    """The world's policy as the environment or .env sets it for every dimos run, if at all."""
    environment = configuration_environment(None)
    return next((v for k, v in environment.items() if k.upper() == POLICY_ENV), None)


def _environment(slot: int) -> dict[str, str]:
    """A private LCM multicast port for the slot. LCM reads this variable itself."""
    env = dict(os.environ)
    env["LCM_DEFAULT_URL"] = f"udpm://{LCM_GROUP}:{LCM_PORT_BASE + slot}?ttl=0"
    return env


def _stop(process: subprocess.Popen[bytes]) -> None:
    if process.poll() is not None:
        return
    group = os.getpgid(process.pid)
    os.killpg(group, signal.SIGTERM)
    try:
        process.wait(timeout=STOP_GRACE_S)
    except subprocess.TimeoutExpired:
        os.killpg(group, signal.SIGKILL)
        process.wait()


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
    if not recording.exists():
        result.error = "no recording"
        return result
    try:
        scored = score(
            Recording.from_store(recording), manifest.rules, case.goal, case.route_length
        )
    except ValueError as error:
        result.error = str(error)
        return result
    result.outcome = scored.outcome
    result.metrics = asdict(scored)
    (episode.dir / SCORE_FILE).write_text(json.dumps(result.metrics, indent=2) + "\n")
    return result
