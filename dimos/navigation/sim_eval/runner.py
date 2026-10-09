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

"""Runs a suite: one blueprint process group per episode, several at a time, scored as they finish.

The first episode runs alone so the native modules build once. Every process gets its
own LCM multicast port and zenoh scouting group for its slot, so parallel instances never link.
An episode ends when its driver writes terminal.json, when the process exits, or at the
wall-clock cap, and is then stopped by its process group.
"""

from __future__ import annotations

from collections.abc import Iterator
from concurrent.futures import ThreadPoolExecutor
from dataclasses import asdict, dataclass, field
import json
import os
from pathlib import Path
import queue
import re
import shutil
import signal
import socket
import subprocess
import threading
import time

from dimos.constants import STATE_DIR
from dimos.navigation.sim_eval.driver import TERMINAL_FILE
from dimos.navigation.sim_eval.scorer import Recording, score
from dimos.navigation.sim_eval.suite import Case, Manifest, Split, _git_state
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

RUNS_DIR = STATE_DIR / "sim-eval"
RESULTS_FILE = "results.jsonl"
RUN_FILE = "run.json"
SCORE_FILE = "score.json"
RECORDING_FILE = "memory.db"
SCORE_TOPICS = "ground_truth,odometry,tf,contacts,goal,planner_path,path,cmd_vel,goal_reached"
LCM_PORT_BASE = 7800
ZENOH_PORT_BASE = 7500
STOP_GRACE_S = 20.0
RECORDING_LINE = re.compile(r"Recording to (\S+memory\.db)")


@dataclass(frozen=True)
class RunConfig:
    suite: Path
    blueprint: str = "go2-sim-nav-episode"
    split: Split | None = "dev"
    repeats: int = 1
    procs: int = 4
    out_dir: Path | None = None
    overrides: tuple[str, ...] = ()
    policy: Path | None = None
    record_topics: str = SCORE_TOPICS
    episode_cap_s: float = 900.0
    command: tuple[str, ...] = ("dimos",)


@dataclass(frozen=True)
class Episode:
    case: Case
    repeat: int
    dir: Path


@dataclass
class Result:
    case_id: str
    tag: str
    split: str
    repeat: int
    episode: str
    terminal: str | None
    outcome: str | None
    error: str | None = None
    metrics: dict[str, object] = field(default_factory=dict)


def run(config: RunConfig) -> Path:
    """Run every selected case the given number of times and return the run directory."""
    manifest = Manifest.load(config.suite)
    manifest.check_drift()
    cases = [c for c in manifest.cases if config.split is None or c.split == config.split]
    if not cases:
        raise ValueError(f"no cases in split {config.split!r} of {config.suite}")
    run_dir = config.out_dir or RUNS_DIR / time.strftime(f"%Y%m%d-%H%M%S-{manifest.suite}")
    run_dir.mkdir(parents=True, exist_ok=False)
    episodes = [
        Episode(case, repeat, run_dir / "episodes" / case.id / f"r{repeat}")
        for case in cases
        for repeat in range(config.repeats)
    ]
    _write_run(run_dir, config, manifest, len(episodes), finished=False)
    results = run_dir / RESULTS_FILE
    lock = threading.Lock()
    slots: queue.Queue[int] = queue.Queue()
    for slot in range(config.procs):
        slots.put(slot)

    def one(episode: Episode) -> Result:
        slot = slots.get()
        try:
            result = _episode(config, manifest, episode, slot)
        finally:
            slots.put(slot)
        with lock, results.open("a") as out:
            out.write(json.dumps(asdict(result)) + "\n")
        logger.info(
            "Episode done",
            case=episode.case.id,
            repeat=episode.repeat,
            outcome=result.outcome,
            error=result.error,
        )
        return result

    first = one(episodes[0])
    if first.terminal is None:
        raise RuntimeError(f"the first episode produced no terminal record, see {episodes[0].dir}")
    with ThreadPoolExecutor(max_workers=config.procs) as pool:
        list(pool.map(one, episodes[1:]))
    _write_run(run_dir, config, manifest, len(episodes), finished=True)
    return run_dir


def rescore(run_dir: Path) -> Path:
    """Score every episode in the run again from its recording, by the suite the run names."""
    meta = json.loads((run_dir / RUN_FILE).read_text())
    manifest = Manifest.load(Path(meta["suite"]))
    cases = {c.id: c for c in manifest.cases}
    results = run_dir / RESULTS_FILE
    results.unlink(missing_ok=True)
    with results.open("a") as out:
        for episode_dir in sorted(_episode_dirs(run_dir)):
            case = cases[episode_dir.parent.name]
            episode = Episode(case, int(episode_dir.name[1:]), episode_dir)
            terminal = _terminal(episode_dir)
            out.write(json.dumps(asdict(_score(manifest, episode, terminal))) + "\n")
    return results


def _episode_dirs(run_dir: Path) -> Iterator[Path]:
    for case_dir in (run_dir / "episodes").iterdir():
        yield from (d for d in case_dir.iterdir() if d.is_dir())


def _write_run(
    run_dir: Path, config: RunConfig, manifest: Manifest, episodes: int, finished: bool
) -> None:
    sha, dirty = _git_state()
    record = {
        "suite": str(config.suite.resolve()),
        "suite_name": manifest.suite,
        "rules_version": manifest.rules.version,
        "blueprint": config.blueprint,
        "split": config.split,
        "repeats": config.repeats,
        "procs": config.procs,
        "overrides": list(config.overrides),
        "policy": str(config.policy) if config.policy else None,
        "record_topics": config.record_topics,
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
        deadline = time.time() + config.episode_cap_s
        while time.time() < deadline and process.poll() is None:
            if (episode.dir / TERMINAL_FILE).exists():
                break
            time.sleep(0.5)
        _stop(process)
    _collect_recording(log, episode.dir)
    return _score(manifest, episode, _terminal(episode.dir))


def _command(config: RunConfig, episode: Episode, slot: int) -> list[str]:
    """The dimos run invocation, on a private zenoh scouting group for its slot."""
    case = episode.case
    command = [
        *config.command,
        "--viewer",
        "none",
        "--zenoh-scout-addr",
        f"224.0.0.224:{ZENOH_PORT_BASE + slot}",
        "run",
        config.blueprint,
        "--record",
        "sqlite",
        "--record-topics",
        config.record_topics,
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


def _environment(slot: int) -> dict[str, str]:
    """A private LCM multicast port for the slot. LCM reads this variable itself."""
    env = dict(os.environ)
    env["LCM_DEFAULT_URL"] = f"udpm://239.255.76.67:{LCM_PORT_BASE + slot}?ttl=0"
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


def _collect_recording(log: Path, episode_dir: Path) -> None:
    """Move the run's recording next to its terminal record, found from the run's log."""
    match = RECORDING_LINE.search(log.read_text(errors="replace"))
    if match is None:
        return
    source = Path(match.group(1))
    if source.exists():
        shutil.move(str(source), episode_dir / RECORDING_FILE)
        if source.parent.exists() and not any(source.parent.iterdir()):
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
        repeat=episode.repeat,
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
