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
"""evo benchmark for the TypeSafe world state: habitat_nav cases, one live run each.

    python misc/evals/ts_state_bench.py --cases <id>[,<id>] [--repeats N] [--min-reached K]

Runs from the worktree it lives in (PYTHONPATH and cwd set to it) with the dimos CLI of the
interpreter's venv. One evo task per (case, repeat). A run without nav_metrics.json is a
harness error and exits 2; it is never scored 0.
"""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import fcntl
from itertools import pairwise
import json
import math
import os
from pathlib import Path
import re
import shutil
import signal
import subprocess
import sys
import tempfile
import time
from typing import Any

ROOT = Path(__file__).resolve().parents[2]  # --root overrides: a pinned copy scores any worktree
STATE = Path(os.environ.get("TS_BENCH_STATE") or Path(tempfile.gettempdir()) / "ts-state-bench")
LOCK = Path(os.environ.get("TS_BENCH_LOCK") or Path(tempfile.gettempdir()) / "dimos-sim.lock")
AGENT_ARGS = [
    "--agent", "dimos.evals.agents.topic",
    "--set", 'modules=["type-safe-agent"]',
    "--set", "trace=TypeSafeAgent",
]  # fmt: skip
UNREACHED_CEILING = 0.4
ROUTE_CORRIDOR_M = 1.5
REFS = Path(__file__).with_name("ts_state_reference_paths.json")

# -- evo inline instrumentation (contract: skills/discover/references/inline_instrumentation.py)
_TRACES = Path(os.environ["EVO_TRACES_DIR"]) if os.environ.get("EVO_TRACES_DIR") else None
_RESULT = os.environ.get("EVO_RESULT_PATH")
_SCORES: dict[str, float] = {}
_STARTED = datetime.now(timezone.utc).isoformat(timespec="seconds")


def log_task(task_id: str, score: float, **extra: Any) -> None:
    _SCORES[task_id] = score
    if _TRACES is None:
        return
    _TRACES.mkdir(parents=True, exist_ok=True)
    trace = {
        "experiment_id": os.environ.get("EVO_EXPERIMENT_ID", "unknown"),
        "task_id": task_id,
        "status": "passed" if score >= 0.5 else "failed",
        "score": score,
        "ended_at": datetime.now(timezone.utc).isoformat(timespec="seconds"),
        **extra,
    }
    (_TRACES / f"task_{task_id}.json").write_text(json.dumps(trace, indent=2))


def write_result() -> float:
    score = round(sum(_SCORES.values()) / len(_SCORES), 4)
    payload = json.dumps(
        {
            "score": score,
            "tasks": _SCORES,
            "started_at": _STARTED,
            "ended_at": datetime.now(timezone.utc).isoformat(timespec="seconds"),
        },
        indent=2,
    )
    if not _RESULT:
        print(payload)
        return score
    target = Path(_RESULT)
    target.parent.mkdir(parents=True, exist_ok=True)
    os.close(os.open(target, os.O_CREAT | os.O_EXCL | os.O_WRONLY))
    tmp = target.with_name(target.name + ".tmp")
    tmp.write_text(payload)
    os.replace(tmp, target)
    return score


# -- scoring


def route_progress(poses: list[tuple[float, float]], ref: list[list[float]]) -> float:
    """Furthest fraction of the reference route (the planner arm's driven path for this case)
    the robot got to: for each pose within ROUTE_CORRIDOR_M of the route, the arc length at the
    nearest route point. Straight-line distance would reward jamming into the wall nearest the
    object and punish a correct detour."""
    arc = [0.0]
    for a, b in pairwise(ref):
        arc.append(arc[-1] + math.dist(a, b))
    best = 0.0
    for x, y in poses:
        d, i = min((math.hypot(x - px, y - py), i) for i, (px, py) in enumerate(ref))
        if d <= ROUTE_CORRIDOR_M:
            best = max(best, arc[i])
    return best / arc[-1] if arc[-1] else 0.0


def wall_between(case: str, pose: tuple[float, float], box: list[float]) -> bool:
    """A ground-truth wall crosses the segment from the final pose to the nearest point of the
    target's box. The suite's `reached` is box distance only, so a robot 0.9 m from a fridge on
    the far side of a wall counts as arrived; this harness does not count that."""
    truth = next(
        (
            g
            for g in (ROOT / "misc/habitat/ground_truth/hssd").glob("*.json")
            if "top_down" not in g.name and case.startswith(g.name.removesuffix(".json"))
        ),
        None,
    )
    if truth is None:
        return False
    x0, y0 = pose
    x1, y1 = min(max(x0, box[0]), box[2]), min(max(y0, box[1]), box[3])
    for d in json.loads(truth.read_text())["detections"]:
        if not str(d["id"]).startswith("wall"):
            continue
        (cx, cy, _), (sx, sy, _) = d["center_xyz"], d["size_xyz"]
        lo, hi = 0.0, 1.0  # slab clip of the segment against the wall's footprint
        for p0, dp, a, b in (
            (x0, x1 - x0, cx - sx / 2, cx + sx / 2),
            (y0, y1 - y0, cy - sy / 2, cy + sy / 2),
        ):
            if abs(dp) < 1e-9:
                if not a <= p0 <= b:
                    lo, hi = 1.0, 0.0
                continue
            t0, t1 = sorted(((a - p0) / dp, (b - p0) / dp))
            lo, hi = max(lo, t0), min(hi, t1)
        if lo <= hi:
            return True
    return False


def objective(m: dict[str, Any], grade: float, progress: float) -> float:
    """Completion first: the suite's grade when reached, else route progress under 0.5."""
    if m["reached"]:
        return grade
    return round(UNREACHED_CEILING * min(1.0, max(0.0, progress)), 4)


def tick_summary(raw: Path) -> dict[str, Any]:
    """What the model saw and picked, for the reader of the trace."""
    n = len(list(raw.glob("*-request.json")))
    picks: dict[str, int] = {}
    flips, last_x, tokens, latency, samples = 0, "none", 0, 0.0, []
    failed, models = 0, set()
    for i in range(1, n + 1):
        try:
            req = json.loads((raw / f"{i}-request.json").read_text())["body"]
            resp = json.loads((raw / f"{i}-response.json").read_text())
        except (OSError, ValueError, KeyError):
            failed += 1
            continue
        ans = resp["body"].get("answers")
        if not ans:
            failed += 1
            continue
        models.add(resp["body"].get("model"))
        pick = "/".join(ans.get(f"drive.{a}", {}).get("choice", "?") for a in ("x", "y", "yaw"))
        picks[pick] = picks.get(pick, 0) + 1
        x = ans.get("drive.x", {}).get("choice", "none")
        if x != "none":
            flips += last_x != "none" and x != last_x
            last_x = x
        tokens += (resp["body"].get("usage") or {}).get("input_tokens", 0)
        latency += resp.get("latency_s", 0.0)
        if i in (1, n // 2, n):
            samples.append({"tick": i, "state": req["state"], "answers": ans})
    top = dict(sorted(picks.items(), key=lambda kv: -kv[1])[:6])
    # No trace at all means every model call raised (no credits, no key, API down).
    if n - failed < 1 or failed > 0.2 * n:  # a premature "finished" is one valid tick
        raise SystemExit(f"harness error: {n - failed} valid model ticks of {n} in {raw}")
    return {
        "ticks": n,
        "failed_ticks": failed,
        "models": sorted(str(m) for m in models),
        "picks": top,
        "drive_x_flips": flips,
        "input_tokens_per_tick": round(tokens / n) if n else 0,
        "latency_s_mean": round(latency / n, 3) if n else 0.0,
        "samples": samples,
    }


# -- one live run


def _poses(recording: Path) -> list[tuple[float, float]]:
    from dimos.evals.nav_metrics import read_poses  # the worktree's dimos, via PYTHONPATH
    from dimos.memory.store.sqlite import SqliteStore

    with SqliteStore(path=str(recording / "memory.db")) as store:
        return [(x, y) for _, x, y, _ in read_poses(store)]


def _wait_for_free_host() -> None:
    # ponytail: one sim per host (shared zenoh). Wait for a foreign eval, then clear leftovers.
    deadline = time.monotonic() + 1200
    while subprocess.run(["pgrep", "-f", "dimos evals ru[n]"], capture_output=True).returncode == 0:
        if time.monotonic() > deadline:
            raise SystemExit("harness error: another eval held the host for 20 min")
        time.sleep(5)
    pattern = f"{ROOT}/(.venv|target)/[a-z]"  # leftovers under this root only
    found = subprocess.run(["pgrep", "-f", pattern], capture_output=True, text=True).stdout.split()
    mine = {os.getpid(), os.getppid()}
    for pid in map(int, found):
        if pid not in mine:
            subprocess.run(["kill", "-KILL", str(pid)], check=False)
    time.sleep(1)


def run_case(case: str, timeout_s: int) -> dict[str, Any]:
    # One simulator per host: every harness copy takes this lock for the length of a run.
    # (Waiting on `pgrep` alone raced: two harnesses launched in the same second and both died.)
    with LOCK.open("w") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        return _run_case_locked(case, timeout_s)


def _run_case_locked(case: str, timeout_s: int) -> dict[str, Any]:
    """Infrastructure deaths (the runner killed or crashed by a signal before it wrote
    anything, seen as rc -9 and -11 on this shared box) are retried; a graded run never is."""
    for attempt in range(3):
        try:
            return _run_case_once(case, timeout_s)
        except _SignalledError as e:
            print(f"{case}: attempt {attempt + 1} died: {e}", flush=True)
    raise SystemExit(f"harness error: {case} died by signal three times")


class _SignalledError(Exception):
    pass


def _run_case_once(case: str, timeout_s: int) -> dict[str, Any]:
    _wait_for_free_host()
    env = {
        **os.environ,
        "PYTHONPATH": str(ROOT),
        "PATH": f"{Path(sys.executable).parent}:{os.environ['PATH']}",
        "XDG_STATE_HOME": str(STATE),
        "DIMOS_EVAL_TIMEOUT_S": str(timeout_s),
        "DIMOS_ZENOH_SHM": "0",
        "CI": "1",
        "PYTEST_VERSION": "1",
    }
    cmd = ["dimos", "evals", "run", "dimos.evals.suites.habitat_nav", "--case", case, *AGENT_ARGS]
    # Own session, so whatever the launch leaves behind (natives, workers) dies with the group.
    proc = subprocess.Popen(
        cmd, cwd=ROOT, env=env, text=True, start_new_session=True,
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
    )  # fmt: skip
    try:
        stdout, _ = proc.communicate(timeout=timeout_s + 1500)
    finally:
        try:
            os.killpg(proc.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
    out = stdout or ""
    STATE.mkdir(parents=True, exist_ok=True)
    (STATE / "last-run.log").write_text(out)  # overwritten per run; the why of a harness error
    found = re.findall(r"cases \|.*?(/\S*run-\S+)", out)
    run = Path(found[-1]) if found else None
    if proc.returncode < 0 and run is None:
        raise _SignalledError(f"rc={proc.returncode}")
    if proc.returncode != 0 or run is None or not (run / case / "nav_metrics.json").exists():
        sys.stderr.write(out[-4000:])
        raise SystemExit(f"harness error: {case} rc={proc.returncode} run={run}")
    m = json.loads((run / case / "nav_metrics.json").read_text())
    row = next(
        r
        for line in (run / "results.jsonl").read_text().splitlines()
        if (r := json.loads(line)).get("case_id") == case
    )
    if row.get("error"):
        raise SystemExit(f"harness error: {case}: {row['error']}")
    try:
        summary = tick_summary(run / case / "raw")
        poses = _poses(Path(m["recording"]))
        if m["reached"] and wall_between(case, poses[-1], m["box"]):
            m["reached"], summary["reached_through_wall"] = False, True
        summary["route_progress"] = round(
            route_progress(poses, json.loads(REFS.read_text())[case]["points"]), 3
        )
    finally:
        # ponytail: traces are 10 MB per timed-out run and the disk is small; the summary keeps
        # three sampled ticks. Keep raw/ if tick-level replay is ever needed.
        recording = Path(m.get("recording") or "/nonexistent").resolve()
        for d in (run / case / "raw", recording):
            if d.is_relative_to(STATE) or d.is_relative_to(ROOT / "recordings"):  # this run's
                shutil.rmtree(d, ignore_errors=True)
    return {"metrics": m, "grade": row["score"], "ended_by": row.get("ended_by"), **summary}


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--cases", required=True)
    ap.add_argument("--repeats", type=int, default=1)
    ap.add_argument("--timeout-s", type=int, default=120)
    ap.add_argument("--min-reached", type=int, default=None, help="gate: exit 1 below this")
    ap.add_argument("--root", default=None, help="worktree to score (default: this file's)")
    args = ap.parse_args()
    global ROOT
    if args.root:
        ROOT = Path(args.root).resolve()
    reached = 0
    keys = (
        "reached final_distance_m min_distance_m time_to_object_s duration_s facing "
        "bumps path_length_m straight_line_m straightness total_turning_rad "
        "turn_reversals_per_m finished_declared declared_at_s"
    ).split()
    for case in args.cases.split(","):
        # One task per case: repeats are noise replicates, not separate Pareto axes.
        repeats = []
        for k in range(args.repeats):
            r = run_case(case, args.timeout_s)
            m = r["metrics"]
            reached += bool(m["reached"])
            j = objective(m, r["grade"], r["route_progress"])
            repeats.append(
                {
                    "J": j,
                    "grade": r["grade"],
                    **{k_: m.get(k_) for k_ in keys},
                    **{k_: v for k_, v in r.items() if k_ not in ("metrics", "grade")},
                }
            )
            print(
                f"{case} r{k}: J={j:.3f} reached={m['reached']} route={r['route_progress']:.0%}",
                flush=True,
            )
        log_task(
            case,
            round(sum(x["J"] for x in repeats) / len(repeats), 4),
            summary="; ".join(
                f"r{i}: reached={x['reached']} route={x['route_progress']:.0%} "
                f"final={x['final_distance_m']:.2f}m "
                f"t={x['time_to_object_s']:.0f}s path={x['path_length_m']:.1f}m "
                f"bumps={x['bumps']} flips={x['drive_x_flips']} ended_by={x['ended_by']}"
                for i, x in enumerate(repeats)
            ),
            repeats=repeats,
        )
    score = write_result()
    print(f"score {score:.4f} reached {reached}/{len(_SCORES) * args.repeats}")
    if args.min_reached is not None and reached < args.min_reached:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
