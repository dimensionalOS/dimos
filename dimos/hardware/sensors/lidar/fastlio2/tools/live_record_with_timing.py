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

"""Record a short live FAST-LIO run with per-section timing instrumentation.

Counterpart to replay_segment_1330_1390.py — same orchestration shape
(orchestrator + worker, auto-increment attempt_NNN/, stdout/stderr capture,
meta.json with commit hash) but with the binary in LIVE mode (no
``replay_pcap``) and ``debug=True`` so the cpp-side ``timing::Section``
counters in main.cpp emit one summary line per section per wall second.

The goal is to know, on this box with a real Mid-360 attached, how often
the main loop fires and how long each part of run_main_iter takes — so
the un-instrumented replay's ~22x slowdown has a baseline to compare against.

Also records the wire pcap (``record_pcap=True``) so the same window can
later be replayed bit-for-bit through the same binary if desired.

Run from the dimos venv with a Mid-360 plugged in:

    cd ~/repos/dimos
    source .venv/bin/activate
    python -m dimos.hardware.sensors.lidar.fastlio2.tools.live_record_with_timing
"""

from __future__ import annotations

from collections.abc import AsyncIterator
import json
import os
from pathlib import Path
import subprocess
import sys
import time

from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In
from dimos.msgs.nav_msgs.Odometry import Odometry

# ---------------- Configuration (hardcoded; bump and recommit to change) -----

# Lidar IP. Override at the source-of-truth (recommit) when the bench
# wiring changes.
LIDAR_IP = "192.168.1.155"

RUNS_ROOT = Path("/media/dimos/USB/fastlio_recordings/live_timing_records")

# How long the live recording runs once the binary has started publishing.
# 30 s is long enough to get a steady-state timing read past initialisation.
RECORD_SEC = 30.0

# Plumbing-only env var the worker reads. Behavior knobs are all constants
# above; this just carries the auto-incremented dir from parent → child.
_ATTEMPT_DIR_ENV = "_LIVE_RECORD_ATTEMPT_DIR"

# Hard ceiling on a single run's wall-clock (startup + RECORD_SEC + shutdown).
MAX_WALL_SEC = RECORD_SEC + 60.0


# ---------------- attempt-dir auto-increment --------------------------------


def _next_attempt_dir() -> Path:
    RUNS_ROOT.mkdir(parents=True, exist_ok=True)
    existing = sorted(p.name for p in RUNS_ROOT.iterdir() if p.name.startswith("attempt_"))
    n = 0
    for name in existing:
        try:
            n = max(n, int(name.split("_", 1)[1]) + 1)
        except ValueError:
            continue
    attempt = RUNS_ROOT / f"attempt_{n:03d}"
    attempt.mkdir()
    return attempt


def _commit_hash() -> str:
    try:
        return subprocess.check_output(
            ["git", "-C", str(Path(__file__).resolve().parents[6]), "rev-parse", "HEAD"],
            text=True,
        ).strip()
    except subprocess.CalledProcessError:
        return "unknown"


# ---------------- Rec module (module-level so multiprocessing can pickle) --


class RecConfig(ModuleConfig):
    """Configures Rec with the per-attempt sqlite db path."""

    db_path: str = ""


_EPS = 1e-9


class Rec(Module):
    """Mirror live FastLio2 odometry into a SqliteStore at config.db_path."""

    config: RecConfig
    fastlio_odometry: In[Odometry]
    _last_o: float = 0.0

    async def main(self) -> AsyncIterator[None]:
        # Local import — SqliteStore is only needed in the worker process.
        from dimos.memory2.store.sqlite import SqliteStore

        self._store = SqliteStore(path=self.config.db_path)
        self._os = self._store.stream("fastlio_odometry", Odometry)
        yield
        self._store.stop()

    async def handle_fastlio_odometry(self, v: Odometry) -> None:
        ts = max(getattr(v, "ts", None) or time.time(), self._last_o + _EPS)
        self._last_o = ts
        pose = getattr(v, "pose", None)
        pose_inner = getattr(pose, "pose", None) if pose is not None else None
        self._os.append(v, ts=ts, pose=pose_inner)


# ---------------- orchestrator (parent) -------------------------------------


def _orchestrate() -> int:
    attempt_dir = _next_attempt_dir()
    stdout_path = attempt_dir / "stdout.txt"
    stderr_path = attempt_dir / "stderr.txt"
    meta = {
        "attempt_dir": str(attempt_dir),
        "lidar_ip": LIDAR_IP,
        "record_sec": RECORD_SEC,
        "commit": _commit_hash(),
        "started_at": time.time(),
    }
    print(f"[live_record] attempt {attempt_dir.name}  commit {meta['commit'][:8]}", flush=True)
    t0 = time.time()
    env = {**os.environ, _ATTEMPT_DIR_ENV: str(attempt_dir)}
    with stdout_path.open("w") as out, stderr_path.open("w") as err:
        rc = subprocess.run(
            [sys.executable, "-m", __spec__.name, "--worker"],
            stdout=out,
            stderr=err,
            env=env,
            check=False,
        ).returncode
    meta["finished_at"] = time.time()
    meta["wall_sec"] = meta["finished_at"] - t0
    meta["worker_rc"] = rc
    (attempt_dir / "meta.json").write_text(json.dumps(meta, indent=2))
    print(
        f"[live_record] done attempt {attempt_dir.name} rc={rc} wall={meta['wall_sec']:.1f}s",
        flush=True,
    )
    return rc


# ---------------- worker (child) --------------------------------------------


def _worker() -> int:
    """Run the live FastLio2 + Rec stack for RECORD_SEC, then exit."""
    attempt_dir = Path(os.environ[_ATTEMPT_DIR_ENV])
    db_path = attempt_dir / "fastlio.db"
    if db_path.exists():
        db_path.unlink()
    db_path_str = str(db_path)

    pcap_path = attempt_dir / "mid360.pcap"

    from dimos.core.coordination.blueprints import autoconnect
    from dimos.core.coordination.module_coordinator import ModuleCoordinator
    from dimos.hardware.sensors.lidar.fastlio2.module import FastLio2

    # Kill orphan dimos / fastlio2_native / tcpdump processes so the new
    # run starts clean. `kd` knows the right kill patterns.
    subprocess.run(["kd"], check=False)
    time.sleep(1.0)

    bp = autoconnect(
        FastLio2.blueprint(
            frame_id="world",
            map_freq=-1,
            lidar_ip=LIDAR_IP,
            debug=True,
            record_pcap=True,
            record_pcap_path=pcap_path,
            deterministic_clock=True,
        ).remappings(
            [
                (FastLio2, "odometry", "fastlio_odometry"),
            ]
        ),
        Rec.blueprint(db_path=db_path_str),
    ).global_config(n_workers=4, robot_model="mid360_fastlio2_live_timing")
    coord = ModuleCoordinator.build(bp)

    t0 = time.time()
    try:
        # Just sleep for RECORD_SEC + slack and let the binary publish.
        # Watch the wall-clock ceiling so a wedged binary can't hang us.
        deadline = t0 + RECORD_SEC
        while time.time() < deadline and time.time() - t0 < MAX_WALL_SEC:
            time.sleep(0.5)
    finally:
        coord.stop()

    if db_path.exists():
        size_mb = db_path.stat().st_size / 1e6
        print(
            f"[live_record.worker] db_size={size_mb:.2f}MB wall={time.time() - t0:.1f}s",
            flush=True,
        )
    return 0


# ---------------- entry -----------------------------------------------------


def main(argv: list[str]) -> int:
    if len(argv) >= 2 and argv[1] == "--worker":
        return _worker()
    return _orchestrate()


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
