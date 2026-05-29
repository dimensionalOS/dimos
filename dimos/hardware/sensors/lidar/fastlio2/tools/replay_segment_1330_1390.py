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

"""Replay the 60-s window [rec_t=1330, 1390] of the 7:37pm pcap through fastlio2.

One invocation = one attempt. Each invocation:
  1. Slices the source pcap to the [1330, 1390] window (cached on disk).
  2. Picks the next attempt_NNN/ directory under RUNS_ROOT.
  3. Spawns itself with `--worker <attempt_dir>` and redirects the child's
     stdout/stderr to attempt_dir/stdout.txt and stderr.txt.
  4. The worker brings up a dimos Coordinator with FastLio2 (replay mode) +
     a tiny Rec module that mirrors fastlio_odometry into a sqlite db.
  5. Writes attempt_dir/meta.json with the dimos commit hash and wall times.

Config is hardcoded — no CLI args, no env vars affecting behavior. Edit the
constants in this file and recommit when you want a different experiment.
The cached slice lives outside the repo (under RUNS_ROOT on the USB drive).

Run from the dimos venv:

    cd ~/repos/dimos
    source .venv/bin/activate
    python -m dimos.hardware.sensors.lidar.fastlio2.tools.replay_segment_1330_1390
"""

from __future__ import annotations

from collections.abc import AsyncIterator
import json
from pathlib import Path
import struct
import subprocess
import sys
import time

# ---------------- Configuration (hardcoded; bump and recommit to change) -----

SOURCE_PCAP = Path(
    "/media/dimos/USB/fastlio_recordings/recording_go2_mid360_2026-05-28_7-37pm-PST.pcap"
)
REC_START_EPOCH = 1780020531.706  # epoch sec of the first pcap packet
T_LO_REC_SEC = 1330.0  # window start (sec into recording)
T_HI_REC_SEC = 1390.0  # window end   (sec into recording)

RUNS_ROOT = Path("/media/dimos/USB/fastlio_recordings/segment_replays_1330_1390")
SLICE_PCAP = RUNS_ROOT / "segment_1330_1390.pcap"

# Hard ceiling on a single run's wall-clock. The 60-s window at live rate
# costs ~60 s of wall + ~30 s of startup/teardown. 300 s is comfortable.
MAX_WALL_SEC = 300.0


# ---------------- pcap slicing (libpcap classic format, little-endian) ------


def _slice_pcap_if_needed() -> None:
    """Slice SOURCE_PCAP to SLICE_PCAP for the configured window.

    No-op if SLICE_PCAP already exists. The pcap format we read is the
    classic libpcap one tcpdump writes: 24-byte global header copied as-is,
    then a sequence of 16-byte record headers + payloads. `ts_sub` is
    microseconds for magic 0xa1b2c3d4 (the one tcpdump uses here), so the
    division by 1e6 is correct.
    """
    if SLICE_PCAP.exists():
        return
    SLICE_PCAP.parent.mkdir(parents=True, exist_ok=True)
    t_lo_epoch = REC_START_EPOCH + T_LO_REC_SEC
    t_hi_epoch = REC_START_EPOCH + T_HI_REC_SEC
    written = 0
    with SOURCE_PCAP.open("rb") as src, SLICE_PCAP.open("wb") as dst:
        global_hdr = src.read(24)
        if len(global_hdr) != 24:
            raise RuntimeError(f"short read on pcap global header: {SOURCE_PCAP}")
        dst.write(global_hdr)
        while True:
            rec_hdr = src.read(16)
            if len(rec_hdr) < 16:
                break
            ts_sec, ts_usec, incl_len, _orig_len = struct.unpack("<IIII", rec_hdr)
            payload = src.read(incl_len)
            if len(payload) != incl_len:
                break
            ts = ts_sec + ts_usec / 1e6
            if ts < t_lo_epoch:
                continue
            if ts > t_hi_epoch:
                break
            dst.write(rec_hdr)
            dst.write(payload)
            written += 1
    print(f"[replay_segment] sliced {written} pcap records → {SLICE_PCAP}", flush=True)


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


# ---------------- orchestrator (parent) -------------------------------------


def _orchestrate() -> int:
    if not SOURCE_PCAP.exists():
        print(f"[replay_segment] missing source pcap: {SOURCE_PCAP}", file=sys.stderr)
        return 2
    _slice_pcap_if_needed()
    attempt_dir = _next_attempt_dir()
    stdout_path = attempt_dir / "stdout.txt"
    stderr_path = attempt_dir / "stderr.txt"
    meta = {
        "attempt_dir": str(attempt_dir),
        "slice_pcap": str(SLICE_PCAP),
        "t_lo_rec_sec": T_LO_REC_SEC,
        "t_hi_rec_sec": T_HI_REC_SEC,
        "commit": _commit_hash(),
        "started_at": time.time(),
    }
    print(f"[replay_segment] attempt {attempt_dir.name}  commit {meta['commit'][:8]}", flush=True)
    t0 = time.time()
    with stdout_path.open("w") as out, stderr_path.open("w") as err:
        rc = subprocess.run(
            [sys.executable, "-m", __spec__.name, "--worker", str(attempt_dir)],
            stdout=out,
            stderr=err,
            check=False,
        ).returncode
    meta["finished_at"] = time.time()
    meta["wall_sec"] = meta["finished_at"] - t0
    meta["worker_rc"] = rc
    (attempt_dir / "meta.json").write_text(json.dumps(meta, indent=2))
    print(
        f"[replay_segment] done attempt {attempt_dir.name} rc={rc} wall={meta['wall_sec']:.1f}s",
        flush=True,
    )
    return rc


# ---------------- worker (child) --------------------------------------------


def _worker(attempt_dir: Path) -> int:
    """Run the replay inside the dimos Coordinator.

    Lives in the same module as the orchestrator so the script stays
    self-contained; only invoked by the orchestrator via
    `python -m <module> --worker <attempt_dir>`.
    """
    # Imports are inside the worker so the orchestrator can run without
    # bringing up the dimos stack (faster CI/sanity checks).
    from dimos.core.coordination.blueprints import autoconnect
    from dimos.core.coordination.module_coordinator import ModuleCoordinator
    from dimos.core.module import Module, ModuleConfig
    from dimos.core.stream import In
    from dimos.hardware.sensors.lidar.fastlio2.module import FastLio2
    from dimos.memory2.store.sqlite import SqliteStore
    from dimos.msgs.nav_msgs.Odometry import Odometry

    db_path = attempt_dir / "fastlio.db"
    if db_path.exists():
        db_path.unlink()
    db_path_str = str(db_path)

    # End-of-pcap detection: convert the window's upper bound from epoch
    # → sensor-boot seconds (which is what fastlio publishes when
    # deterministic_clock=True). Constant from prior session memory.
    SENSOR_BOOT_EPOCH_OFFSET = 1780018948.01
    PCAP_END_SENSOR_BOOT = REC_START_EPOCH + T_HI_REC_SEC - SENSOR_BOOT_EPOCH_OFFSET

    class RecConfig(ModuleConfig):
        db_path: str = db_path_str

    _EPS = 1e-9

    class Rec(Module):
        config: RecConfig
        fastlio_odometry: In[Odometry]
        _store: SqliteStore
        _last_o: float = 0.0

        async def main(self) -> AsyncIterator[None]:
            self._store = SqliteStore(path=self.config.db_path)
            self._os = self._store.stream("fastlio_odometry", Odometry)
            yield
            self._store.stop()

        async def handle_fastlio_odometry(self, v: Odometry) -> None:
            ts = max(getattr(v, "ts", None) or time.time(), self._last_o + _EPS)
            self._last_o = ts
            self._os.append(v, ts=ts)

    # Kill orphan dimos / fastlio2_native processes from any previous run
    # so their LCM multicast traffic can't bleed into this Rec module.
    # `kd` is a user-local script that knows the right kill patterns.
    subprocess.run(["kd"], check=False)
    time.sleep(1.0)

    bp = autoconnect(
        FastLio2.blueprint(
            frame_id="world",
            map_freq=-1,
            replay_pcap=SLICE_PCAP,
            deterministic_clock=True,
            debug=False,
        ).remappings(
            [
                (FastLio2, "odometry", "fastlio_odometry"),
            ]
        ),
        Rec.blueprint(),
    ).global_config(n_workers=4, robot_model="mid360_fastlio2_replay")
    coord = ModuleCoordinator.build(bp)

    import sqlite3

    t0 = time.time()
    last_ts_seen = 0.0
    stagnant_since: float | None = None
    saw_first_row = False
    try:
        while time.time() - t0 < MAX_WALL_SEC:
            time.sleep(1.0)
            if not db_path.exists():
                continue
            try:
                con = sqlite3.connect(f"file:{db_path_str}?mode=ro", uri=True, timeout=0.5)
                row = con.execute("SELECT MAX(ts), COUNT(*) FROM fastlio_odometry").fetchone()
                con.close()
            except Exception:
                continue
            last_ts = row[0] if row and row[0] else 0.0
            cnt = row[1] if row else 0
            if cnt > 0:
                saw_first_row = True
            if last_ts >= PCAP_END_SENSOR_BOOT - 2:
                break  # processed past pcap end
            if not saw_first_row:
                continue
            # Stagnation by ts progress, not file size: WAL can grow
            # without new rows landing in the table.
            if last_ts == last_ts_seen:
                if stagnant_since is None:
                    stagnant_since = time.time()
                elif time.time() - stagnant_since > 3.0:
                    break  # no new rows for 3 s = binary done
            else:
                last_ts_seen = last_ts
                stagnant_since = None
    finally:
        coord.stop()

    if db_path.exists():
        size_mb = db_path.stat().st_size / 1e6
        print(
            f"[replay_segment.worker] db_size={size_mb:.2f}MB wall={time.time() - t0:.1f}s",
            flush=True,
        )
    return 0


# ---------------- entry -----------------------------------------------------


def main(argv: list[str]) -> int:
    if len(argv) >= 3 and argv[1] == "--worker":
        return _worker(Path(argv[2]))
    return _orchestrate()


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
