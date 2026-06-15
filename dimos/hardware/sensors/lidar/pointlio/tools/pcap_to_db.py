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

"""Run Point-LIO over a .pcap (via the rust virtual_mid360 replay) → .db.

Point-LIO has no in-process replay anymore; the only replay path is the
``virtual_mid360`` rust module, which replays a recorded Mid-360 pcap *over the
wire* so an unmodified live Point-LIO connects to it as real hardware. This tool
orchestrates that end to end and records Point-LIO's outputs into a memory2
SQLite database:

* ``pointlio_odometry`` — the IESKF pose at the native odom rate.
* ``pointlio_lidar``    — the sensor-frame point cloud at the native rate.

By default both ends run in the host namespace with ``host_ip``/``lidar_ip``
aliased on a dummy interface (``--alias-iface``); pass ``--lidar-ns``/``--drv-ns``
to isolate them in separate netns + veth instead (needed to run two replays at
once). Either way it configures interfaces/routes, so **it needs CAP_NET_ADMIN**
and shells out to ``sudo`` for the setup. Live Point-LIO + the recorder run
together (one coordinator, so their LCM streams wire up normally). Replay is real
time (Point-LIO is not deterministic), so two runs over the same pcap differ.

The ``--db`` is optional: with no existing db a fresh one is built from scratch
(defaults to ``<pcap>.db`` next to the pcap). With an existing db the two streams
are appended and time-aligned onto its clock, so Point-LIO output can be compared
against whatever it already holds (e.g. a fastlio replay). If either stream
already exists the run aborts unless ``--force`` drops them first.

Run it as your normal user from the dimos6 venv — it shells out to ``sudo``
for the privileged netns/veth bits itself::

    source .venv/bin/activate
    PCAP=$(python -c "from dimos.utils.data import get_data; \
        print(get_data('mid360_shake_stairs/mid360_shake_stairs.pcap'))")
    python -m dimos.hardware.sensors.lidar.pointlio.tools.pcap_to_db --pcap "$PCAP"
    # -> writes mid360_shake_stairs.db next to the sample.

Two simultaneous runs (e.g. alongside a fastlio replay) must use distinct
namespaces/IPs — see --drv-ns / --lidar-ns / --host-ip / --lidar-ip.
"""

from __future__ import annotations

import argparse
from collections.abc import AsyncIterator
import json
import math
import os
from pathlib import Path
import signal
import sqlite3
import subprocess
import sys
import time

from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2

# Below this the db's existing timestamps are sensor-boot seconds, not unix time.
_SENSOR_CLOCK_MAX = 1e8
# Strictly-increasing tie-breaker so two samples never collide on ts.
_EPS = 1e-9
# Poll the db on this cadence while the replay drains the pcap.
_POLL_SEC = 1.0
# Stop after the odom stream has been stagnant this long (pcap fully drained).
_STAGNANT_SEC = 5.0
# No odometry row within this long after start = binary failed to come up
# (missing artifact, bad pcap, SLAM-init crash); bounds the poll loop. Generous
# to cover Point-LIO's IMU-init latency.
_STARTUP_TIMEOUT_SEC = 60.0
# virtual_mid360 crate dir (its `nix build .#default` produces result/bin/virtual_mid360).
# .../sensors/lidar/pointlio/tools/pcap_to_db.py -> parents[2] == .../sensors/lidar
_VM_DIR = Path(__file__).resolve().parents[2] / "livox" / "virtual_mid360"


class _RecConfig(ModuleConfig):
    """Configures the recorder with the target db and timing conversion."""

    db_path: str = ""
    # Earliest existing ts in the db, or -1.0 if the db has no timestamped rows.
    ref_start_ts: float = -1.0
    # Explicit offset override; NaN means auto-derive from ref_start_ts.
    time_offset: float = float("nan")


class _Rec(Module):
    """Append Point-LIO odometry + lidar into a SQLite db with ts conversion.

    Underscore-prefixed so the blueprint registry generator skips it — this is
    an internal helper for the tool, not a public robot module.
    """

    config: _RecConfig
    pointlio_odometry: In[Odometry]
    pointlio_lidar: In[PointCloud2]
    _offset: float | None = None
    _last_odom_ts: float = 0.0
    _last_lidar_ts: float = 0.0
    _odom_count: int = 0
    _lidar_count: int = 0

    async def main(self) -> AsyncIterator[None]:
        from dimos.memory2.store.sqlite import SqliteStore

        self._store = SqliteStore(path=self.config.db_path)
        self._os = self._store.stream("pointlio_odometry", Odometry)
        self._ls = self._store.stream("pointlio_lidar", PointCloud2)
        yield
        self._store.stop()

    def _resolve_offset(self, first_ts: float) -> float:
        override = self.config.time_offset
        if not math.isnan(override):
            return override
        ref = self.config.ref_start_ts
        if ref < 0.0 or ref < _SENSOR_CLOCK_MAX:
            # Empty db, or db already on the sensor clock -> exact alignment.
            return 0.0
        # db on wall-clock time -> start-align Point-LIO onto the db's earliest ts.
        return ref - first_ts

    def _aligned_ts(self, raw_ts: float, last_ts: float) -> float:
        """Convert a sensor ts onto the db clock, kept strictly above last_ts."""
        if self._offset is None:
            self._offset = self._resolve_offset(raw_ts)
        return max(raw_ts + self._offset, last_ts + _EPS)

    async def handle_pointlio_odometry(self, msg: Odometry) -> None:
        # `is not None`, not `or`: a real sensor ts of 0.0 must not fall back to
        # wall time (would misclassify the stream's clock in _resolve_offset).
        raw_ts_raw = getattr(msg, "ts", None)
        raw_ts = raw_ts_raw if raw_ts_raw is not None else time.time()
        ts = self._aligned_ts(raw_ts, self._last_odom_ts)
        self._last_odom_ts = ts
        pose = getattr(msg, "pose", None)
        pose_inner = getattr(pose, "pose", None) if pose is not None else None
        self._os.append(msg, ts=ts, pose=pose_inner)
        self._odom_count += 1

    async def handle_pointlio_lidar(self, msg: PointCloud2) -> None:
        raw_ts_raw = getattr(msg, "ts", None)
        raw_ts = raw_ts_raw if raw_ts_raw is not None else time.time()
        ts = self._aligned_ts(raw_ts, self._last_lidar_ts)
        self._last_lidar_ts = ts
        self._ls.append(msg, ts=ts)
        self._lidar_count += 1


def _db_ref_start_ts(db_path: Path) -> float:
    """Min ts across the db's existing streams, or -1.0 if none/absent."""
    if not db_path.exists():
        return -1.0
    con = sqlite3.connect(f"file:{db_path}?mode=ro", uri=True, timeout=2.0)
    try:
        tables = [
            row[0]
            for row in con.execute("SELECT name FROM sqlite_master WHERE type='table'").fetchall()
        ]
        best: float | None = None
        for table in tables:
            if table.startswith("_") or table.startswith("sqlite_"):
                continue
            try:
                cols = [col[1] for col in con.execute(f"PRAGMA table_info('{table}')").fetchall()]
                if "ts" not in cols:
                    continue
                row = con.execute(f"SELECT MIN(ts) FROM '{table}'").fetchone()
            except sqlite3.OperationalError:
                continue
            if row and row[0] is not None:
                best = row[0] if best is None else min(best, row[0])
        return best if best is not None else -1.0
    finally:
        con.close()


def _table_stats(db_path: Path, table: str) -> tuple[int, float, float]:
    """(count, min_ts, max_ts) for a stream table; zeros if absent."""
    if not db_path.exists():
        return 0, 0.0, 0.0
    con = sqlite3.connect(f"file:{db_path}?mode=ro", uri=True, timeout=2.0)
    try:
        try:
            row = con.execute(f"SELECT COUNT(*), MIN(ts), MAX(ts) FROM '{table}'").fetchone()
        except sqlite3.OperationalError:
            return 0, 0.0, 0.0
        cnt = row[0] or 0
        return cnt, row[1] or 0.0, row[2] or 0.0
    finally:
        con.close()


# Network-namespace orchestration (outer process; needs CAP_NET_ADMIN via sudo).


def _sudo() -> list[str]:
    return ["sudo"]


def _ns(args: list[str], check: bool = True) -> subprocess.CompletedProcess[bytes]:
    return subprocess.run(_sudo() + args, check=check, capture_output=True)


def _teardown(drv: str, lidar: str, veth: str) -> None:
    for cmd in (
        ["ip", "netns", "del", drv],
        ["ip", "netns", "del", lidar],
        ["ip", "link", "del", veth],
    ):
        _ns(cmd, check=False)


def _setup_netns(
    drv: str, lidar: str, veth_drv: str, veth_lidar: str, host_ip: str, lidar_ip: str
) -> None:
    """Recreate the drv/lidar veth pair with the Livox multicast routing."""
    _teardown(drv, lidar, veth_drv)
    steps = [
        ["ip", "netns", "add", drv],
        ["ip", "netns", "add", lidar],
        ["ip", "link", "add", veth_drv, "type", "veth", "peer", "name", veth_lidar],
        ["ip", "link", "set", veth_drv, "netns", drv],
        ["ip", "link", "set", veth_lidar, "netns", lidar],
        ["ip", "netns", "exec", drv, "ip", "addr", "add", f"{host_ip}/24", "dev", veth_drv],
        ["ip", "netns", "exec", lidar, "ip", "addr", "add", f"{lidar_ip}/24", "dev", veth_lidar],
    ]
    for ns in (drv, lidar):
        steps += [
            ["ip", "netns", "exec", ns, "ip", "link", "set", "lo", "up"],
            ["ip", "netns", "exec", ns, "ip", "link", "set", "lo", "multicast", "on"],
            ["ip", "netns", "exec", ns, "ip", "route", "add", "224.0.0.0/4", "dev", "lo"],
        ]
    steps += [
        ["ip", "netns", "exec", drv, "ip", "link", "set", veth_drv, "up"],
        ["ip", "netns", "exec", lidar, "ip", "link", "set", veth_lidar, "up"],
        ["ip", "netns", "exec", drv, "ip", "link", "set", veth_drv, "multicast", "on"],
        ["ip", "netns", "exec", lidar, "ip", "link", "set", veth_lidar, "multicast", "on"],
        # Mid-360 multicasts point/IMU to 224.1.1.5; broadcast detection to 255.255.255.255.
        [
            "ip",
            "netns",
            "exec",
            lidar,
            "ip",
            "route",
            "add",
            "255.255.255.255/32",
            "dev",
            veth_lidar,
        ],
        ["ip", "netns", "exec", lidar, "ip", "route", "add", "224.1.1.5/32", "dev", veth_lidar],
    ]
    for cmd in steps:
        _ns(cmd)


def _teardown_aliases(iface: str) -> None:
    _ns(["ip", "link", "del", iface], check=False)


def _setup_aliases(iface: str, host_ip: str, lidar_ip: str) -> None:
    """No-netns setup: put host_ip + lidar_ip on a dummy interface in the same
    /24 with the Livox multicast + discovery-broadcast routes. Both processes
    then run in the host namespace; multicast loopback (on by default) delivers
    the fake lidar's point/IMU to the local SDK. (Still iproute2/Linux — macOS
    would need the ifconfig-alias equivalent.)"""
    _teardown_aliases(iface)
    steps = [
        ["ip", "link", "add", iface, "type", "dummy"],
        ["ip", "addr", "add", f"{host_ip}/24", "dev", iface],
        ["ip", "addr", "add", f"{lidar_ip}/24", "dev", iface],
        ["ip", "link", "set", iface, "up"],
        ["ip", "link", "set", iface, "multicast", "on"],
        ["ip", "route", "add", "224.1.1.5/32", "dev", iface],
        ["ip", "route", "add", "255.255.255.255/32", "dev", iface],
    ]
    for cmd in steps:
        _ns(cmd)


def _resolve_vm_binary() -> str:
    """Path to the virtual_mid360 binary; build it via nix if not present."""
    env = os.environ.get("DIMOS_MID360_BIN")
    if env:
        return env
    out = _VM_DIR / "result" / "bin" / "virtual_mid360"
    if out.exists():
        return str(out)
    print("[pcap_to_db] building virtual_mid360 (nix build .#default)...", flush=True)
    subprocess.run(["nix", "build", ".#default"], cwd=_VM_DIR, check=True)
    return str(out)


def _run_outer(args: argparse.Namespace) -> int:
    pcap_path = Path(args.pcap).expanduser().resolve()
    if not pcap_path.exists():
        print(f"[pcap_to_db] missing pcap: {pcap_path}", file=sys.stderr)
        return 2
    db_path = Path(args.db).expanduser().resolve() if args.db else pcap_path.with_suffix(".db")
    db_existed = db_path.exists()
    db_path.parent.mkdir(parents=True, exist_ok=True)

    # Fail fast on stream conflicts before touching the network. Only open an
    # *existing* db here — a new db is created by the (root) inner so it owns it
    # outright; SQLite refuses WAL writes when the file owner != the process uid.
    pointlio_streams = ("pointlio_odometry", "pointlio_lidar")
    if db_existed:
        from dimos.memory2.store.sqlite import SqliteStore

        store = SqliteStore(path=str(db_path))
        try:
            existing = sorted(set(store.list_streams()) & set(pointlio_streams))
            if existing and not args.force:
                print(
                    f"[pcap_to_db] {db_path.name} already has {existing}; pass --force to overwrite",
                    file=sys.stderr,
                )
                return 2
            for name in existing:
                store.delete_stream(name)
            if existing:
                print(f"[pcap_to_db] --force: dropped existing {existing}", flush=True)
        finally:
            store.stop()

    ref_start_ts = _db_ref_start_ts(db_path)
    vm_bin = _resolve_vm_binary()
    net = (
        f"netns {args.drv_ns}/{args.lidar_ns}"
        if args.lidar_ns
        else f"aliases on {args.alias_iface}"
    )
    print(
        f"[pcap_to_db] pcap={pcap_path.name} db={db_path.name} "
        f"({'append' if db_existed else 'new'}) rate={args.rate} "
        f"{net} ips={args.host_ip}/{args.lidar_ip}",
        flush=True,
    )

    # netns is opt-in: with --lidar-ns the two ends get isolated namespaces (root
    # inner); without it we alias both IPs on a dummy interface and run everything
    # in the host ns as the normal user (so the db stays user-owned — no chown).
    use_netns = bool(args.lidar_ns)
    vm_proc: subprocess.Popen[bytes] | None = None
    inner: subprocess.Popen[bytes] | None = None
    # Setup lives inside the try so the finally always cleans up + (netns mode)
    # restores db ownership, even if setup raises (e.g. missing CAP_NET_ADMIN).
    try:
        if use_netns:
            # Root inner can't write a user-owned WAL db (SQLite refuses cross-uid
            # writes), so hand it to root; restored in the finally.
            if db_existed:
                for suffix in ("", "-wal", "-shm"):
                    sidecar = Path(f"{db_path}{suffix}")
                    if sidecar.exists():
                        _ns(["chown", "0:0", str(sidecar)], check=False)
            _setup_netns(
                args.drv_ns,
                args.lidar_ns,
                args.veth_drv,
                args.veth_lidar,
                args.host_ip,
                args.lidar_ip,
            )
            inner_prefix = [*_sudo(), "ip", "netns", "exec", args.drv_ns]
            vm_prefix = [*_sudo(), "ip", "netns", "exec", args.lidar_ns]
        else:
            _setup_aliases(args.alias_iface, args.host_ip, args.lidar_ip)
            inner_prefix = []
            vm_prefix = []

        # Recorder + live Point-LIO run together (one coordinator).
        inner = subprocess.Popen(
            [
                *inner_prefix,
                sys.executable,
                "-m",
                "dimos.hardware.sensors.lidar.pointlio.tools.pcap_to_db",
                "--inner",
                "--db",
                str(db_path),
                "--odom-freq",
                str(args.odom_freq),
                "--ref-start-ts",
                repr(ref_start_ts),
                "--time-offset",
                "nan" if args.time_offset is None else repr(args.time_offset),
                "--max-sensor-sec",
                str(args.max_sensor_sec),
                "--host-ip",
                args.host_ip,
                "--lidar-ip",
                args.lidar_ip,
            ],
            cwd=os.getcwd(),
        )

        # Give Point-LIO a moment to come up before the sensor starts streaming.
        time.sleep(args.warmup_sec)
        vm_cfg = json.dumps(
            {
                "topics": {},
                "config": {
                    "pcap": str(pcap_path),
                    "rate": args.rate,
                    "delay": 0.0,
                    "lidar_ip": args.lidar_ip,
                    "host_ip": args.host_ip,
                    "lidar_netns": args.lidar_ns,
                },
            }
        ).encode()
        vm_proc = subprocess.Popen([*vm_prefix, vm_bin], stdin=subprocess.PIPE)
        assert vm_proc.stdin is not None
        vm_proc.stdin.write(vm_cfg)
        vm_proc.stdin.close()

        # The inner exits itself once the odom stream goes stagnant (pcap drained).
        inner.wait()
    finally:
        if use_netns:
            # Kill ONLY this run's processes — those in its (uniquely named)
            # namespaces — as root (the binaries run under sudo). `ip netns pids`
            # scopes precisely, so a concurrent run elsewhere is untouched.
            for ns in (args.lidar_ns, args.drv_ns):
                pids = _ns(["ip", "netns", "pids", ns], check=False).stdout.decode().split()
                if pids:
                    _ns(["kill", "-9", *pids], check=False)
        else:
            # Alias mode: our own user-owned children — signal them directly
            # (pointlio_native dies with its parent via PR_SET_PDEATHSIG).
            for proc in (vm_proc, inner):
                if proc and proc.poll() is None:
                    proc.terminate()
        for proc in (vm_proc, inner):
            if proc and proc.poll() is None:
                try:
                    proc.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    proc.kill()
        if use_netns:
            _teardown(args.drv_ns, args.lidar_ns, args.veth_drv)
            # Root inner owned the db files → hand them back to the invoking user.
            uid, gid = os.getuid(), os.getgid()
            for suffix in ("", "-wal", "-shm"):
                sidecar = Path(f"{db_path}{suffix}")
                if sidecar.exists():
                    _ns(["chown", f"{uid}:{gid}", str(sidecar)], check=False)
        else:
            _teardown_aliases(args.alias_iface)

    o_cnt, o_min, o_max = _table_stats(db_path, "pointlio_odometry")
    l_cnt = _table_stats(db_path, "pointlio_lidar")[0]
    if o_cnt == 0:
        print("[pcap_to_db] no odometry recorded — check the run above", file=sys.stderr)
        return 1
    print(
        f"[pcap_to_db] done odom={o_cnt} lidar={l_cnt} "
        f"ts=[{o_min:.3f}, {o_max:.3f}] span={o_max - o_min:.1f}s",
        flush=True,
    )
    return 0


# Inner process: live Point-LIO + recorder, already inside the drv netns.


def _run_inner(args: argparse.Namespace) -> int:
    from dimos.core.coordination.blueprints import autoconnect
    from dimos.core.coordination.module_coordinator import ModuleCoordinator
    from dimos.hardware.sensors.lidar.pointlio.module import PointLio

    db_path = Path(args.db)
    time_offset = float("nan") if args.time_offset == "nan" else float(args.time_offset)

    blueprint = autoconnect(
        PointLio.blueprint(
            host_ip=args.host_ip,
            lidar_ip=args.lidar_ip,
            odom_freq=args.odom_freq,
            debug=False,
        ).remappings(
            [
                (PointLio, "odometry", "pointlio_odometry"),
                (PointLio, "lidar", "pointlio_lidar"),
            ]
        ),
        _Rec.blueprint(
            db_path=str(db_path),
            ref_start_ts=args.ref_start_ts,
            time_offset=time_offset,
        ),
    ).global_config(n_workers=4, robot_model="mid360_pointlio_pcap_to_db")
    coord = ModuleCoordinator.build(blueprint)

    last_max = 0.0
    first_max: float | None = None
    stagnant_since: float | None = None
    start_time = time.time()
    startup_failed = False
    try:
        while True:
            time.sleep(_POLL_SEC)
            cnt, min_ts, max_ts = _table_stats(db_path, "pointlio_odometry")
            if cnt == 0:
                # Bound the no-output wait so a binary that never starts fails
                # cleanly instead of hanging (stagnation timeout only arms once
                # the first row exists).
                if time.time() - start_time > _STARTUP_TIMEOUT_SEC:
                    print(
                        f"[pcap_to_db] no odometry after {_STARTUP_TIMEOUT_SEC:.0f}s — Point-LIO "
                        "failed to start (check the binary, pcap path, and netns wiring).",
                        file=sys.stderr,
                        flush=True,
                    )
                    startup_failed = True
                    break
                continue
            if first_max is None:
                first_max = min_ts
            if args.max_sensor_sec > 0 and (max_ts - first_max) >= args.max_sensor_sec:
                print(
                    f"[pcap_to_db] reached --max-sensor-sec={args.max_sensor_sec:.1f}s", flush=True
                )
                break
            if max_ts == last_max:
                if stagnant_since is None:
                    stagnant_since = time.time()
                elif time.time() - stagnant_since > _STAGNANT_SEC:
                    break
            else:
                last_max = max_ts
                stagnant_since = None
    finally:
        coord.stop()
    return 1 if startup_failed else 0


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--pcap", help="Livox Mid-360 pcap capture")
    parser.add_argument(
        "--db",
        default=None,
        help="target memory2 SQLite db. Existing -> append/align; missing -> built from "
        "scratch. Omit to default to <pcap>.db next to the pcap.",
    )
    parser.add_argument(
        "--rate", type=float, default=1.0, help="replay-speed multiplier (default 1.0)"
    )
    parser.add_argument(
        "--odom-freq", type=float, default=30.0, help="Point-LIO odometry rate Hz (default 30)"
    )
    parser.add_argument(
        "--max-sensor-sec",
        type=float,
        default=0.0,
        help="stop after N sensor seconds (0 = whole pcap)",
    )
    parser.add_argument(
        "--time-offset",
        type=float,
        default=None,
        help="seconds added to every output ts (auto if omitted)",
    )
    parser.add_argument("--force", action="store_true", help="overwrite existing pointlio streams")
    parser.add_argument(
        "--warmup-sec", type=float, default=4.0, help="wait before streaming starts"
    )
    # Addressing knobs (override to run two replays at once).
    parser.add_argument("--host-ip", default="192.168.1.5")
    parser.add_argument("--lidar-ip", default="192.168.1.155")
    # netns is opt-in. Default (empty --lidar-ns) aliases both IPs on a dummy
    # interface and runs in the host namespace; pass --lidar-ns/--drv-ns to
    # isolate the two ends in separate namespaces instead.
    parser.add_argument("--drv-ns", default="")
    parser.add_argument("--lidar-ns", default="")
    parser.add_argument("--veth-drv", default="veth-drv")
    parser.add_argument("--veth-lidar", default="veth-lidar")
    parser.add_argument(
        "--alias-iface", default="dimos-mid360", help="dummy iface for no-netns mode"
    )
    # Internal: re-exec inside the drv netns to run the coordinator.
    parser.add_argument("--inner", action="store_true", help=argparse.SUPPRESS)
    parser.add_argument("--ref-start-ts", type=float, default=-1.0, help=argparse.SUPPRESS)

    args = parser.parse_args(argv)
    if args.inner:
        return _run_inner(args)
    if not args.pcap:
        parser.error("--pcap is required")
    # Ignore SIGINT in the parent so the finally-block teardown always runs.
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    return _run_outer(args)


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
