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

"""
Usage:
    # fetch a sample Mid-360 capture (the get_data arg is the dir/file inside the
    # LFS archive, NOT the archive name)
    PCAP_PATH="$(python -c "from dimos.utils.data import get_data; print(get_data('mid360_shake_stairs/mid360_shake_stairs.pcap'))")"

    # gen .db from pcap with your config (defaults to <pcap>.db next to the pcap)
    python -m dimos.hardware.sensors.lidar.pointlio.scripts.pcap_to_db \
        --config dimos/hardware/sensors/lidar/pointlio/config/default.yaml \
        --pcap "$PCAP_PATH"

    # add to existing .db
    DB="mem2.db"
    python -m dimos.hardware.sensors.lidar.pointlio.scripts.pcap_to_db --db "$DB"  --pcap "$PCAP_PATH"

    # generate map
    dimos map summary "$DB"
    dimos map pose-fill "$DB" \
        --target pointlio_lidar \
        --pose-source pointlio_odometry \
        --out "${DB%.db}_posed.db"
    dimos map global "${DB%.db}_posed.db" --lidar pointlio_lidar

One coordinator runs three autoconnected modules: a ``VirtualMid360`` replays the
pcap over the Livox wire (aliasing the host/lidar IPs onto a dummy interface on
Linux, or lo0 on macOS — needs CAP_NET_ADMIN/sudo), an unmodified live ``PointLio``
consumes it as real hardware, and a ``PointlioRecorder`` appends PointLio's
odometry/lidar into the db. This script just wires them and stops once the pcap
has drained. Replay is real time (Point-LIO is not deterministic), so runs differ.
"""

from __future__ import annotations

import argparse
from pathlib import Path
import sqlite3
import sys
import time
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from dimos.core.coordination.blueprints import Blueprint

# Poll the db on this cadence while the replay drains the pcap.
_POLL_SEC = 1.0
# Stop after the odom stream has been stagnant this long (pcap fully drained).
_STAGNANT_SEC = 5.0
# No odometry within this long after start = Point-LIO failed to come up (missing
# artifact, bad pcap, SLAM-init crash); bounds the poll loop. Generous to cover
# Point-LIO's IMU-init latency.
_STARTUP_TIMEOUT_SEC = 60.0


def _odom_stats(db_path: Path, table: str) -> tuple[int, float, float]:
    """(count, min_ts, max_ts) for the odom table; zeros if absent."""
    if not db_path.exists():
        return 0, 0.0, 0.0
    con = sqlite3.connect(f"file:{db_path}?mode=ro", uri=True, timeout=2.0)
    try:
        try:
            row = con.execute(f"SELECT COUNT(*), MIN(ts), MAX(ts) FROM '{table}'").fetchone()
        except sqlite3.OperationalError:
            return 0, 0.0, 0.0
        return row[0] or 0, row[1] or 0.0, row[2] or 0.0
    finally:
        con.close()


def _build_blueprint(args: argparse.Namespace, db_path: Path, config_path: str) -> Blueprint:
    """autoconnect(VirtualMid360 + PointLio + PointlioRecorder).

    PointLio's ``odometry``/``lidar`` outputs auto-wire to the recorder's
    same-named inputs. VirtualMid360 carries no dimos streams — it speaks the
    Livox wire protocol, reached by host_ip/lidar_ip, and sets up the NIC itself.
    """
    from dimos.core.coordination.blueprints import autoconnect
    from dimos.hardware.sensors.lidar.pointlio.module import PointLio
    from dimos.hardware.sensors.lidar.pointlio.recorder import PointlioRecorder
    from dimos.hardware.sensors.lidar.virtual_mid360.module import VirtualMid360

    # `config` (not `config_path`, which PointLioConfig derives itself); already
    # an absolute path so it bypasses the config-dir-relative resolution. Omit
    # when empty to keep the default.yaml.
    pointlio_kwargs: dict[str, object] = dict(
        host_ip=args.host_ip, lidar_ip=args.lidar_ip, odom_freq=args.odom_freq, debug=False
    )
    if config_path:
        pointlio_kwargs["config"] = config_path

    return autoconnect(
        VirtualMid360.blueprint(
            pcap=str(args.pcap_path),
            rate=args.rate,
            delay=args.warmup_sec,  # hold streaming until PointLio's SDK is up
            host_ip=args.host_ip,
            lidar_ip=args.lidar_ip,
            alias_iface=args.alias_iface,
            # When the NIC is provisioned by hand, skip the module's own sudo
            # (it runs in a tty-less worker where a password prompt can't appear).
            setup_network=not args.no_network_setup,
        ),
        PointLio.blueprint(**pointlio_kwargs),
        PointlioRecorder.blueprint(
            db_path=str(db_path),
            odom_stream_name=args.odom_stream_name,
            lidar_stream_name=args.lidar_stream_name,
            time_offset=float("nan") if args.time_offset is None else args.time_offset,
            force=args.force,
        ),
    ).global_config(n_workers=4, robot_model="mid360_pointlio_pcap_to_db")


def _poll_until_drained(
    db_path: Path, odom_stream: str, lidar_stream: str, max_sensor_sec: float
) -> bool:
    """Block until the pcap drains or a cap is hit; False if Point-LIO never
    produced odometry within the startup timeout.

    Drain is detected on the *lidar* stream's latest timestamp going flat: lidar
    is input-driven, so it stops advancing the moment the pcap is exhausted. The
    odometry stream can't be used for this — Point-LIO keeps publishing odometry
    (dead-reckoning) at odom_freq after input stops, with ever-advancing
    timestamps, so its stream never looks stagnant and the run would hang."""
    last_lidar_max = 0.0
    first_max: float | None = None
    stagnant_since: float | None = None
    start_time = time.time()
    while True:
        time.sleep(_POLL_SEC)
        odom_cnt, odom_min, odom_max = _odom_stats(db_path, odom_stream)
        if odom_cnt == 0:
            # Stagnation timeout only arms once odometry exists, so bound the
            # no-output wait separately or a dead binary would hang forever.
            if time.time() - start_time > _STARTUP_TIMEOUT_SEC:
                print(
                    f"[pcap_to_db] no odometry after {_STARTUP_TIMEOUT_SEC:.0f}s — Point-LIO "
                    "failed to start (check the binary, pcap path, and interface setup).",
                    file=sys.stderr,
                    flush=True,
                )
                return False
            continue
        if first_max is None:
            first_max = odom_min
        if max_sensor_sec > 0 and (odom_max - first_max) >= max_sensor_sec:
            print(f"[pcap_to_db] reached --max-sensor-sec={max_sensor_sec:.1f}s", flush=True)
            return True
        _, _, lidar_max = _odom_stats(db_path, lidar_stream)
        if lidar_max == last_lidar_max:
            if stagnant_since is None:
                stagnant_since = time.time()
            elif time.time() - stagnant_since > _STAGNANT_SEC:
                return True
        else:
            last_lidar_max = lidar_max
            stagnant_since = None


def _run(args: argparse.Namespace) -> int:
    from dimos.core.coordination.module_coordinator import ModuleCoordinator

    pcap_path = Path(args.pcap).expanduser().resolve()
    if not pcap_path.exists():
        print(f"[pcap_to_db] missing pcap: {pcap_path}", file=sys.stderr)
        return 2
    args.pcap_path = pcap_path
    db_path = Path(args.db).expanduser().resolve() if args.db else pcap_path.with_suffix(".db")
    db_path.parent.mkdir(parents=True, exist_ok=True)
    # Resolve --config against the *invoking* cwd (pwd-relative) up front; the
    # PointLio config field otherwise resolves a relative path against its own
    # config/ dir, never the pwd. Absolute path passes through unchanged.
    config_path = str(Path(args.config).expanduser().resolve()) if args.config else ""
    if config_path and not Path(config_path).exists():
        print(f"[pcap_to_db] missing --config: {config_path}", file=sys.stderr)
        return 2

    print(
        f"[pcap_to_db] pcap={pcap_path.name} db={db_path.name} "
        f"({'append' if db_path.exists() else 'new'}) rate={args.rate} "
        f"ips={args.host_ip}/{args.lidar_ip}",
        flush=True,
    )

    coord = None
    try:
        coord = ModuleCoordinator.build(_build_blueprint(args, db_path, config_path))
        drained = _poll_until_drained(
            db_path, args.odom_stream_name, args.lidar_stream_name, args.max_sensor_sec
        )
    finally:
        if coord is not None:
            coord.stop()

    o_cnt, o_min, o_max = _odom_stats(db_path, args.odom_stream_name)
    if o_cnt == 0 or not drained:
        print("[pcap_to_db] no odometry recorded — check the run above", file=sys.stderr)
        return 1
    print(
        f"[pcap_to_db] done odom={o_cnt} ts=[{o_min:.3f}, {o_max:.3f}] span={o_max - o_min:.1f}s",
        flush=True,
    )
    return 0


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
        "--warmup-sec",
        type=float,
        default=4.0,
        help="seconds the fake lidar waits before streaming (lets Point-LIO come up first)",
    )
    parser.add_argument(
        "--config",
        default="",
        help="Point-LIO YAML config (pwd-relative or absolute; default: module's default.yaml)",
    )
    parser.add_argument(
        "--odom-stream-name",
        default="pointlio_odometry",
        help="db stream/table name for the recorded odometry",
    )
    parser.add_argument(
        "--lidar-stream-name",
        default="pointlio_lidar",
        help="db stream/table name for the recorded point cloud",
    )
    # Addressing knobs (override to run two replays at once).
    parser.add_argument("--host-ip", default="192.168.1.5")
    parser.add_argument("--lidar-ip", default="192.168.1.155")
    parser.add_argument(
        "--alias-iface", default="dimos-mid360", help="dummy iface the host/lidar IPs live on"
    )
    parser.add_argument(
        "--no-network-setup",
        action="store_true",
        help="don't let the module alias the NIC via sudo — you've set up host/lidar IPs "
        "+ multicast routes yourself (e.g. on macOS where worker-side sudo can't prompt)",
    )

    args = parser.parse_args(argv)
    if not args.pcap:
        parser.error("--pcap is required")
    return _run(args)


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
