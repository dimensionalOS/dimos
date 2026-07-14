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

    # gen .db from pcap (defaults to <pcap>.db next to the pcap)
    python -m dimos.hardware.sensors.lidar.fastlio2.tools.pcap_to_db --pcap "$PCAP_PATH"

    # override FastLio2Config tuning via direct flags
    python -m dimos.hardware.sensors.lidar.fastlio2.tools.pcap_to_db \
        --pcap "$PCAP_PATH" --acc-cov 0.5 --filter-size-surf 0.3 --lidar-type livox

    # add to existing .db (a missing --db is fetched via get_data before falling
    # back to building from scratch)
    DB="mem2.db"
    python -m dimos.hardware.sensors.lidar.fastlio2.tools.pcap_to_db --db "$DB"  --pcap "$PCAP_PATH"

    # A quick-look <db>.rrd (aggregated world lidar + pose path) is written next
    # to the db and opened in rerun automatically (--no-gui to skip opening).

One coordinator runs three autoconnected modules: a ``VirtualMid360`` replays the
pcap over the Livox wire (aliasing the host/lidar IPs onto a dummy interface on
Linux, or lo0 on macOS — needs CAP_NET_ADMIN/sudo), an unmodified live ``FastLio2``
consumes it as real hardware, and a ``FastLio2Recorder`` appends FastLio2's
odometry/lidar into the db. This script just wires them and stops once the pcap
has drained. Replay is real time (FAST-LIO is not deterministic), so runs differ.
"""

from __future__ import annotations

import argparse
from pathlib import Path
import sqlite3
import subprocess
import sys
import time
from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:
    from dimos.core.coordination.blueprints import Blueprint

# Poll the db on this cadence while the replay drains the pcap.
_POLL_SEC = 1.0
# Stop after the odom stream has been stagnant this long (pcap fully drained).
_STAGNANT_SEC = 5.0
# No odometry within this long after start = FAST-LIO failed to come up (missing
# artifact, bad pcap, SLAM-init crash); bounds the poll loop. Generous to cover
# FAST-LIO's IMU-init latency.
_STARTUP_TIMEOUT_SEC = 60.0
# Extra seconds past the pcap's own duration before auto-stopping, when no
# explicit --max-sensor-sec is given.
_DRAIN_MARGIN_SEC = 4.0
# FastLio2Config fields exposed as direct CLI flags.
_TUNING_FIELDS = (
    "acc_cov",
    "gyr_cov",
    "b_acc_cov",
    "b_gyr_cov",
    "filter_size_surf",
    "filter_size_map",
    "det_range",
    "blind",
    "fov_degree",
    "scan_line",
    "lidar_type",
    "extrinsic_est_en",
    "scan_publish_en",
    "dense_publish_en",
)
# Max |Δts| to match a lidar frame to an odometry pose when aggregating the .rrd.
_POSE_MATCH_TOL = 0.1
# db stream/table names (= the recorder's In-port names).
_ODOM_STREAM = "fastlio_odometry"
_LIDAR_STREAM = "fastlio_lidar"


def _quat_to_rot(qx: float, qy: float, qz: float, qw: float) -> Any:
    import numpy as np

    return np.array(
        [
            [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
            [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
            [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)],
        ]
    )


def _pcap_sensor_span(pcap_path: Path) -> float:
    """Span (s) between the first and last packet of a classic little-endian pcap,
    walking only record headers (seeking past payloads). 0.0 if not parseable —
    the caller then falls back to stream-stagnation drain detection."""
    import struct

    try:
        with open(pcap_path, "rb") as handle:
            if handle.read(24)[:4] != b"\xd4\xc3\xb2\xa1":
                return 0.0
            first: float | None = None
            last = 0.0
            while True:
                header = handle.read(16)
                if len(header) < 16:
                    break
                ts_sec, ts_usec, incl_len, _orig = struct.unpack("<IIII", header)
                last = ts_sec + ts_usec / 1e6
                if first is None:
                    first = last
                handle.seek(incl_len, 1)
            return max(0.0, last - first) if first is not None else 0.0
    except OSError:
        return 0.0


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


def _write_rrd(db_path: Path, odom_stream: str, lidar_stream: str, voxel: float) -> Path | None:
    """Aggregate the recorded lidar (registered into world via the nearest
    odometry pose) plus the pose path into a ``.rrd`` next to the db.

    FastLio2 publishes its cloud in the sensor/body frame, so each frame is
    transformed to world by its pose here, then voxel-deduped. Best-effort: any
    failure is non-fatal to the recording. Returns the .rrd path, or None."""
    import numpy as np
    import rerun as rr

    from dimos.memory2.store.sqlite import SqliteStore
    from dimos.msgs.nav_msgs.Odometry import Odometry
    from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
    from dimos.visualization.rerun.init import rerun_init

    store = SqliteStore(path=str(db_path), must_exist=True)
    try:
        odom = list(store.stream(odom_stream, Odometry).order_by("ts"))
        if not odom:
            return None
        ots = np.array([o.ts for o in odom])
        opos = np.array(
            [
                [
                    o.data.pose.pose.position.x,
                    o.data.pose.pose.position.y,
                    o.data.pose.pose.position.z,
                ]
                for o in odom
            ]
        )
        oquat = np.array(
            [
                [
                    o.data.pose.pose.orientation.x,
                    o.data.pose.pose.orientation.y,
                    o.data.pose.pose.orientation.z,
                    o.data.pose.pose.orientation.w,
                ]
                for o in odom
            ]
        )
        chunks: list[Any] = []
        for lid in store.stream(lidar_stream, PointCloud2).order_by("ts"):
            j = int(np.argmin(np.abs(ots - lid.ts)))
            if abs(ots[j] - lid.ts) > _POSE_MATCH_TOL:
                continue
            pts = np.asarray(lid.data.as_numpy()[0])[:, :3].astype(np.float64)
            if pts.shape[0] == 0:
                continue
            world = pts @ _quat_to_rot(*oquat[j]).T + opos[j]
            # Per-frame voxel-dedup to bound memory before the global merge.
            _, idx = np.unique(np.floor(world / voxel).astype(np.int64), axis=0, return_index=True)
            chunks.append(world[idx])
        if not chunks:
            return None
        allpts = np.concatenate(chunks)
        _, idx = np.unique(np.floor(allpts / voxel).astype(np.int64), axis=0, return_index=True)
        agg = allpts[idx].astype(np.float32)

        # Height gradient: hot pink (low) -> dark purple (high).
        z = agg[:, 2]
        zn = (z - z.min()) / ((z.max() - z.min()) + 1e-9)
        low = np.array([255, 20, 147], dtype=np.float64)
        high = np.array([60, 0, 80], dtype=np.float64)
        colors = (low * (1 - zn)[:, None] + high * zn[:, None]).astype(np.uint8)

        rrd = db_path.with_suffix(".rrd")
        rerun_init("pcap_to_db")
        rr.save(str(rrd))
        rr.log("world/map", rr.Points3D(positions=agg, colors=colors, radii=[voxel / 8]))
        rr.log(
            "world/path",
            rr.LineStrips3D(strips=[opos.astype(np.float32)], colors=[[231, 76, 60]], radii=[0.05]),
        )
        return rrd
    finally:
        store.stop()


def _build_blueprint(
    args: argparse.Namespace, db_path: Path, overrides: dict[str, Any]
) -> Blueprint:
    """autoconnect(VirtualMid360 + FastLio2 + FastLio2Recorder).

    The recorder's ``fastlio_odometry``/``fastlio_lidar`` In ports (which name
    the db streams) are remapped to FastLio2's ``odometry``/``lidar`` outputs.
    VirtualMid360 carries no dimos streams — it speaks the Livox wire protocol,
    reached by host_ip/lidar_ip, and sets up the NIC itself.
    """
    from dimos.core.coordination.blueprints import autoconnect
    from dimos.hardware.sensors.lidar.fastlio2.module import FastLio2
    from dimos.hardware.sensors.lidar.fastlio2.recorder import FastLio2Recorder
    from dimos.hardware.sensors.lidar.virtual_mid360.module import VirtualMid360

    fastlio_kwargs: dict[str, Any] = dict(
        host_ip=args.host_ip, lidar_ip=args.lidar_ip, odom_freq=args.odom_freq, debug=False
    )
    fastlio_kwargs.update(overrides)

    return (
        autoconnect(
            VirtualMid360.blueprint(
                pcap=str(args.pcap_path),
                rate=args.rate,
                delay=args.warmup_sec,  # hold streaming until FastLio2's SDK is up
                host_ip=args.host_ip,
                lidar_ip=args.lidar_ip,
                alias_iface=args.alias_iface,
                # When the NIC is provisioned by hand, skip the module's own sudo
                # (it runs in a tty-less worker where a password prompt can't appear).
                setup_network=not args.no_network_setup,
            ),
            FastLio2.blueprint(**fastlio_kwargs),
            FastLio2Recorder.blueprint(db_path=str(db_path)),
        )
        .remappings(
            [
                (FastLio2Recorder, "fastlio_odometry", "odometry"),
                (FastLio2Recorder, "fastlio_lidar", "lidar"),
            ]
        )
        .global_config(n_workers=4, robot_model="mid360_fastlio_pcap_to_db")
    )


def _poll_until_drained(
    db_path: Path, odom_stream: str, lidar_stream: str, max_sensor_sec: float
) -> bool:
    """Block until the pcap drains or a cap is hit; False if FAST-LIO never
    produced odometry within the startup timeout.

    Drain is detected on the *lidar* stream's latest timestamp going flat: lidar
    is input-driven, so it stops advancing the moment the pcap is exhausted. The
    odometry stream can't be used for this — FAST-LIO keeps publishing odometry
    (dead-reckoning) at odom_freq after input stops, with ever-advancing
    timestamps, so its stream never looks stagnant and the run would hang."""
    last_lidar_max: float | None = None
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
                    f"[pcap_to_db] no odometry after {_STARTUP_TIMEOUT_SEC:.0f}s — FAST-LIO "
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
        if lidar_max <= 0.0:
            continue
        if lidar_max == last_lidar_max:
            if stagnant_since is None:
                stagnant_since = time.time()
            elif time.time() - stagnant_since > _STAGNANT_SEC:
                return True
        else:
            last_lidar_max = lidar_max
            stagnant_since = None


def _resolve_db_path(args: argparse.Namespace, pcap_path: Path) -> Path:
    """Where to record. Omitted --db -> <pcap>.db. A given --db that's missing is
    fetched via get_data (LFS) before falling back to building from scratch."""
    if not args.db:
        return pcap_path.with_suffix(".db")
    db_path = Path(args.db).expanduser().resolve()
    if not db_path.exists():
        try:
            from dimos.utils.data import get_data

            fetched = get_data(args.db)
            if fetched.exists():
                print(f"[pcap_to_db] fetched --db via get_data: {fetched}", flush=True)
                return fetched.resolve()
        except (FileNotFoundError, RuntimeError, OSError) as exc:  # not an LFS db -> build fresh
            print(
                f"[pcap_to_db] --db not found locally or via get_data ({exc}); "
                "building from scratch",
                file=sys.stderr,
                flush=True,
            )
    return db_path


def _run(args: argparse.Namespace) -> int:
    from dimos.core.coordination.module_coordinator import ModuleCoordinator

    pcap_path = Path(args.pcap).expanduser()
    if not pcap_path.exists():
        try:
            from dimos.utils.data import get_data

            pcap_path = get_data(args.pcap)
        except (FileNotFoundError, RuntimeError, OSError) as exc:
            print(
                f"[pcap_to_db] pcap not found locally or via get_data: {args.pcap} ({exc})",
                file=sys.stderr,
            )
            return 2
    pcap_path = pcap_path.resolve()
    args.pcap_path = pcap_path
    db_path = _resolve_db_path(args, pcap_path)
    db_path.parent.mkdir(parents=True, exist_ok=True)
    overrides = {
        field: getattr(args, field) for field in _TUNING_FIELDS if getattr(args, field) is not None
    }

    # Default the stop bound to the pcap's own duration: FAST-LIO keeps
    # dead-reckoning (publishing at full rate) after the pcap drains, so the
    # stream-stagnation check never fires on its own. Adding the real span makes
    # the run stop shortly after the data ends. --max-sensor-sec overrides.
    max_sensor_sec = args.max_sensor_sec
    if max_sensor_sec <= 0:
        span = _pcap_sensor_span(pcap_path)
        if span > 0:
            max_sensor_sec = span + _DRAIN_MARGIN_SEC

    print(
        f"[pcap_to_db] pcap={pcap_path.name} db={db_path.name} "
        f"({'append' if db_path.exists() else 'new'}) rate={args.rate} "
        f"ips={args.host_ip}/{args.lidar_ip} stop_at={max_sensor_sec or 'drain'}",
        flush=True,
    )

    coord = None
    try:
        coord = ModuleCoordinator.build(_build_blueprint(args, db_path, overrides))
        drained = _poll_until_drained(db_path, _ODOM_STREAM, _LIDAR_STREAM, max_sensor_sec)
    finally:
        if coord is not None:
            coord.stop()

    o_cnt, o_min, o_max = _odom_stats(db_path, _ODOM_STREAM)
    if o_cnt == 0 or not drained:
        print("[pcap_to_db] no odometry recorded — check the run above", file=sys.stderr)
        return 1
    print(
        f"[pcap_to_db] done odom={o_cnt} ts=[{o_min:.3f}, {o_max:.3f}] span={o_max - o_min:.1f}s",
        flush=True,
    )
    if not args.no_rrd:
        try:
            rrd = _write_rrd(db_path, _ODOM_STREAM, _LIDAR_STREAM, args.voxel)
            if rrd is not None:
                print(f"[pcap_to_db] wrote {rrd.name} (aggregated lidar + pose path)", flush=True)
                if not args.no_gui:
                    subprocess.Popen(["rerun", str(rrd)])
        except Exception as exc:  # viz is a non-fatal bonus
            print(f"[pcap_to_db] .rrd generation skipped ({exc})", file=sys.stderr, flush=True)
    return 0


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--pcap", help="Livox Mid-360 pcap capture")
    parser.add_argument(
        "--db",
        default=None,
        help="target memory2 SQLite db. Existing -> append/align; missing -> fetched via "
        "get_data (LFS), else built from scratch. Omit to default to <pcap>.db.",
    )
    parser.add_argument(
        "--rate", type=float, default=1.0, help="replay-speed multiplier (default 1.0)"
    )
    parser.add_argument(
        "--odom-freq", type=float, default=30.0, help="FAST-LIO odometry rate Hz (default 30)"
    )
    parser.add_argument(
        "--max-sensor-sec",
        type=float,
        default=0.0,
        help="stop after N sensor seconds (0 = whole pcap)",
    )
    parser.add_argument(
        "--no-rrd",
        action="store_true",
        help="skip writing the <db>.rrd quick-look (aggregated world lidar + pose path)",
    )
    parser.add_argument(
        "--no-gui", action="store_true", help="write the <db>.rrd but don't open it in rerun"
    )
    parser.add_argument(
        "--voxel", type=float, default=0.2, help="voxel size (m) for the .rrd aggregated map"
    )
    parser.add_argument(
        "--warmup-sec",
        type=float,
        default=4.0,
        help="seconds the fake lidar waits before streaming (lets FAST-LIO come up first)",
    )
    # FastLio2Config tuning as direct flags.
    tuning = parser.add_argument_group("FastLio2 tuning")
    tuning.add_argument("--acc-cov", type=float, help="IMU accel covariance")
    tuning.add_argument("--gyr-cov", type=float, help="IMU gyro covariance")
    tuning.add_argument("--b-acc-cov", type=float, help="IMU accel bias covariance")
    tuning.add_argument("--b-gyr-cov", type=float, help="IMU gyro bias covariance")
    tuning.add_argument("--filter-size-surf", type=float, help="IESKF scan voxel leaf (m)")
    tuning.add_argument("--filter-size-map", type=float, help="ikd-tree map voxel leaf (m)")
    tuning.add_argument("--det-range", type=float, help="max detection range (m)")
    tuning.add_argument("--blind", type=float, help="spherical min range (m)")
    tuning.add_argument("--fov-degree", type=int, help="sensor FOV (deg)")
    tuning.add_argument("--scan-line", type=int, help="lidar scan lines")
    tuning.add_argument("--lidar-type", choices=("livox", "velodyne", "ouster"))
    tuning.add_argument(
        "--extrinsic-est-en",
        action=argparse.BooleanOptionalAction,
        default=None,
        help="online IMU-LiDAR extrinsic estimation",
    )
    tuning.add_argument(
        "--scan-publish-en",
        action=argparse.BooleanOptionalAction,
        default=None,
        help="publish the lidar cloud",
    )
    tuning.add_argument(
        "--dense-publish-en",
        action=argparse.BooleanOptionalAction,
        default=None,
        help="publish the full (vs voxel-downsampled) cloud",
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
