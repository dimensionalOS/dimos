#!/usr/bin/env python3
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

"""Capture and replay M20 SLAM LCM logs.

The canonical replay boundary for M20 FAST-LIO2 debugging is the DimOS LCM
sensor interface:

    /raw_points#sensor_msgs.PointCloud2
    /airy_imu_{front|rear}#sensor_msgs.Imu

Capture records those inputs plus reference SLAM outputs by default. Replay is
input-only by default so a recorded session can be fed back into a local or NOS
FAST-LIO2 process without also publishing stale odometry or command topics.
"""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import shutil
import socket
import subprocess
import sys
import time
from typing import Any

VALID_LIDAR_SOURCES = frozenset({"front", "rear"})
RAW_POINTS_TOPIC = "/raw_points#sensor_msgs.PointCloud2"
YESENSE_IMU_TOPIC = "/yesense_imu#sensor_msgs.Imu"
REFERENCE_TOPICS = (
    "/odometry#nav_msgs.Odometry",
    "/registered_scan#sensor_msgs.PointCloud2",
    "/global_map_fastlio#sensor_msgs.PointCloud2",
)
NAV_DEBUG_TOPICS = (
    "/terrain_map#sensor_msgs.PointCloud2",
    "/terrain_map_ext#sensor_msgs.PointCloud2",
    "/costmap_cloud#sensor_msgs.PointCloud2",
    "/path#nav_msgs.Path",
    "/goal#geometry_msgs.PointStamped",
    "/way_point#geometry_msgs.PointStamped",
)
SIDECAR_LOGS = (
    "/tmp/m20_nav_stack_native.log",
    "/var/log/drdds_recv.log",
)


def _validate_lidar_source(lidar_source: str) -> str:
    source = lidar_source.strip().lower()
    if source not in VALID_LIDAR_SOURCES:
        raise ValueError(f"lidar_source must be one of {sorted(VALID_LIDAR_SOURCES)}")
    return source


def airy_imu_topic(lidar_source: str) -> str:
    return f"/airy_imu_{_validate_lidar_source(lidar_source)}#sensor_msgs.Imu"


def record_topics(
    *,
    lidar_source: str,
    include_reference: bool = True,
    include_nav_debug: bool = False,
    include_yesense: bool = False,
) -> list[str]:
    """Return the LCM topics to record for a reusable M20 SLAM fixture."""
    topics = [RAW_POINTS_TOPIC, airy_imu_topic(lidar_source)]
    if include_yesense:
        topics.append(YESENSE_IMU_TOPIC)
    if include_reference:
        topics.extend(REFERENCE_TOPICS)
    if include_nav_debug:
        topics.extend(NAV_DEBUG_TOPICS)
    return list(dict.fromkeys(topics))


def replay_topics(*, lidar_source: str, mode: str = "inputs") -> list[str]:
    """Return safe replay topics.

    ``inputs`` is the default for feeding FAST-LIO2. ``reference`` replays only
    the original SLAM outputs for visual inspection. ``all`` replays both input
    and reference topics, but still omits nav command topics.
    """
    if mode == "inputs":
        return [RAW_POINTS_TOPIC, airy_imu_topic(lidar_source)]
    if mode == "reference":
        return list(REFERENCE_TOPICS)
    if mode == "all":
        return record_topics(lidar_source=lidar_source, include_nav_debug=True)
    raise ValueError("mode must be one of: inputs, reference, all")


def _escape_posix_ere_literal(value: str) -> str:
    """Escape only characters that appear in our topic names and affect POSIX ERE."""
    return value.replace("\\", "\\\\").replace(".", "\\.").replace("|", "\\|")


def channel_regex(topics: list[str]) -> str:
    if not topics:
        raise ValueError("at least one topic is required")
    return "^(" + "|".join(_escape_posix_ere_literal(topic) for topic in topics) + ")$"


def build_logger_command(
    *,
    log_path: Path,
    topics: list[str],
    lcm_url: str | None = None,
    force: bool = False,
    flush_interval_ms: int = 100,
    max_unwritten_mb: int = 1024,
) -> list[str]:
    command = ["lcm-logger"]
    if force:
        command.append("-f")
    command.extend(
        [
            f"--flush-interval={flush_interval_ms}",
            f"--max-unwritten-mb={max_unwritten_mb}",
        ]
    )
    if lcm_url:
        command.append(f"--lcm-url={lcm_url}")
    command.extend(["-c", channel_regex(topics), str(log_path)])
    return command


def build_logplayer_command(
    *,
    log_path: Path,
    lidar_source: str,
    mode: str = "inputs",
    speed: float = 1.0,
    lcm_url: str | None = None,
    verbose: bool = False,
) -> list[str]:
    command = ["lcm-logplayer"]
    if verbose:
        command.append("--verbose")
    command.append(f"--speed={speed:g}")
    if lcm_url:
        command.append(f"--lcm-url={lcm_url}")
    command.append(f"--regexp={channel_regex(replay_topics(lidar_source=lidar_source, mode=mode))}")
    command.append(str(log_path))
    return command


def _git_sha() -> str | None:
    try:
        result = subprocess.run(
            ["git", "rev-parse", "HEAD"],
            check=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            text=True,
        )
    except Exception:
        return None
    return result.stdout.strip() or None


def _env_snapshot() -> dict[str, str]:
    prefixes = ("M20_", "DIMOS_", "LCM_")
    return {key: value for key, value in sorted(os.environ.items()) if key.startswith(prefixes)}


def write_metadata(
    *,
    outdir: Path,
    log_path: Path,
    topics: list[str],
    command: list[str],
    label: str | None,
    duration_sec: float | None,
) -> Path:
    metadata: dict[str, Any] = {
        "label": label,
        "created_wall_time": time.time(),
        "hostname": socket.gethostname(),
        "cwd": os.getcwd(),
        "git_sha": _git_sha(),
        "duration_sec": duration_sec,
        "log_path": str(log_path),
        "topics": topics,
        "logger_command": command,
        "env": _env_snapshot(),
    }
    path = outdir / "metadata.json"
    path.write_text(json.dumps(metadata, indent=2, sort_keys=True) + "\n")
    return path


def copy_sidecars(outdir: Path) -> None:
    for source in SIDECAR_LOGS:
        src = Path(source)
        if src.exists():
            shutil.copy2(src, outdir / src.name)


def record_exit_code(*, returncode: int | None, timed_stop: bool) -> int:
    if returncode is None:
        return 0
    if timed_stop and returncode < 0:
        return 0
    return int(returncode)


def _terminate_process(proc: subprocess.Popen[Any]) -> None:
    proc.terminate()
    try:
        proc.wait(timeout=10.0)
    except subprocess.TimeoutExpired:
        proc.kill()
        proc.wait()


def run_record(args: argparse.Namespace) -> int:
    outdir = Path(args.outdir)
    outdir.mkdir(parents=True, exist_ok=True)
    log_path = outdir / args.log_name
    topics = record_topics(
        lidar_source=args.lidar_source,
        include_reference=not args.inputs_only,
        include_nav_debug=args.include_nav_debug,
        include_yesense=args.include_yesense,
    )
    command = build_logger_command(
        log_path=log_path,
        topics=topics,
        lcm_url=args.lcm_url,
        force=args.force,
        flush_interval_ms=args.flush_interval_ms,
        max_unwritten_mb=args.max_unwritten_mb,
    )
    metadata_path = write_metadata(
        outdir=outdir,
        log_path=log_path,
        topics=topics,
        command=command,
        label=args.label,
        duration_sec=args.duration,
    )

    print(f"[slam_lcm_log] recording {len(topics)} topics to {log_path}", flush=True)
    print(f"[slam_lcm_log] metadata: {metadata_path}", flush=True)
    print("[slam_lcm_log] command:", " ".join(command), flush=True)

    proc = subprocess.Popen(command)
    timed_stop = False
    try:
        if args.duration is None:
            proc.wait()
        else:
            time.sleep(args.duration)
            timed_stop = True
            _terminate_process(proc)
    except KeyboardInterrupt:
        _terminate_process(proc)
    finally:
        if args.copy_sidecars:
            copy_sidecars(outdir)

    return record_exit_code(returncode=proc.returncode, timed_stop=timed_stop)


def run_replay(args: argparse.Namespace) -> int:
    command = build_logplayer_command(
        log_path=Path(args.log),
        lidar_source=args.lidar_source,
        mode=args.mode,
        speed=args.speed,
        lcm_url=args.lcm_url,
        verbose=args.verbose,
    )
    print("[slam_lcm_log] command:", " ".join(command), flush=True)
    return subprocess.call(command)


def _default_outdir() -> str:
    return f"/var/opt/robot/data/tmp/m20_slam_lcm_{int(time.time())}"


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)

    record = subparsers.add_parser("record", help="record an M20 SLAM LCM dataset")
    record.add_argument("--outdir", default=_default_outdir())
    record.add_argument("--log-name", default="session.lcm")
    record.add_argument("--duration", type=float, default=None)
    record.add_argument("--label", default=None)
    record.add_argument("--lidar-source", choices=sorted(VALID_LIDAR_SOURCES), default="front")
    record.add_argument(
        "--inputs-only", action="store_true", help="record only raw_points + Airy IMU"
    )
    record.add_argument("--include-nav-debug", action="store_true")
    record.add_argument("--include-yesense", action="store_true")
    record.add_argument("--lcm-url", default=None)
    record.add_argument("--flush-interval-ms", type=int, default=100)
    record.add_argument("--max-unwritten-mb", type=int, default=1024)
    record.add_argument("--force", action="store_true", help="overwrite an existing log file")
    record.add_argument("--no-copy-sidecars", dest="copy_sidecars", action="store_false")
    record.set_defaults(copy_sidecars=True, func=run_record)

    replay = subparsers.add_parser("replay", help="replay an M20 SLAM LCM dataset")
    replay.add_argument("log")
    replay.add_argument("--lidar-source", choices=sorted(VALID_LIDAR_SOURCES), default="front")
    replay.add_argument("--mode", choices=("inputs", "reference", "all"), default="inputs")
    replay.add_argument("--speed", type=float, default=1.0)
    replay.add_argument("--lcm-url", default=None)
    replay.add_argument("--verbose", action="store_true")
    replay.set_defaults(func=run_replay)

    return parser


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    return int(args.func(args))


if __name__ == "__main__":
    sys.exit(main())
