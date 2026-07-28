#!/usr/bin/env python3
"""Capture DimOS costmap and odometry frames into JSON plus compressed NPZ.

This is a read-only sidecar subscriber. It never publishes robot commands and
does not create a robot connection or a second DimOS runtime.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from datetime import datetime, timezone
import json
from pathlib import Path
import threading
import time
from typing import Any

import numpy as np

from dimos.core.global_config import global_config
from dimos.core.transport_factory import make_transport
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid

ODOM_COLUMNS = ("ts", "x", "y", "z", "qx", "qy", "qz", "qw")


@dataclass(frozen=True)
class CostmapFrame:
    ts: float
    frame_id: str
    resolution: float
    origin: tuple[float, float, float, float, float, float, float]
    grid: np.ndarray


@dataclass(frozen=True)
class OdomFrame:
    ts: float
    frame_id: str
    pose: tuple[float, float, float, float, float, float, float]


def _costmap_frame(message: OccupancyGrid) -> CostmapFrame:
    origin = message.origin
    return CostmapFrame(
        ts=float(message.ts),
        frame_id=str(message.frame_id),
        resolution=float(message.resolution),
        origin=(
            float(origin.position.x),
            float(origin.position.y),
            float(origin.position.z),
            float(origin.orientation.x),
            float(origin.orientation.y),
            float(origin.orientation.z),
            float(origin.orientation.w),
        ),
        grid=np.asarray(message.grid, dtype=np.int8).copy(),
    )


def _odom_frame(message: PoseStamped) -> OdomFrame:
    return OdomFrame(
        ts=float(message.ts),
        frame_id=str(message.frame_id),
        pose=(
            float(message.position.x),
            float(message.position.y),
            float(message.position.z),
            float(message.orientation.x),
            float(message.orientation.y),
            float(message.orientation.z),
            float(message.orientation.w),
        ),
    )


def write_dataset(
    output_dir: Path,
    *,
    costmaps: list[CostmapFrame],
    odometry: list[OdomFrame],
    source: str,
    transport: str,
    costmap_topic: str,
    odom_topic: str,
) -> tuple[Path, Path]:
    """Write a self-describing manifest and lossless numeric arrays."""

    output_dir.mkdir(parents=True, exist_ok=True)
    manifest_path = output_dir / "manifest.json"
    npz_path = output_dir / "costmap_odom.npz"

    odom_array = np.asarray(
        [[frame.ts, *frame.pose] for frame in odometry],
        dtype=np.float64,
    ).reshape((-1, len(ODOM_COLUMNS)))
    arrays: dict[str, np.ndarray] = {
        "odom": odom_array,
        "costmap_ts": np.asarray([frame.ts for frame in costmaps], dtype=np.float64),
        "costmap_resolution": np.asarray(
            [frame.resolution for frame in costmaps],
            dtype=np.float64,
        ),
        "costmap_origin_xyzw": np.asarray(
            [frame.origin for frame in costmaps],
            dtype=np.float64,
        ).reshape((-1, 7)),
    }

    shapes = {frame.grid.shape for frame in costmaps}
    stacked_costmaps = bool(costmaps) and len(shapes) == 1
    if stacked_costmaps:
        arrays["costmaps"] = np.stack([frame.grid for frame in costmaps])
    else:
        for index, frame in enumerate(costmaps):
            arrays[f"costmap_{index:03d}"] = frame.grid
    np.savez_compressed(npz_path, **arrays)

    manifest: dict[str, Any] = {
        "schema_version": "1.0",
        "created_at": datetime.now(timezone.utc).isoformat(),
        "source": source,
        "transport": transport,
        "topics": {
            "costmap": costmap_topic,
            "odom": odom_topic,
        },
        "costmap": {
            "frame_count": len(costmaps),
            "value_semantics": {
                "-1": "unknown",
                "0": "free",
                "1..99": "increasing traversal cost",
                "100": "occupied_or_lethal",
            },
            "array_layout": (
                "costmaps[N,H,W]"
                if stacked_costmaps
                else "costmap_NNN[H,W] (variable-size frames)"
            ),
            "row_axis": "grid_y",
            "column_axis": "grid_x",
            "frames": [
                {
                    "index": index,
                    "ts": frame.ts,
                    "frame_id": frame.frame_id,
                    "shape": list(frame.grid.shape),
                    "resolution_m_per_cell": frame.resolution,
                    "origin_pose_xyzw": list(frame.origin),
                    "npz_key": "costmaps" if stacked_costmaps else f"costmap_{index:03d}",
                }
                for index, frame in enumerate(costmaps)
            ],
        },
        "odom": {
            "frame_count": len(odometry),
            "npz_key": "odom",
            "columns": list(ODOM_COLUMNS),
            "frame_ids": [frame.frame_id for frame in odometry],
            "samples": [
                {
                    "ts": frame.ts,
                    "frame_id": frame.frame_id,
                    "position_xyz": list(frame.pose[:3]),
                    "orientation_xyzw": list(frame.pose[3:]),
                }
                for frame in odometry
            ],
        },
    }
    manifest_path.write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    return manifest_path, npz_path


def capture(
    *,
    frames: int,
    timeout_s: float,
    output_dir: Path,
    source: str,
    transport: str,
    costmap_topic: str,
    odom_topic: str,
) -> tuple[Path, Path, int, int]:
    """Subscribe to one active Runtime and collect up to ``frames`` per stream."""

    if frames <= 0:
        raise ValueError("frames must be positive")
    if timeout_s <= 0:
        raise ValueError("timeout must be positive")

    global_config.update(transport=transport)
    costmap_transport = make_transport(costmap_topic, OccupancyGrid)
    odom_transport = make_transport(odom_topic, PoseStamped)
    costmaps: list[CostmapFrame] = []
    odometry: list[OdomFrame] = []
    lock = threading.Lock()
    complete = threading.Event()

    def update_complete() -> None:
        if len(costmaps) >= frames and len(odometry) >= frames:
            complete.set()

    def on_costmap(message: OccupancyGrid) -> None:
        with lock:
            if len(costmaps) < frames:
                costmaps.append(_costmap_frame(message))
            update_complete()

    def on_odom(message: PoseStamped) -> None:
        with lock:
            if len(odometry) < frames:
                odometry.append(_odom_frame(message))
            update_complete()

    unsubscribe_costmap = costmap_transport.subscribe(on_costmap)
    unsubscribe_odom = odom_transport.subscribe(on_odom)
    try:
        complete.wait(timeout_s)
    finally:
        unsubscribe_costmap()
        unsubscribe_odom()
        costmap_transport.stop()
        odom_transport.stop()

    with lock:
        captured_costmaps = list(costmaps)
        captured_odometry = list(odometry)
    if not captured_costmaps or not captured_odometry:
        raise RuntimeError(
            "No complete costmap+odom pair was observed. "
            "Check that one DimOS Runtime is active and the topic/transport names match."
        )

    manifest_path, npz_path = write_dataset(
        output_dir,
        costmaps=captured_costmaps,
        odometry=captured_odometry,
        source=source,
        transport=transport,
        costmap_topic=costmap_topic,
        odom_topic=odom_topic,
    )
    return manifest_path, npz_path, len(captured_costmaps), len(captured_odometry)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--frames", type=int, default=40, help="Frames per stream")
    parser.add_argument("--timeout", type=float, default=30.0, help="Capture timeout in seconds")
    parser.add_argument("--transport", choices=("lcm", "zenoh"), default=global_config.transport)
    parser.add_argument("--source", choices=("live", "replay"), default="live")
    parser.add_argument("--costmap-topic", default="global_costmap")
    parser.add_argument("--odom-topic", default="odom")
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("runtime/exports/costmap-odom") / time.strftime("%Y%m%d-%H%M%S"),
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    manifest, npz, costmap_count, odom_count = capture(
        frames=args.frames,
        timeout_s=args.timeout,
        output_dir=args.output_dir,
        source=args.source,
        transport=args.transport,
        costmap_topic=args.costmap_topic,
        odom_topic=args.odom_topic,
    )
    print(
        json.dumps(
            {
                "ok": True,
                "source": args.source,
                "costmap_frames": costmap_count,
                "odom_frames": odom_count,
                "manifest": str(manifest.resolve()),
                "npz": str(npz.resolve()),
            },
            ensure_ascii=False,
        )
    )


if __name__ == "__main__":
    main()
