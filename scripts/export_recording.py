#!/usr/bin/env python3
# Copyright 2025-2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.

"""Export a memory2 recording into the robomoo web app.

Reads a SqliteStore `.db` and pushes three structured artifacts over the same
token-guarded robot-ingest endpoints the live skills use (`map_uploader.py`,
`take_picture_skill.py`):

  1. occupancy map   — top-down lidar histogram → value-encoded grayscale PNG
                       → POST /api/robot/map
  2. trajectory      — downsampled odom path [{ts,x,y,theta}] (JSON)
                       → POST /api/robot/trajectory
  3. embedded frames — throttled color_image keyframes, each CLIP-embedded
                       (same model the web searches with) + thumbnail + pose
                       → POST /api/robot/frame (with an `embedding` field)

The web then renders an interactive vector map (pan/zoom), the driven path, and
in-browser CLIP semantic search over the frames.

Usage:
  ROBOMOO_URL=http://localhost:4470 ROBOT_INGEST_TOKEN=... \
    uv run python scripts/export_recording.py recording_go2.db
"""

from __future__ import annotations

import argparse
import json
import math
import os

import cv2
import httpx
import numpy as np

from dimos.memory2.store.sqlite import SqliteStore
from dimos.memory2.transform import downsample, throttle
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


def quat_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
    """Heading (yaw) about world +z, radians."""
    return math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))


def quat_to_matrix(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    """3x3 rotation matrix from a (x, y, z, w) quaternion."""
    return np.array(
        [
            [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
            [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
            [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)],
        ],
        dtype=np.float64,
    )


def post(url: str, token: str, path: str, **kwargs) -> None:
    resp = httpx.post(
        f"{url.rstrip('/')}{path}",
        headers={"Authorization": f"Bearer {token}"},
        timeout=60.0,
        **kwargs,
    )
    resp.raise_for_status()


def build_and_push_map(
    store: SqliteStore, url: str, token: str, resolution: float, every: int
) -> None:
    """Accumulate lidar into a world-frame top-down occupancy and upload it."""
    if "lidar" not in store.list_streams():
        logger.warning("no lidar stream — skipping map")
        return

    pts_world: list[np.ndarray] = []
    for obs in store.streams.lidar.transform(downsample(every)):
        if obs.pose is None:
            continue
        x, y, z, qx, qy, qz, qw = obs.pose
        pts = np.asarray(obs.data.points_f32(), dtype=np.float64)
        if pts.size == 0:
            continue
        world = pts @ quat_to_matrix(qx, qy, qz, qw).T + np.array([x, y, z])
        pts_world.append(world[:, :2])  # XY only
    if not pts_world:
        logger.warning("no lidar points with pose — skipping map")
        return

    xy = np.concatenate(pts_world, axis=0)
    min_x, min_y = xy.min(axis=0)
    max_x, max_y = xy.max(axis=0)
    width = max(1, int(math.ceil((max_x - min_x) / resolution)))
    height = max(1, int(math.ceil((max_y - min_y) / resolution)))

    cols = np.clip(((xy[:, 0] - min_x) / resolution).astype(int), 0, width - 1)
    rows = np.clip(((xy[:, 1] - min_y) / resolution).astype(int), 0, height - 1)
    counts = np.zeros((height, width), dtype=np.int32)
    np.add.at(counts, (rows, cols), 1)

    # value-encoded grayscale (matches map_uploader.py): unknown=255, occupied=100,
    # lightly-hit cells scaled, free space left unknown (transparent on the web).
    enc = np.full((height, width), 255, dtype=np.uint8)
    hit = counts > 0
    enc[hit] = np.clip(counts[hit] * 25, 1, 100).astype(np.uint8)

    ok, buf = cv2.imencode(".png", enc)
    if not ok:
        logger.warning("map PNG encode failed")
        return
    post(
        url,
        token,
        "/api/robot/map",
        files={"file": ("map.png", buf.tobytes(), "image/png")},
        data={
            "resolution": str(resolution),
            "originX": str(min_x),
            "originY": str(min_y),
            "width": str(width),
            "height": str(height),
        },
    )
    logger.info("pushed map %dx%d (res %.3f)", width, height, resolution)


def push_trajectory(store: SqliteStore, url: str, token: str, interval: float) -> None:
    if "odom" not in store.list_streams():
        logger.warning("no odom stream — skipping trajectory")
        return
    points = []
    for obs in store.streams.odom.transform(throttle(interval)):
        if obs.pose is None:
            continue
        x, y, _z, qx, qy, qz, qw = obs.pose
        points.append({"ts": obs.ts, "x": x, "y": y, "theta": quat_to_yaw(qx, qy, qz, qw)})
    if not points:
        logger.warning("no odom poses — skipping trajectory")
        return
    blob = json.dumps(points).encode("utf-8")
    post(
        url,
        token,
        "/api/robot/trajectory",
        files={"file": ("trajectory.json", blob, "application/json")},
    )
    logger.info("pushed trajectory (%d points)", len(points))


def push_frames(
    store: SqliteStore, url: str, token: str, interval: float, max_frames: int, thumb_w: int
) -> None:
    if "color_image" not in store.list_streams():
        logger.warning("no color_image stream — skipping frames")
        return
    from dimos.models.embedding.clip import CLIPModel

    clip = CLIPModel()
    n = 0
    for obs in store.streams.color_image.transform(throttle(interval)):
        if n >= max_frames:
            break
        img = obs.data
        emb = clip.embed(img)
        vec = emb.to_numpy().astype(float).ravel().tolist()

        bgr = img.to_opencv()
        h, w = bgr.shape[:2]
        if w > thumb_w:
            bgr = cv2.resize(bgr, (thumb_w, int(h * thumb_w / w)))
        ok, buf = cv2.imencode(".jpg", bgr, [cv2.IMWRITE_JPEG_QUALITY, 80])
        if not ok:
            continue

        data = {"label": "frame", "embedding": json.dumps(vec)}
        if obs.pose is not None:
            data["poseX"] = str(obs.pose[0])
            data["poseY"] = str(obs.pose[1])
        post(
            url,
            token,
            "/api/robot/frame",
            files={"file": ("frame.jpg", buf.tobytes(), "image/jpeg")},
            data=data,
        )
        n += 1
    logger.info("pushed %d embedded frames", n)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("db", help="path to the recording .db")
    ap.add_argument("--robomoo-url", default=os.getenv("ROBOMOO_URL", ""))
    ap.add_argument("--token", default=os.getenv("ROBOT_INGEST_TOKEN", ""))
    ap.add_argument("--map-resolution", type=float, default=0.05)
    ap.add_argument("--lidar-every", type=int, default=10, help="use every Nth lidar scan")
    ap.add_argument("--traj-interval", type=float, default=0.5, help="seconds between path points")
    ap.add_argument("--frame-interval", type=float, default=2.0, help="seconds between keyframes")
    ap.add_argument("--max-frames", type=int, default=120)
    ap.add_argument("--thumb-width", type=int, default=384)
    ap.add_argument("--no-map", action="store_true")
    ap.add_argument("--no-trajectory", action="store_true")
    ap.add_argument("--no-frames", action="store_true")
    args = ap.parse_args()

    if not args.robomoo_url or not args.token:
        raise SystemExit("set ROBOMOO_URL and ROBOT_INGEST_TOKEN (or pass --robomoo-url/--token)")

    store = SqliteStore(path=args.db)
    logger.info("opened %s — streams: %s", args.db, store.list_streams())

    if not args.no_map:
        build_and_push_map(store, args.robomoo_url, args.token, args.map_resolution, args.lidar_every)
    if not args.no_trajectory:
        push_trajectory(store, args.robomoo_url, args.token, args.traj_interval)
    if not args.no_frames:
        push_frames(
            store,
            args.robomoo_url,
            args.token,
            args.frame_interval,
            args.max_frames,
            args.thumb_width,
        )
    logger.info("done")


if __name__ == "__main__":
    main()
