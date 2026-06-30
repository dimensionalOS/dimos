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

"""Publish deterministic color bars through the same DimOS→Rerun image path.

This is a narrow smoke test for the camera visualization plumbing. It avoids
Robosuite entirely and checks each transport layer with known RGB values before
publishing them to Rerun:

1. RGB array -> `.npy` bytes -> RGB array
2. RGB array -> DimOS Image LCM encode/decode -> RGB array
3. RGB array -> private DimOS LCM topics -> RerunBridgeModule
"""

from __future__ import annotations

import argparse
from io import BytesIO
import json
from pathlib import Path
import sys
import time

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT))
sys.path.insert(0, str(REPO_ROOT / "packages" / "dimos-runtime-protocol" / "src"))
sys.path.insert(0, str(REPO_ROOT / "packages" / "dimos-robosuite-sidecar" / "src"))

from dimos.benchmark.runtime.artifacts import write_json
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from scripts.benchmarks.demo_robosuite_panda_lift import (
    RerunStreamPublisher,
    _free_tcp_port,
)


def _color_bars(height: int, width: int) -> np.ndarray:
    if height < 3 or width < 1:
        raise ValueError("height must be >= 3 and width must be >= 1")
    image = np.zeros((height, width, 3), dtype=np.uint8)
    first = height // 3
    second = 2 * height // 3
    image[:first, :, :] = [255, 0, 0]
    image[first:second, :, :] = [0, 255, 0]
    image[second:, :, :] = [0, 0, 255]
    return image


def _npy_round_trip(image: np.ndarray) -> np.ndarray:
    buffer = BytesIO()
    np.save(buffer, image, allow_pickle=False)
    return np.load(BytesIO(buffer.getvalue()), allow_pickle=False)


def _lcm_round_trip(image: np.ndarray) -> np.ndarray:
    encoded = Image.from_numpy(image, format=ImageFormat.RGB).lcm_encode()
    decoded = Image.lcm_decode(encoded)
    if decoded.format != ImageFormat.RGB:
        raise AssertionError(f"expected RGB after LCM round trip, got {decoded.format}")
    return decoded.data


def _pixel_summary(image: np.ndarray) -> dict[str, list[int]]:
    return {
        "top_left_rgb": [int(value) for value in image[0, 0, :3]],
        "center_rgb": [int(value) for value in image[image.shape[0] // 2, image.shape[1] // 2, :3]],
        "bottom_left_rgb": [int(value) for value in image[-1, 0, :3]],
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--height", type=int, default=120)
    parser.add_argument("--width", type=int, default=160)
    parser.add_argument("--frames", type=int, default=60)
    parser.add_argument("--hz", type=float, default=10.0)
    parser.add_argument("--rerun-memory-limit", default="128MB")
    parser.add_argument("--rerun-grpc-port", type=int, default=0)
    parser.add_argument("--rerun-lcm-port", type=int, default=0)
    parser.add_argument(
        "--artifact-dir",
        type=Path,
        default=REPO_ROOT / "artifacts" / "benchmark" / "rerun-color-smoke",
    )
    args = parser.parse_args()

    source = _color_bars(args.height, args.width)
    npy_decoded = _npy_round_trip(source)
    lcm_decoded = _lcm_round_trip(npy_decoded)
    npy_matches = bool(np.array_equal(source, npy_decoded))
    lcm_matches = bool(np.array_equal(source, lcm_decoded))
    if not npy_matches or not lcm_matches:
        raise AssertionError(
            f"color smoke mismatch: npy_matches={npy_matches}, lcm_matches={lcm_matches}"
        )

    grpc_port = args.rerun_grpc_port if args.rerun_grpc_port > 0 else _free_tcp_port()
    lcm_port = args.rerun_lcm_port if args.rerun_lcm_port > 0 else _free_tcp_port()
    publisher = RerunStreamPublisher(
        grpc_port=grpc_port,
        lcm_port=lcm_port,
        memory_limit=args.rerun_memory_limit,
        max_hz=args.hz,
        topic_prefix="/rerun_color_smoke",
    )
    published = 0
    try:
        publisher.start()
        period_s = 1.0 / args.hz if args.hz > 0.0 else 0.0
        for _ in range(args.frames):
            publisher.publish_rgb(lcm_decoded, fov_y_deg=45.0, frame_id="color_smoke_camera")
            published += 1
            if period_s > 0.0:
                time.sleep(period_s)
    finally:
        publisher.stop()

    summary = {
        "ok": True,
        "expected_display": "top red, middle green, bottom blue; colors should not change over time",
        "npy_matches": npy_matches,
        "lcm_matches": lcm_matches,
        "published_frames": published,
        "height": args.height,
        "width": args.width,
        "rerun_grpc_port": grpc_port,
        "rerun_lcm_port": lcm_port,
        "rerun_memory_limit": args.rerun_memory_limit,
        "source": _pixel_summary(source),
        "npy_decoded": _pixel_summary(npy_decoded),
        "lcm_decoded": _pixel_summary(lcm_decoded),
    }
    write_json(args.artifact_dir / "color_smoke_summary.json", summary)
    print(json.dumps({"ok": True, "artifact_dir": str(args.artifact_dir), **summary}, indent=2))


if __name__ == "__main__":
    main()
