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

"""Golden voxels.zlib.v1 vectors: the Python encoder is the reference.

Writes web/shared/fixtures/voxel_frames.json, pinning the encoder's exact
zlib bytes: vitest inflates payload_b64 and compares the unpacked voxel
indices against `voxels`, pytest (test_voxel_encoding.py) re-encodes and
byte-compares against payload_b64, so drift on either side fails a suite.

Regenerate with:  uv run python -m dimos.web.relay_bridge.gen_voxel_fixtures

gen.ts does not write this file: the payloads must be Python zlib output
(CompressionStream compresses to different bytes).
"""

from __future__ import annotations

import base64
import json
from typing import Any

import numpy as np

from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.web.relay_bridge.builtin_codecs import encode_voxels
from dimos.web.relay_bridge.locate import find_web_dir


def cloud_msg(points: list[list[float]]) -> PointCloud2:
    """Fixture-shaped PointCloud2; also used by test_voxel_encoding.py."""
    return PointCloud2.from_numpy(
        np.array(points, dtype=np.float32).reshape(-1, 3), frame_id="world", timestamp=1752576000.5
    )


def voxel_indices(points: list[list[float]], res: float) -> list[list[int]]:
    """Sorted unique floor(point / res) rows, the encoder's quantization."""
    idx = np.floor(np.array(points, dtype=np.float32).reshape(-1, 3) / res).astype(np.int64)
    rows: list[list[int]] = np.unique(idx, axis=0).tolist()
    return rows


# name -> (points, res). At res 0.05 voxel centres sit at odd multiples of
# 0.025. one_chunk's second point shares a voxel with its first (deduped);
# across_chunks has negative and positive chunk coords (chunk = index >> 4,
# so -17 lands in chunk -2 at local 15); coarse_res quantizes at 0.1; empty
# is the zero-record frame that clears a panel.
CASES: dict[str, tuple[list[list[float]], float]] = {
    "one_chunk": (
        [
            [0.025, 0.025, 0.025],
            [0.026, 0.024, 0.026],
            [0.075, 0.025, 0.025],
            [0.425, 0.725, 0.375],
        ],
        0.05,
    ),
    "across_chunks": (
        [
            [-0.025, -0.025, -0.025],
            [0.025, 0.025, 0.025],
            [0.825, -0.825, 0.125],
            [-1.625, 1.575, -0.925],
        ],
        0.05,
    ),
    "coarse_res": ([[0.15, 0.25, 0.05], [0.16, 0.24, 0.06], [1.75, 1.95, 0.55]], 0.1),
    "empty": ([], 0.05),
}


def build_vectors() -> list[dict[str, Any]]:
    vectors: list[dict[str, Any]] = []
    for name, (points, res) in CASES.items():
        encoded = encode_voxels(cloud_msg(points), {"res": res})
        vectors.append(
            {
                "name": name,
                "params": {"res": res},
                "meta": encoded.meta,
                "points": points,
                "voxels": voxel_indices(points, res),
                "payload_b64": base64.b64encode(encoded.payload).decode(),
            }
        )
    return vectors


def main() -> None:
    path = find_web_dir() / "shared" / "fixtures" / "voxel_frames.json"
    path.write_text(json.dumps({"vectors": build_vectors()}, indent=2) + "\n")
    print(f"wrote {path}")


if __name__ == "__main__":
    main()
