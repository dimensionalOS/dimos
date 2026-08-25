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

"""Golden costmap.zlib.v1 vectors: the Python encoder is the reference.

Writes web/shared/fixtures/costmap_frames.json, pinning the encoder's exact
zlib bytes: vitest inflates payload_b64 and byte-compares against grid_b64,
pytest (test_costmap_encoding.py) re-encodes and byte-compares against
payload_b64, so drift on either side fails a suite.

Regenerate with:  uv run python -m dimos.web.relay_bridge.gen_costmap_fixtures

gen.ts does not write this file: the payloads must be Python zlib output
(CompressionStream compresses to different bytes).
"""

from __future__ import annotations

import base64
import json
from typing import TYPE_CHECKING, Any, cast

import numpy as np

from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid
from dimos.web.relay_bridge.locate import find_web_dir
from dimos.web.relay_bridge.relay_bridge_module import _encode_costmap

if TYPE_CHECKING:
    from dimos.web.relay_bridge.relay_bridge_module import RelayBridgeModule


def grid_msg(rows: list[list[int]], res: float, x: float, y: float, yaw: float) -> OccupancyGrid:
    """Fixture-shaped OccupancyGrid; also used by test_costmap_encoding.py."""
    quat = Quaternion.from_euler(Vector3(0.0, 0.0, yaw))
    origin = Pose(x, y, 0.0, quat.x, quat.y, quat.z, quat.w)
    grid = np.array(rows, dtype=np.int8)
    return OccupancyGrid(grid=grid, resolution=res, origin=origin, ts=1752576000.5)


# name -> (rows, res, origin x, origin y, origin yaw). small_map covers every
# value class of the wire contract (-1 unknown, 0 free, graded, 100 lethal).
CASES: dict[str, tuple[list[list[int]], float, float, float, float]] = {
    "small_map": (
        [
            [-1, -1, 0, 0, 0],
            [-1, 0, 1, 50, 0],
            [0, 0, 99, 100, 0],
            [0, -1, -1, 0, 100],
        ],
        0.05,
        -1.25,
        2.5,
        0.0,
    ),
    "with_yaw": (
        [
            [0, 100, -1],
            [0, 50, 0],
            [-1, 0, 25],
        ],
        0.1,
        1.5,
        -0.75,
        0.25,
    ),
    "single_row": ([[0, -1, 1, 99, 100, 0]], 0.25, -0.5, 0.125, 0.0),
}


def build_vectors() -> list[dict[str, Any]]:
    vectors: list[dict[str, Any]] = []
    for name, (rows, res, x, y, yaw) in CASES.items():
        msg = grid_msg(rows, res, x, y, yaw)
        encoded = _encode_costmap(cast("RelayBridgeModule", None), msg)  # module unused
        assert encoded is not None
        payload, meta = encoded
        cells = np.where(msg.grid == -1, 255, msg.grid).astype(np.uint8)
        vectors.append(
            {
                "name": name,
                "meta": meta,
                "grid_b64": base64.b64encode(cells.tobytes()).decode(),
                "payload_b64": base64.b64encode(payload).decode(),
            }
        )
    return vectors


def main() -> None:
    path = find_web_dir() / "shared" / "fixtures" / "costmap_frames.json"
    path.write_text(json.dumps({"vectors": build_vectors()}, indent=2) + "\n")
    print(f"wrote {path}")


if __name__ == "__main__":
    main()
