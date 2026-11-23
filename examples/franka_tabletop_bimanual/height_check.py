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

"""Put one arm's fingertips 3.0 cm above the table, as that arm measures it, and hold.

The arm's wrist camera measures the tabletop under a free spot on its side, then the
fingertips (pointing down) go to 3.0 cm above that. Measure the real gap with a ruler.

    ~/dimos-franka/.venv/bin/python height_check.py right|left [up]
"""

import sys

import numpy as np

from handoff import (
    APPROACH,
    FINGERTIP_REACH,
    LOOK_HEIGHT,
    READY,
    Arms,
    bearing,
    camera_optical,
    other,
    seeds,
    solve,
)
import tabletop

SPOTS = {"right": (0.42, -0.10), "left": (0.47, 0.88)}
GAP = 0.03


def main() -> None:
    side = sys.argv[1]
    arms = Arms(dry=False)
    try:
        if len(sys.argv) > 2:  # back up and home
            arms.straight(side, APPROACH, "up")
            arms.move(side, READY, "to ready")
            return
        arms.add_table()
        x, y = SPOTS[side]
        look = np.eye(4)
        look[:3, :3] = np.diag([1.0, -1.0, -1.0])
        look[:3, 3] = [x, y, LOOK_HEIGHT]
        q = solve(
            arms.model,
            side,
            arms.q(other(side)),
            f"{side}/fr3_hand",
            camera_optical(side),
            look,
            seeds(arms.q(side), bearing(side, look[:3, 3])),
            free_yaw=True,
        )
        arms.move(side, q, "to look")
        found, _ = tabletop.look(arms.app, arms.model, cameras=(f"{side}_wrist",))
        cam = f"{side}_wrist"
        # The table under the spot, from this camera's own plane fit.
        info = getattr(arms.app, cam).peek_stream("camera_info", 5.0)
        depth = (
            np.asarray(getattr(arms.app, cam).peek_stream("depth_image", 5.0).data).astype(float)
            / 1000.0
        )
        T = tabletop.world_from_optical(cam, {side: arms.q(side)}, arms.model)
        n, d = tabletop.fit_table(tabletop.world_points(depth, np.asarray(info.K).reshape(3, 3), T))
        table_z = float((d - n[0] * x - n[1] * y) / n[2])
        print(f"  {side} camera: the table at ({x}, {y}) is at z = {table_z * 1000:+.1f} mm")
        tcp = np.eye(4)
        target = np.eye(4)
        target[:3, :3] = np.diag([1.0, -1.0, -1.0])
        target[:3, 3] = [x, y, table_z + GAP + FINGERTIP_REACH]
        above = target.copy()
        above[2, 3] += APPROACH
        q_above = solve(
            arms.model,
            side,
            arms.q(other(side)),
            f"{side}/fr3_hand_tcp",
            tcp,
            above,
            seeds(arms.q(side), bearing(side, above[:3, 3])),
            free_yaw=True,
        )
        arms.move(side, q_above, "above the spot")
        arms.straight(side, -APPROACH, "down")
        print(
            f"\n  {side} fingertips should now be {GAP * 100:.1f} cm above the table: measure the real gap"
        )
    finally:
        arms.close()


if __name__ == "__main__":
    main()
