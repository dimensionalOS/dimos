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

"""Re-measure the tripod's reference board (the 4x3 ChArUco "marker") after moving it.

The right wrist camera looks straight down at it from 35 cm; its pose in the world (the
right arm's base) replaces arm1_to_marker in bimanual_cal.json, and its outline replaces
the masked areas in fixtures.json. Backups of both go to cal_backup_<time>/.

    ~/dimos-franka/.venv/bin/python remeasure_marker_board.py
"""

from __future__ import annotations

import json
from pathlib import Path
import shutil
import time

import cv2
import numpy as np
from scipy.spatial.transform import Rotation

import bimanual_cal as bc
import handoff as ho
import marker_demo as md
import tabletop as tt

HERE = Path(__file__).parent


def detect_board(app, cam: str, frames: int = 8):
    mod = getattr(app, cam)
    info = mod.peek_stream("camera_info", 5.0)
    K, dist = np.asarray(info.K).reshape(3, 3), np.asarray(info.D, float)
    imgs = [
        np.asarray(mod.peek_stream("color_image", 5.0).data).astype(np.float32)
        for _ in range(frames)
    ]
    img = cv2.cvtColor(np.mean(imgs, 0).astype(np.uint8), cv2.COLOR_RGB2BGR)
    return bc.detect("marker", img, K, dist)


def main() -> None:
    arms = ho.Arms(dry=False)
    try:
        T_opt, n, err = detect_board(arms.app, "scene")
        if T_opt is None:
            raise SystemExit(f"the tripod does not see the board whole ({n} corners)")
        w, h = bc.SIZE["marker"]
        rough = tt._scene_from_calibration() @ T_opt
        centre = (rough @ np.r_[w / 2, h / 2, 0, 1])[:3]
        print(f"tripod: board centre roughly ({centre[0]:.3f}, {centre[1]:.3f})")
        md.look_with(arms, "right", centre, 0.35)
        time.sleep(1.0)
        joints = {s: arms.q(s) for s in ho.SIDES}
        T_opt, n, err = detect_board(arms.app, "right_wrist")
        if T_opt is None:
            arms.move("right", ho.READY, "to ready", md.FREE)
            raise SystemExit(f"the right wrist camera does not see the board whole ({n} corners)")
        T = tt.world_from_optical("right_wrist", joints, arms.model) @ T_opt
        arms.move("right", ho.READY, "to ready", md.FREE)
    finally:
        arms.close()
    centre = (T @ np.r_[w / 2, h / 2, 0, 1])[:3]
    tilt = np.degrees(np.arccos(abs(T[2, 2])))
    print(
        f"right wrist ({n} corners, {err:.2f} px): board centre ({centre[0]:.3f}, {centre[1]:.3f}, "
        f"{centre[2] * 1000:.1f} mm up), {tilt:.1f} deg from flat; {np.linalg.norm(centre[:2] - (rough @ np.r_[w / 2, h / 2, 0, 1])[:2]) * 1000:.0f} mm from the tripod's rough guess"
    )
    backup = HERE / f"cal_backup_{time.strftime('%Y-%m-%d_%H%M')}"
    backup.mkdir(exist_ok=True)
    for f in ("bimanual_cal.json", "fixtures.json"):
        shutil.copy(HERE / f, backup / f)
    cal = json.loads((HERE / "bimanual_cal.json").read_text())
    cal["arm1_to_marker"] = {
        "translation_m": T[:3, 3].tolist(),
        "rotation_xyzw": Rotation.from_matrix(T[:3, :3]).as_quat().tolist(),
    }
    cal["arm1_marker_centre"] = centre.tolist()
    (HERE / "bimanual_cal.json").write_text(json.dumps(cal, indent=1))
    corners = [
        (T @ np.r_[x, y, 0, 1])[:2].round(4).tolist() for x, y in ((0, 0), (w, 0), (w, h), (0, h))
    ]
    (HERE / "fixtures.json").write_text(json.dumps({"marker": corners}, indent=1))
    print(f"saved; the old files are in {backup.name}/ (the 8x5 board is no longer masked)")


if __name__ == "__main__":
    main()
