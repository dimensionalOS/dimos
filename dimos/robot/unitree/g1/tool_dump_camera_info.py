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

"""Dump the G1 head camera's live intrinsics to the YAML the demo loads.

Marker pose estimation scales linearly with focal length, so guessed
intrinsics silently put the pot at the wrong distance. Run this once per
robot, with the camera connected, and commit the result.

    .venv/bin/python -m dimos.robot.unitree.g1.tool_dump_camera_info
"""

from __future__ import annotations

from pathlib import Path
import sys

import yaml

from dimos.hardware.sensors.camera.realsense.camera import RealSenseCamera
from dimos.robot.unitree.g1.head_camera import HEAD_CAMERA_INFO_YAML

_TIMEOUT_S = 15.0


def main() -> int:
    camera = RealSenseCamera(enable_depth=False, enable_pointcloud=False)
    camera.start()
    try:
        info = camera.get_color_camera_info()
    finally:
        camera.stop()

    if info is None:
        print("No CameraInfo from the device — is the RealSense plugged in?", file=sys.stderr)
        return 1

    out = Path(HEAD_CAMERA_INFO_YAML)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(
        yaml.safe_dump(
            {
                "image_width": info.width,
                "image_height": info.height,
                "camera_name": "g1_head_d435",
                "distortion_model": info.distortion_model or "plumb_bob",
                "camera_matrix": {"rows": 3, "cols": 3, "data": list(info.K)},
                "distortion_coefficients": {
                    "rows": 1,
                    "cols": len(info.D),
                    "data": list(info.D),
                },
                "projection_matrix": {"rows": 3, "cols": 4, "data": list(info.P)},
                "rectification_matrix": {
                    "rows": 3,
                    "cols": 3,
                    "data": [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
                },
            },
            sort_keys=False,
        )
    )
    print(f"Wrote {out} ({info.width}x{info.height}, fx={info.K[0]:.1f})")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
