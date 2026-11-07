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

"""Where the xArm6's cameras sit, from hand-eye calibration.

Two RealSense D435: one on the kit wrist mount, ``camera_link`` hung on ``link6``, and one on
a post beside the bench, ``chest/camera_link`` in the arm's base, which is the world. Each
has a measured result shipped in ``calibration/``, written by ``xarm6-handeye-calibrate`` and
``xarm6-handeye-calibrate-chest`` on the bench arm. A run on this machine, under
``STATE_DIR/calibration``, takes precedence over the shipped file, so a re-calibration needs
no code change. See docs/usage/xarm6_cameras.md.
"""

from __future__ import annotations

from pathlib import Path

from dimos.manipulation.calibration.store import load_hand_eye, load_hand_eye_file
from dimos.msgs.geometry_msgs.Transform import Transform

# Measured results shipped with the package, in the save_hand_eye layout.
XARM_CALIBRATION_DIR = Path(__file__).resolve().parent / "calibration"
XARM6_WRIST_CAMERA = "xarm6_wrist_camera"
XARM6_CHEST_CAMERA = "xarm6_chest_camera"
# The chest RealSense runs under this blueprint namespace, so its frames and topics do not
# collide with the wrist camera's: chest/camera_link, chest/color_image, chest/pointcloud.
CHEST_NAMESPACE = "chest"


def calibrated_camera(name: str) -> Transform | None:
    """The mount of a passed calibration: this machine's run first, then the shipped result."""
    return load_hand_eye(name) or load_hand_eye_file(XARM_CALIBRATION_DIR / f"{name}.json")


def xarm6_wrist_camera() -> Transform:
    """``link6`` to ``camera_link`` for the D435 on the kit wrist mount."""
    mount = calibrated_camera(XARM6_WRIST_CAMERA)
    assert mount is not None, f"{XARM6_WRIST_CAMERA}.json ships with the package"
    return mount


def xarm6_chest_camera() -> Transform:
    """``world`` to ``chest/camera_link`` for the D435 on the post beside the bench.

    The calibration writes the camera as ``camera_link``, the RealSense module's default; the
    two-camera stack runs the chest camera under the ``chest`` namespace, so its mount is
    renamed to match here, whichever file it came from."""
    mount = calibrated_camera(XARM6_CHEST_CAMERA)
    assert mount is not None, f"{XARM6_CHEST_CAMERA}.json ships with the package"
    return Transform(
        translation=mount.translation,
        rotation=mount.rotation,
        frame_id=mount.frame_id,
        child_frame_id=f"{CHEST_NAMESPACE}/{mount.child_frame_id}",
    )
