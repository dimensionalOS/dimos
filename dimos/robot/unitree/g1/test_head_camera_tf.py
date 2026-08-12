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

from __future__ import annotations

import numpy as np
import pytest

from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.perception.fiducial.marker_latch_module import MarkerLatchModule
from dimos.robot.unitree.g1.head_camera_tf import G1HeadCameraTf


@pytest.mark.parametrize("module_cls", [G1HeadCameraTf, MarkerLatchModule])
def test_lifecycle_methods_stay_rpc(module_cls):
    # Overriding start/stop without @rpc drops the RPC registration, so the
    # coordinator falls back to fetching the attribute — which pickles the
    # bound method, and therefore the module's threading.Lock. The worker dies
    # with "cannot pickle '_thread.lock' object".
    for name in ("start", "stop"):
        assert getattr(module_cls, name).__rpc__, f"{module_cls.__name__}.{name} must be @rpc"


def test_stream_size_matches_the_captured_intrinsics():
    # Marker detection skips any frame whose size differs from CameraInfo, with
    # no error — the camera looks alive and detection just never fires.
    from dimos.robot.unitree.g1.head_camera import CAMERA_STREAM_CONFIG, head_camera_info

    info = head_camera_info()
    assert info is not None, "intrinsics artifact should be committed"
    assert (info.width, info.height) == (
        CAMERA_STREAM_CONFIG["width"],
        CAMERA_STREAM_CONFIG["height"],
    )


def test_camera_transform_tracks_the_waist():
    mod = G1HeadCameraTf()
    try:
        rest = mod._transform()
        # Rest pose: the D435 sits forward and up from the pelvis, pitched down.
        assert rest.frame_id == "pelvis"
        assert rest.child_frame_id == "d435_link"
        assert rest.translation.z == pytest.approx(0.4739, abs=1e-3)

        mod._on_joint_state(JointState(name=["g1/waist_yaw"], position=[np.pi / 2]))
        yawed = mod._transform()

        # Yawing the waist swings the camera; a static transform would not move.
        assert yawed.translation.x != pytest.approx(rest.translation.x, abs=1e-3)
    finally:
        mod.stop()
