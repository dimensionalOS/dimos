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

import pytest

from dimos.robot.deeprobotics.m20 import basic_server
from dimos.robot.deeprobotics.m20.joint_state import urdf_joints_from_motor_status

# A 1002/4 report captured from a real M20 lying down (MotionState 4).
LYING_FRAME = bytes.fromhex("eb91eb90930300000100000000000000") + (
    b'{"PatrolDevice":{"Type":1002,"Command":4,"Time":"2026-11-04 23:08:57.073","Items":{"MotionStatus":'
    b'{"Roll":-0.0038725025951862335,"Pitch":-0.018899837508797646,"Yaw":-0.8044768571853638,'
    b'"OmegaZ":0.0018627026583999395,"LinearX":0.0,"LinearY":0.0,"Height":0.0990581065416336,"Payload":0.0,'
    b'"RemainMile":6.630000114440918},"MotorStatus":{"LeftFrontHipX":0.6066128015518188,'
    b'"LeftFrontHipY":1.3911378383636475,"LeftFrontKnee":0.16135327517986298,"LeftFrontWheel":0.02052883803844452,'
    b'"RightFrontHipX":-0.6164506077766418,"RightFrontHipY":-1.3813812732696533,'
    b'"RightFrontKnee":-0.15967299044132233,"RightFrontWheel":0.07643042504787445,'
    b'"LeftBackHipX":-0.5917340517044067,"LeftBackHipY":-1.387388825416565,"LeftBackKnee":-0.16378337144851685,'
    b'"LeftBackWheel":0.009547926485538483,"RightBackHipX":0.6267040371894836,"RightBackHipY":1.3704233169555664,'
    b'"RightBackKnee":0.13960884511470795,"RightBackWheel":0.030264783650636673}}}}'
)


def test_lying_report_maps_to_a_symmetric_folded_urdf_pose() -> None:
    device = basic_server.decode_frame(LYING_FRAME)
    assert device is not None
    assert (device["Type"], device["Command"]) == (1002, 4)

    angles, wheel_speeds = urdf_joints_from_motor_status(device["Items"]["MotorStatus"])

    # Fully folded legs: front and hind knees fold opposite ways, left and right hips mirror each other.
    for leg, hipx_sign, fold_sign in (("fl", 1, 1), ("fr", -1, 1), ("hl", 1, -1), ("hr", -1, -1)):
        assert angles[f"{leg}_hipx_joint"] == pytest.approx(hipx_sign * 0.17, abs=0.03)
        assert angles[f"{leg}_hipy_joint"] == pytest.approx(-fold_sign * 0.90, abs=0.03)
        assert angles[f"{leg}_knee_joint"] == pytest.approx(fold_sign * 2.63, abs=0.03)
    # The right wheels' motors spin opposite the URDF axis.
    assert wheel_speeds["fl_wheel_joint"] == pytest.approx(0.0205, abs=1e-3)
    assert wheel_speeds["fr_wheel_joint"] == pytest.approx(-0.0764, abs=1e-3)
