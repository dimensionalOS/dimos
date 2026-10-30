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

"""The A8 stand-ins of px4-sitl-follow."""

from __future__ import annotations

import math
import time
from typing import Any

import pytest

from dimos.hardware.gimbal.siyi.frame import GIMBAL_JOINTS, GIMBAL_TARGET_JOINTS, joints
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.px4.sitl import BrightBlobDetector, FakeA8, FakeA8Camera


def test_fake_camera_streams_a_square_the_blob_detector_finds_in_the_middle() -> None:
    cam = FakeA8Camera(fps=50.0)
    frames: list[Any] = []
    cam.color_image.publish = frames.append
    try:
        cam.start()
        deadline = time.monotonic() + 2.0
        while len(frames) < 3 and time.monotonic() < deadline:
            time.sleep(0.01)
    finally:
        cam.stop()
    assert len(frames) >= 3
    image = frames[0]
    assert (image.frame_id, image.data.shape) == ("a8_optical", (180, 320, 3))
    (det,) = BrightBlobDetector().process_image(image).detections
    assert det.bbox == (148.0, 78.0, 172.0, 102.0)


def test_fake_gimbal_slews_toward_the_target_at_the_a8_rate() -> None:
    a8 = FakeA8(initial_pitch_deg=-20.0, initial_yaw_deg=30.0)
    sent: list[JointState] = []
    a8.gimbal_attitude.publish = sent.append
    target = [math.radians(-30.0), math.radians(60.0)]
    try:
        a8._on_target(JointState(name=list(GIMBAL_TARGET_JOINTS), position=target))
        for _ in range(4):
            a8._step()
    finally:
        a8.stop()
    angles = [[math.degrees(a) for a in joints(m, GIMBAL_JOINTS) or []] for m in sent]
    # 90 deg/s at 10 Hz: at most 9 deg a step.
    assert [a[1] for a in angles] == pytest.approx([-29.0, -30.0, -30.0, -30.0])
    assert [a[2] for a in angles] == pytest.approx([39.0, 48.0, 57.0, 60.0])
