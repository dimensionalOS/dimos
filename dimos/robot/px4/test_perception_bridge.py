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

"""PerceptionBridge on the blob detector's arrays. No camera, no GPU, no transports."""

from __future__ import annotations

from collections.abc import Iterator
import math
from typing import Any

import numpy as np
import pytest

from dimos.hardware.gimbal.siyi.replay import AttitudeSample
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.msgs.vision_msgs.Detection2DArray import Detection2DArray
from dimos.robot.px4.perception_bridge import PerceptionBridge, followable
from dimos.robot.px4.sitl import BrightBlobDetector

T0 = 1_700_000_000.0
W, H = 320, 180
VEHICLE_YAW_DEG = 10.0  # NED heading
GIMBAL_YAW_DEG = 30.0
GIMBAL_PITCH_DEG = -20.0
_F = (W / 2) / math.tan(math.radians(81.0) / 2)
INFO = CameraInfo.from_intrinsics(_F, _F, W / 2, H / 2, W, H, "a8_optical")
BLOB = BrightBlobDetector()

Bridge = tuple[PerceptionBridge, dict[str, list[Any]]]


@pytest.fixture
def bridge() -> Iterator[Bridge]:
    b = PerceptionBridge(camera_info=INFO)
    published: dict[str, list[Any]] = {"target_state": [], "target_valid": [], "target_los": []}
    for name, sink in published.items():
        getattr(b, name).publish = sink.append
    # Vehicle 10 m up, heading 10 deg; gimbal 30 deg right, 20 deg down.
    yaw_flu = -math.radians(VEHICLE_YAW_DEG)
    b._on_odometry(
        Odometry(
            ts=T0,
            frame_id="odom",
            child_frame_id="base_link",
            pose=Pose(Vector3(0.0, 0.0, 10.0), Quaternion.from_euler(Vector3(0.0, 0.0, yaw_flu))),
            twist=Twist(),
        )
    )
    b._on_gimbal(AttitudeSample(T0, 0.0, GIMBAL_PITCH_DEG, GIMBAL_YAW_DEG).joint_state())
    b._on_global_pose(PoseStamped(ts=T0, frame_id="home", position=Vector3(0.0, 0.0, 10.0)))
    yield b, published
    b.stop()


def _detections(i: int, square: bool = True, wire: bool = True) -> Detection2DArray:
    """What Detection2DModule publishes for one synthetic frame, by default after the wire."""
    data = np.full((H, W, 3), 96, dtype=np.uint8)
    if square:
        data[H // 2 - 12 : H // 2 + 12, W // 2 - 12 : W // 2 + 12] = 255
    image = Image(data=data, format=ImageFormat.RGB, frame_id="a8_optical", ts=T0 + i / 25)
    msg: Detection2DArray = BLOB.process_image(image).filter(followable).to_ros_detection2d_array()
    if wire:
        msg = Detection2DArray.lcm_decode(msg.lcm_encode())
    return msg


def test_blob_detector_boxes_the_bright_square() -> None:
    msg = _detections(0)
    assert msg.detections_length == 1
    det = msg.detections[0]
    assert det.id == "1"
    assert (det.bbox.center.position.x, det.bbox.center.position.y) == (W / 2, H / 2)
    assert (det.bbox.size_x, det.bbox.size_y) == (24.0, 24.0)
    assert _detections(0, square=False).detections_length == 0


def test_ports(bridge: Bridge) -> None:
    b, _ = bridge
    assert set(b.outputs) == {"target_state", "target_valid", "target_los"}
    assert set(b.inputs) == {"detections", "odometry", "gimbal_attitude", "global_pose"}


@pytest.mark.parametrize("wire", [True, False])
def test_selection_yields_a_valid_target(bridge: Bridge, wire: bool) -> None:
    b, published = bridge
    b.process(_detections(0, wire=wire), now=T0 + 0.01)
    assert published["target_valid"][-1].data is False  # nothing selected yet
    assert b.status()["los"]["reason"] == "no selection"
    assert b.status()["tracks"] == [{"track_id": 1, "class_name": "person"}]

    b.select_track(1)
    state = b.process(_detections(1, wire=wire), now=T0 + 1 / 25 + 0.01)
    assert state.valid, state.reason
    assert published["target_valid"][-1].data is True
    los = published["target_los"][-1]
    # Square at the image centre: azimuth is heading + gimbal yaw; the LOS pose carries the
    # gimbal body yaw as an FLU (counter-clockwise) yaw and the elevation as nose-down pitch.
    assert b.status()["los"]["azimuth_deg"] == pytest.approx(
        VEHICLE_YAW_DEG + GIMBAL_YAW_DEG, abs=0.05
    )
    assert math.degrees(los.yaw) == pytest.approx(-GIMBAL_YAW_DEG, abs=0.05)
    assert math.degrees(los.pitch) == pytest.approx(-GIMBAL_PITCH_DEG, abs=0.05)
    assert los.ts == pytest.approx(T0 + 1 / 25)  # the frame's capture stamp
    target = published["target_state"][-1]
    horizontal = 9.0 / math.tan(math.radians(-GIMBAL_PITCH_DEG))  # 10 m up, aim 1.0
    az = math.radians(VEHICLE_YAW_DEG + GIMBAL_YAW_DEG)
    assert target.x == pytest.approx(horizontal * math.cos(az), abs=0.05)
    assert target.y == pytest.approx(-horizontal * math.sin(az), abs=0.05)  # east is -y in FLU
    assert target.child_frame_id == "target"


def test_selection_persists_while_the_target_is_lost(bridge: Bridge) -> None:
    b, published = bridge
    b.select_track(1)
    b.process(_detections(0), now=T0 + 0.01)
    b.process(_detections(1), now=T0 + 0.05)
    assert published["target_valid"][-1].data is True
    n_los = len(published["target_los"])
    state = b.process(_detections(7, square=False), now=T0 + 0.31)
    assert b.status()["selected_track_id"] == 1
    assert b.status()["los"]["reason"] == "selected target not visible"
    assert len(published["target_los"]) == n_los  # no line of sight without an observation
    assert state.valid  # the Kalman filter coasts within max_meas_age_s


def test_untracked_detections_are_not_selectable(bridge: Bridge) -> None:
    b, _ = bridge
    msg = _detections(0)
    msg.detections[0].id = "-1"  # what a detector without a tracker reports
    b.process(msg, now=T0 + 0.01)
    assert b.status()["tracks"] == []


def _odometry(ts: float) -> Odometry:
    return Odometry(
        ts=ts,
        frame_id="odom",
        child_frame_id="base_link",
        pose=Pose(Vector3(0.0, 0.0, 10.0), Quaternion()),
        twist=Twist(),
    )


def test_frame_outside_the_buffered_attitude_is_stale(bridge: Bridge) -> None:
    b, _ = bridge
    b.select_track(1)
    # The odometry stopped at T0: a frame one second on has no heading to go with it.
    state = b.process(_detections(25), now=T0 + 1.01)
    assert not state.valid and b.status()["los"]["reason"] == "vehicle attitude stale"
    # Vehicle stamps 10 s ahead of the frame clock: the lookup would clamp to the oldest sample.
    for i in range(3):
        b._on_odometry(_odometry(T0 + 10.0 + i / 30))
    b._on_gimbal(AttitudeSample(T0 + 1.04, 0.0, GIMBAL_PITCH_DEG, GIMBAL_YAW_DEG).joint_state())
    b.process(_detections(26), now=T0 + 1.05)
    assert b.status()["los"]["reason"] == "vehicle attitude stale"


def test_estimator_config_is_validated() -> None:
    with pytest.raises(ValueError, match="agl_source"):
        PerceptionBridge(camera_info=INFO, estimator={"agl_source": "bogus"})
