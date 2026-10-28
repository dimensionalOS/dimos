# Copyright 2026 Dimensional Inc.
# SPDX-License-Identifier: Apache-2.0

import numpy as np

from dimos.messages.sensor_msgs.msg import CameraInfo
from dimos.messages.tf2_msgs.msg import TFMessage
from dimos.robot.deeprobotics.m20.camera import M20CameraRelay


def test_camera_metadata_preserves_mounts_and_shared_frame_time(mocker):
    module = M20CameraRelay()
    tf = mocker.patch.object(module.tf, "publish")
    front = mocker.patch.object(module.front_camera_info, "publish")
    rear = mocker.patch.object(module.rear_camera_info, "publish")
    mocker.patch.object(module._stop_event, "is_set", side_effect=[False, True])
    mocker.patch.object(module._stop_event, "wait")
    mocker.patch("dimos.robot.deeprobotics.m20.camera.time.time", return_value=17.125)
    try:
        module._publish_camera_metadata()
        transforms = tf.call_args.args[0]
        assert type(transforms) is TFMessage
        assert [(t.header.frame_id, t.child_frame_id) for t in transforms.transforms] == [
            ("base_link", "front_camera_link"),
            ("front_camera_link", "front_camera_optical"),
            ("base_link", "rear_camera_link"),
            ("rear_camera_link", "rear_camera_optical"),
        ]
        assert [(t.header.stamp.sec, t.header.stamp.nanosec) for t in transforms.transforms] == [
            (17, 125_000_000)
        ] * 4
        position = transforms.transforms[0].transform.translation
        np.testing.assert_allclose([position.x, position.y, position.z], [0.37646, 0.0, 0.03738])
        for value, frame in [
            (front.call_args.args[0], "front_camera_optical"),
            (rear.call_args.args[0], "rear_camera_optical"),
        ]:
            assert type(value) is CameraInfo
            assert value.header.frame_id == frame
            assert (value.header.stamp.sec, value.header.stamp.nanosec) == (17, 125_000_000)
            assert (value.width, value.height) == (800, 600)
            assert value.k[0] == 379.375
    finally:
        module.stop()
