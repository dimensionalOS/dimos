# Copyright 2026 Dimensional Inc.
# SPDX-License-Identifier: Apache-2.0

import importlib.util
import math
from pathlib import Path

import numpy as np

from dimos.message_utils import geometry, image
from dimos.messages.builtin_interfaces.msg import Time
from dimos.messages.geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from dimos.messages.sensor_msgs.msg import JointState
from dimos.messages.std_msgs.msg import Header

ROOT = Path(__file__).resolve().parents[1]


def _load(relative):
    spec = importlib.util.spec_from_file_location(Path(relative).stem, ROOT / relative)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


GRAYSCALE = _load("examples/camera_grayscale.py")
SIMPLE_ROBOT = _load("examples/simplerobot/simplerobot.py")
G1_REPLAY = _load("scripts/g1_replay.py")


def test_grayscale_demo_preserves_source_header_and_rgb_luminance(mocker):
    module = GRAYSCALE.Grayscale()
    publish = mocker.patch.object(module.gray_image, "publish")
    pixels = np.array([[[255, 0, 0], [0, 255, 0], [0, 0, 255]]], dtype=np.uint8)
    source = image.from_array(pixels, encoding="rgb8", header=Header(Time(17, 1), "camera"))
    try:
        module._publish_grayscale(source)
        result = publish.call_args.args[0]
        np.testing.assert_array_equal(image.view(result), [[76, 150, 29]])
        assert result.encoding == "mono8"
        assert (result.header.stamp.sec, result.header.stamp.nanosec, result.header.frame_id) == (
            17,
            1,
            "camera",
        )
        np.testing.assert_array_equal(image.view(source), pixels)
    finally:
        module.stop()


def test_simple_robot_keeps_its_unicycle_integration_and_command_timeout(mocker):
    pose = Pose(Point(0.0, 0.0, 2.0), Quaternion(0.0, 0.0, 0.0, 1.0))
    result = SIMPLE_ROBOT.apply_twist(
        pose, Twist(Vector3(1.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.5)), 0.5
    )
    np.testing.assert_allclose(
        [result.position.x, result.position.y, result.position.z],
        [0.5 * math.cos(0.25), 0.5 * math.sin(0.25), 2.0],
    )
    assert geometry.yaw(result.orientation) == 0.25
    robot = SIMPLE_ROBOT.SimpleRobot(update_rate=2.0, cmd_timeout=0.5)
    publish = mocker.patch.object(robot.pose, "publish")
    mocker.patch.object(SIMPLE_ROBOT.time, "time", return_value=17.125)
    robot._vel = Twist(Vector3(1.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0))
    robot._vel_time = 16.0
    try:
        robot._update()
        value = publish.call_args.args[0]
        assert (value.pose.position.x, value.pose.position.y) == (0.0, 0.0)
        assert (value.header.stamp.sec, value.header.stamp.nanosec, value.header.frame_id) == (
            17,
            125_000_000,
            "world",
        )
    finally:
        robot.stop()


def test_g1_replay_builds_generated_numeric_joint_commands(mocker):
    mocker.patch.object(G1_REPLAY.message_time, "header_now", return_value=Header(Time(17, 1), ""))
    positions = [float(i) for i in range(29)]
    value = G1_REPLAY.make_joint_state(positions)
    positions[0] = 99.0
    assert type(value) is JointState
    assert (len(value.name), value.position[0], value.position[-1]) == (29, 0.0, 28.0)
    np.testing.assert_array_equal(value.velocity, np.zeros(29))
    np.testing.assert_array_equal(value.effort, np.zeros(29))
    assert value.header.stamp.nanosec == 1
