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

from collections.abc import Iterator
import io
import json
import time
from unittest.mock import MagicMock

import numpy as np
from PIL import Image as PILImage
import pytest

from dimos.message_utils import geometry, image as image_utils, pointcloud, time as message_time
from dimos.messages.geometry_msgs.msg import (
    Point,
    Pose,
    PoseStamped,
    Quaternion,
    Transform,
    TransformStamped,
    Vector3,
)
from dimos.messages.sensor_msgs.msg import JointState
from dimos.messages.std_msgs.msg import Header
from dimos.messages.tf2_msgs.msg import TFMessage
from dimos.robot.raw_robot_bridge import (
    Deadman,
    RawRobotBridge,
    RawTopics,
    depth_f32,
    jpeg_bytes,
    odom_json,
    xyz_f32,
)

ENDPOINT = "tcp/127.0.0.1:17448"


def wait_for(condition, timeout_s: float = 5.0) -> None:  # type: ignore[no-untyped-def]
    deadline = time.monotonic() + timeout_s
    while not condition():
        assert time.monotonic() < deadline, "timed out"
        time.sleep(0.02)


@pytest.fixture
def topics() -> Iterator[tuple[RawTopics, RawTopics]]:
    robot = RawTopics(ENDPOINT, listen=True)
    client = RawTopics(ENDPOINT, listen=False)
    try:
        yield robot, client
    finally:
        client.close()
        robot.close()


def test_client_receives_sensor_topics_and_robot_receives_commands(
    topics: tuple[RawTopics, RawTopics],
) -> None:
    robot, client = topics
    got: list[tuple[str, bytes, float | None]] = []
    client.subscribe("odom/json", lambda b, ts: got.append(("odom", b, ts)))
    client.subscribe("camera/jpeg", lambda b, ts: got.append(("camera", b, ts)))
    commands: list[bytes] = []
    robot.subscribe("cmd_vel/json", lambda b, _ts: commands.append(b))
    time.sleep(0.5)  # subscriptions propagate over the link

    pose = PoseStamped(
        Header(message_time.time_from_seconds(3.5), ""),
        Pose(Point(*(1.0, 2.0, 0.5)), Quaternion(*(0.0, 0.0, 0.0, 1.0))),
    )
    robot.put("odom/json", odom_json(pose))
    robot.put("camera/jpeg", b"\xff\xd8jpeg", ts=4.25)
    client.put("cmd_vel/json", json.dumps({"vx": 0.3, "vy": 0.0, "wz": 0.1, "t": 1.0}))
    wait_for(lambda: len(got) == 2 and len(commands) == 1)

    odom = json.loads(next(b for k, b, _ in got if k == "odom"))
    assert (odom["t"], odom["x"], odom["y"], odom["z"], odom["qw"]) == (3.5, 1.0, 2.0, 0.5, 1.0)
    assert next((b, ts) for k, b, ts in got if k == "camera") == (b"\xff\xd8jpeg", 4.25)
    assert json.loads(commands[0])["vx"] == 0.3


def test_encoders_are_plain_formats() -> None:
    frame = np.zeros((6, 8, 3), dtype=np.uint8)
    frame[:, :, 0] = 200
    decoded = PILImage.open(
        io.BytesIO(
            jpeg_bytes(
                image_utils.from_array(
                    frame, encoding="bgr8", header=Header(message_time.time_from_seconds(1.0), "")
                )
            )
        )
    )
    assert decoded.format == "JPEG" and decoded.size == (8, 6)

    points = np.arange(12, dtype=np.float32).reshape(4, 3)
    raw = xyz_f32(
        pointcloud.from_xyz(points, header=Header(message_time.time_from_seconds(2.0), ""))
    )
    assert np.frombuffer(raw, dtype="<f4").reshape(-1, 3).tolist() == points.tolist()

    fields = json.loads(
        odom_json(
            PoseStamped(
                Header(message_time.time_from_seconds(9.0), ""),
                Pose(Point(*(0, 0, 0)), Quaternion(*(0, 0, 0, 1))),
            )
        )
    )
    assert list(fields) == ["t", "x", "y", "z", "qx", "qy", "qz", "qw"]


def test_deadman_holds_then_stops_and_caps_the_hold() -> None:
    deadman = Deadman(max_s=0.2)
    deadman.set(json.dumps({"vx": 0.5, "wz": -0.2, "t": 0.05}))
    assert deadman.current() == (0.5, 0.0, -0.2)
    time.sleep(0.08)
    assert deadman.current() == (0.0, 0.0, 0.0)

    deadman.set(json.dumps({"vx": 1.0, "t": 60}))  # capped to max_s
    time.sleep(0.25)
    assert deadman.current() == (0.0, 0.0, 0.0)

    with pytest.raises(ValueError):
        deadman.set(b"not json")


def test_deadman_rejects_non_finite_and_clamps_speed() -> None:
    deadman = Deadman(max_s=2.0, max_linear=1.0, max_angular=1.5)
    for bad in (
        '{"vx": NaN, "t": 1}',
        '{"vx": Infinity, "t": 1}',
        '{"wz": -Infinity, "t": 1}',
        '{"vx": 1, "t": NaN}',
    ):
        with pytest.raises(ValueError):
            deadman.set(bad)
    assert deadman.current() == (0.0, 0.0, 0.0)  # nothing armed by the bad packets
    deadman.set(json.dumps({"vx": 1e308, "vy": -7.0, "wz": 40.0, "t": 1}))
    assert deadman.current() == (1.0, -1.0, 1.5)


@pytest.mark.parametrize(
    "kwargs", [{"max_linear": -1.0}, {"max_angular": 0.0}, {"max_s": float("inf")}]
)
def test_deadman_rejects_non_positive_limits(kwargs: dict[str, float]) -> None:
    with pytest.raises(ValueError, match="finite positive limit"):
        Deadman(**kwargs)


@pytest.fixture
def bridge() -> Iterator[RawRobotBridge]:
    module = RawRobotBridge(
        camera_frame="wrist_optical",
        ee_frame="link_tcp",
        gripper_joint="arm/gripper",
        gripper_range=(0.0, 0.85),
    )
    module._topics = MagicMock()
    module.cmd_vel = MagicMock()
    module.ee_twist_command = MagicMock()
    module.gripper_command = MagicMock()
    yield module
    module._topics = None
    module.stop()


def _drive(module: RawRobotBridge, ticks: int) -> None:
    stop = module._stop
    module._stop = MagicMock()
    module._stop.wait.side_effect = [False] * ticks + [True]
    module._drive()
    module._stop = stop


def _published(module: RawRobotBridge, key: str) -> list[dict]:  # type: ignore[type-arg]
    return [json.loads(c.args[1]) for c in module._topics.put.call_args_list if c.args[0] == key]


def test_arm_twist_is_held_clamped_then_zeroed(bridge: RawRobotBridge) -> None:
    bridge._arm.set(json.dumps({"vx": 0.05, "vz": 9, "wz": -9, "t": 60}))
    _drive(bridge, 2)
    twists = [c.args[0] for c in bridge.ee_twist_command.publish.call_args_list]
    assert [
        (*geometry.vector_array(t.twist.linear), *geometry.vector_array(t.twist.angular))
        for t in twists
    ] == [(0.05, 0, 0.1, 0, 0, -0.5)] * 2
    bridge._arm.until = 0.0  # deadline passed
    _drive(bridge, 2)
    last = bridge.ee_twist_command.publish.call_args_list[2:]
    assert len(last) == 1 and not any(geometry.vector_array(last[0].args[0].twist.linear))
    bridge.cmd_vel.publish.assert_not_called()  # the base deadman is independent


@pytest.mark.parametrize("payload", [b"{}", b'{"opening": 1.5}', b'{"opening": NaN}', b"nope"])
def test_bad_gripper_commands_are_dropped(bridge: RawRobotBridge, payload: bytes) -> None:
    bridge._command(bridge._on_gripper)(payload, None)
    bridge.gripper_command.publish.assert_not_called()


def test_gripper_opening_passes_through(bridge: RawRobotBridge) -> None:
    bridge._command(bridge._on_gripper)(json.dumps({"opening": 0.25, "t": 1}).encode(), None)
    assert bridge.gripper_command.publish.call_args.args[0].data == pytest.approx(0.25)


def test_state_reports_joints_tcp_pose_and_normalized_gripper(bridge: RawRobotBridge) -> None:
    tcp = TransformStamped(
        Header(message_time.time_from_seconds(1.0), "world"),
        "link_tcp",
        Transform(Vector3(0.5, 0.0, 0.3), Quaternion(1, 0, 0, 0)),
    )
    hidden = TransformStamped(
        Header(message_time.time_from_seconds(1.0), "world"),
        "cup",
        Transform(Vector3(0, 0, 0), Quaternion(0, 0, 0, 1)),
    )
    bridge._on_tf(TFMessage([tcp, hidden]))
    bridge._on_joint_state(
        JointState(
            header=Header(message_time.time_from_seconds(7.0), ""),
            name=["j1", "arm/gripper"],
            position=np.asarray([0.1, 0.425], dtype=np.float64),
            velocity=np.asarray([0.5, 0.0], dtype=np.float64),
            effort=np.asarray([], dtype=np.float64),
        )
    )
    (state,) = _published(bridge, "arm/state/json")
    assert state == {
        "t": 7.0,
        "joint_names": ["j1"],
        "positions": [0.1],
        "velocities": [0.5],
        "ee_pose": {"frame": "world", "xyz": [0.5, 0.0, 0.3], "quaternion_xyzw": [1, 0, 0, 0]},
        "gripper_opening": pytest.approx(0.5),
    }
    assert [c.args[0] for c in bridge._topics.put.call_args_list] == ["arm/state/json"]


def test_unconfigured_robot_reports_plain_joints() -> None:
    module = RawRobotBridge()
    module._topics = MagicMock()
    try:
        module._on_joint_state(
            JointState(
                header=Header(message_time.time_from_seconds(3.0), ""),
                name=["a", "b"],
                position=np.asarray([1.0, 2.0], dtype=np.float64),
                velocity=np.asarray([], dtype=np.float64),
                effort=np.asarray([], dtype=np.float64),
            )
        )
        (state,) = _published(module, "arm/state/json")
        assert state == {
            "t": 3.0,
            "joint_names": ["a", "b"],
            "positions": [1.0, 2.0],
            "velocities": [0.0, 0.0],
        }
        module._on_tf(
            TFMessage(
                [
                    TransformStamped(
                        Header(message_time.time_from_seconds(time.time()), "world"),
                        "link_tcp",
                        Transform(Vector3(0, 0, 0), Quaternion(0, 0, 0, 1)),
                    )
                ]
            )
        )
        assert len(module._topics.put.call_args_list) == 1
    finally:
        module._topics = None
        module.stop()


def test_camera_pose_and_depth_keep_capture_timestamps(bridge: RawRobotBridge) -> None:
    camera = TransformStamped(
        Header(message_time.time_from_seconds(5.0), "world"),
        "wrist_optical",
        Transform(Vector3(1, 2, 3), Quaternion(0, 0, 0, 1)),
    )
    bridge._on_tf(TFMessage([camera]))
    (pose,) = _published(bridge, "camera_pose/json")
    assert pose == {"t": 5.0, "frame": "world", "xyz": [1, 2, 3], "quaternion_xyzw": [0, 0, 0, 1]}
    bridge._on_depth(
        image_utils.from_array(
            np.ones((2, 3), dtype=np.float32),
            encoding="32FC1",
            header=Header(message_time.time_from_seconds(6), ""),
        )
    )
    info, depth = bridge._topics.put.call_args_list[-2:]
    assert info.args[0] == "camera/depth_info/json" and json.loads(info.args[1])["dtype"] == "<f4"
    assert depth.args[0] == "camera/depth_f32" and depth.args[2] == info.args[2] == 6


def test_depth_round_trip_preserves_metric_values_and_invalid_pixels() -> None:
    source = np.array([[0.123456, 0.5, 1.25], [np.nan, np.inf, 0]], dtype=">f4")[:, ::-1]
    encoded = depth_f32(
        image_utils.from_array(
            source, encoding="32FC1", header=Header(message_time.time_from_seconds(time.time()), "")
        )
    )
    np.testing.assert_allclose(
        np.frombuffer(encoded, dtype="<f4").reshape(2, 3), source, equal_nan=True
    )


@pytest.mark.parametrize(
    "robot,connected",
    [
        (
            "xarm-sim",
            {
                "color_image",
                "depth_image",
                "camera_info",
                "coordinator_joint_state",
                "tf",
                "odom",
                "ee_twist_command",
                "gripper_command",
            },
        ),
        ("unitree-go2", {"color_image", "camera_info", "lidar", "odom", "tf", "cmd_vel"}),
    ],
)
def test_one_bridge_wires_to_whatever_the_robot_provides(robot: str, connected: set[str]) -> None:
    from importlib import import_module

    from dimos.robot.all_blueprints import all_blueprints

    module, attr = all_blueprints[robot].split(":")
    provided = {
        (s.name, s.type, s.direction)
        for atom in getattr(import_module(module), attr).blueprints
        for s in atom.streams
    }
    bridge = {
        (s.name, s.type, s.direction) for s in RawRobotBridge.blueprint().blueprints[0].streams
    }
    opposite = {"in": "out", "out": "in"}
    wired = {
        name
        for name, type_, direction in bridge
        if (name, type_, opposite[direction]) in provided or (name, type_, "inout") in provided
    }
    assert wired == connected


def test_state_with_a_non_finite_joint_is_not_sent(bridge: RawRobotBridge) -> None:
    bridge._on_joint_state(
        JointState(
            header=Header(message_time.time_from_seconds(1.0), ""),
            name=["j1"],
            position=np.asarray([float("nan")], dtype=np.float64),
            velocity=np.asarray([], dtype=np.float64),
            effort=np.asarray([], dtype=np.float64),
        )
    )
    assert not _published(bridge, "arm/state/json")


def test_rejected_depth_warns_once(bridge: RawRobotBridge, mocker) -> None:  # type: ignore[no-untyped-def]
    warn = mocker.patch("dimos.robot.raw_robot_bridge.logger.warning")
    for _ in range(3):
        bridge._on_depth(
            image_utils.from_array(
                np.zeros((2, 3, 3), dtype=np.uint8),
                encoding="rgb8",
                header=Header(message_time.time_from_seconds(time.time()), ""),
            )
        )
    assert warn.call_count == 1
