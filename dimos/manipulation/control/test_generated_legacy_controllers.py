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

from types import SimpleNamespace
from unittest.mock import MagicMock

from dimos_generated.builtin_interfaces.msg import Time
from dimos_generated.dimos_msgs.msg import JointCommand
from dimos_generated.geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from dimos_generated.sensor_msgs.msg import JointState
from dimos_generated.std_msgs.msg import Header
from dimos_generated.trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from dimos_message_build.registry import decode as cdr_decode, encode as cdr_encode
import numpy as np
import pytest

from dimos.manipulation.control.servo_control.cartesian_motion_controller import (
    CartesianMotionController,
)
from dimos.manipulation.control.trajectory_controller.joint_trajectory_controller import (
    JointTrajectoryController,
)
from dimos.msgs.time import duration_from_seconds
from dimos.msgs.trajectory import TrajectoryState


def test_trajectory_loop_publishes_generated_sample_with_source_time(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    controller = JointTrajectoryController()
    controller._trajectory = JointTrajectory(
        joint_names=["a", "b"],
        points=[
            JointTrajectoryPoint(
                positions=np.array([0.0, 0.0], dtype=np.float64),
                time_from_start=duration_from_seconds(0),
                velocities=np.array([], dtype=np.float64),
                accelerations=np.array([], dtype=np.float64),
                effort=np.array([], dtype=np.float64),
            ),
            JointTrajectoryPoint(
                positions=np.array([1.0, 2.0], dtype=np.float64),
                time_from_start=duration_from_seconds(1),
                velocities=np.array([], dtype=np.float64),
                accelerations=np.array([], dtype=np.float64),
                effort=np.array([], dtype=np.float64),
            ),
        ],
        header=Header(stamp=Time(sec=0, nanosec=0), frame_id=""),
    )
    controller._state = TrajectoryState.EXECUTING
    controller._start_time = 100.0
    controller.joint_position_command = MagicMock()
    controller.joint_position_command.publish.side_effect = (
        lambda message: controller._stop_event.set()
    )
    monkeypatch.setattr(
        "dimos.manipulation.control.trajectory_controller.joint_trajectory_controller.time",
        SimpleNamespace(time=lambda: 100.5, sleep=lambda seconds: None),
    )
    monkeypatch.setattr("dimos.msgs.time.time.time_ns", lambda: 100500000000)
    try:
        controller._execution_loop()
        controller.joint_position_command.publish.assert_called_once()
        message = cdr_decode(
            cdr_encode(controller.joint_position_command.publish.call_args.args[0]),
            JointCommand,
        )
        assert list(message.positions) == [0.5, 1.0]
        assert (message.header.stamp.sec, message.header.stamp.nanosec) == (100, 500000000)
    finally:
        controller.stop()


def test_cartesian_generated_command_preserves_joint_count_and_pid_clamp(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    controller = CartesianMotionController()
    driver = MagicMock()
    driver.get_forward_kinematics.return_value = (0, [0.0] * 6)
    driver.get_inverse_kinematics.return_value = (0, [0.1, 0.2, 0.3])
    controller._arm_driver = driver
    controller._latest_joint_state = JointState(
        position=np.array([0.0, 0.0], dtype=np.float64),
        header=Header(stamp=Time(sec=0, nanosec=0), frame_id=""),
        name=[],
        velocity=np.array([], dtype=np.float64),
        effort=np.array([], dtype=np.float64),
    )
    controller._target_pose_ = PoseStamped(
        pose=Pose(
            position=Point(x=0.1, y=0.0, z=0.0), orientation=Quaternion(w=1.0, x=0.0, y=0.0, z=0.0)
        ),
        header=Header(stamp=Time(sec=0, nanosec=0), frame_id=""),
    )
    controller._is_tracking = True
    controller._last_target_time = 100.0
    controller.current_pose = MagicMock()
    controller.cartesian_velocity = MagicMock()
    controller.joint_position_command = MagicMock()
    controller.joint_position_command.publish.side_effect = (
        lambda message: controller._stop_event.set()
    )
    monkeypatch.setattr(
        "dimos.manipulation.control.servo_control.cartesian_motion_controller.time",
        SimpleNamespace(time=lambda: 100.5),
    )
    try:
        controller._control_loop()
        controller.joint_position_command.publish.assert_called_once()
        message = cdr_decode(
            cdr_encode(controller.joint_position_command.publish.call_args.args[0]),
            JointCommand,
        )
        assert list(message.positions) == [0.1, 0.2]
        assert (message.header.stamp.sec, message.header.stamp.nanosec) == (100, 500000000)
        velocity = controller.cartesian_velocity.publish.call_args.args[0]
        assert velocity.linear.x == pytest.approx(controller.config.max_linear_velocity)
        # FK/IK only invoked on the fake driver; no driver actuation interface exists here.
        assert driver.get_forward_kinematics.call_count == 1
        assert driver.get_inverse_kinematics.call_count == 1
    finally:
        controller.stop()
