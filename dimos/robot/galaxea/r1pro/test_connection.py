# Copyright 2025-2026 Dimensional Inc.
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

"""Tests for ``R1ProConnection`` against a fake vendor ROS stack.

No ROS 2, no robot, no vendor stack: the fakes in ``_connection_test_fakes``
stand in for the ``RawROS`` nodes and the ROS message types, record every
command the module sends to the vendor topics, and feed feedback in. Every
port gets an in-process transport, so commands enter exactly as the
coordinator sends them and feedback leaves exactly as the bus carries it.
"""

from __future__ import annotations

from collections.abc import Callable, Iterator
import threading
import time
from typing import Any

import pytest

from dimos.control.contract.limits import limits
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.galaxea.r1pro import connection as connection_module
from dimos.robot.galaxea.r1pro._connection_test_fakes import (
    LEFT_Q,
    MEASURED_Q,
    RIGHT_Q,
    TORSO_Q,
    FakeRos,
    LocalTransport,
    RosFeedbackStatus,
    RosHeader,
    RosImu,
    RosJointState,
    RosStatusError,
    chassis_speed,
    feed_all_segments,
    gripper_feedback,
    install_fake_ros,
    joint_feedback,
)
from dimos.robot.galaxea.r1pro.connection import (
    R1PRO_UPPER_BODY_JOINTS,
    R1ProConnection,
    R1ProConnectionConfig,
)
from dimos.utils.testing.waiting import wait_until

# Long enough that the deadman never fires mid-test, except where a test sets its own.
DEADMAN_S = 5.0

TORSO_TOPIC = "/motion_target/target_joint_state_torso"
LEFT_TOPIC = "/motion_target/target_joint_state_arm_left"
RIGHT_TOPIC = "/motion_target/target_joint_state_arm_right"
SPEED_TOPIC = "/motion_target/target_speed_chassis"


class Rig:
    """One ``R1ProConnection`` on the fake vendor stack, every port in-process."""

    def __init__(self, fake: FakeRos, module: R1ProConnection) -> None:
        self.fake = fake
        self.module = module
        self.ports = {name: LocalTransport() for name in {**module.inputs, **module.outputs}}
        for name, transport in self.ports.items():
            module.set_transport(name, transport)  # type: ignore[arg-type]

    def start(self, feedback: bool = True) -> None:
        """Start the module, feeding it one frame per segment once it subscribes."""
        if feedback:
            threading.Thread(target=self._feed_once_subscribed, daemon=True).start()
        self.module.start()

    def _feed_once_subscribed(self) -> None:
        def subscribed() -> bool:
            return bool(self.fake.nodes) and self.fake.control.listener_count() >= 4

        try:
            wait_until(subscribed, timeout=5.0, interval=0.01)
        except TimeoutError:
            return
        self.feed()

    def feed(self, ts: float | None = None) -> None:
        feed_all_segments(self.fake, ts)

    def move(self, positions: dict[str, float]) -> None:
        """Send joint targets in rad on ``position_command``, named ``r1pro/<joint>``."""
        msg = JointState(name=list(positions), position=list(positions.values()))
        self.ports["position_command"].publish(msg)

    def drive(self, vx: float, vy: float, wz: float) -> None:
        """Send chassis speeds on ``cmd_vel``."""
        self.ports["cmd_vel"].publish(
            Twist(linear=Vector3(vx, vy, 0.0), angular=Vector3(0.0, 0.0, wz))
        )

    def joint_targets(self) -> dict[str, Any]:
        """The last command sent to each vendor joint topic that got one, by topic."""
        return {
            topic: msgs[-1]
            for topic in (TORSO_TOPIC, LEFT_TOPIC, RIGHT_TOPIC)
            if (msgs := self.fake.control.sent_to(topic))
        }

    def speeds(self) -> list[Any]:
        """Every TwistStamped sent to the chassis speed topic."""
        return self.fake.control.sent_to(SPEED_TOPIC)

    def readings(self) -> list[JointState]:
        return list(self.ports["joint_state"].sent)

    def feed_grippers(
        self, left: float | None = None, right: float | None = None, ts: float | None = None
    ) -> None:
        """One feedback frame per named jaw, an opening in mm as the vendor sends it."""
        if left is not None:
            self.fake.control.feed("/hdas/feedback_gripper_left", gripper_feedback(left, ts))
        if right is not None:
            self.fake.control.feed("/hdas/feedback_gripper_right", gripper_feedback(right, ts))

    def feed_status(self, segment: str, errors: list[RosStatusError]) -> None:
        """One segment's fault report, as the vendor's hdas_msg status topics carry it."""
        self.fake.control.feed(f"/hdas/feedback_status_{segment}", RosFeedbackStatus(errors=errors))

    def jaws(self, side: str) -> list[Any]:
        """Every message sent to one jaw's vendor command topic, oldest first."""
        return self.fake.control.sent_to(f"/motion_target/target_position_gripper_{side}")

    def jaw_readings(self) -> list[JointState]:
        """The joint_state messages that carry a jaw, separate from the arms' frame."""
        return [msg for msg in self.readings() if any("gripper" in name for name in msg.name)]

    def move_arm(self, position: float = 0.0) -> None:
        """A full 18-joint command that carries no jaw — the intent-withdrawal tick."""
        self.move(dict.fromkeys(R1PRO_UPPER_BODY_JOINTS, position))


@pytest.fixture
def fake_ros(monkeypatch: pytest.MonkeyPatch) -> FakeRos:
    return install_fake_ros(monkeypatch)


@pytest.fixture
def rig(fake_ros: FakeRos) -> Iterator[Callable[..., Rig]]:
    """Build an ``R1ProConnection`` on the fake vendor stack; stop every one at the end."""
    made: list[Rig] = []

    def build(start: bool = True, **config: Any) -> Rig:
        config.setdefault("boot_vendor_stack", False)
        config.setdefault("deadman_timeout_s", DEADMAN_S)
        config.setdefault("sensor_stats_interval_s", 0.0)
        config.setdefault("enable_wrist_color", False)
        r = Rig(fake_ros, R1ProConnection(**config))
        made.append(r)
        if start:
            r.start()
        return r

    yield build
    for each in made:
        each.module.stop()


def test_a_config_without_a_model_fails_start_before_any_ros_is_touched(
    rig: Callable[..., Rig], fake_ros: FakeRos
) -> None:
    r = rig(start=False, model=None)
    with pytest.raises(ValueError, match="no model"):
        r.module.start()
    assert fake_ros.nodes == []
    assert not r.module.status().connected


def test_the_limits_table_is_the_18_urdf_joints_the_two_jaws_and_the_clamped_base() -> None:
    table = limits(R1ProConnectionConfig())
    # The URDF's joint order: torso, left arm, its jaw, right arm, its jaw;
    # then the base's three axes.
    keys = list(table)
    assert [key.rsplit("/", 1)[0] for key in keys[:20]] == [
        *R1PRO_UPPER_BODY_JOINTS[:11],
        "r1pro/left_gripper",
        *R1PRO_UPPER_BODY_JOINTS[11:],
        "r1pro/right_gripper",
    ]
    assert keys[20:] == ["r1pro/base/vx", "r1pro/base/vy", "r1pro/base/wz"]
    # Positions only: the vendor tracker owns speeds, so no velocity keys.
    assert not any(key.endswith("/velocity") for key in keys)

    # Bounds are the vendor URDF's, in radians.
    torso = table["r1pro/torso_joint1/position"]
    assert (torso.lo, torso.hi) == pytest.approx((-1.2735467075, 1.8125467075))
    arm = table["r1pro/left_arm_joint1/position"]
    assert (arm.lo, arm.hi) == pytest.approx((-4.415693415, 1.274093415))
    # The jaws: the vendor's 0-100 mm opening, on the contract in metres.
    for side in ("left", "right"):
        jaw = table[f"r1pro/{side}_gripper/position"]
        assert (jaw.lo, jaw.hi) == pytest.approx((0.0, 0.1))
    for axis, top in (("vx", 1.0), ("vy", 1.0), ("wz", 2.0)):
        limit = table[f"r1pro/base/{axis}"]
        assert (limit.lo, limit.hi, limit.clamp) == (-top, top, True)


def test_a_position_command_reaches_the_three_vendor_topics_in_layout_order(
    rig: Callable[..., Rig],
) -> None:
    r = rig()
    targets = [0.01 * (i + 1) for i in range(18)]
    r.move(dict(zip(R1PRO_UPPER_BODY_JOINTS, targets, strict=True)))

    sent = r.joint_targets()
    assert list(sent) == [TORSO_TOPIC, LEFT_TOPIC, RIGHT_TOPIC]
    assert list(sent[TORSO_TOPIC].position) == pytest.approx(targets[0:4])
    assert list(sent[LEFT_TOPIC].position) == pytest.approx(targets[4:11])
    assert list(sent[RIGHT_TOPIC].position) == pytest.approx(targets[11:18])
    # The vendor tracker follows each target at the configured tracking speed.
    for msg in sent.values():
        assert list(msg.velocity) == [r.module.config.tracking_speed] * len(msg.position)


def test_a_command_missing_joints_is_an_error_and_the_robot_is_held(
    rig: Callable[..., Rig],
) -> None:
    r = rig()
    wait_until(lambda: bool(r.readings()), timeout=5.0)
    before = len(r.fake.control.published)

    # 17 of the 18 joints: a partial command must never reach the tracker.
    r.move(dict.fromkeys(R1PRO_UPPER_BODY_JOINTS[:17], 0.05))

    status = r.module.status()
    assert status.last_error is not None
    assert "must cover all 18" in status.last_error
    # The failure halted the robot: chassis zeroed, arms held at the last
    # measured positions, and nothing else sent.
    new = r.fake.control.published[before:]
    assert [name for name, _ in new] == [SPEED_TOPIC, TORSO_TOPIC, LEFT_TOPIC, RIGHT_TOPIC]
    twist = new[0][1]
    assert (twist.twist.linear.x, twist.twist.linear.y, twist.twist.angular.z) == (0.0, 0.0, 0.0)
    held = [msg for name, msg in new if "joint_state" in name]
    assert list(held[0].position) == pytest.approx(TORSO_Q)
    assert list(held[1].position) == pytest.approx(LEFT_Q)
    assert list(held[2].position) == pytest.approx(RIGHT_Q)


def test_an_out_of_range_command_is_refused_and_nothing_reaches_the_vendor(
    rig: Callable[..., Rig],
) -> None:
    r = rig()
    before = list(r.fake.control.published)

    # torso_joint1 stops at ~1.81 rad in the URDF; 2.0 is past it.
    targets = dict.fromkeys(R1PRO_UPPER_BODY_JOINTS, 0.0)
    targets["r1pro/torso_joint1"] = 2.0
    r.move(targets)

    assert r.fake.control.published == before
    rejection = r.module.status().last_rejection
    assert rejection is not None
    assert "r1pro/torso_joint1/position" in rejection
    assert "above its limit" in rejection


def test_unknown_and_foreign_keys_never_reach_the_vendor(rig: Callable[..., Rig]) -> None:
    r = rig()
    before = list(r.fake.control.published)

    r.move({"r1pro/gripper_joint1": 0.5})
    rejection = r.module.status().last_rejection
    assert rejection is not None and "unknown_key" in rejection

    # Keys of other robots are skipped, not refused: nothing happens at all.
    r.move({"other/joint1": 0.5, "chassis/base/vx": 0.5})
    assert r.fake.control.published == before


def test_a_non_finite_command_is_refused(rig: Callable[..., Rig]) -> None:
    r = rig()
    r.drive(float("nan"), 0.0, 0.0)
    assert r.speeds() == []
    rejection = r.module.status().last_rejection
    assert rejection is not None and "non-finite" in rejection


def test_the_chassis_is_told_its_speeds_clamped_to_the_base_range(
    rig: Callable[..., Rig],
) -> None:
    r = rig()
    r.drive(0.5, -0.2, 0.7)
    r.drive(1.5, -2.0, 3.0)

    first, second = r.speeds()
    assert (first.twist.linear.x, first.twist.linear.y, first.twist.angular.z) == (0.5, -0.2, 0.7)
    assert (second.twist.linear.x, second.twist.linear.y, second.twist.angular.z) == (
        1.0,
        -1.0,
        2.0,
    )


def test_the_deadman_waits_for_the_first_command_then_halts_the_robot(
    rig: Callable[..., Rig],
) -> None:
    r = rig(deadman_timeout_s=0.05)
    wait_until(lambda: bool(r.readings()), timeout=5.0)

    # No command yet: the deadman is not armed, nothing is sent.
    time.sleep(0.15)
    assert r.fake.control.published == []
    assert not r.module.status().deadman_fired

    r.drive(0.3, 0.0, 0.0)
    assert len(r.speeds()) == 1
    wait_until(lambda: r.module.status().deadman_fired, timeout=5.0)

    # The halt: chassis zeroed once, arms held where they were last measured.
    assert r.speeds()[-1].twist.linear.x == 0.0
    held = r.joint_targets()
    assert list(held[TORSO_TOPIC].position) == pytest.approx(TORSO_Q)

    # A new command rearms the deadman.
    r.drive(0.2, 0.0, 0.0)
    assert not r.module.status().deadman_fired
    assert r.speeds()[-1].twist.linear.x == 0.2


def test_stop_halts_the_robot_closes_ros_and_is_idempotent(rig: Callable[..., Rig]) -> None:
    r = rig()
    wait_until(lambda: bool(r.readings()), timeout=5.0)
    r.drive(0.3, 0.0, 0.0)

    r.module.stop()

    assert not r.module.status().connected
    assert all(node.stopped for node in r.fake.nodes)
    # The stop-time halt: chassis zeroed, arms held at the measured positions.
    assert r.speeds()[-1].twist.linear.x == 0.0
    assert list(r.joint_targets()[TORSO_TOPIC].position) == pytest.approx(TORSO_Q)

    sent = list(r.fake.control.published)
    r.module.stop()
    assert r.fake.control.published == sent


def test_feedback_is_aggregated_into_joint_state_in_layout_order(rig: Callable[..., Rig]) -> None:
    r = rig()
    wait_until(lambda: bool(r.readings()), timeout=5.0)
    msg = r.readings()[-1]

    assert list(msg.name) == R1PRO_UPPER_BODY_JOINTS
    assert list(msg.position) == pytest.approx(MEASURED_Q)
    assert len(msg.velocity) == 18
    assert len(msg.effort) == 18
    assert msg.frame_id == "base_link"


def test_nothing_is_published_until_every_segment_has_reported(
    rig: Callable[..., Rig], monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setattr(connection_module, "_FEEDBACK_DISCOVERY_TIMEOUT_S", 0.2)
    r = rig(start=False)
    r.start(feedback=False)

    # A zero-position snapshot would walk the arms to home, so silence in.
    time.sleep(0.3)
    assert r.readings() == []

    r.fake.control.feed("/hdas/feedback_torso", joint_feedback(TORSO_Q, 100.1))
    r.fake.control.feed("/hdas/feedback_arm_left", joint_feedback(LEFT_Q, 100.2))
    time.sleep(0.1)
    assert r.readings() == []

    r.fake.control.feed("/hdas/feedback_arm_right", joint_feedback(RIGHT_Q, 100.3))
    wait_until(lambda: bool(r.readings()), timeout=5.0)
    # The fused snapshot is only as fresh as its stalest segment.
    assert r.readings()[-1].ts == pytest.approx(100.1)


def test_the_imus_are_republished_once_feedback_is_flowing(rig: Callable[..., Rig]) -> None:
    r = rig()
    wait_until(lambda: bool(r.readings()), timeout=5.0)
    sensors = r.fake.sensors
    sensors.feed("/hdas/imu_chassis", RosImu(header=RosHeader(frame_id="chassis_imu")))
    sensors.feed("/hdas/imu_torso", RosImu(header=RosHeader(frame_id="torso_imu")))

    wait_until(lambda: bool(r.ports["imu_chassis"].sent), timeout=5.0)
    wait_until(lambda: bool(r.ports["imu_torso"].sent), timeout=5.0)
    assert r.ports["imu_chassis"].sent[-1].frame_id == "chassis_imu"
    assert r.ports["imu_torso"].sent[-1].frame_id == "torso_imu"


def test_executed_chassis_speed_is_integrated_into_odometry(rig: Callable[..., Rig]) -> None:
    r = rig()
    r.fake.control.feed("/motion_control/chassis_speed", chassis_speed(0.0, 0.0, 0.0, 100.0))
    r.fake.control.feed("/motion_control/chassis_speed", chassis_speed(0.4, 0.0, 0.0, 100.5))

    wait_until(lambda: bool(r.ports["odometry"].sent), timeout=5.0)
    odom = r.ports["odometry"].sent[-1]
    assert odom.frame_id == "odom"
    assert odom.child_frame_id == "base_link"
    assert odom.pose.position.x == pytest.approx(0.2)
    assert odom.twist.linear.x == pytest.approx(0.4)
    # The pose and tf share the same gate.
    assert r.ports["odom"].sent[-1].position.x == pytest.approx(0.2)
    assert r.ports["tf"].sent


def test_the_odom_pose_and_tf_follow_their_gate(rig: Callable[..., Rig]) -> None:
    r = rig(publish_odom_tf=False)
    r.fake.control.feed("/motion_control/chassis_speed", chassis_speed(0.0, 0.0, 0.0, 100.0))
    r.fake.control.feed("/motion_control/chassis_speed", chassis_speed(0.4, 0.0, 0.0, 100.5))

    wait_until(lambda: bool(r.ports["odometry"].sent), timeout=5.0)
    assert r.ports["odom"].sent == []
    assert r.ports["tf"].sent == []


def test_status_reports_the_session_commands_and_rejections(rig: Callable[..., Rig]) -> None:
    r = rig(start=False)
    assert not r.module.status().connected

    r.start()
    status = r.module.status()
    assert status.connected
    assert status.session_id
    assert status.last_command_time is None
    assert status.last_rejection is None
    assert status.last_error is None

    r.drive(0.1, 0.0, 0.0)
    assert r.module.status().last_command_time is not None


# The jaws: vendor interface in millimetres, fault/staleness semantics, and the
# no-replay rule, all against the fake vendor stack.


def test_a_jaw_command_reaches_only_its_own_vendor_topic_in_millimetres(
    rig: Callable[..., Rig],
) -> None:
    r = rig()
    # Idle: a started connection never makes up a jaw target.
    assert r.jaws("left") == []
    assert r.jaws("right") == []

    r.feed_grippers(left=20.0, right=40.0)
    r.move({"r1pro/left_gripper": 0.05})
    sent = r.jaws("left")
    assert len(sent) == 1
    assert list(sent[0].position) == [50.0]  # 0.05 m is 50 mm on the vendor topic
    assert sent[0].name == ["left_gripper"]
    # The other jaw is never touched by its sibling's command.
    assert r.jaws("right") == []

    r.move({"r1pro/right_gripper": 0.03})
    assert [list(msg.position) for msg in r.jaws("right")] == [[30.0]]
    assert len(r.jaws("left")) == 1

    # The vendor controller holds its target, so a repeat is not re-sent.
    r.move({"r1pro/right_gripper": 0.03})
    assert len(r.jaws("right")) == 1


def test_the_jaw_readback_is_a_separate_position_only_message_in_metres(
    rig: Callable[..., Rig],
) -> None:
    r = rig()
    r.feed_grippers(left=25.0, right=50.0)

    wait_until(lambda: bool(r.jaw_readings()), timeout=5.0)
    msg = r.jaw_readings()[-1]
    assert list(msg.name) == ["r1pro/left_gripper", "r1pro/right_gripper"]
    assert list(msg.position) == pytest.approx([0.025, 0.05])  # measured, in metres
    assert len(msg.velocity) == 0
    assert len(msg.effort) == 0


def test_a_jaw_without_feedback_takes_no_command_and_reports_nothing(
    rig: Callable[..., Rig],
) -> None:
    # The arms report as usual; the jaws never do.
    r = rig()
    wait_until(lambda: bool(r.readings()), timeout=5.0)

    r.move({"r1pro/left_gripper": 0.05})

    assert r.jaws("left") == []
    rejection = r.module.status().last_rejection
    assert rejection is not None
    assert "gripper_left" in rejection and "dropped" in rejection
    # No reading is made up for a jaw that never reported.
    assert r.jaw_readings() == []


def test_stale_jaw_feedback_drops_the_command_and_withholds_the_readback(
    rig: Callable[..., Rig],
) -> None:
    r = rig(gripper_feedback_timeout_s=0.05)
    r.feed_grippers(left=20.0)
    r.move({"r1pro/left_gripper": 0.05})
    assert len(r.jaws("left")) == 1
    wait_until(lambda: bool(r.jaw_readings()), timeout=5.0)

    # The jaw's feedback goes quiet while the arms keep reporting.
    time.sleep(0.1)
    readings = len(r.jaw_readings())
    r.move({"r1pro/left_gripper": 0.06})

    # The command is dropped, not sent to a jaw whose state is unknown...
    assert len(r.jaws("left")) == 1
    rejection = r.module.status().last_rejection
    assert rejection is not None and "stale" in rejection
    # ...and the stale measurement stops being published.
    time.sleep(0.1)
    assert len(r.jaw_readings()) == readings


def test_a_malformed_jaw_frame_interrupts_the_intent_until_it_is_replaced(
    rig: Callable[..., Rig],
) -> None:
    r = rig()
    r.feed_grippers(left=20.0)
    r.move({"r1pro/left_gripper": 0.05})
    assert len(r.jaws("left")) == 1

    # A frame that is not one finite position breaks the feedback: the intent
    # it carried is interrupted, even between two ticks.
    r.fake.control.feed("/hdas/feedback_gripper_left", RosJointState(position=[1.0, 2.0]))
    r.move({"r1pro/left_gripper": 0.06})
    assert len(r.jaws("left")) == 1
    rejection = r.module.status().last_rejection
    assert rejection is not None and "gripper_left" in rejection

    # Feedback recovers, but the interrupted intent is not replayed — a
    # repeated value is the old intent, not a new command.
    r.feed_grippers(left=20.0)
    r.move({"r1pro/left_gripper": 0.06})
    assert len(r.jaws("left")) == 1

    # The task withdraws it (a tick's command without the jaw), then commands
    # the same opening again: an explicit new command, so it is sent.
    r.move_arm()
    r.move({"r1pro/left_gripper": 0.06})
    assert [list(msg.position) for msg in r.jaws("left")] == [[50.0], [60.0]]


def test_a_fault_report_blocks_the_jaw_and_recovery_never_replays(rig: Callable[..., Rig]) -> None:
    r = rig()
    r.feed_grippers(left=20.0, right=40.0)
    r.move({"r1pro/left_gripper": 0.05})
    assert len(r.jaws("left")) == 1

    # The vendor reports a fault on the jaw's own segment.
    r.feed_status(
        "gripper_left", [RosStatusError(name="motor", error_code=7, error_description="stalled")]
    )
    r.move({"r1pro/left_gripper": 0.06})
    assert len(r.jaws("left")) == 1
    rejection = r.module.status().last_rejection
    assert rejection is not None and "stalled" in rejection

    # A torso fault blocks both jaws: both arms hang from it. (One command
    # carries both jaws here — a jaw absent from a command is withdrawn.)
    r.feed_status("torso", [RosStatusError(name="drive", error_code=1, error_description="fault")])
    r.move({"r1pro/left_gripper": 0.07, "r1pro/right_gripper": 0.05})
    assert len(r.jaws("left")) == 1
    assert r.jaws("right") == []

    # Clean reports clear their segments, but neither interrupted intent is
    # replayed while the tasks still hold it.
    r.feed_status("gripper_left", [RosStatusError(name="motor")])
    r.feed_status("torso", [RosStatusError(name="drive")])
    r.feed_grippers(left=20.0, right=40.0)
    r.move({"r1pro/left_gripper": 0.07, "r1pro/right_gripper": 0.05})
    assert len(r.jaws("left")) == 1
    assert r.jaws("right") == []

    # Withdrawn, then explicitly re-commanded at the same openings: sent.
    r.move_arm()
    r.move({"r1pro/left_gripper": 0.07, "r1pro/right_gripper": 0.05})
    assert [list(msg.position) for msg in r.jaws("left")] == [[50.0], [70.0]]
    assert [list(msg.position) for msg in r.jaws("right")] == [[50.0]]


def test_a_jaw_command_outside_the_vendor_range_is_refused(rig: Callable[..., Rig]) -> None:
    r = rig()
    r.feed_grippers(left=20.0)

    r.move({"r1pro/left_gripper": 0.2})  # the vendor opens 0-100 mm
    assert r.jaws("left") == []
    rejection = r.module.status().last_rejection
    assert rejection is not None and "r1pro/left_gripper/position" in rejection


def test_stop_and_halt_send_the_jaws_nothing(rig: Callable[..., Rig]) -> None:
    r = rig()
    r.feed_grippers(left=20.0)
    r.move({"r1pro/left_gripper": 0.05})
    assert len(r.jaws("left")) == 1

    # Halt holds the arms and zeroes the chassis, but there is no verified
    # vendor cancel: an accepted jaw target is never recalled, and the jaws
    # are sent nothing.
    r.module.halt()
    assert len(r.jaws("left")) == 1

    r.module.stop()
    assert len(r.jaws("left")) == 1
    assert r.jaws("right") == []
