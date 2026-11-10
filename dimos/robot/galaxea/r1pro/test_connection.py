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
import math
import threading
import time
from typing import Any

import pytest

from dimos.control.contract.limits import limits
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.galaxea.r1pro import connection as connection_module
from dimos.robot.galaxea.r1pro._connection_test_fakes import (
    LEFT_Q,
    MEASURED_Q,
    RIGHT_Q,
    TORSO_Q,
    FakeRos,
    LocalTransport,
    RosHeader,
    RosImu,
    feed_all_segments,
    install_fake_ros,
    joint_feedback,
    odometry_feedback,
)
from dimos.robot.galaxea.r1pro.connection import (
    R1PRO_BASE_JOINTS,
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
        """Send world-frame ordinary joint velocities, using measured yaw."""
        self.fake.control.feed("/odom", odometry_feedback())
        self.ports["velocity_command"].publish(
            JointState(name=R1PRO_BASE_JOINTS, velocity=[vx, vy, wz])
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


def test_the_limits_table_is_the_18_urdf_joints_and_three_planar_joints() -> None:
    table = limits(R1ProConnectionConfig())
    # 18 position keys in the flat command order, then the base's three axes.
    keys = list(table)
    assert [key.rsplit("/", 1)[0] for key in keys[:18]] == R1PRO_UPPER_BODY_JOINTS
    assert keys[18:] == [f"{joint}/velocity" for joint in R1PRO_BASE_JOINTS]
    # Positions only: the vendor tracker owns speeds, so no velocity keys.
    assert not any(key.endswith("/velocity") for key in keys[:18])

    # Bounds are the vendor URDF's, in radians.
    torso = table["r1pro/torso_joint1/position"]
    assert (torso.lo, torso.hi) == pytest.approx((-1.2735467075, 1.8125467075))
    arm = table["r1pro/left_arm_joint1/position"]
    assert (arm.lo, arm.hi) == pytest.approx((-4.415693415, 1.274093415))
    for joint, top in zip(R1PRO_BASE_JOINTS, (1.0, 1.0, 2.0), strict=True):
        limit = table[f"{joint}/velocity"]
        assert (limit.lo, limit.hi) == (-top, top)


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


def test_the_chassis_speed_limit_rejects_rather_than_clamps(
    rig: Callable[..., Rig],
) -> None:
    r = rig()
    r.drive(0.5, -0.2, 0.7)
    r.drive(1.5, -2.0, 3.0)

    (first,) = r.speeds()
    assert (first.twist.linear.x, first.twist.linear.y, first.twist.angular.z) == (0.5, -0.2, 0.7)
    assert r.module.status().last_rejection is not None


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
    # A late transport callback cannot restart motion after shutdown.
    r.ports["velocity_command"].publish(
        JointState(name=R1PRO_BASE_JOINTS, velocity=[0.4, 0.0, 0.0])
    )
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


def test_actual_odometry_is_decoded_with_world_joint_velocities(rig: Callable[..., Rig]) -> None:
    r = rig()
    r.fake.control.feed("/odom", odometry_feedback(x=0.2, yaw=math.pi / 2, vx=0.4, vy=0.1, wz=0.3))
    base = [s for s in r.readings() if list(s.name) == R1PRO_BASE_JOINTS][-1]
    assert list(base.position) == pytest.approx([0.2, 0.0, math.pi / 2])
    assert list(base.velocity) == pytest.approx([-0.1, 0.4, 0.3])

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
    r.fake.control.feed("/odom", odometry_feedback(vx=0.4))

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


def test_world_velocity_uses_nonzero_actual_yaw(rig: Callable[..., Rig]) -> None:
    r = rig()
    r.fake.control.feed("/odom", odometry_feedback(yaw=math.pi / 2))
    r.ports["velocity_command"].publish(
        JointState(name=R1PRO_BASE_JOINTS, velocity=[0.2, 0.5, 0.7])
    )
    sent = r.speeds()[-1].twist
    assert (sent.linear.x, sent.linear.y, sent.angular.z) == pytest.approx((0.5, -0.2, 0.7))


@pytest.mark.parametrize(
    "invalid", ["absent", "old_stamp", "stale_receipt", "nan", "nonunit", "frame"]
)
def test_yaw_authority_is_independent_of_arm_imu_and_publish_traffic(
    rig: Callable[..., Rig],
    invalid: str,
) -> None:
    r = rig()
    msg = odometry_feedback(yaw=0.8)
    if invalid == "old_stamp":
        msg = odometry_feedback(ts=time.time() - 1)
    elif invalid == "nan":
        msg.pose.pose.orientation.z = float("nan")
    elif invalid == "nonunit":
        msg.pose.pose.orientation.w = 2.0
    elif invalid == "frame":
        msg.header.frame_id = "wrong"
    if invalid != "absent":
        r.fake.control.feed("/odom", msg)
    if invalid == "stale_receipt":
        time.sleep(0.22)
    r.feed()
    r.fake.sensors.feed("/hdas/imu_chassis", RosImu(header=RosHeader()))
    time.sleep(0.02)  # The arm publish timer has run, but cannot refresh yaw.
    before = list(r.fake.control.published)
    batch = {f"{j}/position": 0.05 for j in R1PRO_UPPER_BODY_JOINTS}
    batch.update(
        {f"{j}/velocity": v for j, v in zip(R1PRO_BASE_JOINTS, [0.2, 0.0, 0.0], strict=True)}
    )
    with pytest.raises(ValueError, match="fresh measured odometry"):
        r.module.write(batch)
    assert r.fake.control.published == before  # No partial arm send.
    r.module.command(batch)
    assert "fresh measured odometry" in (r.module.status().last_error or "")
    assert r.speeds()[-1].twist.linear.x == 0.0  # Error halt works without yaw.
    assert list(r.joint_targets()[TORSO_TOPIC].position) == pytest.approx(TORSO_Q)


def test_zero_velocity_and_halt_need_no_odometry(rig: Callable[..., Rig]) -> None:
    r = rig()
    r.ports["velocity_command"].publish(JointState(name=R1PRO_BASE_JOINTS, velocity=[0.0] * 3))
    assert r.module.status().last_error is None
    r.module.halt()
    assert all(m.twist.linear.x == m.twist.linear.y == m.twist.angular.z == 0 for m in r.speeds())
