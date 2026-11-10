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

"""Tests for ``R1ProConnection`` against a fake vendor ROS stack."""

from __future__ import annotations

from collections.abc import Callable, Iterator
import math
import time
from types import SimpleNamespace
from typing import Any

import pytest

from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.galaxea.r1pro import connection as connection_module
from dimos.robot.galaxea.r1pro._connection_test_fakes import (
    CHASSIS_FEEDBACK_TOPIC,
    LEFT_Q,
    MEASURED_Q,
    RIGHT_Q,
    TORSO_Q,
    FakeRos,
    LocalTransport,
    RosFeedbackStatus,
    RosHeader,
    RosImu,
    RosStatusError,
    chassis_speed,
    feed_all_segments,
    feed_chassis_interval,
    gripper_feedback,
    install_fake_ros,
    joint_feedback,
)
from dimos.robot.galaxea.r1pro.connection import (
    R1PRO_BASE_JOINTS,
    R1PRO_UPPER_BODY_JOINTS,
    R1ProConnection,
)
from dimos.utils.testing.waiting import wait_until

# Long enough that the deadman never fires mid-test, except where a test sets its own.
DEADMAN_S = 5.0

TORSO_TOPIC = "/motion_target/target_joint_state_torso"
LEFT_TOPIC = "/motion_target/target_joint_state_arm_left"
RIGHT_TOPIC = "/motion_target/target_joint_state_arm_right"
SPEED_TOPIC = "/motion_target/target_speed_chassis"


class Rig:
    """One concrete connection with in-process ports."""

    def __init__(self, fake: FakeRos, module: R1ProConnection) -> None:
        self.fake = fake
        self.module = module
        self.ports = {name: LocalTransport() for name in {**module.inputs, **module.outputs}}
        for name, transport in self.ports.items():
            module.set_transport(name, transport)  # type: ignore[arg-type]

    def start(self, feedback: bool = True) -> None:
        self.module.start()
        if feedback:
            self.feed()

    def feed(self, ts: float | None = None) -> None:
        feed_all_segments(self.fake, ts)

    def move(self, positions: dict[str, float]) -> None:
        msg = JointState(name=list(positions), position=list(positions.values()))
        self.ports["position_command"].publish(msg)

    def drive(self, vx: float, vy: float, wz: float) -> None:
        feed_chassis_interval(self.fake)
        self.ports["velocity_command"].publish(
            JointState(name=R1PRO_BASE_JOINTS, velocity=[vx, vy, wz])
        )

    def joint_targets(self) -> dict[str, Any]:
        return {
            topic: msgs[-1]
            for topic in (TORSO_TOPIC, LEFT_TOPIC, RIGHT_TOPIC)
            if (msgs := self.fake.control.sent_to(topic))
        }

    def speeds(self) -> list[Any]:
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


@pytest.fixture
def fake_ros(monkeypatch: pytest.MonkeyPatch) -> FakeRos:
    return install_fake_ros(monkeypatch)


@pytest.fixture
def rig(fake_ros: FakeRos, monkeypatch: pytest.MonkeyPatch) -> Iterator[Callable[..., Rig]]:
    """Build an ``R1ProConnection`` on the fake vendor stack; stop every one at the end."""
    monkeypatch.setattr(connection_module, "_FEEDBACK_DISCOVERY_TIMEOUT_S", 0.0)
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


def test_nothing_is_published_until_every_segment_has_reported(
    rig: Callable[..., Rig], monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setattr(connection_module, "_FEEDBACK_DISCOVERY_TIMEOUT_S", 0.0)
    r = rig(start=False)
    r.start(feedback=False)

    assert r.readings() == []

    r.fake.control.feed("/hdas/feedback_torso", joint_feedback(TORSO_Q, 100.1))
    r.fake.control.feed("/hdas/feedback_arm_left", joint_feedback(LEFT_Q, 100.2))
    time.sleep(2.0 / r.module.config.publish_rate_hz)
    assert r.readings() == []

    r.fake.control.feed("/hdas/feedback_arm_right", joint_feedback(RIGHT_Q, 100.3))
    wait_until(lambda: bool(r.readings()), timeout=5.0)
    # The fused snapshot is only as fresh as its stalest segment.
    msg = r.readings()[-1]
    assert msg.ts == pytest.approx(100.1)
    assert list(msg.name) == R1PRO_UPPER_BODY_JOINTS
    assert list(msg.position) == pytest.approx(MEASURED_Q)
    assert len(msg.velocity) == len(msg.effort) == 18
    assert msg.frame_id == "base_link"
    for part in ("chassis", "torso"):
        r.fake.sensors.feed(f"/hdas/imu_{part}", RosImu(header=RosHeader(frame_id=part)))
        wait_until(lambda part=part: bool(r.ports[f"imu_{part}"].sent), timeout=5.0)
        assert r.ports[f"imu_{part}"].sent[-1].frame_id == part

    before = list(r.fake.control.published)
    with pytest.raises(ValueError, match="must cover all 18"):
        r.module.write({f"{j}/position": 0.05 for j in R1PRO_UPPER_BODY_JOINTS[:17]})
    assert r.fake.control.published == before
    workers = list(r.module._sensor_workers)
    r.module.stop()
    assert all(node.stopped and node.listener_count() == 0 for node in r.fake.nodes)
    assert all(not worker.is_alive() for worker in workers)
    assert not r.module.status().connected
    sent = list(r.fake.control.published)
    r.drive(0.4, 0.0, 0.0)  # Disposed ports cannot restart motion.
    assert r.fake.control.published == sent


@pytest.mark.parametrize("publish_tf", [True, False])
def test_measured_speed_integrates_local_pose_and_world_joint_velocities(
    rig: Callable[..., Rig], publish_tf: bool
) -> None:
    r = rig(publish_odom_tf=publish_tf)
    assert CHASSIS_FEEDBACK_TOPIC in r.fake.control.subs
    assert "/odom" not in r.fake.control.subs
    feed_chassis_interval(r.fake, turn=math.pi / 2)
    ts = time.time()
    previous = r.module._odom_last_ts
    assert previous is not None
    r.fake.control.feed(CHASSIS_FEEDBACK_TOPIC, chassis_speed(0.4, 0.1, 0.0, ts))
    base = [s for s in r.readings() if list(s.name) == R1PRO_BASE_JOINTS][-1]
    assert list(base.position) == pytest.approx(
        [-0.1 * (ts - previous), 0.4 * (ts - previous), math.pi / 2]
    )
    assert list(base.velocity) == pytest.approx([-0.1, 0.4, 0.0])
    r.ports["velocity_command"].publish(
        JointState(name=R1PRO_BASE_JOINTS, velocity=[0.2, 0.5, 0.7])
    )
    sent = r.speeds()[-1].twist
    assert (sent.linear.x, sent.linear.y, sent.angular.z) == pytest.approx((0.5, -0.2, 0.7))

    wait_until(lambda: bool(r.ports["odometry"].sent), timeout=5.0)
    odom = r.ports["odometry"].sent[-1]
    assert odom.frame_id == "odom"
    assert odom.child_frame_id == "base_link"
    assert odom.pose.position.x == pytest.approx(base.position[0])
    assert odom.twist.linear.x == pytest.approx(0.4)
    # The pose and tf share the same gate.
    assert bool(r.ports["odom"].sent) is publish_tf
    assert bool(r.ports["tf"].sent) is publish_tf
    if publish_tf:
        assert r.ports["odom"].sent[-1].position.x == pytest.approx(base.position[0])


def test_gripper_units_readback_and_dedup(rig: Callable[..., Rig]) -> None:
    r = rig()
    assert not r.jaws("left") and not r.jaws("right")
    r.feed_grippers(left=20.0, right=40.0, ts=time.time())
    wait_until(lambda: bool(r.jaw_readings()), timeout=5.0)
    assert list(r.jaw_readings()[-1].position) == pytest.approx([0.02, 0.04])
    r.move({"r1pro/left_gripper": 0.02})  # Initial measured-position hold.
    assert not r.jaws("left")
    r.move({"r1pro/left_gripper": 0.05})
    r.move({"r1pro/left_gripper": 0.05})
    assert [list(msg.position) for msg in r.jaws("left")] == [[50.0]]
    assert not r.jaws("right")
    r.move({"r1pro/right_gripper": 0.03})
    assert [list(msg.position) for msg in r.jaws("right")] == [[30.0]]
    r.module.stop()
    r.module._send_gripper_target("left", 0.06, None)
    assert len(r.jaws("left")) == 1


def test_gripper_invalid_feedback_blocks_without_replay(rig, monkeypatch):
    clock = [time.monotonic()]
    monkeypatch.setattr(
        connection_module, "time", SimpleNamespace(**{**vars(time), "monotonic": lambda: clock[0]})
    )
    r = rig(gripper_feedback_timeout_s=0.1)
    r.move({"r1pro/left_gripper": 0.05})
    assert not r.jaws("left")
    r.feed_grippers(left=20.0)
    r.move({"r1pro/left_gripper": 0.06})
    assert len(r.jaws("left")) == 1
    clock[0] += 0.2
    r.move({"r1pro/left_gripper": 0.07})
    r.feed_grippers(left=20.0)
    r.move({"r1pro/left_gripper": 0.07})
    assert len(r.jaws("left")) == 1
    r.feed_status("gripper_left", [RosStatusError(error_code=7)])
    r.move({"r1pro/left_gripper": 0.08})
    assert len(r.jaws("left")) == 1
    r.feed_status("gripper_left", [RosStatusError()])
    r.move({"r1pro/left_gripper": 0.09})
    assert [list(msg.position) for msg in r.jaws("left")] == [[60.0], [90.0]]
    r.fake.control.feed("/hdas/feedback_gripper_left", gripper_feedback(math.nan))
    r.move({"r1pro/left_gripper": 0.1})
    assert len(r.jaws("left")) == 2
    r.feed_grippers(left=20.0)
    r.move({"r1pro/left_gripper": 0.095})
    assert list(r.jaws("left")[-1].position) == [95.0]


def test_stale_feedback_rejects_mixed_commands_but_allows_halt(rig, monkeypatch):
    r = rig(start=False)
    r.module.connect()
    r.feed()
    feed_chassis_interval(r.fake)
    received = r.module._odom_received_mono
    monkeypatch.setattr(connection_module.time, "monotonic", lambda: received + 10.0)
    batch = {f"{j}/position": 0.05 for j in R1PRO_UPPER_BODY_JOINTS}
    batch[f"{R1PRO_BASE_JOINTS[0]}/velocity"] = 0.2
    before = list(r.fake.control.published)
    with pytest.raises(ValueError, match="fresh measured odometry"):
        r.module.write(batch)
    assert r.fake.control.published == before
    r.module.halt()
    stopped = r.speeds()[-1].twist
    assert (stopped.linear.x, stopped.linear.y, stopped.angular.z) == (0.0, 0.0, 0.0)
    for topic, q in zip(
        (TORSO_TOPIC, LEFT_TOPIC, RIGHT_TOPIC), (TORSO_Q, LEFT_Q, RIGHT_Q), strict=True
    ):
        assert list(r.joint_targets()[topic].position) == pytest.approx(q)
