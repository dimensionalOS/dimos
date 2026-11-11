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
    RosHeader,
    RosImu,
    chassis_speed,
    feed_all_segments,
    feed_chassis_interval,
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
def test_measured_body_rates_and_independent_odometry(
    rig: Callable[..., Rig], publish_tf: bool
) -> None:
    r = rig(publish_odom_tf=publish_tf)
    assert CHASSIS_FEEDBACK_TOPIC in r.fake.control.subs
    assert "/odom" not in r.fake.control.subs
    first_ts = time.time()
    r.fake.control.feed(CHASSIS_FEEDBACK_TOPIC, chassis_speed(0.4, 0.1, 0.0, first_ts))
    first = [s for s in r.readings() if list(s.name) == R1PRO_BASE_JOINTS][-1]
    assert first.ts == first_ts and list(first.position) == []
    assert list(first.velocity) == pytest.approx([0.4, 0.1, 0.0])
    assert r.ports["odometry"].sent == []
    feed_chassis_interval(r.fake, turn=math.pi / 2)
    ts = time.time()
    previous = r.module._odom_last_ts
    assert previous is not None
    r.fake.control.feed(CHASSIS_FEEDBACK_TOPIC, chassis_speed(0.4, 0.1, 0.0, ts))
    base = [s for s in r.readings() if list(s.name) == R1PRO_BASE_JOINTS][-1]
    assert list(base.position) == []
    assert base.frame_id == "base_link"
    assert base.ts == ts
    assert list(base.velocity) == pytest.approx([0.4, 0.1, 0.0])
    r.ports["velocity_command"].publish(
        JointState(name=R1PRO_BASE_JOINTS, velocity=[0.2, 0.5, 0.7])
    )
    sent = r.speeds()[-1].twist
    assert (sent.linear.x, sent.linear.y, sent.angular.z) == pytest.approx((0.2, 0.5, 0.7))

    wait_until(lambda: bool(r.ports["odometry"].sent), timeout=5.0)
    odom = r.ports["odometry"].sent[-1]
    assert odom.frame_id == "odom"
    assert odom.child_frame_id == "base_link"
    assert odom.ts == ts
    assert odom.pose.position.x == pytest.approx(-0.1 * (ts - previous))
    assert odom.twist.linear.x == pytest.approx(0.4)
    # The pose and tf share the same gate.
    assert bool(r.ports["odom"].sent) is publish_tf
    assert bool(r.ports["tf"].sent) is publish_tf
    if publish_tf:
        assert r.ports["odom"].sent[-1].position.x == pytest.approx(odom.pose.position.x)


def test_body_write_needs_no_odometry_and_halt_holds_measured_arms(rig):
    r = rig(start=False)
    r.module.connect()
    r.feed()
    batch = {f"{j}/position": 0.05 for j in R1PRO_UPPER_BODY_JOINTS}
    batch[f"{R1PRO_BASE_JOINTS[0]}/velocity"] = 0.2
    r.module.write(batch)
    assert r.speeds()[-1].twist.linear.x == 0.2
    r.module.halt()
    stopped = r.speeds()[-1].twist
    assert (stopped.linear.x, stopped.linear.y, stopped.angular.z) == (0.0, 0.0, 0.0)
    for topic, q in zip(
        (TORSO_TOPIC, LEFT_TOPIC, RIGHT_TOPIC), (TORSO_Q, LEFT_Q, RIGHT_Q), strict=True
    ):
        assert list(r.joint_targets()[topic].position) == pytest.approx(q)
