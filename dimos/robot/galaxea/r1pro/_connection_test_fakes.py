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

"""Fakes for the R1 Pro's vendor ROS stack: no ROS 2, no robot, no vendor stack."""

from __future__ import annotations

from dataclasses import dataclass, field
import sys
import time
import types
from typing import Any

import pytest

from dimos.msgs.geometry_msgs.Twist import Twist as RosTwist
from dimos.msgs.geometry_msgs.Vector3 import Vector3 as RosVector3


@dataclass
class RosTime:
    sec: int = 0
    nanosec: int = 0


@dataclass
class RosHeader:
    stamp: RosTime = field(default_factory=RosTime)
    frame_id: str = ""


@dataclass
class RosJointState:
    """The vendor's feedback/command message: header, names, and parallel arrays."""

    header: RosHeader = field(default_factory=RosHeader)
    name: list[str] = field(default_factory=list)
    position: list[float] = field(default_factory=list)
    velocity: list[float] = field(default_factory=list)
    effort: list[float] = field(default_factory=list)


@dataclass
class RosTwistStamped:
    header: RosHeader = field(default_factory=RosHeader)
    twist: RosTwist = field(default_factory=RosTwist)


@dataclass
class RosImu:
    header: RosHeader = field(default_factory=RosHeader)


CHASSIS_FEEDBACK_TOPIC = "/motion_control/chassis_speed"


def feed_chassis_interval(fake: FakeRos, turn: float = 0.0) -> None:
    """Two real-topic measured samples, defining origin then integrating a turn."""
    now = time.time()
    start = now - 0.1
    fake.control.feed(CHASSIS_FEEDBACK_TOPIC, chassis_speed(0.0, 0.0, 0.0, start))
    fake.control.feed(CHASSIS_FEEDBACK_TOPIC, chassis_speed(0.0, 0.0, turn / (now - start), now))


def joint_feedback(positions: list[float], ts: float | None = None) -> RosJointState:
    """One segment's feedback frame: positions, at rest, no torque, stamped ``ts``."""
    stamp = RosTime()
    if ts is not None:
        stamp.sec = int(ts)
        stamp.nanosec = int((ts - stamp.sec) * 1e9)
    return RosJointState(
        header=RosHeader(stamp=stamp),
        position=list(positions),
        velocity=[0.0] * len(positions),
        effort=[0.0] * len(positions),
    )


def chassis_speed(vx: float, vy: float, wz: float, ts: float) -> RosTwistStamped:
    """The executed chassis speed the vendor stack reports, stamped ``ts``."""
    stamp = RosTime(sec=int(ts), nanosec=int((ts - int(ts)) * 1e9))
    return RosTwistStamped(
        header=RosHeader(stamp=stamp),
        twist=RosTwist(linear=RosVector3(vx, vy, 0.0), angular=RosVector3(0.0, 0.0, wz)),
    )


@dataclass
class FakeRawROSTopic:
    name: str
    msg_type: Any
    qos: Any = None


class FakeRawROS:
    """Stands in for ``RawROS``: records publishes, dispatches subscriptions."""

    def __init__(self, node_name: str = "") -> None:
        self.node_name = node_name
        self.started = False
        self.stopped = False
        #: Every (topic name, message) ever published, oldest first.
        self.published: list[tuple[str, Any]] = []
        self.subs: dict[str, list[Any]] = {}

    def start(self) -> None:
        self.started = True

    def stop(self) -> None:
        self.stopped = True

    def subscribe(self, topic: FakeRawROSTopic, cb: Any) -> Any:
        self.subs.setdefault(topic.name, []).append(cb)

        def unsubscribe() -> None:
            self.subs[topic.name].remove(cb)

        return unsubscribe

    def publish(self, topic: FakeRawROSTopic, msg: Any) -> None:
        self.published.append((topic.name, msg))

    def feed(self, topic_name: str, msg: Any) -> None:
        """Deliver ``msg`` to every subscriber of ``topic_name``, as the vendor would."""
        for cb in list(self.subs.get(topic_name, [])):
            cb(msg, None)

    def listener_count(self) -> int:
        """How many live subscriptions this node has (unsubscribed topics don't count)."""
        return sum(len(cbs) for cbs in self.subs.values())

    def sent_to(self, topic_name: str) -> list[Any]:
        """Every message published to ``topic_name``, oldest first."""
        return [msg for name, msg in self.published if name == topic_name]


class FakeRos:
    """The installed fakes: every ``FakeRawROS`` the module under test made."""

    def __init__(self) -> None:
        self.nodes: list[FakeRawROS] = []

    def _raw_ros(self, node_name: str = "") -> FakeRawROS:
        node = FakeRawROS(node_name)
        self.nodes.append(node)
        return node

    @property
    def control(self) -> FakeRawROS:
        """The control node of the most recent connection (the first RawROS it makes)."""
        return self.nodes[-2]

    @property
    def sensors(self) -> FakeRawROS:
        """The sensor node of the most recent connection (its second RawROS)."""
        return self.nodes[-1]


def install_fake_ros(monkeypatch: pytest.MonkeyPatch) -> FakeRos:
    """Point the connection's lazy ROS imports at the fakes; returns their registry."""
    fake = FakeRos()

    # Real module objects preserve lazy import behavior without a ROS installation.
    modules: dict[str, dict[str, Any]] = {
        "rclpy.qos": {
            "QoSProfile": types.SimpleNamespace,
            "ReliabilityPolicy": types.SimpleNamespace(BEST_EFFORT=1),
            "DurabilityPolicy": types.SimpleNamespace(VOLATILE=1),
        },
        "builtin_interfaces.msg": {"Time": RosTime},
        "sensor_msgs.msg": {
            "JointState": RosJointState,
            "Image": type("Image", (), {}),
            "CompressedImage": type("CompressedImage", (), {}),
            "Imu": RosImu,
        },
        "geometry_msgs.msg": {"TwistStamped": RosTwistStamped},
        "dimos.protocol.pubsub.impl.rospubsub": {
            "RawROS": fake._raw_ros,
            "RawROSTopic": FakeRawROSTopic,
        },
        "dimos.protocol.pubsub.impl.rospubsub_conversion": {
            "ros_to_dimos": lambda msg, cls: cls(
                frame_id=getattr(getattr(msg, "header", None), "frame_id", "")
            ),
        },
    }
    for name, attrs in modules.items():
        module = types.ModuleType(name)
        for attr, value in attrs.items():
            setattr(module, attr, value)
        monkeypatch.setitem(sys.modules, name, module)
        if not name.startswith("dimos."):
            parent, child = name.rsplit(".", 1)
            package = types.ModuleType(parent)
            setattr(package, child, module)
            monkeypatch.setitem(sys.modules, parent, package)
    return fake


class LocalTransport:
    """An in-process transport: publishing records and calls every subscriber."""

    def __init__(self) -> None:
        self.sent: list[Any] = []
        self.subscribers: list[Any] = []

    def broadcast(self, selfstream: Any, value: Any) -> None:
        self.sent.append(value)
        for cb in list(self.subscribers):
            cb(value)

    def publish(self, value: Any) -> None:
        self.broadcast(None, value)

    def subscribe(self, cb: Any, selfstream: Any = None) -> Any:
        self.subscribers.append(cb)
        return lambda: self.subscribers.remove(cb)

    def start(self) -> None:
        pass

    def stop(self) -> None:
        pass


#: Feedback the fake robot stands at: distinct per segment so a mix-up in the
#: flat layout shows up as a wrong value, not a coincidental zero. Every value
#: is well inside the URDF limits (the tightest is right_arm_joint2 at
#: ±0.1396 rad), so the coordinator's holds are never clamped.
TORSO_Q = [0.011, 0.012, 0.013, 0.014]
LEFT_Q = [0.021, 0.022, 0.023, 0.024, 0.025, 0.026, 0.027]
RIGHT_Q = [0.031, 0.032, 0.033, 0.034, 0.035, 0.036, 0.037]
MEASURED_Q = TORSO_Q + LEFT_Q + RIGHT_Q


def feed_all_segments(fake: FakeRos, ts: float | None = None, q: list[float] | None = None) -> None:
    """One feedback frame per segment, as the vendor stack publishes them."""
    values = MEASURED_Q if q is None else q
    stamp = time.time() if ts is None else ts
    fake.control.feed("/hdas/feedback_torso", joint_feedback(values[0:4], stamp))
    fake.control.feed("/hdas/feedback_arm_left", joint_feedback(values[4:11], stamp))
    fake.control.feed("/hdas/feedback_arm_right", joint_feedback(values[11:18], stamp))
