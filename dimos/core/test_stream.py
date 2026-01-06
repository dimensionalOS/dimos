# Copyright 2025 Dimensional Inc.
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

from collections.abc import Callable
import time

import pytest

from dimos.core import (
    In,
    LCMTransport,
    Module,
    rpc,
)
from dimos.core.testing import MockRobotClient, dimos
from dimos.robot.unitree_webrtc.type.lidar import LidarMessage
from dimos.robot.unitree_webrtc.type.odometry import Odometry

assert dimos


class SubscriberBase(Module):
    sub1_msgs: list[Odometry] = None
    sub2_msgs: list[Odometry] = None

    def __init__(self) -> None:
        self.sub1_msgs = []
        self.sub2_msgs = []
        super().__init__()

    @rpc
    def sub1(self) -> None: ...

    @rpc
    def sub2(self) -> None: ...

    @rpc
    def active_subscribers(self):
        return self.odom.transport.active_subscribers

    @rpc
    def sub1_msgs_len(self) -> int:
        return len(self.sub1_msgs)

    @rpc
    def sub2_msgs_len(self) -> int:
        return len(self.sub2_msgs)


class ClassicSubscriber(SubscriberBase):
    odom: In[Odometry]
    unsub: Callable[[], None] | None = None
    unsub2: Callable[[], None] | None = None

    @rpc
    def sub1(self) -> None:
        self.unsub = self.odom.subscribe(self.sub1_msgs.append)

    @rpc
    def sub2(self) -> None:
        self.unsub2 = self.odom.subscribe(self.sub2_msgs.append)

    @rpc
    def stop(self) -> None:
        if self.unsub:
            self.unsub()
            self.unsub = None
        if self.unsub2:
            self.unsub2()
            self.unsub2 = None


class RXPYSubscriber(SubscriberBase):
    odom: In[Odometry]
    unsub: Callable[[], None] | None = None
    unsub2: Callable[[], None] | None = None

    hot: Callable[[], None] | None = None

    @rpc
    def sub1(self) -> None:
        self.unsub = self.odom.observable().subscribe(self.sub1_msgs.append)

    @rpc
    def sub2(self) -> None:
        self.unsub2 = self.odom.observable().subscribe(self.sub2_msgs.append)

    @rpc
    def stop(self) -> None:
        if self.unsub:
            self.unsub.dispose()
            self.unsub = None
        if self.unsub2:
            self.unsub2.dispose()
            self.unsub2 = None

    @rpc
    def get_next(self):
        return self.odom.get_next()

    @rpc
    def start_hot_getter(self) -> None:
        self.hot = self.odom.hot_latest()

    @rpc
    def stop_hot_getter(self) -> None:
        self.hot.dispose()

    @rpc
    def get_hot(self):
        return self.hot()


class SpyLCMTransport(LCMTransport):
    active_subscribers: int = 0

    def __reduce__(self):
        return (SpyLCMTransport, (self.topic.topic, self.topic.lcm_type))

    def __init__(self, topic: str, type: type, **kwargs) -> None:
        super().__init__(topic, type, **kwargs)
        self._subscriber_map = {}  # Maps unsubscribe functions to track active subs

    def subscribe(self, selfstream: In, callback: Callable) -> Callable[[], None]:
        # Call parent subscribe to get the unsubscribe function
        unsubscribe_fn = super().subscribe(selfstream, callback)

        # Increment counter
        self.active_subscribers += 1

        def wrapped_unsubscribe() -> None:
            # Create wrapper that decrements counter when called
            if wrapped_unsubscribe in self._subscriber_map:
                self.active_subscribers -= 1
                del self._subscriber_map[wrapped_unsubscribe]
            unsubscribe_fn()

        # Track this subscription
        self._subscriber_map[wrapped_unsubscribe] = True

        return wrapped_unsubscribe


@pytest.mark.parametrize("subscriber_class", [ClassicSubscriber, RXPYSubscriber])
@pytest.mark.module
def test_subscription(dimos, subscriber_class) -> None:
    robot = dimos.deploy(MockRobotClient)

    robot.lidar.transport = SpyLCMTransport("/lidar", LidarMessage)
    robot.odometry.transport = SpyLCMTransport("/odom", Odometry)

    subscriber = dimos.deploy(subscriber_class)

    subscriber.odom.connect(robot.odometry)

    robot.start()
    subscriber.sub1()
    time.sleep(0.25)

    assert subscriber.sub1_msgs_len() > 0
    assert subscriber.sub2_msgs_len() == 0
    assert subscriber.active_subscribers() == 1

    subscriber.sub2()

    time.sleep(0.25)
    subscriber.stop()

    assert subscriber.active_subscribers() == 0
    assert subscriber.sub1_msgs_len() != 0
    assert subscriber.sub2_msgs_len() != 0

    total_msg_n = subscriber.sub1_msgs_len() + subscriber.sub2_msgs_len()

    time.sleep(0.25)

    # ensuring no new messages have passed through
    assert total_msg_n == subscriber.sub1_msgs_len() + subscriber.sub2_msgs_len()

    robot.stop()


@pytest.mark.module
def test_get_next(dimos) -> None:
    robot = dimos.deploy(MockRobotClient)

    robot.lidar.transport = SpyLCMTransport("/lidar", LidarMessage)
    robot.odometry.transport = SpyLCMTransport("/odom", Odometry)

    subscriber = dimos.deploy(RXPYSubscriber)
    subscriber.odom.connect(robot.odometry)

    robot.start()
    time.sleep(0.1)

    odom = subscriber.get_next()

    assert isinstance(odom, Odometry)
    assert subscriber.active_subscribers() == 0

    time.sleep(0.2)

    next_odom = subscriber.get_next()

    assert isinstance(next_odom, Odometry)
    assert subscriber.active_subscribers() == 0

    assert next_odom != odom
    robot.stop()


@pytest.mark.module
def test_hot_getter(dimos) -> None:
    robot = dimos.deploy(MockRobotClient)

    robot.lidar.transport = SpyLCMTransport("/lidar", LidarMessage)
    robot.odometry.transport = SpyLCMTransport("/odom", Odometry)

    subscriber = dimos.deploy(RXPYSubscriber)
    subscriber.odom.connect(robot.odometry)

    robot.start()

    # we are robust to multiple calls
    subscriber.start_hot_getter()
    time.sleep(0.2)
    odom = subscriber.get_hot()
    subscriber.stop_hot_getter()

    assert isinstance(odom, Odometry)
    time.sleep(0.3)

    # there are no subs
    assert subscriber.active_subscribers() == 0

    # we can restart though
    subscriber.start_hot_getter()
    time.sleep(0.3)

    next_odom = subscriber.get_hot()
    assert isinstance(next_odom, Odometry)
    assert next_odom != odom
    subscriber.stop_hot_getter()

    robot.stop()
