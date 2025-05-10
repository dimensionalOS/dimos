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
import asyncio
import functools
import enum
import reactivex as rx
from reactivex import operators as ops
from reactivex.disposable import Disposable
from reactivex.scheduler import ThreadPoolScheduler
from rxpy_backpressure import BackPressure

from nav_msgs import msg
from dimos.utils.logging_config import setup_logger
from dimos.utils.threadpool import get_scheduler
from dimos.types.costmap import Costmap
from dimos.types.vector import Vector

from typing import Union, Callable, Any

from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
    QoSDurabilityPolicy,
)

__all__ = ["ROSObservableTopicAbility", "QOS"]

ConversionType = Costmap
TopicType = Union[ConversionType, msg.OccupancyGrid, msg.Odometry]


class QOS(enum.Enum):
    SENSOR = "sensor"
    COMMAND = "command"

    def to_profile(self) -> QoSProfile:
        if self == QOS.SENSOR:
            return QoSProfile(
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
                history=QoSHistoryPolicy.KEEP_LAST,
                durability=QoSDurabilityPolicy.VOLATILE,
                depth=1,
            )
        if self == QOS.COMMAND:
            return QoSProfile(
                reliability=QoSReliabilityPolicy.RELIABLE,
                history=QoSHistoryPolicy.KEEP_LAST,
                durability=QoSDurabilityPolicy.VOLATILE,
                depth=10,  # Higher depth for commands to ensure delivery
            )

        raise ValueError(f"Unknown QoS enum value: {self}")


logger = setup_logger("dimos.robot.ros_control.observable_topic")


class ROSObservableTopicAbility:
    # Ensures that we can return multiple observables which have multiple subscribers
    # consuming the same topic at different (blocking) rates while:
    #
    # - immediately returning latest value received to new subscribers
    # - allowing slow subscribers to consume the topic without blocking fast ones
    # - dealing with backpressure from slow subscribers (auto dropping unprocessed messages)
    #
    # (for more details see corresponding test file)
    #
    # ROS thread ─► ReplaySubject─► observe_on(pool) ─► backpressure.latest ─► sub1 (fast)
    #                          ├──► observe_on(pool) ─► backpressure.latest ─► sub2 (slow)
    #                          └──► observe_on(pool) ─► backpressure.latest ─► sub3 (slower)
    #
    def _maybe_conversion(self, msg_type: TopicType, callback) -> Callable[[TopicType], Any]:
        if msg_type == Costmap:
            return lambda msg: callback(Costmap.from_msg(msg))
        # just for test, not sure if this Vector auto-instantiation is used irl
        if msg_type == Vector:
            return lambda msg: callback(Vector.from_msg(msg))
        return callback

    def _sub_msg_type(self, msg_type):
        if msg_type == Costmap:
            return msg.OccupancyGrid

        if msg_type == Vector:
            return msg.Odometry

        return msg_type

    @functools.lru_cache(maxsize=None)
    def topic(
        self,
        topic_name: str,
        msg_type: TopicType,
        qos=QOS.SENSOR,
        scheduler: ThreadPoolScheduler | None = None,
        drop_unprocessed: bool = True,
    ) -> rx.Observable:
        if scheduler is None:
            scheduler = get_scheduler()

        # Convert QOS to QoSProfile
        qos_profile = qos.to_profile()

        # upstream ROS callback
        def _on_subscribe(obs, _):
            ros_sub = self._node.create_subscription(
                self._sub_msg_type(msg_type), topic_name, self._maybe_conversion(msg_type, obs.on_next), qos_profile
            )
            return Disposable(lambda: self._node.destroy_subscription(ros_sub))

        upstream = rx.create(_on_subscribe)

        # hot, latest-cached core
        core = upstream.pipe(
            ops.replay(buffer_size=1),
            ops.ref_count(),  # still synchronous!
        )

        # per-subscriber factory
        def per_sub():
            # hop off the ROS thread into the pool
            base = core.pipe(ops.observe_on(scheduler))

            # optional back-pressure handling
            if not drop_unprocessed:
                return base

            def _subscribe(observer, sch=None):
                return base.subscribe(BackPressure.LATEST(observer), scheduler=sch)

            return rx.create(_subscribe)

        # each `.subscribe()` call gets its own async backpressure chain
        return rx.defer(lambda *_: per_sub())

    # If you are not interested in processing streams, just want to fetch the latest stream
    # value use this function. It runs a subscription in the background.
    # caches latest value for you, always ready to return.
    #
    # odom = robot.topic_latest("/odom", msg.Odometry)
    # the initial call to odom() will block until the first message is received
    #
    # any time you'd like you can call:
    #
    # print(f"Latest odom: {odom()}")
    # odom.dispose()  # clean up the subscription
    #
    # see test_ros_observable_topic.py test_topic_latest for more details
    def topic_latest(self, topic_name: str, msg_type: TopicType, timeout: float | None = 30.0, qos=QOS.SENSOR):
        """
        Blocks the current thread until the first message is received, then
        returns `reader()` (sync) and keeps one ROS subscription alive
        in the background.

            latest_scan = robot.ros_control.topic_latest_blocking("scan", LaserScan)
            do_something(latest_scan())       # instant
            latest_scan.dispose()             # clean up
        """
        # one shared observable with a 1-element replay buffer
        core = self.topic(topic_name, msg_type, qos=qos).pipe(ops.replay(buffer_size=1))
        conn = core.connect()  # starts the ROS subscription immediately

        try:
            first_val = core.pipe(ops.first(), *([ops.timeout(timeout)] if timeout is not None else [])).run()
        except Exception:
            conn.dispose()
            msg = f"{topic_name} message not received after {timeout} seconds. Is robot connected?"
            logger.error(msg)
            raise Exception(msg)

        cache = {"val": first_val}
        sub = core.subscribe(lambda v: cache.__setitem__("val", v))

        def reader():
            return cache["val"]

        reader.dispose = lambda: (sub.dispose(), conn.dispose())
        return reader

    # If you are not interested in processing streams, just want to fetch the latest stream
    # value use this function. It runs a subscription in the background.
    # caches latest value for you, always ready to return
    #
    # odom = await robot.topic_latest_async("/odom", msg.Odometry)
    #
    # async nature of this function allows you to do other stuff while you wait
    # for a first message to arrive
    #
    # any time you'd like you can call:
    #
    # print(f"Latest odom: {odom()}")
    # odom.dispose()  # clean up the subscription
    #
    # see test_ros_observable_topic.py test_topic_latest for more details
    async def topic_latest_async(self, topic_name: str, msg_type: TopicType, qos=QOS.SENSOR, timeout: float = 30.0):
        loop = asyncio.get_running_loop()
        first = loop.create_future()
        cache = {"val": None}

        core = self.topic(topic_name, msg_type, qos=qos)  # single ROS callback

        def _on_next(v):
            cache["val"] = v
            if not first.done():
                loop.call_soon_threadsafe(first.set_result, v)

        subscription = core.subscribe(_on_next)

        try:
            await asyncio.wait_for(first, timeout)
        except Exception:
            subscription.dispose()
            raise

        def reader():
            return cache["val"]

        reader.dispose = subscription.dispose
        return reader
