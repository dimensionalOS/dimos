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
import reactivex as rx
from reactivex import operators as ops
from reactivex.disposable import Disposable
from reactivex.scheduler import ThreadPoolScheduler
from rxpy_backpressure import BackPressure

from nav_msgs import msg
from dimos.utils.logging_config import setup_logger
from dimos.utils.threadpool import get_scheduler

from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
    QoSDurabilityPolicy,
)

__all__ = ["ROSObservableTopicAbility"]

# TODO: should go to some shared file, this is copy pasted from ros_control.py
sensor_qos = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    durability=QoSDurabilityPolicy.VOLATILE,
    depth=1,
)

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
    @functools.lru_cache(maxsize=None)
    def topic(
        self,
        topic_name: str,
        msg_type: msg,
        qos=sensor_qos,
        scheduler: ThreadPoolScheduler | None = None,
        drop_unprocessed: bool = True,
    ) -> rx.Observable:
        if scheduler is None:
            scheduler = get_scheduler()

        # upstream ROS callback
        def _on_subscribe(obs, _):
            ros_sub = self._node.create_subscription(msg_type, topic_name, obs.on_next, qos)
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

    # if you are not interested in processing streams, just want to fetch the latest stream
    # value in a sync fashion, use this function.
    #
    # odom = await robot.topic_latest("/odom", msg.Odometry)
    #
    # any time you'd like you can call:
    # print(f"Latest odom: {odom()}")
    # odom.dispose()  # clean up the subscription
    #
    # see test_ros_observable_topic.py test_topic_latest for more details
    async def topic_latest(self, topic_name: str, msg_type: msg, qos=sensor_qos, timeout: float = 10.0):
        loop = asyncio.get_running_loop()
        first = loop.create_future()
        cache = {"val": None}

        core = self.topic(topic_name, msg_type)  # single ROS callback

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
