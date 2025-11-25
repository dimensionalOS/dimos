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
import pickle

import reactivex as rx
from reactivex import operators as ops
from reactivex.disposable import Disposable
from reactivex.observable import Observable

from dimos.msgs.nav_msgs import OccupancyGrid
from dimos.msgs.sensor_msgs import PointCloud2
from dimos.protocol.pubsub import lcm  # type: ignore[attr-defined]
from dimos.robot.unitree_webrtc.type.lidar import LidarMessage
from dimos.robot.unitree_webrtc.type.map import Map
from dimos.utils.data import get_data
from dimos.utils.reactive import backpressure
from dimos.utils.testing import TimedSensorReplay

Connection = Callable[[], Observable[OccupancyGrid]]


def live_connection() -> Observable[OccupancyGrid]:
    def subscribe(observer, scheduler=None):  # type: ignore[no-untyped-def]
        lcm.autoconf()
        l = lcm.LCM()

        def on_message(grid: OccupancyGrid, _) -> None:  # type: ignore[no-untyped-def]
            observer.on_next(grid)

        l.subscribe(lcm.Topic("/global_costmap", OccupancyGrid), on_message)
        l.start()

        def dispose() -> None:
            l.stop()

        return Disposable(dispose)

    return rx.create(subscribe)


def recorded_connection() -> Observable[OccupancyGrid]:
    lidar_store = TimedSensorReplay("unitree_office_walk/lidar", autocast=LidarMessage.from_msg)
    mapper = Map()
    return backpressure(
        lidar_store.stream(speed=1).pipe(
            ops.map(mapper.add_frame),
            ops.map(lambda _: mapper.costmap().inflate(0.1).gradient()),  # type: ignore[attr-defined]
        )
    )


def single_message() -> Observable[OccupancyGrid]:
    pointcloud_pickle = get_data("lcm_msgs") / "sensor_msgs/PointCloud2.pickle"
    with open(pointcloud_pickle, "rb") as f:
        pointcloud = PointCloud2.lcm_decode(pickle.load(f))
    mapper = Map()
    mapper.add_frame(pointcloud)
    return rx.just(mapper.costmap())  # type: ignore[attr-defined]
