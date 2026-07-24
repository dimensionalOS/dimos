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

from __future__ import annotations

from reactivex.disposable import Disposable

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.protocol.pubsub.impl.lcmpubsub import LCMPubSubBase, Topic as LCMTopic
from dimos.protocol.pubsub.impl.zenohpubsub import (
    QOS_LATEST_WINS,
    Topic as ZenohTopic,
    ZenohPubSubBase,
)

_LIDAR_CHANNEL = "dimos/lidar/sensor_msgs.PointCloud2"
_ODOMETRY_CHANNEL = "dimos/odometry/nav_msgs.Odometry"


class PointLioZenohRelay(Module):
    config: ModuleConfig

    @rpc
    def start(self) -> None:
        super().start()
        lcm = LCMPubSubBase()
        zenoh = ZenohPubSubBase()
        lcm.start()
        zenoh.start()

        lidar = ZenohTopic("dimos/lidar", PointCloud2, qos=QOS_LATEST_WINS)
        odometry = ZenohTopic("dimos/odometry", Odometry)
        self.register_disposable(
            Disposable(
                lcm.subscribe(
                    LCMTopic(_LIDAR_CHANNEL),
                    lambda payload, _: zenoh.publish(lidar, payload),
                )
            )
        )
        self.register_disposable(
            Disposable(
                lcm.subscribe(
                    LCMTopic(_ODOMETRY_CHANNEL),
                    lambda payload, _: zenoh.publish(odometry, payload),
                )
            )
        )
        self.register_disposable(Disposable(lcm.stop))
        self.register_disposable(Disposable(zenoh.stop))
