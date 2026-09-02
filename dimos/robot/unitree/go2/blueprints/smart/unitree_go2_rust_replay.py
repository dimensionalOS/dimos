# Copyright 2026 Dimensional Inc.
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

"""Full Go2 stack driven by the opt-in native Memory2 replayer."""

import copy
from threading import Event, Thread

from reactivex.disposable import Disposable

from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.stream import In, Out
from dimos.experimental.memory.rust_replayer import RustReplayer
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.robot.unitree.go2.blueprints.basic.unitree_go2_basic import _with_vis
from dimos.robot.unitree.go2.blueprints.smart.unitree_go2 import _unitree_go2_stack
from dimos.robot.unitree.go2.connection import GO2Connection


class Go2RustReplayer(RustReplayer):
    """Native source for the three high-rate streams in Go2 recordings."""

    lidar: Out[PointCloud2]
    odom: Out[PoseStamped]
    color_image: Out[Image]


class Go2ReplaySupport(Module):
    """Derive the Go2 signals that are not stored in older recordings."""

    odom: In[PoseStamped]
    tf: Out[TFMessage]
    camera_info: Out[CameraInfo]

    _camera_info_stop: Event
    _camera_info_thread: Thread | None = None

    def __init__(self, **kwargs: object) -> None:
        super().__init__(**kwargs)
        self._camera_info_stop = Event()

    @rpc
    def start(self) -> None:
        super().start()
        self.register_disposable(Disposable(self.odom.subscribe(self._publish_tf)))
        self._camera_info_stop.clear()
        self._camera_info_thread = Thread(
            target=self._publish_camera_info,
            daemon=True,
            name="go2-replay-camera-info",
        )
        self._camera_info_thread.start()

    @rpc
    def stop(self) -> None:
        self._camera_info_stop.set()
        if self._camera_info_thread is not None:
            self._camera_info_thread.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)
            self._camera_info_thread = None
        super().stop()

    def _publish_tf(self, odom: PoseStamped) -> None:
        self.tf.publish(TFMessage(*GO2Connection._odom_to_tf(odom)))

    def _publish_camera_info(self) -> None:
        camera_info = copy.copy(GO2Connection.camera_info_static)
        while not self._camera_info_stop.is_set():
            self.camera_info.publish(camera_info)
            self._camera_info_stop.wait(1.0)


_rust_replay_source = autoconnect(
    _with_vis,
    Go2RustReplayer.blueprint(),
    Go2ReplaySupport.blueprint(),
)

unitree_go2_rust_replay = _unitree_go2_stack(_rust_replay_source).global_config(
    replay=True,
    n_workers=11,
    robot_model="unitree_go2",
)
