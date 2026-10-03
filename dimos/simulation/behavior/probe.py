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

"""A stream observer and command publisher used by integration demos."""

from functools import partial
import threading
from typing import Any

import numpy as np
from reactivex.disposable import Disposable

from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.tf2_msgs.TFMessage import TFMessage


class BehaviorProbe(Module):
    color_image: In[Image]
    depth_image: In[Image]
    camera_info: In[CameraInfo]
    left_wrist_image: In[Image]
    left_wrist_depth: In[Image]
    left_wrist_camera_info: In[CameraInfo]
    right_wrist_depth: In[Image]
    right_wrist_camera_info: In[CameraInfo]
    right_wrist_image: In[Image]
    joint_state: In[JointState]
    odometry: In[Odometry]
    tf: In[TFMessage]
    cmd_vel: Out[Twist]
    joint_command: Out[JointState]
    native_action: Out[list[float]]

    def __init__(self, **kwargs: Any) -> None:
        self._lock = threading.Lock()
        self._latest: dict[str, Any] = {}
        super().__init__(**kwargs)

    @rpc
    def start(self) -> None:
        super().start()
        for name, stream in self.inputs.items():
            self.register_disposable(Disposable(stream.subscribe(partial(self._record, name))))

    def _record(self, name: str, message: Any) -> None:
        with self._lock:
            self._latest[name] = message

    @rpc
    def snapshot(self) -> dict[str, Any]:
        with self._lock:
            result: dict[str, Any] = {"received": sorted(self._latest)}
            if "odometry" in self._latest:
                pose = self._latest["odometry"].position
                result["position"] = [pose.x, pose.y, pose.z]
            if "joint_state" in self._latest:
                msg = self._latest["joint_state"]
                result["joints"] = dict(zip(msg.name, msg.position, strict=True))
            result["sensors"] = {}
            for name, message in self._latest.items():
                if isinstance(message, Image):
                    result["sensors"][name] = {
                        "shape": list(message.data.shape),
                        "dtype": str(message.data.dtype),
                        "frame": message.frame_id,
                        "ts": message.ts,
                    }
                    if "depth" in name:
                        result["sensors"][name]["valid_depth_pixels"] = int(
                            np.count_nonzero(np.isfinite(message.data) & (message.data > 0))
                        )
                elif isinstance(message, CameraInfo):
                    result["sensors"][name] = {
                        "width": message.width,
                        "height": message.height,
                        "K": list(message.K),
                        "frame": message.frame_id,
                        "ts": message.ts,
                    }
            return result

    @rpc
    def drive(self, x: float = 0, y: float = 0, yaw: float = 0) -> None:
        self.cmd_vel.publish(Twist(linear=[x, y, 0], angular=[0, 0, yaw]))

    @rpc
    def set_joints(self, targets: dict[str, float]) -> None:
        self.joint_command.publish(JointState(name=list(targets), position=list(targets.values())))

    @rpc
    def send_native(self, action: list[float]) -> None:
        self.native_action.publish(action)
