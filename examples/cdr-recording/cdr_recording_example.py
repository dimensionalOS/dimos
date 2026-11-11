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

"""Publish a standard ROS 2 pose for the CLI Rust recorder."""

import math
import time

from dimos_messages.builtin_interfaces.msg import Time
from dimos_messages.geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from dimos_messages.std_msgs.msg import Header
from reactivex import interval

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.core import rpc
from dimos.core.global_config import global_config
from dimos.core.module import Module
from dimos.core.stream import Out
from dimos.visualization.vis_module import vis_module


class MovingPose(Module):
    """Publish a pose moving around a one-metre circle at 10 Hz."""

    pose: Out[PoseStamped]

    @rpc
    def start(self) -> None:
        super().start()
        self.register_disposable(interval(0.1).subscribe(self._publish))

    def _publish(self, tick: int) -> None:
        angle = tick * 0.05
        sec, nanosec = divmod(time.time_ns(), 1_000_000_000)
        self.pose.publish(
            PoseStamped(
                header=Header(stamp=Time(sec=sec, nanosec=nanosec), frame_id="map"),
                pose=Pose(
                    position=Point(x=math.cos(angle), y=math.sin(angle), z=0.0),
                    orientation=Quaternion(
                        x=0.0, y=0.0, z=math.sin(angle / 2), w=math.cos(angle / 2)
                    ),
                ),
            )
        )


# The same sample with the standard viewer next to it: `--viewer rerun` adds the
# existing Rerun bridge (`RerunBridgeModule`), `--viewer none` adds nothing. One
# process then records the stream and shows it at the same time.
pose_view = autoconnect(MovingPose.blueprint(), vis_module(global_config.viewer))
