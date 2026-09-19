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

"""The Go2 as it appears on the graph when it runs the go2web zenoh bridge.

The bridge (go2web ``src/dimos_zenoh.rs``) publishes Point-LIO odom, the clouds and H.264
video and consumes ``cmd_vel``/``command``; nothing here produces them, declaring the ports
is what puts them on the graph. On top of :class:`Go2Base` this adds the
``odom -> mid360_link`` tf edge the bridge does not send.
"""

from __future__ import annotations

from reactivex.disposable import Disposable

from dimos.core.core import rpc
from dimos.core.stream import Out
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.robot.unitree.go2.base import Go2Base, Go2BaseConfig
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class GO2ZenohConfig(Go2BaseConfig):
    pass


class GO2Zenoh(Go2Base):
    """The go2's zenoh-side streams, plus the static data the robot doesn't send."""

    config: GO2ZenohConfig

    lidar: Out[PointCloud2]  # per-scan, in the LIO's own sensor frame
    pointlio_map: Out[PointCloud2]  # accumulated world map, frame `odom`

    @rpc
    def start(self) -> None:
        super().start()
        self.register_disposable(
            Disposable(self.odometry.transport.subscribe(self._publish_tf, self.odometry))
        )

    def _startup_pose(self) -> None:
        """Stand, and drop the head L1: Point-LIO runs off the MID-360, not that one."""
        super()._startup_pose()
        self.set_lidar(False)

    @rpc
    def stop(self) -> None:
        self.liedown()
        super().stop()

    def transforms(self) -> list[Transform]:
        """The mount tree, rooted at mid360_link because Point-LIO owns that frame.

        Measured outward from the body, but odom -> mid360_link is the only live edge, so
        the two edges above the lidar are inverted, otherwise mid360_link has two parents
        and the body snaps between them at 35 Hz.
        """
        edges = self.mount_edges()
        return [-edges["mid360_link"], -edges["front_camera"], edges["camera_optical"]]

    # The bridge knows sport verbs, api ids and the L1 switch only.
    def _unsupported(self, name: str) -> None:
        logger.warning("%s is not a go2web bridge verb; use GO2DDS", name)

    @rpc
    def set_obstacle_avoidance(self, enabled: bool = True) -> None:
        self._unsupported("set_obstacle_avoidance")

    @rpc
    def set_rage_mode(self, enable: bool) -> None:
        self._unsupported("set_rage_mode")

    @rpc
    def switch_joystick(self, enable: bool = True) -> None:
        self._unsupported("switch_joystick")

    @rpc
    def set_light(self, level: int) -> None:
        self._unsupported("set_light")

    @rpc
    def set_led(self, color: str) -> None:
        self._unsupported("set_led")

    @rpc
    def set_volume(self, level: int) -> None:
        self._unsupported("set_volume")

    def _publish_tf(self, odom: Odometry) -> None:
        """The one moving edge, odom -> mid360_link; the bridge publishes no tf."""
        self.tf.publish(TFMessage(Transform.from_pose(odom.child_frame_id, odom.to_pose_stamped())))
