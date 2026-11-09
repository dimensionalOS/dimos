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

"""Twist-driven Habitat robot connection: RGB-D, pose and a world-frame scan."""

from __future__ import annotations

from functools import partial
from pathlib import Path
from typing import Any
from uuid import uuid4

from dimos_lcm.geometry_msgs.Twist import Twist as WireTwist
from dimos_lcm.geometry_msgs.Vector3 import Vector3 as WireVector3
from pydantic import Field
from reactivex.disposable import Disposable

from dimos.core.native_module import LogFormat, NativeModule, NativeModuleConfig
from dimos.core.stream import In, Out
from dimos.memory.codecs.legacy import decode_lcm
from dimos.messages.geometry_msgs.msg import Twist
from dimos.messages.nav_msgs.msg import Odometry
from dimos.messages.sensor_msgs.msg import CameraInfo, Image, PointCloud2
from dimos.messages.tf2_msgs.msg import TFMessage
from dimos.messages.vision_msgs.msg import Detection3DArray
from dimos.protocol.pubsub.impl.zenohpubsub import Topic, ZenohPubSubBase
from dimos.simulation.habitat.constants import HABITAT_ROOT, HM3D_EXAMPLE_DATASET_CONFIG
from dimos.simulation.habitat.server import HabitatProp


def _default_scene_dataset_config(config: dict[str, Any]) -> str:
    executable = Path(config["executable"])
    if config["source_dir"] is None:
        executable = executable.absolute()
    # The installer places data next to habitat-native in both launch modes.
    return str(executable.parent / HM3D_EXAMPLE_DATASET_CONFIG.relative_to(HABITAT_ROOT))


class HabitatConnectionConfig(NativeModuleConfig):
    """Scene and camera settings for the Habitat native process."""

    # habitat-sim is python 3.9 conda-only, so it runs in its own env under
    # target/habitat (outside the package tree); the wrapper is the build sentinel.
    source_dir: str | None = "dimos/simulation/habitat/nix"
    executable: str = "../../../../target/habitat/habitat-native"
    build_command: str | None = "nix develop path:. -c ./install.sh"
    stdin_config: bool = True
    log_format: LogFormat = LogFormat.TEXT

    # Annotated HM3D house, no Matterport credentials needed.
    scene_dataset_config: str = Field(default_factory=_default_scene_dataset_config)
    scene_id: str = "00861-GLAQ4DNUx5U"
    # ROS yaw, +left. 90 faces into the room in the default scene.
    start_yaw_deg: float = 90.0
    start_position_ros: tuple[float, float, float] | None = None

    # Defaults are Go2-ish, not GOAT-Bench's 1.41 m Stretch.
    width: int = 640
    height: int = 360
    hfov_deg: float = Field(default=90.0, gt=0.0, lt=180.0)
    camera_height_m: float = Field(default=0.45, gt=0.0)
    max_depth_m: float = Field(default=5.0, gt=0.0)

    sim_rate_hz: float = Field(default=10.0, gt=0.0)
    # Zero the command when stale, like the real connections.
    cmd_vel_timeout_s: float = Field(default=0.2, gt=0.0)
    # Subsample depth before unprojection; 2 is 4x fewer points.
    scan_stride: int = Field(default=2, ge=1)
    seed: int = 0
    publish_semantic: bool = False
    # Visible annotated instances as world-frame boxes (semantic ids + depth).
    publish_objects: bool = False
    objects_hz: float = Field(default=1.0, gt=0.0)
    # Static models placed in the scene.
    props: tuple[HabitatProp, ...] = ()
    # Unprojection is the frame's main cost; off for teleop-only stacks.
    publish_scan: bool = True
    # "world" pre-registers the scan for VoxelGridMapper; "camera_optical" lets
    # RayTracingVoxelMap register it via tf.
    scan_frame: str = "world"


class HabitatConnection(NativeModule):
    """Drive a Habitat scene with Twist; publish what a depth robot would see."""

    config: HabitatConnectionConfig

    cmd_vel: In[Twist]

    color_image: Out[Image]
    depth_image: Out[Image]
    camera_info: Out[CameraInfo]
    registered_scan: Out[PointCloud2]
    odometry: Out[Odometry]
    tf: Out[TFMessage]
    semantic_image: Out[Image]
    objects: Out[Detection3DArray]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._wire_prefix = f"habitat-native/{uuid4().hex}"
        self._wire: ZenohPubSubBase | None = None

    def _collect_topics(self) -> dict[str, str]:
        return {name: f"{self._wire_prefix}/{name}" for name in super()._collect_topics()}

    def _receive_wire(self, name: str, kind: type, data: bytes, _: Topic) -> None:
        getattr(self, name).publish(decode_lcm(data, kind))

    def _send_command(self, command: Twist) -> None:
        assert self._wire is not None
        linear, angular = command.linear, command.angular
        payload = WireTwist(
            WireVector3(linear.x, linear.y, linear.z), WireVector3(angular.x, angular.y, angular.z)
        ).lcm_encode()
        self._wire.publish(Topic(f"{self._wire_prefix}/cmd_vel"), payload)

    def start(self) -> None:
        # Habitat's independent Python 3.9 process keeps its raw LCM ABI. Only
        # this process boundary translates it; every DimOS port is generated.
        wire = ZenohPubSubBase()
        wire.start()
        self._wire = wire
        self.register_disposable(Disposable(wire.stop))
        kinds = {
            "color_image": Image,
            "depth_image": Image,
            "camera_info": CameraInfo,
            "registered_scan": PointCloud2,
            "odometry": Odometry,
            "tf": TFMessage,
            "semantic_image": Image,
            "objects": Detection3DArray,
        }
        topics = self._collect_topics()
        for name, kind in kinds.items():
            if name in topics:
                wire.subscribe(Topic(topics[name]), partial(self._receive_wire, name, kind))
        if "cmd_vel" in topics:
            self.register_disposable(Disposable(self.cmd_vel.subscribe(self._send_command)))
        super().start()
