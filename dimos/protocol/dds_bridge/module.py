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

"""DdsBridge: join a DDS domain and republish ROS 2 topics as DimOS streams, in rust.

Each entry of ``dds_to_dimos`` becomes one output stream named ``zenoh_topic``, on the
same channel ``make_transport`` gives any other module's stream of that name and type,
so a consumer declaring e.g. ``odom: In[Odometry]`` receives it::

    DdsBridge.blueprint(
        domain_id=0,
        dds_to_dimos=[
            DdsToDimos(dds_topic="rt/SLAM_ODOM", dds_type="nav_msgs/msg/Odometry", zenoh_topic="odom"),
            DdsToDimos(
                dds_topic="rt/SLAM_CLOUD_REGISTERED_BODY",
                dds_type="sensor_msgs/msg/PointCloud2",
                zenoh_topic="lidar",
            ),
        ],
    )
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Any, Literal

from pydantic import BaseModel, ConfigDict, Field, field_validator, model_validator

from dimos.core.native_module import NativeModule, NativeModuleConfig
from dimos.core.transport_factory import make_transport
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.std_msgs.Bool import Bool
from dimos.msgs.std_msgs.Int32 import Int32
from dimos.msgs.std_msgs.UInt32 import UInt32
from dimos.msgs.tf2_msgs.TFMessage import TFMessage

if TYPE_CHECKING:
    from dimos.core.transport import PubSubTransport

# The stream type of each ROS 2 type; mirrors TABLE in rust/src/translate.rs, which checks the pairing at startup.
DIMOS_TYPE_OF_ROS_TYPE: dict[str, type] = {
    "std_msgs/msg/Int32": Int32,
    "std_msgs/msg/UInt32": UInt32,
    "std_msgs/msg/Bool": Bool,
    "geometry_msgs/msg/Twist": Twist,
    "geometry_msgs/msg/PoseStamped": PoseStamped,
    "nav_msgs/msg/Odometry": Odometry,
    "sensor_msgs/msg/PointCloud2": PointCloud2,
    "sensor_msgs/msg/Imu": Imu,
    "tf2_msgs/msg/TFMessage": TFMessage,
}


class DdsQos(BaseModel):
    """The reader QoS; DDS only matches a reader whose QoS the writer's satisfies."""

    model_config = ConfigDict(extra="forbid")

    # best_effort matches both reliable and best-effort writers.
    reliability: Literal["reliable", "best_effort"] = "best_effort"
    # volatile matches every writer; transient_local also receives a late-joiner's backlog.
    durability: Literal["volatile", "transient_local"] = "volatile"
    history_depth: int = Field(default=10, ge=1)


class DdsToDimos(BaseModel):
    """One DDS topic republished as one DimOS stream."""

    model_config = ConfigDict(extra="forbid")

    # The DDS topic as on the wire: ROS 2 topic `/odom` is DDS topic `rt/odom`.
    dds_topic: str = Field(min_length=1)
    # The ROS 2 interface, e.g. "nav_msgs/msg/Odometry".
    dds_type: str
    # The DimOS stream name the samples are published on.
    zenoh_topic: str = Field(min_length=1)
    # The registered DDS type name, for writers that do not follow ROS 2's `pkg::msg::dds_::Name_`.
    dds_type_name: str | None = None
    qos: DdsQos = Field(default_factory=DdsQos)

    @model_validator(mode="after")
    def _type_is_translated(self) -> DdsToDimos:
        if self.dds_type not in DIMOS_TYPE_OF_ROS_TYPE:
            raise ValueError(
                f"DDS topic {self.dds_topic!r} has type {self.dds_type!r}, which the bridge "
                f"cannot translate (supported: {', '.join(DIMOS_TYPE_OF_ROS_TYPE)})"
            )
        return self

    @property
    def dimos_type(self) -> type:
        return DIMOS_TYPE_OF_ROS_TYPE[self.dds_type]


class DdsBridgeConfig(NativeModuleConfig):
    source_dir: str | None = "dimos/protocol/dds_bridge/rust"
    executable: str = "result/bin/dds_bridge"
    build_command: str | None = "nix build -L path:."
    stdin_config: bool = True

    # The DDS domain to join (ROS_DOMAIN_ID for ROS 2).
    domain_id: int = Field(default=0, ge=0, le=232)
    # Local interface addresses DDS may use; empty uses every interface.
    network_addresses: list[str] = Field(default_factory=list)
    dds_to_dimos: list[DdsToDimos] = Field(default_factory=list)

    @field_validator("dds_to_dimos")
    @classmethod
    def _one_topic_per_stream(cls, bridges: list[DdsToDimos]) -> list[DdsToDimos]:
        streams = [bridge.zenoh_topic for bridge in bridges]
        duplicates = sorted({stream for stream in streams if streams.count(stream) > 1})
        if duplicates:
            raise ValueError(f"several DDS topics publish onto the stream(s) {duplicates}")
        return bridges


class DdsBridge(NativeModule):
    """Republishes the configured DDS topics as DimOS streams.

    Its streams come from config, not class annotations, so blueprint wiring does
    not see them: each lands on the default channel for its name and type, and a
    blueprint transport override for that name does not reach it.
    """

    config: DdsBridgeConfig

    def _bridge_transports(self) -> dict[str, PubSubTransport[Any]]:
        return {
            bridge.zenoh_topic: make_transport(f"/{bridge.zenoh_topic}", bridge.dimos_type)
            for bridge in self.config.dds_to_dimos
        }

    def _collect_topics(self) -> dict[str, str]:
        topics = super()._collect_topics()
        topics.update({port: t.channel for port, t in self._bridge_transports().items()})
        return topics

    def _collect_output_qos(self) -> dict[str, dict[str, str]]:
        qos = super()._collect_output_qos()
        for transport in self._bridge_transports().values():
            publish_qos = getattr(transport, "publish_qos", None)
            if publish_qos:
                qos[transport.channel] = publish_qos
        return qos


if TYPE_CHECKING:
    DdsBridge()
