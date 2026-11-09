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

"""Python NativeModule wrapper for the Rust Livox Mid-360 driver.

Usage::
    from dimos.hardware.sensors.lidar.livox.module import Mid360
    from dimos.core.coordination.blueprints import autoconnect

    from dimos.core.coordination.module_coordinator import ModuleCoordinator
    ModuleCoordinator.build(autoconnect(
        Mid360.blueprint(lidar_ip="192.168.1.155"),  # host_ip auto-detected
        SomeConsumer.blueprint(),
    )).loop()

The sensor also describes itself as USD over the ``get_usd`` RPC, so a UsdManager can take it
as a ``mounts`` value. The layer is baked into ``_MID360_USD`` rather than converted on the
fly, because usd-core has no linux-aarch64 wheel and this module runs on a Jetson; the
constant is the whole artifact, so its provenance is written above it.
"""

from __future__ import annotations

from pathlib import Path
import sys
from typing import TYPE_CHECKING, Any, Literal

from pydantic import Field, field_validator

from dimos.core.core import rpc
from dimos.core.native_module import NativeModule, NativeModuleConfig
from dimos.core.stream import Out
from dimos.hardware.sensors.lidar.livox.net import resolve_host_ip
from dimos.hardware.sensors.lidar.livox.ports import (
    SDK_CMD_DATA_PORT,
    SDK_HOST_CMD_DATA_PORT,
    SDK_HOST_IMU_DATA_PORT,
    SDK_HOST_POINT_DATA_PORT,
    SDK_HOST_PUSH_MSG_PORT,
    SDK_IMU_DATA_PORT,
    SDK_MULTICAST_GROUP,
    SDK_POINT_DATA_PORT,
    SDK_PUSH_MSG_PORT,
)
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.spec import perception


class Mid360Config(NativeModuleConfig):
    source_dir: str | None = "dimos/hardware/sensors/lidar/livox/rust"
    executable: str = "result/bin/mid360_native"
    build_command: str | None = "nix build -L path:."
    stdin_config: bool = True
    base_fields: frozenset[str] = frozenset({"frame_id"})
    # None derives host_ip from a NIC on the lidar's subnet; a box with two links
    # into that subnet sets it explicitly.
    host_ip: str | None = None
    # Required for a live sensor.
    lidar_ip: str | None = None
    frequency: float = 10.0
    enable_imu: bool = True
    # Replay this capture instead of a live sensor. host_ip/lidar_ip are unused.
    pcap: str | None = None
    # Replay speed relative to capture time. None runs flat-out.
    replay_rate: float | None = Field(default=1.0, gt=0)
    # Seconds a replay holds its first packet, so consumers can subscribe first.
    replay_delay: float = Field(default=0.0, ge=0)
    # Multicast group the device streams to. None receives unicast only, which
    # loopback replay needs and macOS requires (see virtual_mid360).
    multicast_ip: str | None = Field(
        default_factory=lambda: None if sys.platform == "darwin" else SDK_MULTICAST_GROUP
    )
    # Wire layout per point:
    #   "minimal" x,y,z,offset_time                    - 16 B (default)
    #   "full"    x,y,z,intensity,offset_time,tag,line - 22 B
    #   "legacy"  x,y,z,intensity                      - 16 B
    point_format: Literal["full", "minimal", "legacy"] = "minimal"
    frame_id: str = "lidar_link"
    imu_frame_id: str = "imu_link"

    # SDK port configuration (see livox/ports.py for defaults)
    cmd_data_port: int = SDK_CMD_DATA_PORT
    push_msg_port: int = SDK_PUSH_MSG_PORT
    point_data_port: int = SDK_POINT_DATA_PORT
    imu_data_port: int = SDK_IMU_DATA_PORT
    host_cmd_data_port: int = SDK_HOST_CMD_DATA_PORT
    host_push_msg_port: int = SDK_HOST_PUSH_MSG_PORT
    host_point_data_port: int = SDK_HOST_POINT_DATA_PORT
    host_imu_data_port: int = SDK_HOST_IMU_DATA_PORT

    @field_validator("pcap")
    @classmethod
    def _pcap_path_present(cls, value: str | None) -> str | None:
        if value == "":
            raise ValueError("pcap is empty; set DIMOS_MID360_PCAP or pass a path")
        return value

    def to_config_dict(self) -> dict[str, Any]:
        config = super().to_config_dict()
        # The rust struct has every key. None crosses as an explicit null.
        for key in ("host_ip", "lidar_ip", "pcap", "replay_rate", "multicast_ip"):
            config[key] = getattr(self, key)
        return config


# The MID-360 as a USD layer, baked rather than converted at runtime: usd-core has no
# linux-aarch64 wheel, and this module's whole job happens on a Jetson.
#
# This constant is the artifact. It was produced one-off by writing the sensor as a URDF and
# running it through dimos.robot.assets.usd.urdf_to_usd; that URDF is not checked in, so the
# numbers here are the record. To change it, write the URDF again from these notes, convert,
# and re-insert the @LIDAR@ / @IMU@ placeholders into the displayName lines.
#
# Where the numbers come from:
#   * the 65 x 65 x 60 mm shell, from the Livox MID-360 specs (265 g, four M3 holes in the base)
#   * the lidar_link_to_imu_link offset (0.011, 0.02329, -0.04412) m, which is
#     PointLioConfig.extrinsic_t -- the lidar origin in the IMU frame -- negated. That default
#     is the in-repo source of truth for this extrinsic; keep the two in step.
#   * the -0.020 m shell offset is a PLACEHOLDER, inferred from the IMU sitting 44.12 mm below
#     the origin and the body being 60 mm tall. It moves the drawn box and nothing measured.
#
# The root link is the point cloud origin -- the frame the driver stamps its cloud with -- not
# the mounting face, so a mount transform onto this points at the optical origin.
#
# @LIDAR@ / @IMU@ stand in for the configured frame names and appear only in displayName, the
# name that travels; prim names stay fixed USD identifiers, so none of this depends on pxr.
_LIDAR_TOKEN = "@LIDAR@"
_IMU_TOKEN = "@IMU@"

_MID360_USD = """\
#usda 1.0
(
    defaultPrim = "mid360"
    metersPerUnit = 1
    upAxis = "Z"
)

def Xform "mid360"
{
    def Xform "lidar_link" (
        prepend apiSchemas = ["PhysicsRigidBodyAPI"]
        displayName = "@LIDAR@"
    )
    {
        def Xform "lidar_link_body" (
            prepend apiSchemas = ["PhysicsRigidBodyAPI"]
            displayName = "@LIDAR@_body"
        )
        {
            matrix4d xformOp:transform = ( (1, 0, 0, 0), (0, 1, 0, 0), (0, 0, 1, 0), (0, 0, -0.02, 1) )
            uniform token[] xformOpOrder = ["xformOp:transform"]

            def Cube "visual_0"
            {
                color3f[] primvars:displayColor = [(0.25, 0.25, 0.27)]
                double size = 1
                float3 xformOp:scale = (0.065, 0.065, 0.06)
                matrix4d xformOp:transform = ( (1, 0, 0, 0), (0, 1, 0, 0), (0, 0, 1, 0), (0, 0, 0, 1) )
                uniform token[] xformOpOrder = ["xformOp:scale", "xformOp:transform"]
            }

            def Cube "collision_1" (
                prepend apiSchemas = ["PhysicsCollisionAPI"]
            )
            {
                double size = 1
                token visibility = "invisible"
                float3 xformOp:scale = (0.065, 0.065, 0.06)
                matrix4d xformOp:transform = ( (1, 0, 0, 0), (0, 1, 0, 0), (0, 0, 1, 0), (0, 0, 0, 1) )
                uniform token[] xformOpOrder = ["xformOp:scale", "xformOp:transform"]
            }
        }

        def Xform "imu_link" (
            prepend apiSchemas = ["PhysicsRigidBodyAPI"]
            displayName = "@IMU@"
        )
        {
            matrix4d xformOp:transform = ( (1, 0, 0, 0), (0, 1, 0, 0), (0, 0, 1, 0), (0.011, 0.02329, -0.04412, 1) )
            uniform token[] xformOpOrder = ["xformOp:transform"]
        }
    }

    def Scope "joints"
    {
        def PhysicsFixedJoint "lidar_link_to_body" (
            displayName = "@LIDAR@_to_body"
        )
        {
            rel physics:body0 = </mid360/lidar_link>
            rel physics:body1 = </mid360/lidar_link/lidar_link_body>
            point3f physics:localPos0 = (0, 0, -0.02)
            point3f physics:localPos1 = (0, 0, 0)
            quatf physics:localRot0 = (1, 0, 0, 0)
            quatf physics:localRot1 = (1, 0, 0, 0)
        }

        def PhysicsFixedJoint "lidar_link_to_imu_link" (
            displayName = "@LIDAR@_to_@IMU@"
        )
        {
            rel physics:body0 = </mid360/lidar_link>
            rel physics:body1 = </mid360/lidar_link/imu_link>
            point3f physics:localPos0 = (0.011, 0.02329, -0.04412)
            point3f physics:localPos1 = (0, 0, 0)
            quatf physics:localRot0 = (1, 0, 0, 0)
            quatf physics:localRot1 = (1, 0, 0, 0)
        }
    }
}

"""


def mid360_usd(lidar_frame: str, imu_frame: str) -> str:
    """The baked MID-360 layer with its frame names filled in."""
    return _MID360_USD.replace(_LIDAR_TOKEN, lidar_frame).replace(_IMU_TOKEN, imu_frame)


def _resolved_host_ip(config: Mid360Config) -> str | None:
    """Live mode derives host_ip from a NIC on the lidar's subnet. Replay skips it."""
    if config.pcap is not None:
        return config.host_ip
    if not config.lidar_ip:
        raise RuntimeError(
            "Mid360: lidar_ip is not set. Pass --mid360.lidar-ip, set it in the blueprint, "
            "or set MID360__LIDAR_IP in the environment."
        )
    return resolve_host_ip(config.lidar_ip, config.host_ip, label="Mid360")


class Mid360(NativeModule, perception.Lidar, perception.IMU):
    config: Mid360Config

    lidar: Out[PointCloud2]
    imu: Out[Imu]

    @rpc
    def start(self) -> None:
        self.config.host_ip = _resolved_host_ip(self.config)
        super().start()

    @rpc
    def stop(self) -> None:
        super().stop()

    @rpc
    def get_usd(self) -> list[str]:
        """``[asset_root, usd]``: this sensor, in the frames this instance is configured to stamp.

        A UsdManager asks for this when the MID-360 is one of its ``mounts``. The root is this
        module's own directory -- the only one this driver can point at and be right about wherever it
        is installed -- resolved at call time rather than baked into the constant, which is written on
        a build machine whose paths mean nothing on the robot.
        """
        return [
            str(Path(__file__).parent),
            mid360_usd(self.config.frame_id, self.config.imu_frame_id),
        ]


# Verify protocol port compliance (mypy will flag missing ports)
if TYPE_CHECKING:
    Mid360()
