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

"""DrddsLidarBridge — NativeModule for M20 lidar + IMU via POSIX SHM.

Reads point cloud and IMU data from shared memory segments written by
drdds_recv (the host-side drdds subscriber) and publishes them as LCM
messages for AriseSLAM and SmartNav consumption.

This is the M20 equivalent of the Mid360 module — it sits at the sensor
input layer and feeds raw_points + imu into the navigation stack.

Usage::

    from dimos.robot.deeprobotics.m20.drdds_bridge.module import DrddsLidarBridge

    autoconnect(
        DrddsLidarBridge.blueprint(),
        AriseSLAM.blueprint(mount=...),
        smart_nav(...),
    ).remappings([
        (DrddsLidarBridge, "lidar", "raw_points"),
    ])
"""

from __future__ import annotations

from typing import TYPE_CHECKING

from dimos.core.native_module import NativeModule, NativeModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.spec import perception


class DrddsLidarBridgeConfig(NativeModuleConfig):
    """Config for the drdds SHM → LCM bridge NativeModule."""

    cwd: str | None = "cpp"
    executable: str = "result/bin/drdds_lidar_bridge"
    build_command: str | None = "nix build .#drdds_lidar_bridge"


class DrddsLidarBridge(
    NativeModule[DrddsLidarBridgeConfig], perception.Lidar, perception.IMU
):
    """M20 lidar + IMU bridge from drdds POSIX SHM to LCM.

    Reads from shared memory segments created by drdds_recv on the NOS host:
    - /drdds_bridge_lidar: merged dual RSAIRY 192-ch point clouds
    - /drdds_bridge_imu: Yesense IMU at ~200Hz

    Performs ring remapping (192→64 for ARISE N_SCANS=64) and time
    relativization (f64 absolute → f32 relative) in the C++ hot path.

    Ports:
        lidar (Out[PointCloud2]): Point cloud with ring + time fields.
        imu (Out[Imu]): IMU data at ~200 Hz.
    """

    config: DrddsLidarBridgeConfig
    default_config = DrddsLidarBridgeConfig

    lidar: Out[PointCloud2]
    imu: Out[Imu]


class AiryImuBridgeConfig(NativeModuleConfig):
    """Config for the Airy integrated-IMU multicast → LCM bridge."""

    cwd: str | None = "cpp"
    executable: str = "result/bin/airy_imu_bridge"
    build_command: str | None = "nix build .#drdds_lidar_bridge"
    which: str = "front"  # "front" or "rear"
    # Output IMU frame. "base_link" (default) rotates into the body frame
    # via R_base_from_{front|rear}. "sensor" passes the raw Airy-frame
    # measurements through unchanged — pair with rsdriver's per-lidar
    # extrinsic zeroed so the lidar cloud stays in the same sensor frame
    # and FAST-LIO2's extrinsic is truly identity.
    frame: str = "base_link"


class AiryImuBridge(NativeModule[AiryImuBridgeConfig], perception.IMU):
    """Taps the RoboSense Airy integrated IMU (front or rear) from its UDP
    multicast stream and publishes a base_link-frame LCM sensor_msgs/Imu.

    Parses the 51-byte RSAIRY IMU packet per rs_driver's decoder_RSAIRY.hpp,
    applies FSR-aware unit conversion (g → m/s², dps → rad/s), rotates the
    accel + gyro into base_link using the lidar→base transform from rsdriver
    config, and drops packets failing a PTP-lock sanity gate (UTC < 2024-01-01).

    Unlike the yesense path, Airy's IMU shares a PTP-locked hardware clock
    with the Airy lidar optics — eliminating the clock-domain mismatch that
    plagued earlier FAST-LIO2 runs (FASTLIO2_LOG Findings #7, #8).

    Ports:
        imu (Out[Imu]): base_link-frame IMU samples at 200 Hz.
    """

    config: AiryImuBridgeConfig
    default_config = AiryImuBridgeConfig

    imu: Out[Imu]


class NavCmdPubConfig(NativeModuleConfig):
    """Config for the raw FastDDS /NAV_CMD publisher."""

    cwd: str | None = "cpp"
    executable: str = "result/bin/nav_cmd_pub"
    build_command: str | None = None


class NavCmdPub(NativeModule[NavCmdPubConfig]):
    """Publishes velocity commands to /NAV_CMD via raw FastDDS.

    Subscribes to cmd_vel on LCM and publishes to DDS topic rt/NAV_CMD
    using FastDDS directly (no ROS, no drdds wrapper). Uses the ROS2
    topic naming convention so basic_server on AOS can receive commands.

    Ports:
        cmd_vel (In[Twist]): Velocity commands from CmdVelMux.
    """

    config: NavCmdPubConfig
    default_config = NavCmdPubConfig

    cmd_vel: In[Twist]


if TYPE_CHECKING:
    DrddsLidarBridge()
    AiryImuBridge()
    NavCmdPub()
