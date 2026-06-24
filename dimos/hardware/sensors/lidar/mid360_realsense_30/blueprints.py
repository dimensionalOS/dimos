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

"""Record blueprints for the RealSense D435i + Mid-360 rig.

Point-LIO odom+lidar and the RealSense color/depth/pointcloud streams are recorded
into a memory2 db, with the rig's mount frames published continuously onto tf. Two
variants: ``mid360_realsense_record`` (db only) and ``mid360_realsense_record_with_pcap``
(also captures a raw .pcap of the Mid-360 UDP stream).

The lidar IPs come from each module's own config (``DIMOS_MID360_LIDAR_IP`` for the
Mid-360 / pcap capture, ``DIMOS_POINTLIO_LIDAR_IP`` for Point-LIO)::

    export DIMOS_MID360_LIDAR_IP=192.168.1.155 DIMOS_POINTLIO_LIDAR_IP=192.168.1.155
    dimos run mid360-realsense-record            # db only
    dimos run mid360-realsense-record-with-pcap  # db + raw pcap
"""

from dimos.core.coordination.blueprints import autoconnect
from dimos.hardware.sensors.camera.realsense.camera import RealSenseCamera
from dimos.hardware.sensors.lidar.livox.module import Mid360
from dimos.hardware.sensors.lidar.mid360_realsense_30.recorder import Mid360RealsenseRecorder
from dimos.hardware.sensors.lidar.mid360_realsense_30.static_transforms import (
    Mid360RealsenseStaticTf,
)
from dimos.hardware.sensors.lidar.pointlio.module import PointLio
from dimos.hardware.sensors.lidar.virtual_mid360.recorder import Mid360PcapRecorder

mid360_realsense_record = autoconnect(
    RealSenseCamera.blueprint().remappings(
        [
            (RealSenseCamera, "depth_image", "realsense_depth_image"),
            (RealSenseCamera, "pointcloud", "realsense_pointcloud"),
            (RealSenseCamera, "camera_info", "realsense_camera_info"),
            (RealSenseCamera, "depth_camera_info", "realsense_depth_camera_info"),
        ]
    ),
    Mid360.blueprint().remappings(
        [
            (Mid360, "lidar", "livox_lidar"),
            (Mid360, "imu", "livox_imu"),
        ]
    ),
    PointLio.blueprint(frame_id="world").remappings(
        [
            (PointLio, "lidar", "pointlio_lidar"),
            (PointLio, "odometry", "pointlio_odometry"),
        ]
    ),
    Mid360RealsenseRecorder.blueprint(),
    # Continuously republishes the rig's mount frames onto tf (no latched static tf).
    Mid360RealsenseStaticTf.blueprint(),
).global_config(n_workers=8)

# Same rig, also capturing a raw .pcap of the Mid-360 UDP stream.
mid360_realsense_record_with_pcap = autoconnect(
    mid360_realsense_record,
    Mid360PcapRecorder.blueprint(),
)
