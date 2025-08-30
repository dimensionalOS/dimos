# Copyright 2025 Dimensional Inc.
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

import numpy as np
import cv2
import open3d as o3d
from typing import Optional, Tuple, Dict, Any
import logging
import time
import threading
from reactivex import interval
from reactivex import operators as ops

try:
    import pyzed.sl as sl
except ImportError:
    sl = None
    logging.warning("ZED SDK not found. Please install pyzed to use ZED camera functionality.")

from dimos.hardware.stereo_camera import StereoCamera
from dimos.core import Module, Out, rpc
from dimos.utils.logging_config import setup_logger
from dimos.protocol.tf import TF
from dimos.msgs.geometry_msgs import Transform, Vector3, Quaternion

from dimos.msgs.sensor_msgs import Image, ImageFormat
from dimos_lcm.sensor_msgs import CameraInfo
from dimos.msgs.geometry_msgs import PoseStamped
from dimos.msgs.std_msgs import Header
from dimos.robot.unitree_webrtc.type.lidar import LidarMessage

logger = setup_logger(__name__)


class ZedCameraThread(threading.Thread):
    def __init__(self, publish_pointcloud):
        super().__init__(daemon=True)
        self._stop_event = threading.Event()
        self.pymesh = None
        self._publish_pointcloud_cb = publish_pointcloud

    def stop_publishing(self):
        self._stop_event.set()

    def run(self):
        init = sl.InitParameters()
        init.depth_mode = sl.DEPTH_MODE.NEURAL
        init.coordinate_units = sl.UNIT.METER
        init.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Z_UP_X_FWD
        init.depth_maximum_distance = 8.0

        zed = sl.Camera()

        status = zed.open(init)
        if status != sl.ERROR_CODE.SUCCESS:
            print("Camera Open : " + repr(status) + ".")
            return

        zed.get_camera_information()
        pose = sl.Pose()

        positional_tracking_parameters = sl.PositionalTrackingParameters()
        positional_tracking_parameters.set_floor_as_origin = True
        returned_state = zed.enable_positional_tracking(positional_tracking_parameters)
        if returned_state != sl.ERROR_CODE.SUCCESS:
            print("Enable Positional Tracking : " + repr(returned_state) + ".")
            return

        spatial_mapping_parameters = sl.SpatialMappingParameters(
            resolution=sl.MAPPING_RESOLUTION.MEDIUM,
            mapping_range=sl.MAPPING_RANGE.MEDIUM,
            max_memory_usage=2048,
            save_texture=False,
            use_chunk_only=True,
            reverse_vertex_order=False,
            map_type=sl.SPATIAL_MAP_TYPE.FUSED_POINT_CLOUD,
        )
        self.pymesh = sl.FusedPointCloud()

        tracking_state = sl.POSITIONAL_TRACKING_STATE.OFF
        mapping_state = sl.SPATIAL_MAPPING_STATE.NOT_ENABLED

        runtime_parameters = sl.RuntimeParameters()
        runtime_parameters.confidence_threshold = 50

        mapping_activated = False

        image = sl.Mat()
        point_cloud = sl.Mat()
        pose = sl.Pose()

        last_call = time.time()

        while True:
            if self._stop_event.is_set():
                break
            time.sleep(0.02)

            if zed.grab(runtime_parameters) > sl.ERROR_CODE.SUCCESS:
                continue

            print("grabbed")

            zed.retrieve_image(image, sl.VIEW.LEFT)
            tracking_state = zed.get_position(pose)
            print("tracking_state", tracking_state)

            if tracking_state != sl.POSITIONAL_TRACKING_STATE.OK:
                continue

            if not mapping_activated:
                print("turning mapping on")

                init_pose = sl.Transform()
                zed.reset_positional_tracking(init_pose)

                spatial_mapping_parameters.resolution_meter = (
                    sl.SpatialMappingParameters().get_resolution_preset(
                        sl.MAPPING_RESOLUTION.MEDIUM
                    )
                )
                zed.enable_spatial_mapping(spatial_mapping_parameters)

                self.pymesh.clear()

                last_call = time.time()

                mapping_activated = True

            if mapping_activated:
                mapping_state = zed.get_spatial_mapping_state()
                print("mapping_state", mapping_state)
                duration = time.time() - last_call
                if duration > 0.5:
                    print("requested spatial map")
                    zed.request_spatial_map_async()
                    last_call = time.time()

                if zed.get_spatial_map_request_status_async() == sl.ERROR_CODE.SUCCESS:
                    print("received spatial map")
                    zed.retrieve_spatial_map_async(self.pymesh)
                    zed.extract_whole_spatial_map(self.pymesh)
                    self._send_pymesh()
                else:
                    print("spatial map not received yet")

        # Turn everything off.
        mapping_state = sl.SPATIAL_MAPPING_STATE.NOT_ENABLED
        mapping_activated = False
        image.free(memory_type=sl.MEM.CPU)
        self.pymesh.clear()
        zed.disable_spatial_mapping()
        zed.disable_positional_tracking()
        zed.close()
        image.free()
        point_cloud.free()

    def _send_pymesh(self):
        print("sending pymesh")
        status = self.pymesh.save(f"mesh_gen_dimos_{time.time()}.obj")
        print("saved mesh", status)

        vertices = self.pymesh.vertices
        if len(vertices) > 0:
            print("have points")
            points = np.array(vertices, dtype=np.float32).reshape(-1, 4)[:, :3]  # XYZ only
            valid = np.isfinite(points).all(axis=1)
            valid_points = points[valid]
            pcd = o3d.geometry.PointCloud()
            if len(valid_points) > 0:
                print("have valid points")
                pcd.points = o3d.utility.Vector3dVector(valid_points)
                self._publish_pointcloud_cb(pcd)
            else:
                print("no valid points")
        else:
            print("no points in pymesh")


class ZedModuleSingle(Module):
    pointcloud_msg: Out[LidarMessage] = None

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self._zed_camera_thread = ZedCameraThread(publish_pointcloud=self._publish_pointcloud)
        logger.info("ZEDModuleSingle initialized")

    @rpc
    def start(self):
        self._zed_camera_thread.start()

    @rpc
    def stop(self):
        self._zed_camera_thread.stop_publishing()
        logger.info("ZED module stopped")

    def cleanup(self):
        self.stop()

    def _publish_pointcloud(self, pcd: o3d.geometry.PointCloud):
        if self.pointcloud_msg is not None and pcd is not None and len(pcd.points) > 0:
            lidar_msg = LidarMessage(
                pointcloud=pcd,
                origin=[0.0, 0.0, 0.0],
                resolution=0.05,
                ts=time.time(),
                frame_id="world",
            )
            self.pointcloud_msg.publish(lidar_msg)
            logger.info(f"Published pointcloud with {len(pcd.points)} points")
        else:
            logger.info("No pointcloud data to publish")
