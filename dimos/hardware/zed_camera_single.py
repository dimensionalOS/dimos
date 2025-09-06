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
import open3d as o3d
import logging
import time
import threading

try:
    import pyzed.sl as sl
except ImportError:
    sl = None
    logging.warning("ZED SDK not found. Please install pyzed to use ZED camera functionality.")

from dimos.core import Module, Out, rpc
from dimos.utils.logging_config import setup_logger
from dimos.msgs.geometry_msgs import PoseStamped
from dimos.msgs.std_msgs import Header
from dimos.msgs.geometry_msgs import Transform, Vector3, Quaternion
from dimos.robot.unitree_webrtc.type.lidar import LidarMessage

logger = setup_logger(__name__)


class ZedCameraThread(threading.Thread):
    def __init__(self, publish_pointcloud, publish_pose, voxel_size, map_publish_interval):
        super().__init__(daemon=True)
        self._stop_event = threading.Event()
        self.pymesh = None
        self._publish_pointcloud_cb = publish_pointcloud
        self._publish_pose_cb = publish_pose
        self.voxel_size = voxel_size
        self.zed = None
        self.pose_thread = None
        self.check_interval = 0.02
        self.runtime_parameters = sl.RuntimeParameters()
        self.map_publish_interval = map_publish_interval

    def stop_publishing(self):
        self._stop_event.set()
        if self.pose_thread:
            self.pose_thread.join(timeout=1.0)

    def _pose_publisher_thread(self):
        pose = sl.Pose()

        while not self._stop_event.is_set():
            time.sleep(self.check_interval)

            grab_status = self.zed.grab(self.runtime_parameters)
            if grab_status != sl.ERROR_CODE.SUCCESS:
                continue

            tracking_state = self.zed.get_position(pose)
            if tracking_state != sl.POSITIONAL_TRACKING_STATE.OK:
                continue

            self._publish_pose_cb(pose)

    def run(self):
        init = sl.InitParameters()
        init.depth_mode = sl.DEPTH_MODE.NEURAL_PLUS
        init.coordinate_units = sl.UNIT.METER
        init.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Z_UP_X_FWD
        init.depth_maximum_distance = 10.0

        self.zed = sl.Camera()

        status = self.zed.open(init)
        if status != sl.ERROR_CODE.SUCCESS:
            print("Camera Open : " + repr(status) + ".")
            return

        self.zed.get_camera_information()
        pose = sl.Pose()

        positional_tracking_parameters = sl.PositionalTrackingParameters()
        positional_tracking_parameters.enable_area_memory = True
        positional_tracking_parameters.enable_pose_smoothing = True
        positional_tracking_parameters.enable_imu_fusion = True
        positional_tracking_parameters.set_floor_as_origin = True
        returned_state = self.zed.enable_positional_tracking(positional_tracking_parameters)
        if returned_state != sl.ERROR_CODE.SUCCESS:
            print("Enable Positional Tracking : " + repr(returned_state) + ".")
            return

        spatial_mapping_parameters = sl.SpatialMappingParameters(
            max_memory_usage=4096,
            save_texture=False,
            use_chunk_only=True,
            reverse_vertex_order=False,
            map_type=sl.SPATIAL_MAP_TYPE.MESH,
        )
        spatial_mapping_parameters.resolution_meter = 0.1
        spatial_mapping_parameters.range_meter = 10.0

        self.pymesh = sl.Mesh()

        tracking_state = sl.POSITIONAL_TRACKING_STATE.OFF
        mapping_state = sl.SPATIAL_MAPPING_STATE.NOT_ENABLED

        # Exclude points with confidence level > 25. 0 is the highest confidence, 100 the lowest. 50 is the default.
        self.runtime_parameters.confidence_threshold = 25
        self.runtime_parameters.enable_fill_mode = True

        image = sl.Mat()
        point_cloud = sl.Mat()
        pose = sl.Pose()

        last_call = time.time()

        self.pose_thread = threading.Thread(target=self._pose_publisher_thread, daemon=True)
        self.pose_thread.start()

        while True:
            if self._stop_event.is_set():
                break
            time.sleep(self.check_interval)

            grab_status = self.zed.grab(self.runtime_parameters)

            if grab_status != sl.ERROR_CODE.SUCCESS:
                print(f"Grab: {repr(grab_status)}")
                continue

            self.zed.retrieve_image(image, sl.VIEW.LEFT)
            tracking_state = self.zed.get_position(pose)

            if tracking_state == sl.POSITIONAL_TRACKING_STATE.OK:
                break

        self.zed.enable_spatial_mapping(spatial_mapping_parameters)
        self.pymesh.clear()
        last_call = time.time()

        while True:
            if self._stop_event.is_set():
                break
            time.sleep(self.check_interval)

            mapping_state = self.zed.get_spatial_mapping_state()
            if mapping_state != sl.SPATIAL_MAPPING_STATE.OK:
                print("mapping not ok", mapping_state)
                continue

            duration = time.time() - last_call

            if duration < self.map_publish_interval:
                continue

            self.zed.request_spatial_map_async()
            last_call = time.time()

            if self.zed.get_spatial_map_request_status_async() == sl.ERROR_CODE.SUCCESS:
                self.zed.retrieve_spatial_map_async(self.pymesh)
                self.pymesh.update_mesh_from_chunklist()
                self._send_pymesh()

        # Turn everything off.
        mapping_state = sl.SPATIAL_MAPPING_STATE.NOT_ENABLED
        image.free(memory_type=sl.MEM.CPU)
        self.pymesh.clear()
        self.zed.disable_spatial_mapping()
        self.zed.disable_positional_tracking()
        self.zed.close()
        image.free()
        point_cloud.free()

    def _send_pymesh(self):
        vertices = self.pymesh.vertices
        if not len(vertices):
            print("no vertices in pymesh")
            return

        points = np.array(vertices, dtype=np.float32).reshape(-1, 3)  # XYZ format
        valid = np.isfinite(points).all(axis=1)
        valid_points = points[valid]

        # Filter out points with Z > 1.5m
        z_filter = valid_points[:, 2] <= 1.5
        valid_points = valid_points[z_filter]

        if not len(valid_points):
            print("no valid points")
            return

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(valid_points)
        pcd = pcd.voxel_down_sample(voxel_size=self.voxel_size)
        print(f"downsampled {len(vertices)} to {len(pcd.points)} points")
        self._publish_pointcloud_cb(pcd)


class ZedModuleSingle(Module):
    pointcloud_msg: Out[LidarMessage] = None
    pose: Out[PoseStamped] = None

    def __init__(self, voxel_size, map_publish_interval, **kwargs):
        super().__init__(**kwargs)
        self.voxel_size = voxel_size
        self.map_publish_interval = map_publish_interval
        self._zed_camera_thread = ZedCameraThread(
            publish_pointcloud=self._publish_pointcloud,
            publish_pose=self._publish_pose,
            voxel_size=self.voxel_size,
            map_publish_interval=self.map_publish_interval,
        )
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

    def _publish_pose(self, pose: sl.Pose):
        header = Header("camera_link")
        position = pose.get_translation().get().tolist()
        rotation = pose.get_orientation().get().tolist()

        msg = PoseStamped(
            ts=header.ts,
            position=position,
            orientation=rotation,
            frame_id="world",
        )

        self.pose.publish(msg)

        # World → base_link (from ZED tracking)
        # Note: ZED tracking gives camera pose, we need to transform to base_link
        base_tf = Transform(
            translation=Vector3(position),
            rotation=Quaternion(rotation),
            frame_id="world",
            child_frame_id="base_link",
            ts=header.ts,
        )

        # base_link → camera_link (physical offset - camera mounted 30cm forward)
        camera_link = Transform(
            translation=Vector3(0.3, 0.0, 0.1),  # 30cm forward, 10cm up
            rotation=Quaternion.from_euler(Vector3([0, 0, 0])),
            frame_id="base_link",
            child_frame_id="camera_link",
            ts=header.ts,
        )
        self.tf.publish(base_tf)
        self.tf.publish(camera_link)

    def _publish_pointcloud(self, pcd: o3d.geometry.PointCloud):
        if self.pointcloud_msg is not None and pcd is not None and len(pcd.points) > 0:
            lidar_msg = LidarMessage(
                pointcloud=pcd,
                origin=[0.0, 0.0, 0.0],
                resolution=self.voxel_size,
                ts=time.time(),
                frame_id="world",
            )
            self.pointcloud_msg.publish(lidar_msg)
