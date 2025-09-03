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
    def __init__(self, publish_pointcloud, publish_pose):
        super().__init__(daemon=True)
        self._stop_event = threading.Event()
        self.pymesh = None
        self._publish_pointcloud_cb = publish_pointcloud
        self._publish_pose_cb = publish_pose
        self.voxel_size = 0.1

    def stop_publishing(self):
        self._stop_event.set()

    def run(self):
        init = sl.InitParameters()
        init.depth_mode = sl.DEPTH_MODE.NEURAL_PLUS
        init.coordinate_units = sl.UNIT.METER
        init.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Z_UP_X_FWD
        init.depth_maximum_distance = 10.0

        zed = sl.Camera()

        status = zed.open(init)
        if status != sl.ERROR_CODE.SUCCESS:
            print("Camera Open : " + repr(status) + ".")
            return

        zed.get_camera_information()
        pose = sl.Pose()

        positional_tracking_parameters = sl.PositionalTrackingParameters()
        positional_tracking_parameters.enable_area_memory = True
        positional_tracking_parameters.enable_pose_smoothing = True
        positional_tracking_parameters.enable_imu_fusion = True
        positional_tracking_parameters.set_floor_as_origin = True
        returned_state = zed.enable_positional_tracking(positional_tracking_parameters)
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

        runtime_parameters = sl.RuntimeParameters()
        # Exclude points with confidence level > 35. 0 is the highest confidence, 100 the lowest. 50 is the default.
        runtime_parameters.confidence_threshold = 35

        mapping_activated = False

        image = sl.Mat()
        point_cloud = sl.Mat()
        pose = sl.Pose()

        last_call = time.time()

        while True:
            if self._stop_event.is_set():
                break
            time.sleep(0.05)

            grab_status = zed.grab(runtime_parameters)

            if grab_status != sl.ERROR_CODE.SUCCESS:
                print(f"Grab: {repr(grab_status)}")
                continue

            zed.retrieve_image(image, sl.VIEW.LEFT)
            tracking_state = zed.get_position(pose)

            if tracking_state != sl.POSITIONAL_TRACKING_STATE.OK:
                print("tracking not ok", tracking_state)
                continue

            self._publish_pose_cb(pose)

            if not mapping_activated:
                print("turning mapping on")

                init_pose = sl.Transform()
                zed.reset_positional_tracking(init_pose)

                zed.enable_spatial_mapping(spatial_mapping_parameters)

                self.pymesh.clear()

                last_call = time.time()

                mapping_activated = True

            if mapping_activated:
                mapping_state = zed.get_spatial_mapping_state()
                if mapping_state != sl.SPATIAL_MAPPING_STATE.OK:
                    print("mapping not ok", mapping_state)
                    continue

                duration = time.time() - last_call

                if duration < 5:
                    continue

                zed.request_spatial_map_async()
                last_call = time.time()

                if zed.get_spatial_map_request_status_async() == sl.ERROR_CODE.SUCCESS:
                    print("retreive_spatial_map_async", zed.retrieve_spatial_map_async(self.pymesh))
                    print("extract_whole_spatial_map", zed.extract_whole_spatial_map(self.pymesh))
                    filter_params = sl.MeshFilterParameters()
                    filter_params.set(sl.MESH_FILTER.MEDIUM)
                    self.pymesh.filter(filter_params, True)
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
        self.pymesh.update_mesh_from_chunklist()

        vertices = self.pymesh.vertices
        if len(vertices) > 0:
            print("points", len(vertices))
            points = np.array(vertices, dtype=np.float32).reshape(-1, 3)  # XYZ format
            valid = np.isfinite(points).all(axis=1)
            valid_points = points[valid]
            pcd = o3d.geometry.PointCloud()
            if len(valid_points) > 0:
                pcd.points = o3d.utility.Vector3dVector(valid_points)
                pcd = pcd.voxel_down_sample(voxel_size=self.voxel_size)
                print(f"downsampled to {len(pcd.points)} points")
                self._publish_pointcloud_cb(pcd)
            else:
                print("no valid points")
        else:
            print("no points in pymesh")


class ZedModuleSingle(Module):
    pointcloud_msg: Out[LidarMessage] = None
    pose: Out[PoseStamped] = None

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self._zed_camera_thread = ZedCameraThread(
            publish_pointcloud=self._publish_pointcloud, publish_pose=self._publish_pose
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
                resolution=0.1,
                ts=time.time(),
                frame_id="world",
            )
            self.pointcloud_msg.publish(lidar_msg)
            logger.info(f"Published pointcloud with {len(pcd.points)} points")
        else:
            logger.info("No pointcloud data to publish")
