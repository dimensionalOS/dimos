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

import pickle
from typing import List, Tuple

import numpy as np
from dimos_lcm.sensor_msgs import CameraInfo

from dimos.msgs.geometry_msgs import Transform
from dimos.msgs.sensor_msgs import Image, PointCloud2
from dimos.robot.unitree_webrtc.type.lidar import LidarMessage
from dimos.robot.unitree_webrtc.type.odometry import Odometry


def create_pointcloud2_from_numpy(points, frame_id, timestamp):
    """Create PointCloud2 from numpy array of shape (N, 3)."""
    import open3d as o3d

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    pc2 = PointCloud2(pointcloud=pcd, ts=timestamp, frame_id=frame_id)
    return pc2


def project_points_to_camera(
    points_3d: np.ndarray,
    camera_matrix: np.ndarray,
    extrinsics: np.ndarray,
    timestamp: float = None,
) -> np.ndarray:
    """Project 3D points to 2D camera coordinates.

    Args:
        points_3d: Nx3 array of 3D points in base_link frame (in meters)
        camera_matrix: 3x3 camera intrinsic matrix
        extrinsics: 4x4 transformation matrix from base_link to camera_optical frame

    Returns:
        Nx2 array of projected 2D points in pixels, valid_mask
    """
    # Transform points from base_link to camera_optical frame
    points_homogeneous = np.hstack([points_3d, np.ones((points_3d.shape[0], 1))])
    points_camera = (extrinsics @ points_homogeneous.T).T

    # Debug: print sample of 3D coordinates
    print(f"Sample 3D points in world frame (first 5):")
    print(points_3d[:5])
    print(f"Sample 3D points in camera optical frame (first 5):")
    print(points_camera[:5, :3])
    print(f"  Note: In optical frame - X:right, Y:down, Z:forward")

    # Find points at reasonable heights (positive Y in optical frame means below camera)
    reasonable_height_mask = (points_camera[:, 1] > -1.0) & (points_camera[:, 1] < 2.0)
    print(f"Points at reasonable height (Y in [-1, 2]): {reasonable_height_mask.sum()}")

    # Broadcast the transformed pointcloud in camera_optical frame for debugging
    from dimos.core import LCMTransport

    # First broadcast the original world frame points
    world_pointcloud = create_pointcloud2_from_numpy(
        points_3d, frame_id="world", timestamp=timestamp
    )
    world_transport = LCMTransport("/debug_world_points", PointCloud2)
    world_transport.broadcast(None, world_pointcloud)
    print(f"Published world frame pointcloud with {points_3d.shape[0]} points")

    # Then broadcast the transformed camera optical frame points
    camera_points_3d = points_camera[:, :3]
    camera_pointcloud = create_pointcloud2_from_numpy(
        camera_points_3d, frame_id="camera_optical", timestamp=timestamp
    )
    debug_transport = LCMTransport("/debug_camera_optical_points", PointCloud2)
    debug_transport.broadcast(None, camera_pointcloud)
    print(f"Published camera optical frame pointcloud with {camera_points_3d.shape[0]} points")

    # Filter out points behind the camera
    valid_mask = points_camera[:, 2] > 0
    points_camera = points_camera[valid_mask]

    # Project to 2D
    points_2d_homogeneous = (camera_matrix @ points_camera[:, :3].T).T
    points_2d = points_2d_homogeneous[:, :2] / points_2d_homogeneous[:, 2:3]

    print(f"Sample 2D points (first 5):")
    print(points_2d[:5])

    return points_2d, valid_mask


def filter_points_in_detections(
    pointcloud: PointCloud2,
    image: Image,
    camera_info: CameraInfo,
    detection_list: List,
    world_to_camera_transform: np.ndarray,
) -> List[PointCloud2]:
    """Filter lidar points that fall within detection bounding boxes.

    Args:
        pointcloud: Input point cloud in its original frame
        image: Camera image (for frame_id reference)
        camera_info: Camera calibration information
        detection_list: List of detections in format [bbox, track_id, class_id, confidence, names]
        world_to_camera_transform: 4x4 transformation matrix from point cloud frame to camera_optical frame

    Returns:
        List of PointCloud2 messages, one for each detection
    """
    # Extract camera parameters from camera info
    fx, fy, cx = camera_info.K[0], camera_info.K[4], camera_info.K[2]
    cy = camera_info.K[5]
    image_width = camera_info.width
    image_height = camera_info.height

    camera_matrix = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]])

    # Convert pointcloud to numpy array
    lidar_points = pointcloud.as_numpy()
    # Project all points to camera frame
    points_2d_all, valid_mask = project_points_to_camera(
        lidar_points, camera_matrix, world_to_camera_transform, pointcloud.ts
    )
    valid_3d_points = lidar_points[valid_mask]
    points_2d = points_2d_all.copy()

    print(f"Points after projection: {points_2d.shape[0]} (from {lidar_points.shape[0]})")

    # Filter points within image bounds
    in_image_mask = (
        (points_2d[:, 0] >= 0)
        & (points_2d[:, 0] < image_width)
        & (points_2d[:, 1] >= 0)
        & (points_2d[:, 1] < image_height)
    )
    points_2d = points_2d[in_image_mask]
    valid_3d_points = valid_3d_points[in_image_mask]

    print(f"Points within image bounds: {points_2d.shape[0]}")
    if points_2d.shape[0] > 0:
        print(
            f"2D point range: X=[{points_2d[:, 0].min():.1f}, {points_2d[:, 0].max():.1f}], Y=[{points_2d[:, 1].min():.1f}, {points_2d[:, 1].max():.1f}]"
        )
    else:
        # Show the range of all projected points before filtering
        all_2d = points_2d_all[valid_mask]
        print(f"All projected points range (before image bounds filter):")
        print(f"  X: [{all_2d[:, 0].min():.1f}, {all_2d[:, 0].max():.1f}]")
        print(f"  Y: [{all_2d[:, 1].min():.1f}, {all_2d[:, 1].max():.1f}]")
        print(f"  Image bounds: X=[0, {image_width}], Y=[0, {image_height}]")

    filtered_pointclouds = []

    for detection in detection_list:
        # Detection format: [bbox, track_id, class_id, confidence, names]
        bbox, track_id, class_id, confidence, names = detection
        x_min, y_min, x_max, y_max = bbox

        print(f"  Detection bbox: X=[{x_min:.1f}, {x_max:.1f}], Y=[{y_min:.1f}, {y_max:.1f}]")

        # Check points in extended region for debugging
        extended_mask = (
            (points_2d[:, 0] >= x_min - 50)
            & (points_2d[:, 0] <= x_max + 50)
            & (points_2d[:, 1] >= y_min - 50)
            & (points_2d[:, 1] <= y_max + 50)
        )
        print(f"  Points near detection (±50px): {extended_mask.sum()}")

        # Show where those nearby points are
        if extended_mask.sum() > 0:
            nearby_points = points_2d[extended_mask]
            print(
                f"    Nearby points X range: [{nearby_points[:, 0].min():.1f}, {nearby_points[:, 0].max():.1f}]"
            )
            print(
                f"    Nearby points Y range: [{nearby_points[:, 1].min():.1f}, {nearby_points[:, 1].max():.1f}]"
            )

        # Find points within this detection box (with small margin for lidar sparsity)
        margin = 5  # pixels
        in_box_mask = (
            (points_2d[:, 0] >= x_min - margin)
            & (points_2d[:, 0] <= x_max + margin)
            & (points_2d[:, 1] >= y_min - margin)
            & (points_2d[:, 1] <= y_max + margin)
        )

        detection_points = valid_3d_points[in_box_mask]

        # names might be a string or list
        if isinstance(names, list):
            class_name = ", ".join(names) if names else f"class_{class_id}"
        else:
            class_name = names if names else f"class_{class_id}"
        print(f"Detection '{class_name}': {detection_points.shape[0]} points")

        # Create PointCloud2 message for this detection
        if detection_points.shape[0] > 0:
            detection_pointcloud = create_pointcloud2_from_numpy(
                detection_points,
                frame_id=pointcloud.frame_id,  # Keep original frame
                timestamp=pointcloud.ts,
            )
            filtered_pointclouds.append(detection_pointcloud)
        else:
            filtered_pointclouds.append(None)

    return filtered_pointclouds


def quaternion_to_rotation_matrix(q):
    """Convert quaternion (x,y,z,w) to 3x3 rotation matrix."""
    x, y, z, w = q
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def transform_to_matrix(transform: Transform) -> np.ndarray:
    """Convert a Transform object to a 4x4 transformation matrix."""
    # Convert quaternion to rotation matrix
    q = transform.rotation
    R = quaternion_to_rotation_matrix([q.x, q.y, q.z, q.w])

    # Build 4x4 transformation matrix
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = [transform.translation.x, transform.translation.y, transform.translation.z]

    print(f"Transform {transform.frame_id} -> {transform.child_frame_id}:")
    print(
        f"  Translation: [{transform.translation.x:.3f}, {transform.translation.y:.3f}, {transform.translation.z:.3f}]"
    )
    print(f"  Quaternion: [{q.x:.3f}, {q.y:.3f}, {q.z:.3f}, {q.w:.3f}]")
    print(f"  Matrix:\n{T}")

    return T


def main():
    # Import detect.py's camera_info and transform_chain functions
    from detect import camera_info, transform_chain

    # Load the pickled data
    try:
        with open("filename.pkl", "rb") as file:
            timestamp, lidar_frame, video_frame, odom_frame, detections, annotations = pickle.load(
                file
            )
    except FileNotFoundError:
        print("Run detect.py first to generate the data pickle file")
        return

    # Import Transform utilities
    from dimos.msgs.geometry_msgs import Quaternion, Transform, Vector3

    # Get camera info using detect.py's camera_info function
    cam_info = camera_info()
    tf = transform_chain(odom_frame)
    from dimos.msgs.geometry_msgs import Quaternion, Transform, Vector3

    # Manually receive the transforms to ensure they're in the buffer

    # Get the transform from lidar frame to camera optical frame
    # Lidar is in "world" frame, we need to transform to "camera_optical"
    # Transform chain: world -> base_link -> camera_link -> camera_optical

    # Get individual transforms (no timestamp needed, will use latest)
    world_to_optical_transform = tf.get("world", "camera_optical")
    print("world to camera_optical transform:", world_to_optical_transform)

    # IMPORTANT: The transform from tf.get() represents the camera_optical frame in world coordinates
    # To transform POINTS from world to camera_optical, we need the INVERSE transform!
    world_to_optical_inverse = world_to_optical_transform.inverse()
    print("INVERSE transform for points:", world_to_optical_inverse)

    # Debug: Also get and print intermediate transforms
    world_to_base = tf.get("world", "base_link")
    base_to_camera = tf.get("base_link", "camera_link")
    camera_to_optical = tf.get("camera_link", "camera_optical")
    print(f"world->base: {world_to_base}")
    print(f"base->camera: {base_to_camera}")
    print(f"camera->optical: {camera_to_optical}")

    extrinsics = transform_to_matrix(world_to_optical_inverse)

    # Extract detection list from the detection tuple
    # detections is a tuple: [image, detection_list]
    detection_list = detections[1]
    print(f"Number of detections: {len(detection_list)}")
    print(f"Lidar frame_id: {lidar_frame.frame_id}")
    print(f"Image frame_id: {video_frame.frame_id}")

    # Convert lidar to PointCloud2 if needed
    if isinstance(lidar_frame, LidarMessage):
        lidar_pointcloud = lidar_frame  # LidarMessage inherits from PointCloud2
    else:
        lidar_pointcloud = lidar_frame

    # Filter points for each detection using the original transform
    # But let's see what happens
    filtered_pointclouds = filter_points_in_detections(
        lidar_pointcloud, video_frame, cam_info, detection_list, extrinsics
    )

    # Publish filtered point clouds for each detection
    from dimos.core import LCMTransport
    from dimos.msgs.std_msgs import Header

    valid_pointclouds = []
    for i, (detection, pc) in enumerate(zip(detection_list, filtered_pointclouds)):
        if pc is not None:
            # Extract detection info
            bbox, track_id, class_id, confidence, names = detection
            # names might be a string or list
            if isinstance(names, list):
                class_name = ", ".join(names) if names else f"class_{class_id}"
            else:
                class_name = names if names else f"class_{class_id}"

            # Publish the filtered point cloud
            transport = LCMTransport(f"/filtered_points_{class_name}_{i}", PointCloud2)
            transport.broadcast(None, pc)
            valid_pointclouds.append(pc)

            print(f"Published {pc.as_numpy().shape[0]} points for {class_name}")

    # Also create a combined point cloud with all filtered points
    if valid_pointclouds:
        # Combine all point arrays
        all_points = np.vstack([pc.as_numpy() for pc in valid_pointclouds])
        combined_pointcloud = create_pointcloud2_from_numpy(
            all_points, frame_id=lidar_pointcloud.frame_id, timestamp=timestamp
        )
        combined_transport = LCMTransport("/filtered_points_combined", PointCloud2)
        combined_transport.broadcast(None, combined_pointcloud)
        print(f"Published combined point cloud with {all_points.shape[0]} points")


if __name__ == "__main__":
    main()
