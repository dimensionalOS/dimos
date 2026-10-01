# Copyright 2025-2026 Dimensional Inc.
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


from typing import overload

from dimos_generated.geometry_msgs.msg import (
    Point,
    Pose,
    PoseStamped,
    Quaternion,
    TransformStamped,
    Twist,
    Vector3,
)
from dimos_generated.std_msgs.msg import Header
import numpy as np
from numpy.typing import NDArray
from scipy.spatial.transform import Rotation as R

from dimos.msgs.geometry import transform_matrix


def normalize_angle(angle: float) -> float:
    """Normalize angle to [-pi, pi] range"""
    return np.arctan2(np.sin(angle), np.cos(angle))  # type: ignore[no-any-return]


def twist_to_numpy(twist: Twist) -> NDArray[np.float64]:
    """Convert a Twist-like geometry message to [linear xyz, angular xyz]."""
    return np.array(
        [
            twist.linear.x,
            twist.linear.y,
            twist.linear.z,
            twist.angular.x,
            twist.angular.y,
            twist.angular.z,
        ],
        dtype=np.float64,
    )


def pose_to_matrix(pose: Pose) -> np.ndarray:
    """
    Convert pose to 4x4 homogeneous transform matrix.

    Args:
        pose: Pose object with position and orientation (quaternion)

    Returns:
        4x4 transformation matrix
    """
    # Extract position
    tx, ty, tz = pose.position.x, pose.position.y, pose.position.z

    # Create rotation matrix from quaternion using scipy
    quat = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]

    # Check for zero norm quaternion and use identity if invalid
    quat_norm = np.linalg.norm(quat)
    if quat_norm == 0.0:
        # Use identity quaternion [0, 0, 0, 1] if zero norm detected
        quat = [0.0, 0.0, 0.0, 1.0]

    rotation = R.from_quat(quat)
    Rot = rotation.as_matrix()

    # Create 4x4 transform
    T = np.eye(4)
    T[:3, :3] = Rot
    T[:3, 3] = [tx, ty, tz]

    return T


def matrix_to_pose(T: np.ndarray) -> Pose:
    """
    Convert 4x4 transformation matrix to Pose object.

    Args:
        T: 4x4 transformation matrix

    Returns:
        Pose object with position and orientation (quaternion)
    """
    # Extract position
    pos = Point(x=T[0, 3], y=T[1, 3], z=T[2, 3])

    # Extract rotation matrix and convert to quaternion
    Rot = T[:3, :3]
    rotation = R.from_matrix(Rot)
    quat = rotation.as_quat()  # Returns [x, y, z, w]

    orientation = Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])

    return Pose(position=pos, orientation=orientation)


@overload
def apply_transform(pose: Pose, transform: np.ndarray) -> Pose: ...


@overload
def apply_transform(pose: PoseStamped, transform: TransformStamped) -> PoseStamped: ...


def apply_transform(
    pose: Pose | PoseStamped, transform: np.ndarray | TransformStamped
) -> Pose | PoseStamped:
    """Apply an array to an unstamped pose or a frame-checked stamped TF edge.

    Stamped transforms preserve the pose's source stamp and advertise the
    destination frame. Mixing stamped and unstamped values is an error.
    """
    if isinstance(transform, TransformStamped):
        if not isinstance(pose, PoseStamped):
            raise TypeError("A stamped transform requires a stamped pose")
        if transform.child_frame_id != pose.header.frame_id:
            raise ValueError(
                f"Transform child frame {transform.child_frame_id} does not match pose frame {pose.header.frame_id}"
            )
        result = matrix_to_pose(transform_matrix(transform.transform) @ pose_to_matrix(pose.pose))
        return PoseStamped(
            header=Header(stamp=pose.header.stamp, frame_id=transform.header.frame_id), pose=result
        )
    if isinstance(pose, PoseStamped):
        raise TypeError("A stamped pose requires a stamped transform")
    return matrix_to_pose(transform @ pose_to_matrix(pose))


def optical_to_robot_frame(pose: Pose) -> Pose:
    """
    Convert pose from optical camera frame to robot frame convention.

    Optical Camera Frame (e.g., ZED):
    - X: Right
    - Y: Down
    - Z: Forward (away from camera)

    Robot Frame (ROS/REP-103):
    - X: Forward
    - Y: Left
    - Z: Up

    Args:
        pose: Pose in optical camera frame

    Returns:
        Pose in robot frame
    """
    # Position transformation
    robot_x = pose.position.z  # Forward = Camera Z
    robot_y = -pose.position.x  # Left = -Camera X
    robot_z = -pose.position.y  # Up = -Camera Y

    # Rotation transformation using quaternions
    # First convert quaternion to rotation matrix
    quat_optical = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
    R_optical = R.from_quat(quat_optical).as_matrix()

    # Coordinate frame transformation matrix from optical to robot
    # X_robot = Z_optical, Y_robot = -X_optical, Z_robot = -Y_optical
    T_frame = np.array(
        [
            [0, 0, 1],  # X_robot = Z_optical
            [-1, 0, 0],  # Y_robot = -X_optical
            [0, -1, 0],  # Z_robot = -Y_optical
        ]
    )

    # Transform the rotation matrix
    R_robot = T_frame @ R_optical @ T_frame.T

    # Convert back to quaternion
    quat_robot = R.from_matrix(R_robot).as_quat()  # [x, y, z, w]

    return Pose(
        position=Point(x=robot_x, y=robot_y, z=robot_z),
        orientation=Quaternion(x=quat_robot[0], y=quat_robot[1], z=quat_robot[2], w=quat_robot[3]),
    )


def robot_to_optical_frame(pose: Pose) -> Pose:
    """
    Convert pose from robot frame to optical camera frame convention.
    This is the inverse of optical_to_robot_frame.

    Args:
        pose: Pose in robot frame

    Returns:
        Pose in optical camera frame
    """
    # Position transformation (inverse)
    optical_x = -pose.position.y  # Right = -Left
    optical_y = -pose.position.z  # Down = -Up
    optical_z = pose.position.x  # Forward = Forward

    # Rotation transformation using quaternions
    quat_robot = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
    R_robot = R.from_quat(quat_robot).as_matrix()

    # Coordinate frame transformation matrix from Robot to optical (inverse of optical to Robot)
    # This is the transpose of the forward transformation
    T_frame_inv = np.array(
        [
            [0, -1, 0],  # X_optical = -Y_robot
            [0, 0, -1],  # Y_optical = -Z_robot
            [1, 0, 0],  # Z_optical = X_robot
        ]
    )

    # Transform the rotation matrix
    R_optical = T_frame_inv @ R_robot @ T_frame_inv.T

    # Convert back to quaternion
    quat_optical = R.from_matrix(R_optical).as_quat()  # [x, y, z, w]

    return Pose(
        position=Point(x=optical_x, y=optical_y, z=optical_z),
        orientation=Quaternion(
            x=quat_optical[0], y=quat_optical[1], z=quat_optical[2], w=quat_optical[3]
        ),
    )


def yaw_towards_point(
    position: Point | Vector3, target_point: Point | Vector3 | None = None
) -> float:
    """
    Calculate yaw angle from target point to position (away from target).
    This is commonly used for object orientation in grasping applications.
    Assumes robot frame where X is forward and Y is left.

    Args:
        position: Current position in robot frame
        target_point: Reference point (default: origin)

    Returns:
        Yaw angle in radians pointing from target_point to position
    """
    if target_point is None:
        target_point = Vector3(x=0.0, y=0.0, z=0.0)
    direction_x = position.x - target_point.x
    direction_y = position.y - target_point.y
    return np.arctan2(direction_y, direction_x)  # type: ignore[no-any-return]


def create_transform_from_6dof(translation: Vector3, euler_angles: Vector3) -> np.ndarray:
    """
    Create a 4x4 transformation matrix from 6DOF parameters.

    Args:
        translation: Translation vector [x, y, z] in meters
        euler_angles: Euler angles [rx, ry, rz] in radians (XYZ convention)

    Returns:
        4x4 transformation matrix
    """
    # Create transformation matrix
    T = np.eye(4)

    # Set translation
    T[0:3, 3] = [translation.x, translation.y, translation.z]

    # Set rotation using scipy
    if np.linalg.norm([euler_angles.x, euler_angles.y, euler_angles.z]) > 1e-6:
        rotation = R.from_euler("xyz", [euler_angles.x, euler_angles.y, euler_angles.z])
        T[0:3, 0:3] = rotation.as_matrix()

    return T


def invert_transform(T: np.ndarray) -> np.ndarray:
    """
    Invert a 4x4 transformation matrix efficiently.

    Args:
        T: 4x4 transformation matrix

    Returns:
        Inverted 4x4 transformation matrix
    """
    # For homogeneous transform matrices, we can use the special structure:
    # [R t]^-1 = [R^T -R^T*t]
    # [0 1]      [0    1    ]

    Rot = T[:3, :3]
    t = T[:3, 3]

    T_inv = np.eye(4)
    T_inv[:3, :3] = Rot.T
    T_inv[:3, 3] = -Rot.T @ t

    return T_inv


def compose_transforms(*transforms: np.ndarray) -> np.ndarray:
    """
    Compose multiple transformation matrices.

    Args:
        *transforms: Variable number of 4x4 transformation matrices

    Returns:
        Composed 4x4 transformation matrix (T1 @ T2 @ ... @ Tn)
    """
    result = np.eye(4)
    for T in transforms:
        result = result @ T
    return result


def euler_to_quaternion(euler_angles: Vector3, degrees: bool = False) -> Quaternion:
    """
    Convert euler angles to quaternion.

    Args:
        euler_angles: Euler angles as Vector3 [roll, pitch, yaw] in radians (XYZ convention)

    Returns:
        Quaternion object [x, y, z, w]
    """
    rotation = R.from_euler(
        "xyz", [euler_angles.x, euler_angles.y, euler_angles.z], degrees=degrees
    )
    quat = rotation.as_quat()  # Returns [x, y, z, w]
    return Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])


def quaternion_to_euler(quaternion: Quaternion, degrees: bool = False) -> Vector3:
    """
    Convert quaternion to euler angles.

    Args:
        quaternion: Quaternion object [x, y, z, w]

    Returns:
        Euler angles as Vector3 [roll, pitch, yaw] in radians (XYZ convention)
    """
    quat = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
    rotation = R.from_quat(quat)
    euler = rotation.as_euler("xyz", degrees=degrees)  # Returns [roll, pitch, yaw]
    if not degrees:
        return Vector3(
            x=normalize_angle(euler[0]), y=normalize_angle(euler[1]), z=normalize_angle(euler[2])
        )
    else:
        return Vector3(x=euler[0], y=euler[1], z=euler[2])


def get_distance(pose1: Pose | Vector3, pose2: Pose | Vector3) -> float:
    """
    Calculate Euclidean distance between two poses.

    Args:
        pose1: First pose
        pose2: Second pose

    Returns:
        Euclidean distance between the two poses in meters
    """
    position1 = pose1.position if isinstance(pose1, Pose) else pose1
    position2 = pose2.position if isinstance(pose2, Pose) else pose2

    dx = position1.x - position2.x
    dy = position1.y - position2.y
    dz = position1.z - position2.z

    return np.linalg.norm(np.array([dx, dy, dz]))  # type: ignore[return-value]


def offset_distance(
    target_pose: Pose, distance: float, approach_vector: Vector3 = Vector3(x=0, y=0, z=-1)
) -> Pose:
    """
    Apply distance offset to target pose along its approach direction.

    This is commonly used in grasping to offset the gripper by a certain distance
    along the approach vector before or after grasping.

    Args:
        target_pose: Target pose (e.g., grasp pose)
        distance: Distance to offset along the approach direction (meters)

    Returns:
        Target pose offset by the specified distance along its approach direction
    """
    # Convert pose to transformation matrix to extract rotation
    T_target = pose_to_matrix(target_pose)
    rotation_matrix = T_target[:3, :3]

    # Define the approach vector based on the target pose orientation
    # Assuming the gripper approaches along its local -z axis (common for downward grasps)
    # You can change this to [1, 0, 0] for x-axis or [0, 1, 0] for y-axis based on your gripper
    approach_vector_local = np.array([approach_vector.x, approach_vector.y, approach_vector.z])

    # Transform approach vector to world coordinates
    approach_vector_world = rotation_matrix @ approach_vector_local

    # Apply offset along the approach direction
    offset_position = Point(
        x=target_pose.position.x + distance * approach_vector_world[0],
        y=target_pose.position.y + distance * approach_vector_world[1],
        z=target_pose.position.z + distance * approach_vector_world[2],
    )

    return Pose(position=offset_position, orientation=target_pose.orientation)
