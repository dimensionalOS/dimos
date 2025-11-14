import pickle
import numpy as np
import open3d as o3d
import random

def save_pointcloud(pcd, file_path):
    """
    Save a point cloud to a file using Open3D.
    """
    o3d.io.write_point_cloud(file_path, pcd)

def restore_pointclouds(pointcloud_paths):
    restored_pointclouds = []
    for path in pointcloud_paths:
        restored_pointclouds.append(o3d.io.read_point_cloud(path))
    return restored_pointclouds


def create_point_cloud_from_rgbd(rgb_image, depth_image, intrinsic_parameters):
    rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
        o3d.geometry.Image(rgb_image),
        o3d.geometry.Image(depth_image),
        depth_scale=0.125, #1000.0,
        depth_trunc=10.0, #10.0,
        convert_rgb_to_intensity=False
    )
    intrinsic = o3d.camera.PinholeCameraIntrinsic()
    intrinsic.set_intrinsics(intrinsic_parameters['width'], intrinsic_parameters['height'],
                             intrinsic_parameters['fx'], intrinsic_parameters['fy'],
                             intrinsic_parameters['cx'], intrinsic_parameters['cy'])
    pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_image, intrinsic)
    return pcd

def canonicalize_point_cloud(pcd, canonicalize_threshold=0.3):
    # Segment the largest plane, assumed to be the floor
    plane_model, inliers = pcd.segment_plane(distance_threshold=0.01, ransac_n=3, num_iterations=1000)

    canonicalized = False
    if len(inliers) / len(pcd.points) > canonicalize_threshold:
        canonicalized = True

        # Ensure the plane normal points upwards
        if np.dot(plane_model[:3], [0, 1, 0]) < 0:
            plane_model = -plane_model

        # Normalize the plane normal vector
        normal = plane_model[:3] / np.linalg.norm(plane_model[:3])

        # Compute the new basis vectors
        new_y = normal
        new_x = np.cross(new_y, [0, 0, -1])
        new_x /= np.linalg.norm(new_x)
        new_z = np.cross(new_x, new_y)

        # Create the transformation matrix
        transformation = np.identity(4)
        transformation[:3, :3] = np.vstack((new_x, new_y, new_z)).T
        transformation[:3, 3] = -np.dot(transformation[:3, :3], pcd.points[inliers[0]])

        # Apply the transformation
        pcd.transform(transformation)

        # Additional 180-degree rotation around the Z-axis
        rotation_z_180 = np.array([[np.cos(np.pi), -np.sin(np.pi), 0],
                                   [np.sin(np.pi), np.cos(np.pi), 0],
                                   [0, 0, 1]])
        pcd.rotate(rotation_z_180, center=(0, 0, 0))

        return pcd, canonicalized, transformation
    else:
        return pcd, canonicalized, None


# Distance calculations
def human_like_distance(distance_meters):
    # Define the choices with units included, focusing on the 0.1 to 10 meters range
    if distance_meters < 1:  # For distances less than 1 meter
        choices = [
            (
                round(distance_meters * 100, 2),
                "centimeters",
                0.2,
            ),  # Centimeters for very small distances
            (
                round(distance_meters * 39.3701, 2),
                "inches",
                0.8,
            ),  # Inches for the majority of cases under 1 meter
        ]
    elif distance_meters < 3:  # For distances less than 3 meters
        choices = [
            (round(distance_meters, 2), "meters", 0.5),
            (
                round(distance_meters * 3.28084, 2),
                "feet",
                0.5,
            ),  # Feet as a common unit within indoor spaces
        ]
    else:  # For distances from 3 up to 10 meters
        choices = [
            (
                round(distance_meters, 2),
                "meters",
                0.7,
            ),  # Meters for clarity and international understanding
            (
                round(distance_meters * 3.28084, 2),
                "feet",
                0.3,
            ),  # Feet for additional context
        ]

    # Normalize probabilities and make a selection
    total_probability = sum(prob for _, _, prob in choices)
    cumulative_distribution = []
    cumulative_sum = 0
    for value, unit, probability in choices:
        cumulative_sum += probability / total_probability  # Normalize probabilities
        cumulative_distribution.append((cumulative_sum, value, unit))

    # Randomly choose based on the cumulative distribution
    r = random.random()
    for cumulative_prob, value, unit in cumulative_distribution:
        if r < cumulative_prob:
            return f"{value} {unit}"

    # Fallback to the last choice if something goes wrong
    return f"{choices[-1][0]} {choices[-1][1]}"

def calculate_distances_between_point_clouds(A, B):
    dist_pcd1_to_pcd2 = np.asarray(A.compute_point_cloud_distance(B))
    dist_pcd2_to_pcd1 = np.asarray(B.compute_point_cloud_distance(A))
    combined_distances = np.concatenate((dist_pcd1_to_pcd2, dist_pcd2_to_pcd1))
    avg_dist = np.mean(combined_distances)
    return human_like_distance(avg_dist)

def calculate_centroid(pcd):
    """Calculate the centroid of a point cloud."""
    points = np.asarray(pcd.points)
    centroid = np.mean(points, axis=0)
    return centroid

def calculate_relative_positions(centroids):
    """Calculate the relative positions between centroids of point clouds."""
    num_centroids = len(centroids)
    relative_positions_info = []

    for i in range(num_centroids):
        for j in range(i + 1, num_centroids):
            relative_vector = centroids[j] - centroids[i]

            distance = np.linalg.norm(relative_vector)
            relative_positions_info.append({
                'pcd_pair': (i, j),
                'relative_vector': relative_vector,
                'distance': distance
            })

    return relative_positions_info

def get_bounding_box_height(pcd):
    """
    Compute the height of the bounding box for a given point cloud.

    Parameters:
    pcd (open3d.geometry.PointCloud): The input point cloud.

    Returns:
    float: The height of the bounding box.
    """
    aabb = pcd.get_axis_aligned_bounding_box()
    return aabb.get_extent()[1]  # Assuming the Y-axis is the up-direction

def compare_bounding_box_height(pcd_i, pcd_j):
    """
    Compare the bounding box heights of two point clouds.

    Parameters:
    pcd_i (open3d.geometry.PointCloud): The first point cloud.
    pcd_j (open3d.geometry.PointCloud): The second point cloud.

    Returns:
    bool: True if the bounding box of pcd_i is taller than that of pcd_j, False otherwise.
    """
    height_i = get_bounding_box_height(pcd_i)
    height_j = get_bounding_box_height(pcd_j)

    return height_i > height_j
