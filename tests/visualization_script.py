#!/usr/bin/env python3
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

"""Visualize pickled manipulation pipeline results."""

import os
import sys
import pickle
import numpy as np
import json
import matplotlib

# Try to use TkAgg backend for live display, fallback to Agg if not available
try:
    matplotlib.use("TkAgg")
except:
    try:
        matplotlib.use("Qt5Agg")
    except:
        matplotlib.use("Agg")  # Fallback to non-interactive
import matplotlib.pyplot as plt
import open3d as o3d

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from dimos.perception.pointcloud.utils import visualize_clustered_point_clouds, visualize_voxel_grid
from dimos.perception.grasp_generation.utils import visualize_grasps_3d
from dimos.perception.pointcloud.utils import visualize_pcd
from dimos.utils.logging_config import setup_logger
import trimesh

import tf_lcm_py
import cv2
from contextlib import contextmanager
import lcm_msgs
from lcm_msgs.sensor_msgs import JointState, PointCloud2, CameraInfo, PointCloud2, PointField
from lcm_msgs.std_msgs import Header
from typing import List, Tuple, Optional
import atexit
from datetime import datetime
import time

from pydrake.all import (
    AddMultibodyPlantSceneGraph,
    CoulombFriction,
    Diagram,
    DiagramBuilder,
    InverseKinematics,
    MeshcatVisualizer,
    MeshcatVisualizerParams,
    MultibodyPlant,
    Parser,
    RigidTransform,
    RollPitchYaw,
    RotationMatrix,
    JointIndex,
    Solve,
    StartMeshcat,
)
from pydrake.geometry import (
    CollisionFilterDeclaration,
    Mesh,
    ProximityProperties,
    InMemoryMesh,
    Box,
    Cylinder,
)
from pydrake.math import RigidTransform as DrakeRigidTransform
from pydrake.common import MemoryFile

from pydrake.all import MinimumDistanceLowerBoundConstraint, MultibodyPlant, Parser, DiagramBuilder, AddMultibodyPlantSceneGraph, MeshcatVisualizer, StartMeshcat, RigidTransform, Role, RollPitchYaw, RotationMatrix, Solve, InverseKinematics, MeshcatVisualizerParams, MinimumDistanceLowerBoundConstraint, DoDifferentialInverseKinematics, DifferentialInverseKinematicsStatus, DifferentialInverseKinematicsParameters, DepthImageToPointCloud
from pydrake.geometry import QueryObject, SignedDistancePair

logger = setup_logger("visualization_script")


def create_point_cloud(color_img, depth_img, intrinsics):
    """Create Open3D point cloud from RGB and depth images."""
    fx, fy, cx, cy = intrinsics
    height, width = depth_img.shape

    o3d_intrinsics = o3d.camera.PinholeCameraIntrinsic(width, height, fx, fy, cx, cy)
    color_o3d = o3d.geometry.Image(color_img)
    depth_o3d = o3d.geometry.Image((depth_img * 1000).astype(np.uint16))

    rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
        color_o3d, depth_o3d, depth_scale=1000.0, convert_rgb_to_intensity=False
    )

    return o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, o3d_intrinsics)


def deserialize_point_cloud(data):
    """Reconstruct Open3D PointCloud from serialized data."""
    if data is None:
        return None
    
    pcd = o3d.geometry.PointCloud()
    if 'points' in data and data['points']:
        pcd.points = o3d.utility.Vector3dVector(np.array(data['points']))
    if 'colors' in data and data['colors']:
        pcd.colors = o3d.utility.Vector3dVector(np.array(data['colors']))
    return pcd


def deserialize_voxel_grid(data):
    """Reconstruct Open3D VoxelGrid from serialized data."""
    if data is None:
        return None
    
    # Create a point cloud to convert to voxel grid
    pcd = o3d.geometry.PointCloud()
    voxel_size = data['voxel_size']
    origin = np.array(data['origin'])
    
    # Create points from voxel indices
    points = []
    colors = []
    for voxel in data['voxels']:
        # Each voxel is (i, j, k, r, g, b)
        i, j, k, r, g, b = voxel
        # Convert voxel grid index to 3D point
        point = origin + np.array([i, j, k]) * voxel_size
        points.append(point)
        colors.append([r, g, b])
    
    if points:
        pcd.points = o3d.utility.Vector3dVector(np.array(points))
        pcd.colors = o3d.utility.Vector3dVector(np.array(colors))
        
    # Convert to voxel grid
    voxel_grid = o3d.geometry.VoxelGrid.create_from_point_cloud(pcd, voxel_size)
    return voxel_grid


def visualize_results(pickle_path="manipulation_results.pkl"):
    """Load pickled results and visualize them."""
    print(f"Loading results from {pickle_path}...")
    try:
        with open(pickle_path, "rb") as f:
            data = pickle.load(f)
            
        results = data["results"]
        color_img = data["color_img"]
        depth_img = data["depth_img"]
        intrinsics = data["intrinsics"]
        
        print(f"Loaded results with keys: {list(results.keys())}")
        
    except FileNotFoundError:
        print(f"Error: Pickle file {pickle_path} not found.")
        print("Make sure to run test_manipulation_pipeline_single_frame_lcm.py first.")
        return
    except Exception as e:
        print(f"Error loading pickle file: {e}")
        return
    
    # Determine number of subplots based on what results we have
    num_plots = 0
    plot_configs = []

    if "detection_viz" in results and results["detection_viz"] is not None:
        plot_configs.append(("detection_viz", "Object Detection"))
        num_plots += 1

    if "segmentation_viz" in results and results["segmentation_viz"] is not None:
        plot_configs.append(("segmentation_viz", "Semantic Segmentation"))
        num_plots += 1

    if "pointcloud_viz" in results and results["pointcloud_viz"] is not None:
        plot_configs.append(("pointcloud_viz", "All Objects Point Cloud"))
        num_plots += 1

    if "detected_pointcloud_viz" in results and results["detected_pointcloud_viz"] is not None:
        plot_configs.append(("detected_pointcloud_viz", "Detection Objects Point Cloud"))
        num_plots += 1

    if "misc_pointcloud_viz" in results and results["misc_pointcloud_viz"] is not None:
        plot_configs.append(("misc_pointcloud_viz", "Misc/Background Points"))
        num_plots += 1

    if "grasp_overlay" in results and results["grasp_overlay"] is not None:
        plot_configs.append(("grasp_overlay", "Grasp Overlay"))
        num_plots += 1

    if num_plots == 0:
        print("No visualization results to display")
        return

    # Create subplot layout
    if num_plots <= 3:
        fig, axes = plt.subplots(1, num_plots, figsize=(6 * num_plots, 5))
    else:
        rows = 2
        cols = (num_plots + 1) // 2
        fig, axes = plt.subplots(rows, cols, figsize=(6 * cols, 5 * rows))

    # Ensure axes is always a list for consistent indexing
    if num_plots == 1:
        axes = [axes]
    elif num_plots > 2:
        axes = axes.flatten()

    # Plot each result
    for i, (key, title) in enumerate(plot_configs):
        axes[i].imshow(results[key])
        axes[i].set_title(title)
        axes[i].axis("off")

    # Hide unused subplots if any
    if num_plots > 3:
        for i in range(num_plots, len(axes)):
            axes[i].axis("off")

    plt.tight_layout()

    # Save and show the plot
    output_path = "visualization_results.png"
    plt.savefig(output_path, dpi=150, bbox_inches="tight")
    print(f"Results visualization saved to: {output_path}")

    # Show plot live as well
    plt.show(block=True)
    plt.close()

    # Deserialize and reconstruct 3D objects from the pickle file
    print("\nReconstructing 3D visualization objects from serialized data...")

    # Reconstruct full point cloud if available
    full_pcd = None
    if "full_pointcloud" in results and results["full_pointcloud"] is not None:
        full_pcd = deserialize_point_cloud(results["full_pointcloud"])
        print(f"Reconstructed full point cloud with {len(np.asarray(full_pcd.points))} points")
        
        # Visualize reconstructed full point cloud
        try:
            visualize_pcd(
                full_pcd,
                window_name="Reconstructed Full Scene Point Cloud",
                point_size=2.0,
                show_coordinate_frame=True,
            )
        except (KeyboardInterrupt, EOFError):
            print("\nSkipping full point cloud visualization")
        except Exception as e:
            print(f"Error in point cloud visualization: {e}")
    else:
        print("No full point cloud available for visualization")
    
    # Reconstruct misc clusters if available
    if "misc_clusters" in results and results["misc_clusters"]:
        misc_clusters = [deserialize_point_cloud(cluster) for cluster in results["misc_clusters"]]
        cluster_count = len(misc_clusters)
        total_misc_points = sum(len(np.asarray(cluster.points)) for cluster in misc_clusters)
        print(f"Reconstructed {cluster_count} misc clusters with {total_misc_points} total points")
        
        # Visualize reconstructed misc clusters
        try:
            visualize_clustered_point_clouds(
                misc_clusters,
                window_name="Reconstructed Misc/Background Clusters (DBSCAN)",
                point_size=3.0,
                show_coordinate_frame=True,
            )
        except (KeyboardInterrupt, EOFError):
            print("\nSkipping misc clusters visualization")
        except Exception as e:
            print(f"Error in misc clusters visualization: {e}")
    else:
        print("No misc clusters available for visualization")
    
    # Reconstruct voxel grid if available
    if "misc_voxel_grid" in results and results["misc_voxel_grid"] is not None:
        misc_voxel_grid = deserialize_voxel_grid(results["misc_voxel_grid"])
        if misc_voxel_grid:
            voxel_count = len(misc_voxel_grid.get_voxels())
            print(f"Reconstructed voxel grid with {voxel_count} voxels")
            
            # Visualize reconstructed voxel grid
            try:
                visualize_voxel_grid(
                    misc_voxel_grid,
                    window_name="Reconstructed Misc/Background Voxel Grid",
                    show_coordinate_frame=True,
                )
            except (KeyboardInterrupt, EOFError):
                print("\nSkipping voxel grid visualization")
            except Exception as e:
                print(f"Error in voxel grid visualization: {e}")
        else:
            print("Failed to reconstruct voxel grid")
    else:
        print("No voxel grid available for visualization")

class DrakeKinematicsEnv:
    def __init__(self, urdf_path: str, kinematic_chain_joints: List[str], links_to_ignore: Optional[List[str]] = None, collision_depth_threshold: float = 0.005, use_rectangular_prisms: bool = False):
        self._resources_to_cleanup = []
        self.collision_depth_threshold = collision_depth_threshold
        self.initial_robot_collisions = set()  # Store initial robot-world collisions to ignore
        self.filtered_geometry_names = set()  # Store names of geometries to exclude from IK
        self.ik_plant = None  # Separate plant for collision-aware IK
        self.ik_scene_graph = None
        self.ik_diagram = None
        self.ik_context = None

        # Register cleanup at exit
        atexit.register(self.cleanup_resources)

        # Initialize tf resources once and reuse them
        self.buffer = tf_lcm_py.Buffer(30.0)
        self._resources_to_cleanup.append(self.buffer)
        with self.safe_lcm_instance() as lcm_instance:
            self.tf_lcm_instance = lcm_instance
            self._resources_to_cleanup.append(self.tf_lcm_instance)
            # Create TransformListener with our LCM instance and buffer
            self.listener = tf_lcm_py.TransformListener(self.tf_lcm_instance, self.buffer)
            self._resources_to_cleanup.append(self.listener)

        # Check if URDF file exists
        if not os.path.exists(urdf_path):
            raise FileNotFoundError(f"URDF file not found: {urdf_path}")
        
        # Drake utils initialization
        self.meshcat = StartMeshcat()
        print(f"Meshcat started at: {self.meshcat.web_url()}")
        
        self.urdf_path = urdf_path
        self.links_to_ignore = links_to_ignore
        self.builder = DiagramBuilder()

        self.plant, self.scene_graph = AddMultibodyPlantSceneGraph(self.builder, time_step=0.01)
        self.parser = Parser(self.plant)

        # Load the robot URDF
        print(f"Loading URDF from: {self.urdf_path}")
        self.model_instances = self.parser.AddModelsFromUrl(f"file://{self.urdf_path}")
        self.kinematic_chain_joints = kinematic_chain_joints
        self.model_instance = self.model_instances[0] if self.model_instances else None
        
        if not self.model_instances:
            raise RuntimeError("Failed to load any model instances from URDF")
        
        print(f"Loaded {len(self.model_instances)} model instances")

        # Set up collision filtering
        if links_to_ignore:
            bodies = []
            for link_name in links_to_ignore:
                try:
                    body = self.plant.GetBodyByName(link_name)
                    if body is not None:
                        bodies.extend(self.plant.GetBodiesWeldedTo(body))
                except RuntimeError:
                    print(f"Warning: Link '{link_name}' not found in URDF")

            if bodies:
                arm_geoms = self.plant.CollectRegisteredGeometries(bodies)
                decl = CollisionFilterDeclaration().ExcludeWithin(arm_geoms)
                manager = self.scene_graph.collision_filter_manager()
                manager.Apply(decl)

        # Load and process point cloud data
        self._load_and_process_point_clouds()

        # Finalize the plant before adding visualizer
        self.plant.Finalize()
        
        # Print some debug info about the plant
        print(f"Plant has {self.plant.num_bodies()} bodies")
        print(f"Plant has {self.plant.num_joints()} joints")
        for i in range(self.plant.num_joints()):
            joint = self.plant.get_joint(JointIndex(i))
            print(f"  Joint {i}: {joint.name()} (type: {joint.type_name()})")

        # Add visualizer
        self.visualizer = MeshcatVisualizer.AddToBuilder(
            self.builder, 
            self.scene_graph, 
            self.meshcat, 
            params=MeshcatVisualizerParams()
        )

        # Build the diagram
        self.diagram = self.builder.Build()
        self.diagram_context = self.diagram.CreateDefaultContext()
        self.plant_context = self.plant.GetMyContextFromRoot(self.diagram_context)

        # Set up joint indices
        self.joint_indices = []
        for joint_name in self.kinematic_chain_joints:
            try:
                joint = self.plant.GetJointByName(joint_name)
                if joint.num_positions() > 0:
                    start_index = joint.position_start()
                    for i in range(joint.num_positions()):
                        self.joint_indices.append(start_index + i)
                    print(f"Added joint '{joint_name}' at indices {start_index} to {start_index + joint.num_positions() - 1}")
            except RuntimeError:
                print(f"Warning: Joint '{joint_name}' not found in URDF.")

        # Get important frames/bodies
        try:
            self.end_effector_link = self.plant.GetBodyByName("link6")
            self.end_effector_frame = self.plant.GetFrameByName("link6")
            print("Found end effector link6")
        except RuntimeError:
            print("Warning: link6 not found")
            self.end_effector_link = None
            self.end_effector_frame = None
            
        try:
            self.camera_link = self.plant.GetBodyByName("camera_center_link")
            print("Found camera_center_link")
        except RuntimeError:
            print("Warning: camera_center_link not found")
            self.camera_link = None

        # Set robot joint positions from pickle file data
        self._set_joint_positions_from_pickle()
        
        # Add meshcat sliders for joint control
        self._add_joint_sliders()
        
        # Identify initial robot-world collisions to ignore
        self._identify_initial_robot_collisions()
        
        # Create separate IK environment without filtered geometries
        self._create_ik_environment()
        
        # Force initial visualization update
        self._update_visualization()
        
        print("Drake environment initialization complete!")
        print(f"Visit {self.meshcat.web_url()} to see the visualization")

    def _load_and_process_point_clouds(self):
        """Load point cloud data from pickle file and add to scene"""
        pickle_path = "manipulation_results.pkl"
        try:
            with open(pickle_path, "rb") as f:
                data = pickle.load(f)
                
            results = data["results"]
            print(f"Loaded results with keys: {list(results.keys())}")
            
        except FileNotFoundError:
            print(f"Warning: Pickle file {pickle_path} not found.")
            print("Skipping point cloud loading.")
            return
        except Exception as e:
            print(f"Warning: Error loading pickle file: {e}")
            return

        # Preprocess detected objects to extract planes and remove overlapping points
        if "all_objects" in results:
            # results["all_objects"] = self._preprocess_plane_detection(results["all_objects"])
            results["all_objects"] = self._preprocess_overlapping_points(results["all_objects"])
        
        # if "misc_clusters" in results:
        #     results["misc_clusters"] = self._preprocess_overlapping_points(results["misc_clusters"])
        #     results["misc_clusters"] = self._preprocess_plane_detection(results["misc_clusters"])
        
        full_detected_pcd = o3d.geometry.PointCloud()
        for obj in results["all_objects"]:
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(obj["point_cloud_numpy"])
            full_detected_pcd += pcd
        
        self.process_and_add_object_class("all_objects", results)
        self.process_and_add_object_class("misc_clusters", results)
        # misc_clusters = results["misc_clusters"]
        # print(type(misc_clusters[0]["points"]))
        # print(np.asarray(misc_clusters[0]["points"]).shape)

    def _preprocess_plane_detection(self, detected_objects):
        """
        Preprocess detected objects to extract large planes (like tables) and create separate plane objects.
        For each object, detect planes larger than 5cm and split them into separate objects.
        
        Args:
            detected_objects: List of detected object dictionaries
            
        Returns:
            List of detected objects with planes extracted as separate objects
        """
        if not detected_objects:
            return detected_objects
            
        print(f"Preprocessing {len(detected_objects)} objects for plane detection...")
        
        processed_objects = []
        plane_objects = []
        min_plane_size = 0.05  # 5cm minimum plane size
        
        for i, obj in enumerate(detected_objects):
            if "point_cloud_numpy" in obj:
                points = obj["point_cloud_numpy"]
            elif "point_cloud" in obj and obj["point_cloud"]:
                points = np.array(obj["point_cloud"]["points"])
            else:
                print(f"Warning: No point cloud data found for object {i}")
                processed_objects.append(obj)
                continue
                
            if len(points) < 50:  # Need enough points for plane detection
                print(f"  Object {i}: Too few points ({len(points)}) for plane detection")
                processed_objects.append(obj)
                continue
            
            # Extract planes from this object
            remaining_points, extracted_planes = self._extract_planes_from_points(points, i, min_plane_size)
            
            # Update original object with remaining points
            if len(remaining_points) > 0:
                obj_copy = obj.copy()
                obj_copy["point_cloud_numpy"] = remaining_points
                processed_objects.append(obj_copy)
                print(f"  Object {i}: {len(points)} → {len(remaining_points)} points (extracted {len(extracted_planes)} planes)")
            else:
                print(f"  Object {i}: All points were part of planes, object removed")
            
            # Create new objects for extracted planes
            for j, plane_points in enumerate(extracted_planes):
                plane_obj = obj.copy()
                plane_obj["point_cloud_numpy"] = plane_points
                plane_obj["is_extracted_plane"] = True
                plane_obj["original_object_index"] = i
                plane_obj["plane_index"] = j
                plane_objects.append(plane_obj)
                print(f"    Created plane object from object {i}, plane {j}: {len(plane_points)} points")
        
        # Combine processed objects with extracted plane objects
        all_objects = processed_objects + plane_objects
        print(f"Plane detection complete: {len(processed_objects)} objects + {len(plane_objects)} extracted planes = {len(all_objects)} total")
        
        return all_objects
    
    def _extract_planes_from_points(self, points, object_id, min_plane_size):
        """
        Extract planes from a point cloud using RANSAC plane detection.
        
        Args:
            points: Nx3 numpy array of 3D points
            object_id: ID for debugging/logging
            min_plane_size: Minimum size (in meters) for a plane to be extracted
            
        Returns:
            Tuple of (remaining_points, extracted_planes)
            - remaining_points: Points that don't belong to any large plane
            - extracted_planes: List of numpy arrays, each containing points of a detected plane
        """
        try:
            # Create Open3D point cloud
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(points)
            
            remaining_points = points.copy()
            extracted_planes = []
            
            # Iteratively extract planes
            max_iterations = 3  # Maximum number of planes to extract per object
            for iteration in range(max_iterations):
                if len(remaining_points) < 50:  # Need enough points for plane detection
                    break
                
                # Create point cloud from remaining points
                temp_pcd = o3d.geometry.PointCloud()
                temp_pcd.points = o3d.utility.Vector3dVector(remaining_points)
                
                # RANSAC plane detection
                plane_model, inliers = temp_pcd.segment_plane(
                    distance_threshold=0.01,  # 1cm tolerance
                    ransac_n=3,
                    num_iterations=1000
                )
                
                if len(inliers) < 50:  # Need enough inliers for a valid plane
                    break
                
                # Get plane points
                plane_points = remaining_points[inliers]
                
                # Check if plane is large enough
                plane_pcd = o3d.geometry.PointCloud()
                plane_pcd.points = o3d.utility.Vector3dVector(plane_points)
                
                # Calculate plane bounding box to estimate size
                bbox = plane_pcd.get_axis_aligned_bounding_box()
                bbox_size = bbox.get_max_bound() - bbox.get_min_bound()
                max_dimension = np.max(bbox_size)
                
                if max_dimension >= min_plane_size:
                    # This is a large enough plane, extract it
                    extracted_planes.append(plane_points)
                    
                    # Remove plane points from remaining points
                    mask = np.ones(len(remaining_points), dtype=bool)
                    mask[inliers] = False
                    remaining_points = remaining_points[mask]
                    
                    print(f"    Extracted plane {len(extracted_planes)} from object {object_id}: {len(plane_points)} points, max dimension: {max_dimension:.3f}m")
                else:
                    # Plane is too small, stop looking for more planes
                    print(f"    Found small plane in object {object_id} (max dimension: {max_dimension:.3f}m < {min_plane_size}m), stopping")
                    break
            
            return remaining_points, extracted_planes
            
        except Exception as e:
            print(f"Error in plane extraction for object {object_id}: {e}")
            return points, []  # Return original points if extraction fails

    def _preprocess_overlapping_points(self, detected_objects):
        """
        Preprocess detected objects to remove overlapping points.
        Sort objects by number of points (ascending) and remove points from larger objects
        that are already present in smaller objects.
        
        Args:
            detected_objects: List of detected object dictionaries
            
        Returns:
            List of detected objects with overlapping points removed
        """
        if not detected_objects:
            return detected_objects
            
        print(f"Preprocessing {len(detected_objects)} objects to remove overlapping points...")
        
        # Extract point clouds and sort by number of points (ascending)
        objects_with_points = []
        for i, obj in enumerate(detected_objects):
            if "point_cloud_numpy" in obj:
                points = obj["point_cloud_numpy"]
            elif "point_cloud" in obj and obj["point_cloud"]:
                points = np.array(obj["point_cloud"]["points"])
            else:
                print(f"Warning: No point cloud data found for object {i}")
                continue
                
            objects_with_points.append({
                'original_index': i,
                'points': points,
                'num_points': len(points),
                'object_data': obj.copy()
            })
        
        # Sort by number of points (ascending - smallest first)
        objects_with_points.sort(key=lambda x: x['num_points'])
        
        print("Object sizes before preprocessing:")
        for obj in objects_with_points:
            print(f"  Object {obj['original_index']}: {obj['num_points']} points")
        
        # Process objects from smallest to largest, removing overlapping points
        processed_objects = []
        all_processed_points = set()
        
        # Define a tolerance for point matching (in meters)
        tolerance = 0.01  # 10mm tolerance
        
        for obj_data in objects_with_points:
            points = obj_data['points']
            original_count = len(points)
            
            # Remove points that are too close to already processed points
            if all_processed_points:
                # Convert processed points to numpy array for efficient distance computation
                processed_points_array = np.array(list(all_processed_points))
                
                # Find points that are far enough from all processed points
                unique_points = []
                for point in points:
                    # Calculate distances to all processed points
                    distances = np.linalg.norm(processed_points_array - point, axis=1)
                    # Keep point if it's far enough from all processed points
                    if np.min(distances) > tolerance:
                        unique_points.append(point)
                        # Add to processed points set (rounded for set storage)
                        all_processed_points.add(tuple(np.round(point, 4)))
                    
                unique_points = np.array(unique_points) if unique_points else np.empty((0, 3))
            else:
                # First object - keep all points
                unique_points = points
                # Add all points to processed set
                for point in points:
                    all_processed_points.add(tuple(np.round(point, 4)))
            
            # Update object data with filtered points
            obj_data['object_data']['point_cloud_numpy'] = unique_points
            processed_objects.append(obj_data)
            
            removed_count = original_count - len(unique_points)
            print(f"  Object {obj_data['original_index']}: {original_count} → {len(unique_points)} points (removed {removed_count} overlapping)")
        
        # Sort back to original order and extract the processed object data
        processed_objects.sort(key=lambda x: x['original_index'])
        result = [obj['object_data'] for obj in processed_objects]
        
        print("Preprocessing complete!")
        return result

    def process_and_add_object_class(self, object_key: str, results: dict):
        # Process detected objects
        if object_key in results:
            detected_objects = results[object_key]
            if detected_objects:
                print(f"Processing {len(detected_objects)} {object_key}")
                all_decomposed_meshes = []
                
                transform = self.get_transform("world", "camera_center_link")
                for i in range(len(detected_objects)):
                    try:
                        if object_key == "misc_clusters":
                            points = np.asarray(detected_objects[i]["points"])
                        elif "point_cloud_numpy" in detected_objects[i]:
                            points = detected_objects[i]["point_cloud_numpy"]
                        elif "point_cloud" in detected_objects[i] and detected_objects[i]["point_cloud"]:
                            # Handle serialized point cloud
                            points = np.array(detected_objects[i]["point_cloud"]["points"])
                        else:
                            print(f"Warning: No point cloud data found for object {i}")
                            continue
                            
                        if len(points) < 10:  # Need more points for mesh reconstruction
                            print(f"Warning: Object {i} has too few points ({len(points)}) for mesh reconstruction")
                            continue
                        
                        # Swap y-z axes since this is a common problem
                        points = np.column_stack((points[:, 0], points[:, 2], -points[:, 1]))
                        # Transform points to world frame
                        points = self.transform_point_cloud_with_open3d(points, transform)
                            
                        # Use voxelized clustering + convex hulls approach
                        clustered_hulls = self._create_voxelized_clustered_convex_hulls(points, i)
                        # Store hulls with their object ID for coloring
                        for hull in clustered_hulls:
                            all_decomposed_meshes.append((hull, i))
                        
                        print(f"Created {len(clustered_hulls)} clustered convex hulls for object {i}")
                        
                    except Exception as e:
                        print(f"Warning: Failed to process object {i}: {e}")
                
                if all_decomposed_meshes:
                    self.register_convex_hulls_as_collision(all_decomposed_meshes, object_key, detected_objects)
                    print(f"Registered {len(all_decomposed_meshes)} total clustered convex hulls")
                else:
                    print("Warning: No valid clustered convex hulls created from detected objects")
            else:
                print("No detected objects found")

    def _create_clustered_convex_hulls(self, points: np.ndarray, object_id: int) -> List[o3d.geometry.TriangleMesh]:
        """
        Create convex hulls from DBSCAN clusters of point cloud data.
        Fast approach: cluster points, then convex hull each cluster.
        
        Args:
            points: Nx3 numpy array of 3D points
            object_id: ID for debugging/logging
            
        Returns:
            List of Open3D triangle meshes (convex hulls of clusters)
        """
        try:
            # Create Open3D point cloud
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(points)
            
            # Quick outlier removal (optional, can skip for speed)
            if len(points) > 50:  # Only for larger point clouds
                pcd, _ = pcd.remove_statistical_outlier(nb_neighbors=10, std_ratio=2.0)
                points = np.asarray(pcd.points)
            
            if len(points) < 4:
                print(f"Warning: Too few points after filtering for object {object_id}")
                return []
            
            # Try multiple DBSCAN parameter combinations to find clusters
            clusters = []
            labels = None
            
            # Calculate some basic statistics for parameter estimation
            if len(points) > 10:
                # Compute nearest neighbor distances for better eps estimation
                distances = pcd.compute_nearest_neighbor_distance()
                avg_nn_distance = np.mean(distances)
                std_nn_distance = np.std(distances)
                
                print(f"Object {object_id}: {len(points)} points, avg_nn_dist={avg_nn_distance:.4f}")
                
                for i in range(20):
                    try:
                        eps = avg_nn_distance * (2.0 + (i*0.1))
                        min_samples = 20
                        labels = np.array(pcd.cluster_dbscan(eps=eps, min_points=min_samples))
                        unique_labels = np.unique(labels)
                        clusters = unique_labels[unique_labels >= 0]  # Remove noise label (-1)
                        
                        noise_points = np.sum(labels == -1)
                        clustered_points = len(points) - noise_points
                        
                        print(f"  Try {i+1}: eps={eps:.4f}, min_samples={min_samples} → {len(clusters)} clusters, {clustered_points}/{len(points)} points clustered")
                        
                        # Accept if we found clusters and most points are clustered
                        if len(clusters) > 0 and clustered_points >= len(points) * 0.95:  # At least 30% of points clustered
                            print(f"  ✓ Accepted parameter set {i+1}")
                            break
                            
                    except Exception as e:
                        print(f"  Try {i+1}: Failed with eps={eps:.4f}, min_samples={min_samples}: {e}")
                        continue
            
            if len(clusters) == 0 or labels is None:
                print(f"No clusters found for object {object_id} after all attempts, using entire point cloud")
                # Fallback: use entire point cloud as single convex hull
                hull_mesh, _ = pcd.compute_convex_hull()
                hull_mesh.compute_vertex_normals()
                return [hull_mesh]
            
            print(f"Found {len(clusters)} clusters for object {object_id} (eps={eps:.3f}, min_samples={min_samples})")

            
            # Create convex hull for each cluster
            convex_hulls = []
            for cluster_id in clusters:
                try:
                    # Get points for this cluster
                    cluster_mask = labels == cluster_id
                    cluster_points = points[cluster_mask]
                    
                    if len(cluster_points) < 4:
                        print(f"Skipping cluster {cluster_id} with only {len(cluster_points)} points")
                        continue
                    
                    # Create point cloud for this cluster
                    cluster_pcd = o3d.geometry.PointCloud()
                    cluster_pcd.points = o3d.utility.Vector3dVector(cluster_points)
                    
                    # Compute convex hull
                    hull_mesh, _ = cluster_pcd.compute_convex_hull()
                    hull_mesh.compute_vertex_normals()
                    
                    # Validate hull
                    if len(np.asarray(hull_mesh.vertices)) >= 4 and len(np.asarray(hull_mesh.triangles)) >= 4:
                        convex_hulls.append(hull_mesh)
                        print(f"  Cluster {cluster_id}: {len(cluster_points)} points → convex hull with {len(np.asarray(hull_mesh.vertices))} vertices")
                    else:
                        print(f"  Skipping degenerate hull for cluster {cluster_id}")
                        
                except Exception as e:
                    print(f"Error processing cluster {cluster_id} for object {object_id}: {e}")
            
            if not convex_hulls:
                print(f"No valid convex hulls created for object {object_id}, using entire point cloud")
                # Fallback: use entire point cloud as single convex hull
                hull_mesh, _ = pcd.compute_convex_hull()
                hull_mesh.compute_vertex_normals()
                return [hull_mesh]
            
            return convex_hulls
            
        except Exception as e:
            print(f"Error in DBSCAN clustering for object {object_id}: {e}")
            # Final fallback: single convex hull
            try:
                pcd = o3d.geometry.PointCloud()
                pcd.points = o3d.utility.Vector3dVector(points)
                hull_mesh, _ = pcd.compute_convex_hull()
                hull_mesh.compute_vertex_normals()
                return [hull_mesh]
            except:
                return []

    def _create_voxelized_clustered_convex_hulls(self, points: np.ndarray, object_id: int) -> List[o3d.geometry.TriangleMesh]:
        """
        Create convex hulls from voxelized clusters of point cloud data.
        This method creates a voxel grid and groups points that fall into the same voxel.
        
        Args:
            points: Nx3 numpy array of 3D points
            object_id: ID for debugging/logging
            
        Returns:
            List of Open3D triangle meshes (convex hulls of voxel clusters)
        """
        try:
            if len(points) < 4:
                print(f"Warning: Too few points ({len(points)}) for object {object_id}")
                return []
            
            # Find min and max dimensions
            min_coords = np.min(points, axis=0)
            max_coords = np.max(points, axis=0)
            
            # Calculate voxel size by dividing total range by 10 for each dimension
            ranges = max_coords - min_coords
            voxel_size = ranges / 10.0
            
            # Handle edge case where range is too small
            min_voxel_size = 0.001  # 1mm minimum voxel size
            voxel_size = np.maximum(voxel_size, min_voxel_size)
            
            print(f"Object {object_id}: {len(points)} points, voxel_size={voxel_size}")
            
            # Create voxel indices for each point
            voxel_indices = np.floor((points - min_coords) / voxel_size).astype(int)
            
            # Ensure indices are within bounds (handle numerical precision issues)
            voxel_indices = np.clip(voxel_indices, 0, 9)
            
            # Create a dictionary to group points by voxel
            voxel_clusters = {}
            for i, point in enumerate(points):
                voxel_key = tuple(voxel_indices[i])
                if voxel_key not in voxel_clusters:
                    voxel_clusters[voxel_key] = []
                voxel_clusters[voxel_key].append(point)
            
            print(f"Created {len(voxel_clusters)} voxel clusters for object {object_id}")
            
            # Create convex hull for each voxel cluster
            convex_hulls = []
            for voxel_key, cluster_points in voxel_clusters.items():
                try:
                    cluster_points = np.array(cluster_points)
                    
                    if len(cluster_points) < 4:
                        print(f"Skipping voxel {voxel_key} with only {len(cluster_points)} points")
                        continue
                    
                    # Create point cloud for this voxel cluster
                    cluster_pcd = o3d.geometry.PointCloud()
                    cluster_pcd.points = o3d.utility.Vector3dVector(cluster_points)
                    
                    # Apply statistical outlier removal to this voxel cluster
                    if len(cluster_points) >= 10:  # Only apply outlier removal if we have enough points
                        cluster_pcd_filtered, outlier_indices = cluster_pcd.remove_statistical_outlier(
                            nb_neighbors=min(10, len(cluster_points) - 1),  # Use fewer neighbors for small clusters
                            std_ratio=2.0
                        )
                        
                        # Update cluster points with filtered points
                        filtered_cluster_points = np.asarray(cluster_pcd_filtered.points)
                        num_outliers = len(outlier_indices)
                        
                        if len(filtered_cluster_points) >= 4:
                            cluster_pcd = cluster_pcd_filtered
                            cluster_points = filtered_cluster_points
                            print(f"  Voxel {voxel_key}: Removed {num_outliers} outliers, {len(cluster_points)} points remaining")
                        else:
                            print(f"  Voxel {voxel_key}: Skipping outlier removal - would leave too few points ({len(filtered_cluster_points)})")
                    
                    # Compute convex hull
                    hull_mesh, _ = cluster_pcd.compute_convex_hull()
                    hull_mesh.compute_vertex_normals()
                    
                    # Validate hull
                    if len(np.asarray(hull_mesh.vertices)) >= 4 and len(np.asarray(hull_mesh.triangles)) >= 4:
                        convex_hulls.append(hull_mesh)
                        print(f"  Voxel {voxel_key}: {len(cluster_points)} points → convex hull with {len(np.asarray(hull_mesh.vertices))} vertices")
                    else:
                        print(f"  Skipping degenerate hull for voxel {voxel_key}")
                        
                except Exception as e:
                    print(f"Error processing voxel {voxel_key} for object {object_id}: {e}")
            
            if not convex_hulls:
                print(f"No valid convex hulls created for object {object_id}, using entire point cloud")
                # Fallback: use entire point cloud as single convex hull
                pcd = o3d.geometry.PointCloud()
                pcd.points = o3d.utility.Vector3dVector(points)
                hull_mesh, _ = pcd.compute_convex_hull()
                hull_mesh.compute_vertex_normals()
                return [hull_mesh]
            
            return convex_hulls
            
        except Exception as e:
            print(f"Error in voxelized clustering for object {object_id}: {e}")
            # Final fallback: single convex hull
            try:
                pcd = o3d.geometry.PointCloud()
                pcd.points = o3d.utility.Vector3dVector(points)
                hull_mesh, _ = pcd.compute_convex_hull()
                hull_mesh.compute_vertex_normals()
                return [hull_mesh]
            except:
                return []

    def _set_joint_positions_from_pickle(self):
        """Set robot joint positions from pickle file joint states data"""
        pickle_path = "manipulation_results.pkl"
        try:
            with open(pickle_path, "rb") as f:
                data = pickle.load(f)
            
            if "joint_states" in data and data["joint_states"]:
                joint_states = data["joint_states"]
                joint_names = joint_states.get("name", [])
                joint_positions = joint_states.get("position", [])
                
                if joint_names and joint_positions:
                    print(f"Found {len(joint_names)} joints from pickle file")
                    print(f"Joint names: {joint_names}")
                    print(f"Joint positions: {joint_positions}")
                    
                    # Create a mapping from joint name to position
                    joint_name_to_position = dict(zip(joint_names, joint_positions))
                    
                    # Set positions for known joints
                    positions = self.plant.GetPositions(self.plant_context)
                    joints_set = 0
                    
                    for joint_name in self.kinematic_chain_joints:
                        try:
                            joint = self.plant.GetJointByName(joint_name)
                            if joint.num_positions() > 0 and joint_name in joint_name_to_position:
                                start_index = joint.position_start()
                                joint_position = joint_name_to_position[joint_name]
                                
                                for i in range(joint.num_positions()):
                                    if start_index + i < len(positions):
                                        positions[start_index + i] = joint_position
                                        joints_set += 1
                                        
                                print(f"Set joint '{joint_name}' to position {joint_position}")
                            elif joint_name not in joint_name_to_position:
                                print(f"Joint '{joint_name}' not found in joint states data")
                        except RuntimeError:
                            print(f"Warning: Joint '{joint_name}' not found in URDF.")
                    
                    self.plant.SetPositions(self.plant_context, positions)
                    print(f"Successfully set {joints_set} joint positions from pickle file")
                else:
                    print("No joint names or positions found in joint states")
                    self._set_initial_configuration()
            else:
                print("No joint states found in pickle file")
                self._set_initial_configuration()
                
        except FileNotFoundError:
            print(f"Pickle file {pickle_path} not found, using default configuration")
            self._set_initial_configuration()
        except Exception as e:
            print(f"Error reading joint states from pickle file: {e}")
            self._set_initial_configuration()

    def _set_initial_configuration(self):
        """Set the robot to a reasonable initial joint configuration"""
        # Set all joints to zero initially
        if self.joint_indices:
            q = np.zeros(len(self.joint_indices))
            
            # You can customize these values for a better initial pose
            # For example, if you know good default joint angles:
            if len(q) >= 6:  # Assuming at least 6 DOF arm
                q[1] = 0.0    # joint1
                q[2] = 0.0   # joint2 
                q[3] = 0.0    # joint3
                q[4] = 0.0    # joint4
                q[5] = 0.0    # joint5
                q[6] = 0.0    # joint6
            
            # Set the joint positions in the plant context
            positions = self.plant.GetPositions(self.plant_context)
            for i, joint_idx in enumerate(self.joint_indices):
                if joint_idx < len(positions):
                    positions[joint_idx] = q[i]
            
            self.plant.SetPositions(self.plant_context, positions)
            print(f"Set initial joint configuration: {q}")
        else:
            print("Warning: No joint indices found, using default configuration")

    def _add_joint_sliders(self):
        """Add meshcat sliders for joint control"""
        try:
            # Get joint limits for each joint
            for i, joint_name in enumerate(self.kinematic_chain_joints):
                try:
                    joint = self.plant.GetJointByName(joint_name)
                    if joint.num_positions() > 0:
                        # Get joint limits
                        lower_limit = joint.position_lower_limits()[0] if joint.position_lower_limits().size > 0 else -3.14159
                        upper_limit = joint.position_upper_limits()[0] if joint.position_upper_limits().size > 0 else 3.14159
                        
                        # Get current position
                        current_pos = self.plant.GetPositions(self.plant_context)[joint.position_start()]
                        
                        # Add slider to meshcat
                        self.meshcat.AddSlider(
                            name=f"joint_{joint_name}",
                            min=lower_limit,
                            max=upper_limit,
                            step=0.01,
                            value=current_pos,
                            decrement_keycode="ArrowDown",
                            increment_keycode="ArrowUp"
                        )
                        print(f"Added slider for {joint_name}: [{lower_limit:.2f}, {upper_limit:.2f}], current: {current_pos:.2f}")
                except RuntimeError:
                    print(f"Warning: Could not add slider for joint '{joint_name}'")
            
            # Add XYZ position sliders for end effector
            if self.end_effector_frame is not None:
                # Get current end effector pose
                current_pose = self.plant.CalcRelativeTransform(
                    self.plant_context, 
                    self.plant.world_frame(), 
                    self.end_effector_frame
                )
                
                # Add XYZ sliders
                self.meshcat.AddSlider(
                    name="ee_x",
                    min=-2.0,
                    max=2.0,
                    step=0.01,
                    value=current_pose.translation()[0]
                )
                self.meshcat.AddSlider(
                    name="ee_y", 
                    min=-2.0,
                    max=2.0,
                    step=0.01,
                    value=current_pose.translation()[1]
                )
                self.meshcat.AddSlider(
                    name="ee_z",
                    min=-2.0,
                    max=2.0,
                    step=0.01,
                    value=current_pose.translation()[2]
                )
                
                # Add RPY sliders
                rpy = RollPitchYaw(current_pose.rotation())
                self.meshcat.AddSlider(
                    name="ee_roll",
                    min=-3.14159,
                    max=3.14159,
                    step=0.01,
                    value=rpy.roll_angle()
                )
                self.meshcat.AddSlider(
                    name="ee_pitch",
                    min=-3.14159,
                    max=3.14159,
                    step=0.01,
                    value=rpy.pitch_angle()
                )
                self.meshcat.AddSlider(
                    name="ee_yaw",
                    min=-3.14159,
                    max=3.14159,
                    step=0.01,
                    value=rpy.yaw_angle()
                )
                
                print("Added end effector XYZ and RPY sliders")
                    
            # Add collision detection display
            self.meshcat.SetProperty("collision_status", "visible", True)
            
        except Exception as e:
            print(f"Error adding joint sliders: {e}")

    def _identify_initial_robot_collisions(self):
        """Identify initial robot-world collisions to ignore in future checks"""
        try:
            print("Identifying initial robot-world collisions to ignore...")
            
            # Get the scene graph context
            scene_graph_context = self.scene_graph.GetMyContextFromRoot(self.diagram_context)
            query_object = self.scene_graph.get_query_output_port().Eval(scene_graph_context)
            
            # Check for all collisions initially
            collision_pairs = query_object.ComputePointPairPenetration()
            
            robot_collision_geometries = set()
            
            for pair in collision_pairs:
                # Get geometry names
                geom_A = query_object.inspector().GetName(pair.id_A)
                geom_B = query_object.inspector().GetName(pair.id_B)
                
                # Check if this is a robot-world collision
                is_robot_world_collision = False
                robot_geom = None
                world_geom = None
                
                # Check if one is robot and other is world object
                if ("devkit_base_descr" in geom_A and ("all_objects" in geom_B or "misc_clusters" in geom_B)):
                    is_robot_world_collision = True
                    robot_geom = geom_A
                    world_geom = geom_B
                elif ("devkit_base_descr" in geom_B and ("all_objects" in geom_A or "misc_clusters" in geom_A)):
                    is_robot_world_collision = True
                    robot_geom = geom_B
                    world_geom = geom_A
                
                if is_robot_world_collision:
                    # Store the collision pair to ignore
                    collision_key = tuple(sorted([geom_A, geom_B]))
                    self.initial_robot_collisions.add(collision_key)
                    
                    # Also store the world geometry for potential removal
                    robot_collision_geometries.add(world_geom)
                    self.filtered_geometry_names.add(world_geom)
                    
                    print(f"  Will ignore: {robot_geom} <-> {world_geom}")
            
            print(f"Found {len(self.initial_robot_collisions)} initial robot-world collisions to ignore")
            print(f"Found {len(robot_collision_geometries)} world geometries colliding with robot")
            
            # Apply collision filtering to exclude these geometries from robot collision detection
            if robot_collision_geometries:
                self._apply_collision_filtering(robot_collision_geometries)
            
        except Exception as e:
            print(f"Error identifying initial robot collisions: {e}")

    def _apply_collision_filtering(self, robot_collision_geometries):
        """Apply collision filtering to exclude certain geometries from robot collision detection"""
        try:
            print("Applying collision filtering to remove robot-world collision geometries...")
            
            # Get all registered geometry IDs that match the collision geometries
            geometry_ids_to_filter = []
            
            if hasattr(self, 'registered_geometry_ids'):
                for geom_id, geom_name in self.registered_geometry_ids:
                    if geom_name in robot_collision_geometries:
                        geometry_ids_to_filter.append(geom_id)
                        print(f"  Filtering geometry: {geom_name}")
            
            if geometry_ids_to_filter:
                # Get all robot body geometries
                robot_bodies = []
                for model_instance in self.model_instances:
                    bodies = self.plant.GetBodiesForModelInstance(model_instance)
                    robot_bodies.extend(bodies)
                
                # Get geometry IDs for robot bodies
                robot_geometry_ids = []
                for body in robot_bodies:
                    body_geoms = self.plant.GetCollisionGeometriesForBody(body)
                    robot_geometry_ids.extend(body_geoms)
                
                # Create collision filter to exclude these geometries from robot collision detection
                if robot_geometry_ids and geometry_ids_to_filter:
                    filter_manager = self.scene_graph.collision_filter_manager()
                    decl = CollisionFilterDeclaration()
                    
                    # Exclude collisions between robot geometries and problematic world geometries
                    for robot_geom_id in robot_geometry_ids:
                        for world_geom_id in geometry_ids_to_filter:
                            decl.ExcludeBetween(robot_geom_id, world_geom_id)
                    
                    filter_manager.Apply(decl)
                    print(f"Applied collision filtering for {len(geometry_ids_to_filter)} world geometries")
                else:
                    print("No valid geometries found for collision filtering")
            else:
                print("No geometries to filter found")
                
        except Exception as e:
            print(f"Error applying collision filtering: {e}")

    def _create_ik_environment(self):
        """Create separate Drake environment for collision-aware IK without filtered geometries"""
        try:
            print("Creating separate IK environment without filtered geometries...")
            
            # Create new builder for IK
            ik_builder = DiagramBuilder()
            self.ik_plant, self.ik_scene_graph = AddMultibodyPlantSceneGraph(ik_builder, time_step=0.01)
            ik_parser = Parser(self.ik_plant)
            
            # Load the robot URDF
            ik_model_instances = ik_parser.AddModelsFromUrl(f"file://{self.urdf_path}")
            
            # Set up collision filtering for ignored links (same as main plant)
            if hasattr(self, 'links_to_ignore') and self.links_to_ignore:
                bodies = []
                for link_name in self.links_to_ignore:
                    try:
                        body = self.ik_plant.GetBodyByName(link_name)
                        if body is not None:
                            bodies.extend(self.ik_plant.GetBodiesWeldedTo(body))
                    except RuntimeError:
                        continue

                if bodies:
                    arm_geoms = self.ik_plant.CollectRegisteredGeometries(bodies)
                    decl = CollisionFilterDeclaration().ExcludeWithin(arm_geoms)
                    manager = self.ik_scene_graph.collision_filter_manager()
                    manager.Apply(decl)
            
            # Add only the non-filtered geometries to IK environment
            self._add_non_filtered_geometries_to_ik()
            
            # Finalize the IK plant
            self.ik_plant.Finalize()
            
            # Build the IK diagram
            self.ik_diagram = ik_builder.Build()
            self.ik_context = self.ik_diagram.CreateDefaultContext()
            self.ik_plant_context = self.ik_plant.GetMyContextFromRoot(self.ik_context)
            
            print("IK environment created successfully")
            
        except Exception as e:
            print(f"Error creating IK environment: {e}")

    def _add_non_filtered_geometries_to_ik(self):
        """Add non-filtered geometries to the IK environment"""
        try:
            # Re-process point cloud data but exclude filtered geometries
            pickle_path = "manipulation_results.pkl"
            with open(pickle_path, "rb") as f:
                data = pickle.load(f)
            results = data["results"]
            
            # Process all_objects for IK (excluding filtered ones)
            if "all_objects" in results:
                filtered_objects = []
                for i, obj in enumerate(results["all_objects"]):
                    # Check if any geometry from this object should be filtered
                    should_filter = False
                    for geom_name in self.filtered_geometry_names:
                        if f"all_objects/object_{i}/" in geom_name:
                            should_filter = True
                            break
                    
                    if not should_filter:
                        filtered_objects.append(obj)
                
                print(f"Adding {len(filtered_objects)} non-filtered objects to IK environment")
                self._process_and_add_object_class_to_ik("all_objects", filtered_objects)
            
            # Process misc_clusters for IK (excluding filtered ones)  
            if "misc_clusters" in results:
                filtered_clusters = []
                for i, cluster in enumerate(results["misc_clusters"]):
                    # Check if any geometry from this cluster should be filtered
                    should_filter = False
                    for geom_name in self.filtered_geometry_names:
                        if f"misc_clusters/object_{i}/" in geom_name:
                            should_filter = True
                            break
                    
                    if not should_filter:
                        filtered_clusters.append(cluster)
                
                print(f"Adding {len(filtered_clusters)} non-filtered clusters to IK environment")
                self._process_and_add_object_class_to_ik("misc_clusters", filtered_clusters)
                
        except Exception as e:
            print(f"Error adding geometries to IK environment: {e}")

    def _process_and_add_object_class_to_ik(self, object_key: str, objects: list):
        """Process and add object class to IK environment (similar to main environment)"""
        try:
            if not objects:
                return
                
            all_decomposed_meshes = []
            transform = self.get_transform("world", "camera_center_link")
            
            for i, obj in enumerate(objects):
                try:
                    if object_key == "misc_clusters":
                        points = np.asarray(obj["points"])
                    elif "point_cloud_numpy" in obj:
                        points = obj["point_cloud_numpy"]
                    elif "point_cloud" in obj and obj["point_cloud"]:
                        points = np.array(obj["point_cloud"]["points"])
                    else:
                        continue
                        
                    if len(points) < 10:
                        continue
                    
                    # Transform points (same as main environment)
                    points = np.column_stack((points[:, 0], points[:, 2], -points[:, 1]))
                    points = self.transform_point_cloud_with_open3d(points, transform)
                        
                    # Create convex hulls
                    clustered_hulls = self._create_voxelized_clustered_convex_hulls(points, i)
                    for hull in clustered_hulls:
                        all_decomposed_meshes.append((hull, i))
                        
                except Exception as e:
                    print(f"Warning: Failed to process object {i} for IK: {e}")
            
            if all_decomposed_meshes:
                self._register_convex_hulls_to_ik(all_decomposed_meshes, object_key)
                
        except Exception as e:
            print(f"Error processing object class for IK: {e}")

    def _register_convex_hulls_to_ik(self, meshes_with_ids, hull_type: str):
        """Register convex hulls to IK environment"""
        try:
            world = self.ik_plant.world_body()
            proximity = ProximityProperties()

            for i, (mesh, object_id) in enumerate(meshes_with_ids):
                try:
                    # Convert mesh (same as main environment)
                    vertices = np.asarray(mesh.vertices)
                    faces = np.asarray(mesh.triangles)
                    
                    if len(vertices) == 0 or len(faces) == 0:
                        continue
                        
                    tmesh = trimesh.Trimesh(vertices=vertices, faces=faces)
                    tmesh_obj_blob = tmesh.export(file_type="obj")
                    mem_file = MemoryFile(
                        contents=tmesh_obj_blob,
                        extension=".obj",
                        filename_hint=f"ik_convex_hull_{i}.obj"
                    )
                    in_memory_mesh = InMemoryMesh()
                    in_memory_mesh.mesh_file = mem_file
                    drake_mesh = Mesh(in_memory_mesh, scale=1.0)

                    pos = np.array([0.0, 0.0, 0.0])
                    rpy = RollPitchYaw(0.0, 0.0, 0.0)
                    X_WG = DrakeRigidTransform(RotationMatrix(rpy), pos)

                    # Register only collision geometry (not visual for IK)
                    self.ik_plant.RegisterCollisionGeometry(
                        body=world,
                        X_BG=X_WG,
                        shape=drake_mesh,
                        name=f"{hull_type}/object_{object_id}/ik_collision_hull_{i}",
                        properties=proximity,
                    )
                        
                except Exception as e:
                    print(f"Warning: Failed to register IK mesh {i}: {e}")
                    
        except Exception as e:
            print(f"Error registering convex hulls to IK: {e}")

    def _solve_collision_aware_ik(self, target_pose: RigidTransform) -> Optional[np.ndarray]:
        """Solve collision-aware IK for target end effector pose"""
        try:
            if self.ik_plant is None or self.end_effector_frame is None:
                return None
                
            # Create IK solver
            ik = InverseKinematics(self.ik_plant, self.ik_plant_context)
            
            # Add pose constraint for end effector
            ik.AddPositionConstraint(
                frameB=self.ik_plant.GetFrameByName("link6"),
                p_BQ=np.array([0.0, 0.0, 0.0]),
                frameA=self.ik_plant.world_frame(),
                p_AQ_lower=target_pose.translation(),
                p_AQ_upper=target_pose.translation()
            )
            
            # Add orientation constraint
            ik.AddOrientationConstraint(
                frameAbar=self.ik_plant.world_frame(),
                R_AbarA=target_pose.rotation(),
                frameBbar=self.ik_plant.GetFrameByName("link6"),
                R_BbarB=RotationMatrix.Identity(),
                theta_bound=0.01
            )
            
            # Add collision avoidance constraint
            ik.AddMinimumDistanceLowerBoundConstraint(
                bound=0.01,  # Minimum distance of 1cm
                influence_distance_offset=0.1  # Start constraint when within 10cm
            )
            
            # Add joint limits
            for joint_name in self.kinematic_chain_joints:
                try:
                    joint = self.ik_plant.GetJointByName(joint_name)
                    if joint.num_positions() > 0:
                        lower_limits = joint.position_lower_limits()
                        upper_limits = joint.position_upper_limits()
                        if lower_limits.size > 0 and upper_limits.size > 0:
                            ik.prog().AddBoundingBoxConstraint(
                                lower_limits,
                                upper_limits,
                                ik.q()[joint.position_start():joint.position_start() + joint.num_positions()]
                            )
                except RuntimeError:
                    continue
            
            # Set initial guess (current configuration)
            current_positions = self.plant.GetPositions(self.plant_context)
            ik.prog().SetInitialGuess(ik.q(), current_positions)
            
            # Solve IK
            result = Solve(ik.prog())
            
            if result.is_success():
                solution = result.GetSolution(ik.q())
                print(f"IK solved successfully")
                return solution
            else:
                print(f"IK failed: {result.get_solver_details()}")
                return None
                
        except Exception as e:
            print(f"Error in collision-aware IK: {e}")
            return None

    def _update_visualization(self):
        """Force update the visualization"""
        try:
            # Get the visualizer's context from the diagram context
            visualizer_context = self.visualizer.GetMyContextFromRoot(self.diagram_context)
            self.visualizer.ForcedPublish(visualizer_context)
            print("Visualization updated successfully")
        except Exception as e:
            print(f"Error updating visualization: {e}")
            
    def _check_collisions(self):
        """Check for collisions and return collision information"""
        try:
            # Get the scene graph context
            scene_graph_context = self.scene_graph.GetMyContextFromRoot(self.diagram_context)
            
            # Create collision checker
            query_object = self.scene_graph.get_query_output_port().Eval(scene_graph_context)
            
            # Check for collisions
            collision_pairs = query_object.ComputePointPairPenetration()
            
            colliding_links = set()
            collision_info = []
            
            for pair in collision_pairs:
                # Only consider collisions with depth greater than threshold
                if pair.depth > self.collision_depth_threshold:
                    # Get geometry names
                    geom_A = query_object.inspector().GetName(pair.id_A)
                    geom_B = query_object.inspector().GetName(pair.id_B)
                    
                    # Check if this collision should be ignored (initial robot-world collision)
                    collision_key = tuple(sorted([geom_A, geom_B]))
                    if collision_key in self.initial_robot_collisions:
                        continue  # Skip this collision
                    
                    # Get frame names (link names)
                    frame_A = query_object.inspector().GetFrameId(pair.id_A)
                    frame_B = query_object.inspector().GetFrameId(pair.id_B)
                    
                    try:
                        frame_A_name = query_object.inspector().GetName(frame_A)
                        frame_B_name = query_object.inspector().GetName(frame_B)
                        
                        colliding_links.add(frame_A_name)
                        colliding_links.add(frame_B_name)
                        
                        # Extract object information from geometry names
                        object_info_A = self._extract_object_info(geom_A)
                        object_info_B = self._extract_object_info(geom_B)
                        
                        collision_info.append({
                            'frame_A': frame_A_name,
                            'frame_B': frame_B_name,
                            'geometry_A': geom_A,
                            'geometry_B': geom_B,
                            'object_info_A': object_info_A,
                            'object_info_B': object_info_B,
                            'depth': pair.depth
                        })
                    except:
                        # If we can't get frame names, use geometry names
                        colliding_links.add(geom_A)
                        colliding_links.add(geom_B)
                        
                        # Extract object information from geometry names
                        object_info_A = self._extract_object_info(geom_A)
                        object_info_B = self._extract_object_info(geom_B)
                        
                        collision_info.append({
                            'frame_A': geom_A,
                            'frame_B': geom_B,
                            'geometry_A': geom_A,
                            'geometry_B': geom_B,
                            'object_info_A': object_info_A,
                            'object_info_B': object_info_B,
                            'depth': pair.depth
                        })
            
            return len(collision_info) > 0, list(colliding_links), collision_info
            
        except Exception as e:
            print(f"Error checking collisions: {e}")
            return False, [], []
            
    def _extract_object_info(self, geometry_name: str) -> str:
        """Extract object information from geometry name"""
        try:
            # Parse geometry names like "detected_objects/object_1/collision_hull_2"
            if "detected_objects" in geometry_name:
                parts = geometry_name.split("/")
                if len(parts) >= 3:
                    object_part = parts[1]  # e.g., "object_1"
                    hull_part = parts[2]    # e.g., "collision_hull_2"
                    return f"{object_part} ({hull_part})"
                else:
                    return f"detected_object ({geometry_name})"
            elif "misc_clusters" in geometry_name:
                parts = geometry_name.split("/")
                if len(parts) >= 3:
                    object_part = parts[1]  # e.g., "object_0"
                    hull_part = parts[2]    # e.g., "collision_hull_1"
                    return f"misc_{object_part} ({hull_part})"
                else:
                    return f"misc_cluster ({geometry_name})"
            elif geometry_name == "world":
                return "world"
            else:
                # Robot links or other geometry
                return geometry_name
        except Exception as e:
            return geometry_name
            
    def _update_joint_from_sliders(self):
        """Update joint positions based on slider values"""
        try:
            positions = self.plant.GetPositions(self.plant_context)
            updated = False
            ik_updated = False
            
            # Check if end effector XYZ/RPY sliders have changed
            if self.end_effector_frame is not None:
                try:
                    # Get target pose from sliders
                    target_x = self.meshcat.GetSliderValue("ee_x")
                    target_y = self.meshcat.GetSliderValue("ee_y")
                    target_z = self.meshcat.GetSliderValue("ee_z")
                    target_roll = self.meshcat.GetSliderValue("ee_roll")
                    target_pitch = self.meshcat.GetSliderValue("ee_pitch")
                    target_yaw = self.meshcat.GetSliderValue("ee_yaw")
                    
                    # Get current end effector pose
                    current_pose = self.plant.CalcRelativeTransform(
                        self.plant_context, 
                        self.plant.world_frame(), 
                        self.end_effector_frame
                    )
                    
                    # Check if target pose has changed significantly
                    current_pos = current_pose.translation()
                    current_rpy = RollPitchYaw(current_pose.rotation())
                    
                    pos_diff = np.linalg.norm([target_x - current_pos[0], target_y - current_pos[1], target_z - current_pos[2]])
                    rpy_diff = abs(target_roll - current_rpy.roll_angle()) + abs(target_pitch - current_rpy.pitch_angle()) + abs(target_yaw - current_rpy.yaw_angle())
                    
                    if pos_diff > 0.01 or rpy_diff > 0.01:  # 1cm or 0.01 rad threshold
                        # Create target pose
                        target_translation = np.array([target_x, target_y, target_z])
                        target_rotation = RotationMatrix(RollPitchYaw(target_roll, target_pitch, target_yaw))
                        target_pose = RigidTransform(target_rotation, target_translation)
                        
                        # Solve collision-aware IK
                        print(f"Solving IK for target pose: {target_translation}, RPY: [{target_roll:.3f}, {target_pitch:.3f}, {target_yaw:.3f}]")
                        ik_solution = self._solve_collision_aware_ik(target_pose)
                        
                        if ik_solution is not None:
                            # Update IK plant context with solution
                            self.ik_plant.SetPositions(self.ik_plant_context, ik_solution)
                            
                            # Update main plant with IK solution
                            positions = ik_solution
                            ik_updated = True
                            print("Applied IK solution")
                        else:
                            print("IK failed - keeping current configuration")
                            
                except Exception as e:
                    print(f"Error in IK update: {e}")
            
            # If IK wasn't used, check individual joint sliders
            if not ik_updated:
                for joint_name in self.kinematic_chain_joints:
                    try:
                        joint = self.plant.GetJointByName(joint_name)
                        if joint.num_positions() > 0:
                            # Get slider value
                            slider_value = self.meshcat.GetSliderValue(f"joint_{joint_name}")
                            
                            # Update position
                            start_index = joint.position_start()
                            if abs(positions[start_index] - slider_value) > 0.001:  # Only update if changed
                                positions[start_index] = slider_value
                                updated = True
                                
                    except RuntimeError:
                        continue
                        
            if updated or ik_updated:
                self.plant.SetPositions(self.plant_context, positions)
                self._update_visualization()
                
        except Exception as e:
            print(f"Error updating joints from sliders: {e}")
            
    def run_interactive_loop(self):
        """Run the interactive loop with continuous collision checking"""
        print("Starting interactive joint control...")
        print("Use the sliders in the meshcat window to control joints")
        print(f"Collision detection threshold: {self.collision_depth_threshold:.3f}m")
        print("Press Ctrl+C to exit")
        
        try:
            while True:
                # Update joints from sliders
                self._update_joint_from_sliders()
                
                # Check for collisions
                in_collision, colliding_links, collision_info = self._check_collisions()
                
                if in_collision:
                    print(f"COLLISION DETECTED!")
                    print(f"Colliding links: {colliding_links}")
                    for info in collision_info:
                        # Create a more informative collision message
                        frame_A = info['frame_A']
                        frame_B = info['frame_B']
                        obj_A = info['object_info_A']
                        obj_B = info['object_info_B']
                        
                        # Show which object is being collided with
                        if obj_A != frame_A and obj_B != frame_B:
                            print(f"  {frame_A} [{obj_A}] <-> {frame_B} [{obj_B}]: depth = {info['depth']:.4f}")
                        elif obj_A != frame_A:
                            print(f"  {frame_A} <-> {frame_B} [{obj_A}]: depth = {info['depth']:.4f}")
                        elif obj_B != frame_B:
                            print(f"  {frame_A} [{obj_B}] <-> {frame_B}: depth = {info['depth']:.4f}")
                        else:
                            print(f"  {frame_A} <-> {frame_B}: depth = {info['depth']:.4f}")
                    
                    # Update meshcat with collision status
                    self.meshcat.SetProperty("collision_status", "color", [1.0, 0.0, 0.0, 1.0])  # Red
                    collision_text = f"COLLISION: {', '.join(colliding_links)}"
                else:
                    # Update meshcat with no collision status
                    self.meshcat.SetProperty("collision_status", "color", [0.0, 1.0, 0.0, 1.0])  # Green
                    collision_text = "No collisions"
                
                # Add text to meshcat (if supported)
                try:
                    self.meshcat.SetProperty("collision_text", "text", collision_text)
                except:
                    pass  # Text might not be supported in all meshcat versions
                
                # Small delay to prevent excessive computation
                time.sleep(0.1)
                
        except KeyboardInterrupt:
            print("\nExiting interactive loop...")
        except Exception as e:
            print(f"Error in interactive loop: {e}")
            import traceback
            traceback.print_exc()

    def set_joint_positions(self, joint_positions):
        """Set specific joint positions and update visualization"""
        if len(joint_positions) != len(self.joint_indices):
            raise ValueError(f"Expected {len(self.joint_indices)} joint positions, got {len(joint_positions)}")
        
        positions = self.plant.GetPositions(self.plant_context)
        for i, joint_idx in enumerate(self.joint_indices):
            if joint_idx < len(positions):
                positions[joint_idx] = joint_positions[i]
        
        self.plant.SetPositions(self.plant_context, positions)
        self._update_visualization()
        print(f"Updated joint positions: {joint_positions}")

    def register_convex_hulls_as_collision(self, meshes_with_ids, hull_type: str, detected_objects=None):
        """Register convex hulls as collision and visual geometry"""
        if not meshes_with_ids:
            print("No meshes to register")
            return
            
        world = self.plant.world_body()
        proximity = ProximityProperties()
        
        # Store geometry IDs for potential collision filtering
        self.registered_geometry_ids = getattr(self, 'registered_geometry_ids', [])

        for i, (mesh, object_id) in enumerate(meshes_with_ids):
            try:
                # Convert Open3D → numpy arrays → trimesh.Trimesh
                vertices = np.asarray(mesh.vertices)
                faces = np.asarray(mesh.triangles)
                
                if len(vertices) == 0 or len(faces) == 0:
                    print(f"Warning: Mesh {i} is empty, skipping")
                    continue
                    
                tmesh = trimesh.Trimesh(vertices=vertices, faces=faces)

                # Export to OBJ in memory
                tmesh_obj_blob = tmesh.export(file_type="obj")
                mem_file = MemoryFile(
                    contents=tmesh_obj_blob,
                    extension=".obj",
                    filename_hint=f"convex_hull_{i}.obj"
                )
                in_memory_mesh = InMemoryMesh()
                in_memory_mesh.mesh_file = mem_file
                drake_mesh = Mesh(in_memory_mesh, scale=1.0)

                pos = np.array([0.0, 0.0, 0.0])
                rpy = RollPitchYaw(0.0, 0.0, 0.0)
                X_WG = DrakeRigidTransform(RotationMatrix(rpy), pos)

                # Register collision and visual geometry with object-based naming
                collision_id = self.plant.RegisterCollisionGeometry(
                    body=world,
                    X_BG=X_WG,
                    shape=drake_mesh,
                    name=f"{hull_type}/object_{object_id}/collision_hull_{i}",
                    properties=proximity,
                )
                
                # Store geometry ID for potential collision filtering
                self.registered_geometry_ids.append((collision_id, f"{hull_type}/object_{object_id}/collision_hull_{i}"))
                
                # Generate a unique color for each object (all hulls from same object have same color)
                hull_color = self.get_seeded_random_rgba(object_id)
                visual_id = self.plant.RegisterVisualGeometry(
                    body=world,
                    X_BG=X_WG,
                    shape=drake_mesh,
                    name=f"{hull_type}/object_{object_id}/visual_hull_{i}",
                    diffuse_color=hull_color,
                )

                # print(f"Registered convex hull {i} with {len(vertices)} vertices and {len(faces)} faces")
                
            except Exception as e:
                print(f"Warning: Failed to register mesh {i}: {e}")
    
    def get_seeded_random_rgba(self, id: int):
        np.random.seed(id)
        random_color = np.random.rand(4)
        random_color[3] = 0.9
        return random_color
    
    @contextmanager
    def safe_lcm_instance(self):
        """Context manager for safely managing LCM instance lifecycle"""
        lcm_instance = tf_lcm_py.LCM()
        try:
            yield lcm_instance
        finally:
            pass

    def cleanup_resources(self):
        """Clean up resources before exiting"""
        # Only clean up once when exiting
        print("Cleaning up resources...")
        # Force cleanup of resources in reverse order (last created first)
        for resource in reversed(self._resources_to_cleanup):
            try:
                # For objects like TransformListener that might have a close or shutdown method
                if hasattr(resource, 'close'):
                    resource.close()
                elif hasattr(resource, 'shutdown'):
                    resource.shutdown()
                
                # Explicitly delete the resource
                del resource
            except Exception as e:
                print(f"Error during cleanup: {e}")
        
        # Clear the resources list
        self._resources_to_cleanup = []
    
    def get_transform(self, target_frame, source_frame):
        print("Getting transform from", source_frame, "to", target_frame)
        attempts = 0
        max_attempts = 20  # Reduced from 120 to avoid long blocking
        
        while attempts < max_attempts:
            try:
                # Process LCM messages with error handling
                if not self.tf_lcm_instance.handle_timeout(100):  # 100ms timeout
                    # If handle_timeout returns false, we might need to re-check if LCM is still good
                    if not self.tf_lcm_instance.good():
                        print("WARNING: LCM instance is no longer in a good state")
                
                # Get the most recent timestamp from the buffer instead of using current time
                try:
                    timestamp = self.buffer.get_most_recent_timestamp()
                    if attempts % 10 == 0:
                        print(f"Using timestamp from buffer: {timestamp}")
                except Exception as e:
                    # Fall back to current time if get_most_recent_timestamp fails
                    timestamp = datetime.now()
                    if not hasattr(timestamp, 'timestamp'):
                        timestamp.timestamp = lambda: time.mktime(timestamp.timetuple()) + timestamp.microsecond / 1e6
                    if attempts % 10 == 0:
                        print(f"Falling back to current time: {timestamp}")
                
                # Check if we can find the transform
                if self.buffer.can_transform(target_frame, source_frame, timestamp):
                    # print(f"Found transform between '{target_frame}' and '{source_frame}'!")
                    
                    # Look up the transform with the timestamp from the buffer
                    transform = self.buffer.lookup_transform(target_frame, source_frame, timestamp, 
                                            timeout=10.0, time_tolerance=0.1, lcm_module=lcm_msgs)
                    
                    return transform
                
                # Increment counter and report status every 10 attempts
                attempts += 1
                if attempts % 10 == 0:
                    print(f"Still waiting... (attempt {attempts}/{max_attempts})")
                    frames = self.buffer.get_all_frame_names()
                    if frames:
                        print(f"Frames received so far ({len(frames)} total):")
                        for frame in sorted(frames):
                            print(f"  {frame}")
                    else:
                        print("No frames received yet")
                
                # Brief pause
                time.sleep(0.5)
                
            except Exception as e:
                print(f"Error during transform lookup: {e}")
                attempts += 1
                time.sleep(1)  # Longer pause after an error
        
        print(f"\nERROR: No transform found after {max_attempts} attempts")
        return None
    
    def transform_point_cloud_with_open3d(self, points_np: np.ndarray, transform) -> np.ndarray:
        """
        Transforms a point cloud using Open3D given a transform.
        
        Args:
            points_np (np.ndarray): Nx3 array of 3D points.
            transform: Transform from tf_lcm_py.

        Returns:
            np.ndarray: Nx3 array of transformed 3D points.
        """
        if points_np.shape[1] != 3:
            print("Input point cloud must have shape Nx3.")
            return points_np

        # Convert transform to 4x4 numpy matrix
        tf_matrix = np.eye(4)
        
        # Extract rotation quaternion components
        qw = transform.transform.rotation.w
        qx = transform.transform.rotation.x
        qy = transform.transform.rotation.y
        qz = transform.transform.rotation.z
        
        # Convert quaternion to rotation matrix
        # Formula from: https://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation#Quaternion-derived_rotation_matrix
        tf_matrix[0, 0] = 1 - 2*qy*qy - 2*qz*qz
        tf_matrix[0, 1] = 2*qx*qy - 2*qz*qw
        tf_matrix[0, 2] = 2*qx*qz + 2*qy*qw
        
        tf_matrix[1, 0] = 2*qx*qy + 2*qz*qw
        tf_matrix[1, 1] = 1 - 2*qx*qx - 2*qz*qz
        tf_matrix[1, 2] = 2*qy*qz - 2*qx*qw
        
        tf_matrix[2, 0] = 2*qx*qz - 2*qy*qw
        tf_matrix[2, 1] = 2*qy*qz + 2*qx*qw
        tf_matrix[2, 2] = 1 - 2*qx*qx - 2*qy*qy
        
        # Set translation
        tf_matrix[0, 3] = transform.transform.translation.x
        tf_matrix[1, 3] = transform.transform.translation.y
        tf_matrix[2, 3] = transform.transform.translation.z

        # Create Open3D point cloud
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points_np)

        # Apply transformation
        pcd.transform(tf_matrix)

        # Return as NumPy array
        return np.asarray(pcd.points)


# Updated main function
if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description='Visualize manipulation results')
    parser.add_argument('--visualize-only', action='store_true', help='Only visualize results')
    args = parser.parse_args()
    
    if args.visualize_only:
        visualize_results()
        exit(0)

    try:
        # Then set up Drake environment
        kinematic_chain_joints = [
            "pillar_platform_joint",
            "pan_tilt_pan_joint",
            "pan_tilt_head_joint",
            "joint1", 
            "joint2",
            "joint3",
            "joint4",
            "joint5",
            "joint6",
            "joint7",
            "joint8",
        ]
        
        links_to_ignore = [
            "devkit_base_link",
            "pillar_platform", 
            "piper_angled_mount",
            "pan_tilt_base",
            "pan_tilt_head",
            "pan_tilt_pan",
            "base_link",
            "link1",
            "link2", 
            "link3",
            "link4",
            "link5",
            "link6",
        ]
        
        urdf_path = "./assets/devkit_base_descr.urdf"
        urdf_path = os.path.abspath(urdf_path)
        
        print(f"Attempting to load URDF from: {urdf_path}")
        
        env = DrakeKinematicsEnv(urdf_path, kinematic_chain_joints, links_to_ignore)
        # env.set_joint_positions([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        transform = env.get_transform("world", "camera_center_link")
        print(transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z)
        print(transform.transform.rotation.w, transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z)
        
        # Start the interactive loop with joint sliders and collision detection
        env.run_interactive_loop()
            
    except KeyboardInterrupt:
        print("\nExiting...")
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()