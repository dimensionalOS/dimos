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
import pickle
import sys

import matplotlib
import numpy as np

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

import atexit
from contextlib import contextmanager
from datetime import datetime
import time

import lcm_msgs
from pydrake.all import (
    AddMultibodyPlantSceneGraph,
    DiagramBuilder,
    JointIndex,
    MeshcatVisualizer,
    MeshcatVisualizerParams,
    Parser,
    RigidTransform,
    RollPitchYaw,
    RotationMatrix,
    StartMeshcat,
)
from pydrake.common import MemoryFile
from pydrake.geometry import (
    Box,
    CollisionFilterDeclaration,
    InMemoryMesh,
    Mesh,
    ProximityProperties,
)
from pydrake.math import RigidTransform as DrakeRigidTransform
import tf_lcm_py
import trimesh

from dimos.perception.pointcloud.utils import (
    visualize_clustered_point_clouds,
    visualize_pcd,
    visualize_voxel_grid,
)
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


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
    if data.get("points"):
        pcd.points = o3d.utility.Vector3dVector(np.array(data["points"]))
    if data.get("colors"):
        pcd.colors = o3d.utility.Vector3dVector(np.array(data["colors"]))
    return pcd


def deserialize_voxel_grid(data):
    """Reconstruct Open3D VoxelGrid from serialized data."""
    if data is None:
        return None

    # Create a point cloud to convert to voxel grid
    pcd = o3d.geometry.PointCloud()
    voxel_size = data["voxel_size"]
    origin = np.array(data["origin"])

    # Create points from voxel indices
    points = []
    colors = []
    for voxel in data["voxels"]:
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
        data["color_img"]
        data["depth_img"]
        data["intrinsics"]

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
        _fig, axes = plt.subplots(rows, cols, figsize=(6 * cols, 5 * rows))

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
    if results.get("misc_clusters"):
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
    def __init__(
        self,
        urdf_path: str,
        kinematic_chain_joints: list[str],
        links_to_ignore: list[str] | None = None,
    ):
        self._resources_to_cleanup = []

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
            self.builder, self.scene_graph, self.meshcat, params=MeshcatVisualizerParams()
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
                    print(
                        f"Added joint '{joint_name}' at indices {start_index} to {start_index + joint.num_positions() - 1}"
                    )
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

        # Set robot to a reasonable initial configuration
        self._set_initial_configuration()

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

        full_detected_pcd = o3d.geometry.PointCloud()
        for obj in results["detected_objects"]:
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(obj["point_cloud_numpy"])
            full_detected_pcd += pcd

        self.process_and_add_object_class("all_objects", results)
        self.process_and_add_object_class("misc_clusters", results)
        misc_clusters = results["misc_clusters"]
        print(type(misc_clusters[0]["points"]))
        print(np.asarray(misc_clusters[0]["points"]).shape)

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
                        elif (
                            "point_cloud" in detected_objects[i]
                            and detected_objects[i]["point_cloud"]
                        ):
                            # Handle serialized point cloud
                            points = np.array(detected_objects[i]["point_cloud"]["points"])
                        else:
                            print(f"Warning: No point cloud data found for object {i}")
                            continue

                        if len(points) < 10:  # Need more points for mesh reconstruction
                            print(
                                f"Warning: Object {i} has too few points ({len(points)}) for mesh reconstruction"
                            )
                            continue

                        # Swap y-z axes since this is a common problem
                        points = np.column_stack((points[:, 0], points[:, 2], -points[:, 1]))
                        # Transform points to world frame
                        points = self.transform_point_cloud_with_open3d(points, transform)

                        # Use fast DBSCAN clustering + convex hulls approach
                        clustered_hulls = self._create_clustered_convex_hulls(points, i)
                        all_decomposed_meshes.extend(clustered_hulls)

                        print(
                            f"Created {len(clustered_hulls)} clustered convex hulls for object {i}"
                        )

                    except Exception as e:
                        print(f"Warning: Failed to process object {i}: {e}")

                if all_decomposed_meshes:
                    self.register_convex_hulls_as_collision(all_decomposed_meshes, object_key)
                    print(f"Registered {len(all_decomposed_meshes)} total clustered convex hulls")
                else:
                    print("Warning: No valid clustered convex hulls created from detected objects")
            else:
                print("No detected objects found")

    def _create_clustered_convex_hulls(
        self, points: np.ndarray, object_id: int
    ) -> list[o3d.geometry.TriangleMesh]:
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
                np.std(distances)

                print(
                    f"Object {object_id}: {len(points)} points, avg_nn_dist={avg_nn_distance:.4f}"
                )

                for i in range(20):
                    try:
                        eps = avg_nn_distance * (2.0 + (i * 0.1))
                        min_samples = 20
                        labels = np.array(pcd.cluster_dbscan(eps=eps, min_points=min_samples))
                        unique_labels = np.unique(labels)
                        clusters = unique_labels[unique_labels >= 0]  # Remove noise label (-1)

                        noise_points = np.sum(labels == -1)
                        clustered_points = len(points) - noise_points

                        print(
                            f"  Try {i + 1}: eps={eps:.4f}, min_samples={min_samples} → {len(clusters)} clusters, {clustered_points}/{len(points)} points clustered"
                        )

                        # Accept if we found clusters and most points are clustered
                        if (
                            len(clusters) > 0 and clustered_points >= len(points) * 0.95
                        ):  # At least 30% of points clustered
                            print(f"  ✓ Accepted parameter set {i + 1}")
                            break

                    except Exception as e:
                        print(
                            f"  Try {i + 1}: Failed with eps={eps:.4f}, min_samples={min_samples}: {e}"
                        )
                        continue

            if len(clusters) == 0 or labels is None:
                print(
                    f"No clusters found for object {object_id} after all attempts, using entire point cloud"
                )
                # Fallback: use entire point cloud as single convex hull
                hull_mesh, _ = pcd.compute_convex_hull()
                hull_mesh.compute_vertex_normals()
                return [hull_mesh]

            print(
                f"Found {len(clusters)} clusters for object {object_id} (eps={eps:.3f}, min_samples={min_samples})"
            )

            # Create convex hull for each cluster
            convex_hulls = []
            for cluster_id in clusters:
                try:
                    # Get points for this cluster
                    cluster_mask = labels == cluster_id
                    cluster_points = points[cluster_mask]

                    if len(cluster_points) < 4:
                        print(
                            f"Skipping cluster {cluster_id} with only {len(cluster_points)} points"
                        )
                        continue

                    # Create point cloud for this cluster
                    cluster_pcd = o3d.geometry.PointCloud()
                    cluster_pcd.points = o3d.utility.Vector3dVector(cluster_points)

                    # Compute convex hull
                    hull_mesh, _ = cluster_pcd.compute_convex_hull()
                    hull_mesh.compute_vertex_normals()

                    # Validate hull
                    if (
                        len(np.asarray(hull_mesh.vertices)) >= 4
                        and len(np.asarray(hull_mesh.triangles)) >= 4
                    ):
                        convex_hulls.append(hull_mesh)
                        print(
                            f"  Cluster {cluster_id}: {len(cluster_points)} points → convex hull with {len(np.asarray(hull_mesh.vertices))} vertices"
                        )
                    else:
                        print(f"  Skipping degenerate hull for cluster {cluster_id}")

                except Exception as e:
                    print(f"Error processing cluster {cluster_id} for object {object_id}: {e}")

            if not convex_hulls:
                print(
                    f"No valid convex hulls created for object {object_id}, using entire point cloud"
                )
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

    def _set_initial_configuration(self):
        """Set the robot to a reasonable initial joint configuration"""
        # Set all joints to zero initially
        if self.joint_indices:
            q = np.zeros(len(self.joint_indices))

            # You can customize these values for a better initial pose
            # For example, if you know good default joint angles:
            if len(q) >= 6:  # Assuming at least 6 DOF arm
                q[1] = 0.0  # joint1
                q[2] = 0.0  # joint2
                q[3] = 0.0  # joint3
                q[4] = 0.0  # joint4
                q[5] = 0.0  # joint5
                q[6] = 0.0  # joint6

            # Set the joint positions in the plant context
            positions = self.plant.GetPositions(self.plant_context)
            for i, joint_idx in enumerate(self.joint_indices):
                if joint_idx < len(positions):
                    positions[joint_idx] = q[i]

            self.plant.SetPositions(self.plant_context, positions)
            print(f"Set initial joint configuration: {q}")
        else:
            print("Warning: No joint indices found, using default configuration")

    def _update_visualization(self):
        """Force update the visualization"""
        try:
            # Get the visualizer's context from the diagram context
            visualizer_context = self.visualizer.GetMyContextFromRoot(self.diagram_context)
            self.visualizer.ForcedPublish(visualizer_context)
            print("Visualization updated successfully")
        except Exception as e:
            print(f"Error updating visualization: {e}")

    def set_joint_positions(self, joint_positions):
        """Set specific joint positions and update visualization"""
        if len(joint_positions) != len(self.joint_indices):
            raise ValueError(
                f"Expected {len(self.joint_indices)} joint positions, got {len(joint_positions)}"
            )

        positions = self.plant.GetPositions(self.plant_context)
        for i, joint_idx in enumerate(self.joint_indices):
            if joint_idx < len(positions):
                positions[joint_idx] = joint_positions[i]

        self.plant.SetPositions(self.plant_context, positions)
        self._update_visualization()
        print(f"Updated joint positions: {joint_positions}")

    def register_convex_hulls_as_collision(
        self, meshes: list[o3d.geometry.TriangleMesh], hull_type: str
    ):
        """Register convex hulls as collision and visual geometry"""
        if not meshes:
            print("No meshes to register")
            return

        world = self.plant.world_body()
        proximity = ProximityProperties()

        for i, mesh in enumerate(meshes):
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
                    contents=tmesh_obj_blob, extension=".obj", filename_hint=f"convex_hull_{i}.obj"
                )
                in_memory_mesh = InMemoryMesh()
                in_memory_mesh.mesh_file = mem_file
                drake_mesh = Mesh(in_memory_mesh, scale=1.0)

                pos = np.array([0.0, 0.0, 0.0])
                rpy = RollPitchYaw(0.0, 0.0, 0.0)
                X_WG = DrakeRigidTransform(RotationMatrix(rpy), pos)

                # Register collision and visual geometry
                self.plant.RegisterCollisionGeometry(
                    body=world,
                    X_BG=X_WG,
                    shape=drake_mesh,
                    name=f"convex_hull_collision_{i}_{hull_type}",
                    properties=proximity,
                )
                self.plant.RegisterVisualGeometry(
                    body=world,
                    X_BG=X_WG,
                    shape=drake_mesh,
                    name=f"convex_hull_visual_{i}_{hull_type}",
                    diffuse_color=np.array([0.7, 0.5, 0.3, 0.8]),  # Orange-ish color
                )

                print(
                    f"Registered convex hull {i} with {len(vertices)} vertices and {len(faces)} faces"
                )

            except Exception as e:
                print(f"Warning: Failed to register mesh {i}: {e}")

        # Add a simple table for reference
        try:
            table_shape = Box(1.0, 1.0, 0.1)  # Thinner table
            table_pose = RigidTransform(p=[0.5, 0.0, -0.05])  # In front of robot
            self.plant.RegisterCollisionGeometry(
                world, table_pose, table_shape, "table_collision", proximity
            )
            self.plant.RegisterVisualGeometry(
                world, table_pose, table_shape, "table_visual", [0.8, 0.6, 0.4, 1.0]
            )
            print("Added reference table")
        except Exception as e:
            print(f"Warning: Failed to add table: {e}")

    def get_seeded_random_rgba(self, id: int):
        np.random.seed(id)
        return np.random.rand(4)

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
                if hasattr(resource, "close"):
                    resource.close()
                elif hasattr(resource, "shutdown"):
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
                except Exception:
                    # Fall back to current time if get_most_recent_timestamp fails
                    timestamp = datetime.now()
                    if not hasattr(timestamp, "timestamp"):
                        timestamp.timestamp = (
                            lambda: time.mktime(timestamp.timetuple()) + timestamp.microsecond / 1e6
                        )
                    if attempts % 10 == 0:
                        print(f"Falling back to current time: {timestamp}")

                # Check if we can find the transform
                if self.buffer.can_transform(target_frame, source_frame, timestamp):
                    # print(f"Found transform between '{target_frame}' and '{source_frame}'!")

                    # Look up the transform with the timestamp from the buffer
                    transform = self.buffer.lookup_transform(
                        target_frame,
                        source_frame,
                        timestamp,
                        timeout=10.0,
                        time_tolerance=0.1,
                        lcm_module=lcm_msgs,
                    )

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
        tf_matrix[0, 0] = 1 - 2 * qy * qy - 2 * qz * qz
        tf_matrix[0, 1] = 2 * qx * qy - 2 * qz * qw
        tf_matrix[0, 2] = 2 * qx * qz + 2 * qy * qw

        tf_matrix[1, 0] = 2 * qx * qy + 2 * qz * qw
        tf_matrix[1, 1] = 1 - 2 * qx * qx - 2 * qz * qz
        tf_matrix[1, 2] = 2 * qy * qz - 2 * qx * qw

        tf_matrix[2, 0] = 2 * qx * qz - 2 * qy * qw
        tf_matrix[2, 1] = 2 * qy * qz + 2 * qx * qw
        tf_matrix[2, 2] = 1 - 2 * qx * qx - 2 * qy * qy

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

    parser = argparse.ArgumentParser(description="Visualize manipulation results")
    parser.add_argument("--visualize-only", action="store_true", help="Only visualize results")
    args = parser.parse_args()

    if args.visualize_only:
        visualize_results()
        exit(0)

    try:
        # Then set up Drake environment
        kinematic_chain_joints = [
            "pillar_platform_joint",
            "joint1",
            "joint2",
            "joint3",
            "joint4",
            "joint5",
            "joint6",
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
        env.set_joint_positions([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        transform = env.get_transform("world", "camera_center_link")
        print(
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z,
        )
        print(
            transform.transform.rotation.w,
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z,
        )

        # Keep the visualization alive
        print("\nVisualization is running. Press Ctrl+C to exit.")
        while True:
            time.sleep(1)

    except KeyboardInterrupt:
        print("\nExiting...")
    except Exception as e:
        print(f"Error: {e}")
        import traceback

        traceback.print_exc()
