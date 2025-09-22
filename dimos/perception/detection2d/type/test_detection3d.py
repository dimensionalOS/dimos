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

import os
import pickle

from dimos.core.transport import LCMTransport
from dimos.perception.detection2d.type.detection3d import Detection3D
from dimos.protocol.service import lcmservice as lcm


def test_boundingbox():
    import os

    pkl_path = os.path.join(os.path.dirname(__file__), "detection3d.pkl")
    detection = pickle.load(open(pkl_path, "rb"))
    print(detection)

    # Test bounding box functions
    print("\n=== Testing Bounding Box Functions ===")
    try:
        # Get oriented bounding box
        obb = detection.get_oriented_bounding_box()
        print(f"✓ Oriented bounding box: {obb}")

        # Get bounding box dimensions
        dims = detection.get_bounding_box_dimensions()
        print(f"✓ Bounding box dimensions (W×H×D): {dims[0]:.3f} × {dims[1]:.3f} × {dims[2]:.3f} m")

        # Get axis-aligned bounding box for comparison
        aabb = detection.get_bounding_box()
        print(f"✓ Axis-aligned bounding box: {aabb}")

    except Exception as e:
        print(f"✗ Error getting bounding box: {e}")

    # Test Foxglove scene entity generation
    print("\n=== Testing Foxglove Scene Entity Generation ===")
    try:
        # First, print the point cloud boundaries
        import numpy as np

        # Access points directly from pointcloud
        print(f"\n✓ Point cloud info:")
        pc_points = detection.pointcloud.points()  # Call the method
        print(f"  - Number of points: {len(pc_points)}")
        print(f"  - Frame ID: {detection.pointcloud.frame_id}")

        # Extract xyz coordinates from points
        points = []
        for pt in pc_points:
            points.append([pt[0], pt[1], pt[2]])  # Assuming points are arrays/tuples
        points = np.array(points)

        if len(points) > 0:
            min_pt = np.min(points, axis=0)
            max_pt = np.max(points, axis=0)
            center = np.mean(points, axis=0)
            print(f"\n✓ Point cloud boundaries:")
            print(f"  - Min point: [{min_pt[0]:.3f}, {min_pt[1]:.3f}, {min_pt[2]:.3f}]")
            print(f"  - Max point: [{max_pt[0]:.3f}, {max_pt[1]:.3f}, {max_pt[2]:.3f}]")
            print(f"  - Center: [{center[0]:.3f}, {center[1]:.3f}, {center[2]:.3f}]")
            print(
                f"  - Extent: [{max_pt[0] - min_pt[0]:.3f}, {max_pt[1] - min_pt[1]:.3f}, {max_pt[2] - min_pt[2]:.3f}]"
            )

        # Test generating a Foxglove scene entity
        entity = detection.to_foxglove_scene_entity("test_entity_123")
        print(f"\n✓ Generated Foxglove scene entity:")
        print(f"  - ID: {entity.id}")
        print(f"  - Frame: {entity.frame_id}")
        print(f"  - Cubes: {entity.cubes_length}")
        print(f"  - Texts: {entity.texts_length}")

        if entity.cubes_length > 0:
            cube = entity.cubes[0]
            print(f"\n✓ Cube primitive:")
            print(
                f"  - Position: [{cube.pose.position.x:.3f}, {cube.pose.position.y:.3f}, {cube.pose.position.z:.3f}]"
            )
            print(f"  - Size: [{cube.size.x:.3f} × {cube.size.y:.3f} × {cube.size.z:.3f}] m")
            print(
                f"  - Color: RGBA({cube.color.r}, {cube.color.g}, {cube.color.b}, {cube.color.a})"
            )

        if entity.texts_length > 0:
            text = entity.texts[0]
            print(f"\n✓ Text label:")
            print(f"  - Text: {text.text}")
            print(
                f"  - Position: [{text.pose.position.x:.3f}, {text.pose.position.y:.3f}, {text.pose.position.z:.3f}]"
            )
            print(f"  - Font size: {text.font_size}")

        # Print detection pose/transform info
        print(f"\n✓ Detection pose:")
        print(
            f"  - Position: [{detection.pose.x:.3f}, {detection.pose.y:.3f}, {detection.pose.z:.3f}]"
        )
        print(f"  - Frame: {detection.pose.frame_id}")

    except Exception as e:
        print(f"✗ Error generating Foxglove entity: {e}")
        import traceback

        traceback.print_exc()


def test_publish_foxglove_native(moment, detections3d, publish_lcm):
    """Test publishing detection3d as Foxglove native 3D annotations using fixtures"""
    from dimos.perception.detection2d.type.detection3d import ImageDetections3D

    # Configure LCM
    lcm.autoconf()

    # Add detections to moment
    moment["detections"] = detections3d

    # Create ImageDetections3D and use the new method to generate SceneUpdate
    image_detections = ImageDetections3D(
        image=detections3d[0].image if detections3d else None, detections=detections3d
    )
    scene_update = image_detections.to_foxglove_scene_update()

    # Add scene_update to moment
    moment["scene_update"] = scene_update

    # Publish all data including scene_update
    publish_lcm(moment)

    print(f"\nPublishing Foxglove native 3D annotations for {len(detections3d)} detections:")
    for i, detection in enumerate(detections3d):
        entity = scene_update.entities[i]
        cube = entity.cubes[0]
        text = entity.texts[0]
        print(f"\nDetection {i + 1}:")
        print(f"  - Entity ID: {entity.id}")
        print(f"  - Class: {detection.name} ({detection.confidence:.0%})")
        print(
            f"  - Position: ({cube.pose.position.x:.3f}, {cube.pose.position.y:.3f}, {cube.pose.position.z:.3f})"
        )
        print(f"  - Size: ({cube.size.x:.3f} × {cube.size.y:.3f} × {cube.size.z:.3f}) m")
        print(f"  - Points: {len(detection.pointcloud.points())}")

    print(f"\nPublished channels:")
    print(f"  - /foxglove/scene_update (Foxglove native 3D annotations)")
    print(f"  - /detected/pointcloud/* (Individual point clouds)")
    print(f"  - /detected/image/* (Cropped detection images)")
    print(f"  - /image, /lidar, /odom, /camera_info (Sensor data)")
    print(f"\nView in Foxglove Studio!")
