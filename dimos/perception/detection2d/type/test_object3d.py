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

from dimos.perception.detection2d.type.detection3d import ImageDetections3D
from dimos.protocol.service import lcmservice as lcm


def test_object(moment, detections3d, publish_lcm):
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
