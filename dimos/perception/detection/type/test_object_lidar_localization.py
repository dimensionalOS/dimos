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

"""Real-data test for Object.from_2d_to_list_lidar on a recorded Go2 moment.

Uses the shared detection conftest fixtures (real Go2 image + lidar + tf +
camera_info, with a real YOLO detector run on CPU). Marked self_hosted because
it downloads/decompresses the dataset and the detector weights.
"""

import pytest

from dimos.perception.detection.type.detection3d.object import Object

pytestmark = pytest.mark.self_hosted


def test_from_2d_to_list_lidar_locates_real_go2_objects(get_moment_2d) -> None:
    moment = get_moment_2d(seek=10.0)

    tf = moment["tf"]
    lidar = moment["lidar_frame"]
    detections_2d = moment["detections2d"]
    assert len(detections_2d) > 0, "expected real detections in the seek=10 frame"

    world_to_optical = tf.get("camera_optical", lidar.frame_id)
    assert world_to_optical is not None

    objects = Object.from_2d_to_list_lidar(
        detections_2d=detections_2d,
        world_pointcloud=lidar,
        camera_info=moment["camera_info"],
        world_to_optical_transform=world_to_optical,
    )

    # The lidar path should localize at least one detected object in world frame.
    assert len(objects) > 0

    print(f"\nLocated {len(objects)} object(s) from real Go2 lidar+camera:")
    for obj in objects:
        c = obj.center
        print(
            f"  - {obj.name}: pos=({c.x:.2f}, {c.y:.2f}, {c.z:.2f})m "
            f"conf={obj.confidence:.2f} size={obj.size.length():.2f}m frame={obj.frame_id}"
        )

    pts, _ = lidar.as_numpy()
    lo = pts.min(axis=0)
    hi = pts.max(axis=0)

    for obj in objects:
        assert obj.name
        assert obj.frame_id == lidar.frame_id
        assert obj.confidence > 0.0
        assert obj.size.length() > 0.0  # non-degenerate bounding box
        # Located center must fall within (a small margin of) the lidar cloud.
        assert lo[0] - 1.0 <= obj.center.x <= hi[0] + 1.0
        assert lo[1] - 1.0 <= obj.center.y <= hi[1] + 1.0
        assert lo[2] - 1.0 <= obj.center.z <= hi[2] + 1.0

    # locate_encode surfaces the world position + confidence to the agent.
    encoded = objects[0].locate_encode()
    assert set(encoded) >= {"name", "position", "confidence", "size", "frame"}
    assert set(encoded["position"]) == {"x", "y", "z"}
