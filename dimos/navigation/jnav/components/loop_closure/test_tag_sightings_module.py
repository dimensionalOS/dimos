# Copyright 2026 Dimensional Inc.
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

"""TagSightings publishes a sighting as a body-frame LocationConstraint and supersedes it under the same instance id."""

from __future__ import annotations

from unittest.mock import PropertyMock, patch

from dimos_lcm.vision_msgs import BoundingBox3D
import pytest

from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.std_msgs.Header import Header
from dimos.msgs.vision_msgs.Detection3D import Detection3D
from dimos.msgs.vision_msgs.Detection3DArray import Detection3DArray
from dimos.navigation.jnav.components.loop_closure.tag_sightings_module import TagSightings
from dimos.navigation.jnav.msgs.LocationConstraint import LocationConstraint
from dimos.navigation.jnav.utils.apriltags import SightingClustering
from dimos.protocol.tf.tf import MultiTBuffer


def detections(ts, x):
    detection = Detection3D()
    detection.id = "13"
    detection.bbox = BoundingBox3D(
        center=Pose(position=Vector3(x, 0.0, 0.0), orientation=Quaternion(0.0, 0.0, 0.0, 1.0)),
        size=Vector3(0.1, 0.1, 0.0),
    )
    return Detection3DArray(header=Header(ts, "odom"), detections=[detection], detections_length=1)


def test_republished_sighting_keeps_its_instance_id():
    module = TagSightings(
        camera_frame="",
        clustering=SightingClustering(min_inliers=2, huber_delta_m=0.5, republish_shift_m=0.02),
    )
    published = []
    unsubscribe = module.landmark_factor.subscribe(published.append)
    tf = MultiTBuffer()
    tfbuffer = patch.object(TagSightings, "tfbuffer", new_callable=PropertyMock, return_value=tf)
    tfbuffer.start()
    try:
        for ts in (10.0, 10.5, 11.0, 11.5, 12.0):
            # the body sits 1 m along odom x
            tf.receive_transform(
                Transform(
                    translation=Vector3(1.0, 0.0, 0.0),
                    rotation=Quaternion(0.0, 0.0, 0.0, 1.0),
                    frame_id="odom",
                    child_frame_id="base_link",
                    ts=ts,
                )
            )
        module._on_detections(detections(10.0, 3.0))
        module._on_detections(detections(10.5, 3.0))
        for ts in (11.0, 11.5, 12.0):
            module._on_detections(detections(ts, 3.2))
        assert len(published) >= 2
        assert {constraint.constraint_instance_id for constraint in published} == {"apriltag/13#0"}
        first, newest = published[0], published[-1]
        assert first.to_id == "apriltag/13"
        assert first.frame_id == "base_link"
        assert first.pose.position.x == pytest.approx(2.0)
        assert newest.pose.position.x == pytest.approx(2.2)
        # rotation unknown (a point sighting), position 5 cm
        assert newest.covariance[0] == -1.0
        assert newest.covariance[21] == pytest.approx(0.0025)
        decoded = LocationConstraint.lcm_decode(newest.lcm_encode())
        assert decoded.constraint_instance_id == "apriltag/13#0"
    finally:
        tfbuffer.stop()
        if callable(unsubscribe):
            unsubscribe()
        module.stop()
