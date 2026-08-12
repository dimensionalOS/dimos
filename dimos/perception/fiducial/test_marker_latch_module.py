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

from __future__ import annotations

from contextlib import contextmanager

from dimos_lcm.vision_msgs import BoundingBox3D

from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.std_msgs.Header import Header
from dimos.msgs.vision_msgs.Detection3D import Detection3D
from dimos.msgs.vision_msgs.Detection3DArray import Detection3DArray
from dimos.perception.fiducial.marker_latch_module import MarkerLatchModule


def _detections(marker_id, xyz, frame_id="pelvis", ts=1_000_000.0):
    det = Detection3D()
    det.header = Header(ts, frame_id)
    det.id = str(marker_id)
    det.bbox = BoundingBox3D(
        center=Pose(position=Vector3(*xyz), orientation=Quaternion(0.0, 0.0, 0.0, 1.0)),
        size=Vector3(0.15, 0.15, 0.0),
    )
    return Detection3DArray(header=Header(ts, frame_id), detections=[det], detections_length=1)


@contextmanager
def _latch(**kwargs):
    module = MarkerLatchModule(marker_ids=[0, 1, 2], frame_id="pelvis", **kwargs)
    try:
        yield module
    finally:
        module.stop()


def test_latches_after_enough_agreeing_sightings():
    with _latch(required_sightings=3) as m:
        for _ in range(2):
            m._on_detections(_detections(1, (1.0, 0.2, 0.5)))
        assert m.get_latched_pose() is None

        m._on_detections(_detections(1, (1.0, 0.2, 0.5)))
        pose = m.get_latched_pose()
        assert pose is not None
        assert m.get_latched_marker_id() == 1
        assert pose.frame_id == "pelvis"
        assert round(pose.position.x, 3) == 1.0


def test_holds_the_pose_after_the_tag_leaves_view():
    # The whole point: the robot walks up and the tag goes out of frame.
    with _latch(required_sightings=2) as m:
        for _ in range(2):
            m._on_detections(_detections(2, (2.0, 0.0, 0.4)))
        first = m.get_latched_pose()

        # A later, different sighting must not move the target.
        m._on_detections(_detections(0, (9.0, 9.0, 9.0)))
        held = m.get_latched_pose()
        assert m.get_latched_marker_id() == 2
        assert (held.position.x, held.position.y) == (first.position.x, first.position.y)


def test_jumping_sightings_restart_the_streak():
    with _latch(required_sightings=3, position_tolerance_m=0.05) as m:
        m._on_detections(_detections(0, (1.0, 0.0, 0.0)))
        m._on_detections(_detections(0, (1.0, 0.0, 0.0)))
        # A metre away — a misdetection, not the same pot.
        m._on_detections(_detections(0, (2.0, 0.0, 0.0)))
        assert m.get_latched_pose() is None


def test_ignores_markers_that_are_not_plants():
    with _latch(required_sightings=1) as m:
        m._on_detections(_detections(7, (1.0, 0.0, 0.0)))
        assert m.get_latched_pose() is None


def test_reset_allows_targeting_the_next_plant():
    with _latch(required_sightings=1) as m:
        m._on_detections(_detections(0, (1.0, 0.0, 0.0)))
        assert m.get_latched_marker_id() == 0

        m.reset()
        assert m.get_latched_pose() is None

        m._on_detections(_detections(2, (3.0, 1.0, 0.0)))
        assert m.get_latched_marker_id() == 2
