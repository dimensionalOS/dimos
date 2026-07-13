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

import numpy as np
import pytest

from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.perception.detection.type.detection2d.bbox import Detection2DBBox
from dimos.perception.detection.type.detection2d.imageDetections2D import ImageDetections2D
from dimos.perception.detection.type.detection3d.object import Object
from dimos.perception.object_scene_registration import ObjectSceneRegistrationModule
from dimos.types.robot_location import RobotLocation


def _make_object(
    name: str,
    center: Vector3,
    confidence: float = 0.8,
    track_id: int = 1,
    detections_count: int = 1,
) -> Object:
    """Build a minimal but valid world-frame Object for encoder/DB tests."""
    image = Image.from_numpy(
        np.zeros((4, 4, 3), dtype=np.uint8),
        format=ImageFormat.BGR,
        frame_id="camera_optical",
        ts=1.0,
    )
    pointcloud = PointCloud2.from_numpy(
        np.array([[center.x, center.y, center.z]], dtype=float), frame_id="world", timestamp=1.0
    )
    return Object(
        bbox=(0.0, 0.0, 1.0, 1.0),
        track_id=track_id,
        class_id=0,
        confidence=confidence,
        name=name,
        ts=1.0,
        image=image,
        frame_id="world",
        center=center,
        size=Vector3(0.1, 0.2, 0.3),
        pose=PoseStamped(
            ts=1.0, frame_id="world", position=center, orientation=Quaternion(0.0, 0.0, 0.0, 1.0)
        ),
        pointcloud=pointcloud,
        detections_count=detections_count,
    )


def test_object_db_runs_on_stream_time_not_wall_clock() -> None:
    """Replayed recordings (ts far in the past) must dedup/promote like live data.

    ObjectDB TTL bookkeeping previously compared observation timestamps against
    time.time(), so every replayed pending object looked stale and was pruned
    before it could accumulate detections.
    """
    from dimos.perception.detection.objectDB import ObjectDB

    db = ObjectDB(distance_threshold=0.5, min_detections_for_permanent=3, pending_ttl_s=5.0)
    base_ts = 1_757_960_660.0  # a recorded moment, ~months before any wall clock "now"

    for i in range(3):
        obj = _make_object("chair", Vector3(1.0, 2.0, 0.5), track_id=4)
        obj.ts = base_ts + i * 0.5  # frames 0.5s apart in stream time
        obj.pose.ts = obj.ts
        db.add_objects([obj])

    assert len(db) == 1, "same-spot detections should dedup and promote, not be pruned as stale"
    permanent = db.get_objects()[0]
    assert permanent.detections_count == 3
    assert db.now == base_ts + 1.0  # stream time, not wall clock


def test_object_db_stream_time_still_prunes_stale_pending() -> None:
    from dimos.perception.detection.objectDB import ObjectDB

    db = ObjectDB(distance_threshold=0.5, min_detections_for_permanent=10, pending_ttl_s=5.0)
    base_ts = 1_757_960_660.0

    stale = _make_object("ghost", Vector3(0.0, 0.0, 0.0), track_id=1)
    stale.ts = base_ts
    db.add_objects([stale])

    fresh = _make_object("chair", Vector3(5.0, 5.0, 0.5), track_id=2)
    fresh.ts = base_ts + 60.0  # a minute later in stream time
    db.add_objects([fresh])

    names = [o.name for o in db.get_all_objects()]
    assert "ghost" not in names, "pending object unseen for > ttl (stream time) should be pruned"
    assert "chair" in names


def test_to_detection3d_msg_survives_flat_pointcloud() -> None:
    """A coplanar (e.g. all-floor) cluster makes Qhull's OBB throw; the ROS
    message conversion must fall back to an axis-aligned box, not crash the
    publish path."""
    rng = np.random.default_rng(1)
    flat = np.column_stack(
        [rng.uniform(0.0, 0.4, 40), rng.uniform(2.4, 2.7, 40), np.full(40, -0.075)]
    )
    obj = _make_object("rug", Vector3(0.2, 2.55, -0.075))
    obj.pointcloud = PointCloud2.from_numpy(flat, frame_id="world", timestamp=1.0)

    msg = obj.to_detection3d_msg()

    assert msg.bbox.size.z == pytest.approx(0.0, abs=1e-6)
    assert msg.bbox.center.position.x == pytest.approx(0.2, abs=0.05)
    assert msg.bbox.center.orientation.w == 1.0  # identity fallback


def test_locate_encode_includes_world_position_and_confidence() -> None:
    obj = _make_object("cup", Vector3(1.0, 2.0, 3.0), confidence=0.873)

    encoded = obj.locate_encode()

    assert encoded["name"] == "cup"
    assert encoded["position"] == {"x": 1.0, "y": 2.0, "z": 3.0}
    assert encoded["frame"] == "world"
    assert encoded["confidence"] == 0.87  # rounded to 2 dp
    assert encoded["size"] == {"x": 0.1, "y": 0.2, "z": 0.3}
    assert encoded["detections"] == 1


def test_from_2d_to_list_lidar_places_blob_at_world_center() -> None:
    # Camera at the world origin, looking down +Z (optical convention), so world
    # coordinates equal optical coordinates: a blob 2m in front projects to the
    # principal point.
    camera_info = CameraInfo.from_intrinsics(
        fx=500.0, fy=500.0, cx=320.0, cy=240.0, width=640, height=480, frame_id="camera_optical"
    )
    world_to_optical = Transform.identity()

    rng = np.random.default_rng(0)
    blob = np.array([0.0, 0.0, 2.0]) + rng.uniform(-0.03, 0.03, size=(40, 3))
    world_pc = PointCloud2.from_numpy(blob.astype(float), frame_id="world", timestamp=5.0)

    image = Image.from_numpy(
        np.zeros((480, 640, 3), dtype=np.uint8),
        format=ImageFormat.BGR,
        frame_id="camera_optical",
        ts=5.0,
    )
    det = Detection2DBBox(
        bbox=(300.0, 220.0, 340.0, 260.0),
        track_id=7,
        class_id=0,
        confidence=0.9,
        name="cup",
        ts=5.0,
        image=image,
    )
    detections_2d: ImageDetections2D = ImageDetections2D(image, [det])

    # filters=[] skips the default sparse-cloud filters (which need dozens of
    # neighbors) so the synthetic blob survives.
    objects = Object.from_2d_to_list_lidar(
        detections_2d=detections_2d,
        world_pointcloud=world_pc,
        camera_info=camera_info,
        world_to_optical_transform=world_to_optical,
        filters=[],
    )

    assert len(objects) == 1
    obj = objects[0]
    assert obj.name == "cup"
    assert obj.confidence == pytest.approx(0.9)
    assert obj.frame_id == "world"
    assert obj.center.x == pytest.approx(0.0, abs=0.1)
    assert obj.center.y == pytest.approx(0.0, abs=0.1)
    assert obj.center.z == pytest.approx(2.0, abs=0.1)


def test_from_2d_to_list_lidar_ignores_detection_with_no_points() -> None:
    camera_info = CameraInfo.from_intrinsics(
        fx=500.0, fy=500.0, cx=320.0, cy=240.0, width=640, height=480, frame_id="camera_optical"
    )
    world_pc = PointCloud2.from_numpy(
        (np.array([0.0, 0.0, 2.0]) + np.zeros((10, 3))).astype(float),
        frame_id="world",
        timestamp=1.0,
    )
    image = Image.from_numpy(
        np.zeros((480, 640, 3), dtype=np.uint8),
        format=ImageFormat.BGR,
        frame_id="camera_optical",
        ts=1.0,
    )
    # bbox in a corner where no points project
    det = Detection2DBBox(
        bbox=(0.0, 0.0, 10.0, 10.0),
        track_id=1,
        class_id=0,
        confidence=0.9,
        name="ghost",
        ts=1.0,
        image=image,
    )
    objects = Object.from_2d_to_list_lidar(
        detections_2d=ImageDetections2D(image, [det]),
        world_pointcloud=world_pc,
        camera_info=camera_info,
        world_to_optical_transform=Transform.identity(),
        filters=[],
    )
    assert objects == []


def test_list_observed_items_reports_positions() -> None:
    module = ObjectSceneRegistrationModule(localization="lidar", min_detections_for_permanent=1)
    try:
        # ObjectDB only promotes pending -> permanent, so promote explicitly.
        inserted = module._object_db.add_objects(
            [_make_object("cup", Vector3(1.0, 2.0, 3.0), confidence=0.8)]
        )
        module._object_db.promote(inserted[0].object_id)

        result = module.list_observed_items()

        assert "Observing 1 item(s):" in result
        assert "cup" in result
        assert "pos=(1.00, 2.00, 3.00)m" in result
        assert "conf=0.80" in result
    finally:
        module.stop()


def test_list_observed_items_empty() -> None:
    module = ObjectSceneRegistrationModule(localization="lidar")
    try:
        assert module.list_observed_items() == "No objects observed yet."
    finally:
        module.stop()


def test_get_located_objects_permanent_only_by_default() -> None:
    # min_detections_for_permanent=3 so a single detection stays pending.
    module = ObjectSceneRegistrationModule(localization="lidar", min_detections_for_permanent=3)
    try:
        module._object_db.add_objects([_make_object("mouse", Vector3(0.5, 0.5, 0.5))])

        assert module.get_located_objects() == []  # pending excluded
        pending = module.get_located_objects(include_pending=True)
        assert len(pending) == 1
        assert pending[0]["name"] == "mouse"
        assert pending[0]["position"] == {"x": 0.5, "y": 0.5, "z": 0.5}
    finally:
        module.stop()


class _FakeSceneMemory:
    """Minimal SceneMemorySpec impl for catalog-fusion and locate tests."""

    def __init__(
        self,
        places: list[RobotLocation] | None = None,
        tagged: RobotLocation | None = None,
        text_results: list[dict] | None = None,  # type: ignore[type-arg]
    ) -> None:
        self._places = places or []
        self._tagged = tagged
        self._text_results = text_results or []

    def get_tagged_locations(self) -> list[RobotLocation]:
        return list(self._places)

    def query_tagged_location(self, query: str) -> RobotLocation | None:
        return self._tagged

    def query_by_text(self, text: str, limit: int = 5) -> list[dict]:  # type: ignore[type-arg]
        return list(self._text_results)


def _promote_object(module: ObjectSceneRegistrationModule, obj: Object) -> None:
    inserted = module._object_db.add_objects([obj])
    module._object_db.promote(inserted[0].object_id)


def test_catalog_scene_without_spatial_memory_is_objects_only() -> None:
    module = ObjectSceneRegistrationModule(localization="lidar", min_detections_for_permanent=1)
    try:
        _promote_object(module, _make_object("cup", Vector3(1.0, 2.0, 3.0), confidence=0.8))

        result = module.catalog_scene()

        assert "Known places" not in result
        assert "Observing 1 item(s):" in result
        assert "cup" in result
        assert "near '" not in result  # no place context without spatial memory
    finally:
        module.stop()


def test_catalog_scene_fuses_tagged_places_and_nearest_annotation() -> None:
    module = ObjectSceneRegistrationModule(localization="lidar", min_detections_for_permanent=1)
    module._spatial_memory = _FakeSceneMemory(
        [
            RobotLocation(name="kitchen", position=(1.5, 2.0, 0.0), rotation=(0.0, 0.0, 0.0)),
            RobotLocation(name="garage", position=(20.0, 20.0, 0.0), rotation=(0.0, 0.0, 0.0)),
        ]
    )
    try:
        _promote_object(module, _make_object("cup", Vector3(1.0, 2.0, 3.0), confidence=0.8))

        result = module.catalog_scene()

        # Places section lists every tagged place.
        assert "Known places (2):" in result
        assert "kitchen: (1.50, 2.00)m" in result
        assert "garage: (20.00, 20.00)m" in result
        # The cup (at 1,2) is 0.5 m from the kitchen -> tagged with it, not the garage.
        assert "near 'kitchen' (0.5m)" in result
        assert "near 'garage'" not in result
    finally:
        module.stop()


def test_catalog_scene_place_beyond_radius_not_attached() -> None:
    # place_radius=1.0 so the only place (5 m away) is listed but not attached.
    module = ObjectSceneRegistrationModule(
        localization="lidar", min_detections_for_permanent=1, place_radius=1.0
    )
    module._spatial_memory = _FakeSceneMemory(
        [RobotLocation(name="far room", position=(6.0, 2.0, 0.0), rotation=(0.0, 0.0, 0.0))]
    )
    try:
        _promote_object(module, _make_object("cup", Vector3(1.0, 2.0, 3.0), confidence=0.8))

        result = module.catalog_scene()

        assert "Known places (1):" in result  # still enumerated
        assert "near '" not in result  # but too far to attach to the cup
    finally:
        module.stop()


def test_nearest_place_gating() -> None:
    module = ObjectSceneRegistrationModule(localization="lidar", place_radius=2.0)
    try:
        places = [
            RobotLocation(name="near", position=(0.0, 1.0, 0.0), rotation=(0.0, 0.0, 0.0)),
            RobotLocation(name="far", position=(50.0, 50.0, 0.0), rotation=(0.0, 0.0, 0.0)),
        ]
        assert module._nearest_place(0.0, 0.0, places) == ("near", 1.0)
        # Beyond place_radius -> no match even though a place exists.
        assert module._nearest_place(10.0, 10.0, places) is None
        assert module._nearest_place(0.0, 0.0, []) is None
    finally:
        module.stop()


def test_locate_matches_catalogued_object() -> None:
    module = ObjectSceneRegistrationModule(localization="lidar", min_detections_for_permanent=1)
    module._spatial_memory = _FakeSceneMemory(
        places=[RobotLocation(name="kitchen", position=(1.5, 2.0, 0.0), rotation=(0.0, 0.0, 0.0))]
    )
    try:
        _promote_object(module, _make_object("backpack", Vector3(1.2, 2.1, 0.4), confidence=0.8))

        # Substring-either-direction match: "the backpack" resolves to "backpack".
        result = module.locate("the backpack")

        assert "Located 'the backpack':" in result
        assert "backpack" in result
        assert "pos=(1.20, 2.10, 0.40)m" in result
        assert "near 'kitchen'" in result
    finally:
        module.stop()


def test_locate_falls_back_to_tagged_place() -> None:
    module = ObjectSceneRegistrationModule(localization="lidar", min_detections_for_permanent=1)
    module._spatial_memory = _FakeSceneMemory(
        tagged=RobotLocation(name="kitchen", position=(3.0, 4.0, 0.0), rotation=(0.0, 0.0, 0.0))
    )
    try:
        # No catalogued object matches, but a tagged place does.
        result = module.locate("kitchen")

        assert "tagged place 'kitchen'" in result
        assert "(3.00, 4.00, 0.00)m" in result
    finally:
        module.stop()


def test_locate_semantic_recall_when_confident() -> None:
    module = ObjectSceneRegistrationModule(
        localization="lidar", semantic_locate_threshold=0.2
    )
    module._spatial_memory = _FakeSceneMemory(
        text_results=[{"metadata": {"pos_x": 5.0, "pos_y": 6.0}, "distance": 0.5}]  # sim=0.5
    )
    try:
        result = module.locate("the reading nook")

        assert "best matches" in result
        assert "(5.00, 6.00)m" in result
        assert "similarity 0.50" in result
    finally:
        module.stop()


def test_locate_semantic_recall_below_threshold_reports_not_found() -> None:
    module = ObjectSceneRegistrationModule(
        localization="lidar", semantic_locate_threshold=0.6
    )
    module._spatial_memory = _FakeSceneMemory(
        text_results=[{"metadata": {"pos_x": 5.0, "pos_y": 6.0}, "distance": 0.7}]  # sim=0.3 < 0.6
    )
    try:
        result = module.locate("unicorn")

        assert "Could not locate 'unicorn'" in result
        assert "remember visiting" in result
    finally:
        module.stop()


def test_locate_without_spatial_memory() -> None:
    module = ObjectSceneRegistrationModule(localization="lidar")
    try:
        result = module.locate("anything")

        assert "Could not locate 'anything'" in result
        assert "no spatial memory is available" in result
    finally:
        module.stop()


def test_spatial_vector_db_get_tagged_locations_roundtrip() -> None:
    """SpatialVectorDB can enumerate every tagged place (feeds catalog_scene)."""
    pytest.importorskip("chromadb")
    from dimos.perception.spatial_vector_db import SpatialVectorDB

    db = SpatialVectorDB(collection_name="test_tagged_roundtrip")
    assert db.get_tagged_locations() == []

    db.tag_location(
        RobotLocation(name="kitchen", position=(1.0, 2.0, 0.0), rotation=(0.0, 0.0, 0.5))
    )
    db.tag_location(
        RobotLocation(name="charging dock", position=(0.0, 0.0, 0.0), rotation=(0.0, 0.0, 0.0))
    )

    got = {loc.name: loc for loc in db.get_tagged_locations()}
    assert set(got) == {"kitchen", "charging dock"}
    assert got["kitchen"].position[0] == 1.0
    assert got["kitchen"].position[1] == 2.0
