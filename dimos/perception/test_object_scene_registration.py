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


class _FakePromptDetector:
    """Stands in for the lazily-created promptable detector: records
    set_prompts calls and returns a canned 2D detection on process_image."""

    def __init__(self, detections_2d: ImageDetections2D) -> None:  # type: ignore[type-arg]
        self._detections_2d = detections_2d
        self.set_prompts_calls: list[list[str] | None] = []

    def set_prompts(self, text: list[str] | None = None, bboxes=None) -> None:  # type: ignore[no-untyped-def]
        self.set_prompts_calls.append(text)

    def process_image(self, image: Image) -> ImageDetections2D:  # type: ignore[type-arg]
        return self._detections_2d

    def stop(self) -> None:
        pass


def _fake_lidar_detection_2d() -> ImageDetections2D:  # type: ignore[type-arg]
    """A synthetic 2D "cup" detection over a dense enough lidar blob to
    survive from_2d_to_list_lidar's default sparse-cloud filters (same setup
    as test_from_2d_to_list_lidar_places_blob_at_world_center, but with more
    points -- that test skips the filters explicitly with filters=[])."""
    image = Image.from_numpy(
        np.zeros((480, 640, 3), dtype=np.uint8), format=ImageFormat.BGR, frame_id="camera_optical", ts=5.0
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
    return ImageDetections2D(image, [det])


def test_detect_skill_uses_separate_detector_and_localizes_from_cached_frame() -> None:
    """detect() must:
    1. Use its own promptable detector (_prompt_detector), never touching
       the always-on background detector (_detector) -- that's what lets
       automatic prompt-free cataloging keep running undisturbed.
    2. Localize the match against the *cached* latest lidar frame (the
       background pipeline's job to keep fresh, not detect()'s).
    3. Ingest the result into the shared ObjectDB so it shows up in
       list_observed_items/catalog_scene too.
    """
    detections_2d = _fake_lidar_detection_2d()
    camera_info = CameraInfo.from_intrinsics(
        fx=500.0, fy=500.0, cx=320.0, cy=240.0, width=640, height=480, frame_id="camera_optical"
    )
    rng = np.random.default_rng(0)
    # Dense enough (300 pts) to survive the default lidar sparse-cloud filters.
    blob = np.array([0.0, 0.0, 2.0]) + rng.uniform(-0.03, 0.03, size=(300, 3))
    world_pc = PointCloud2.from_numpy(blob.astype(float), frame_id="world", timestamp=5.0)

    module = ObjectSceneRegistrationModule(localization="lidar", min_detections_for_permanent=1)
    background_detector = object()  # sentinel: must stay untouched by detect()
    module._detector = background_detector  # type: ignore[assignment]
    module._prompt_detector = _FakePromptDetector(detections_2d)  # type: ignore[assignment]
    module._camera_info = camera_info
    module._latest_color_image = detections_2d.image
    module._latest_lidar = world_pc
    module._latest_world_to_optical = Transform.identity()
    try:
        result = module.detect(prompts=["cup"])

        assert module._prompt_detector.set_prompts_calls == [["cup"]]  # type: ignore[union-attr]
        assert module._detector is background_detector  # background detector untouched
        assert "cup" in result
        assert [o.name for o in module._object_db.get_all_objects()] == ["cup"]
    finally:
        module._detector = None  # sentinel has no .stop(); detach before teardown
        module.stop()


def test_detect_skill_no_prompts() -> None:
    module = ObjectSceneRegistrationModule(localization="lidar")
    try:
        assert module.detect(prompts=[]) == "No prompts provided."
    finally:
        module.stop()


def test_detect_skill_no_frame_yet() -> None:
    module = ObjectSceneRegistrationModule(localization="lidar")
    try:
        # No frame *and* an empty catalog: nothing to report from either path.
        assert (
            module.detect(prompts=["cup"])
            == "No matching items in the catalog, and no camera frame available yet."
        )
    finally:
        module.stop()


def test_detect_skill_searches_accumulated_catalog_without_a_frame() -> None:
    """A directed search must surface items already in the catalog even when
    they aren't in the current view (and even with no current frame at all) --
    "search for a chair" should find the chair list_observed_items reports,
    not answer "not in my current view."
    """
    module = ObjectSceneRegistrationModule(localization="lidar", min_detections_for_permanent=1)
    try:
        # Stub the persistent catalog so a "chair" is already observed, with no
        # live camera frame available (_latest_color_image stays None).
        module._match_located_objects = lambda query: (  # type: ignore[method-assign]
            [
                {
                    "name": "chair",
                    "object_id": "obj-chair-1",
                    "position": {"x": 1.0, "y": 2.0, "z": 0.0},
                    "size": {"x": 0.5, "y": 0.5, "z": 1.0},
                    "confidence": 0.9,
                    "detections": 7,
                    "last_seen": "just now",
                }
            ]
            if "chair" in query.lower()
            else []
        )
        result = module.detect(prompts=["chair", "backpack"])

        assert "already catalogued" in result
        assert "chair" in result
        assert "obj-chair-1" in result
        assert "current view" not in result
    finally:
        module.stop()


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


# --- Frame-quality gate + sharpest-appearance retention (per docs/capabilities/
# memory/plot.md: dark/blurry frames are useless and embed close to everything) ---


def _textured_image(seed: int) -> Image:
    """A high-frequency (sharp) frame: random noise -> large Laplacian variance."""
    rng = np.random.default_rng(seed)
    data = rng.integers(0, 256, size=(64, 64, 3), dtype=np.uint8)
    return Image.from_numpy(data, format=ImageFormat.BGR, frame_id="camera_optical", ts=1.0)


def _flat_image(value: int = 128) -> Image:
    """A featureless (blurry) frame of constant color -> ~zero Laplacian variance.

    Its brightness is ``value/255``, so ``value`` also controls dark-vs-bright.
    """
    data = np.full((64, 64, 3), value, dtype=np.uint8)
    return Image.from_numpy(data, format=ImageFormat.BGR, frame_id="camera_optical", ts=1.0)


def test_frame_quality_gate_rejects_dark_and_blurry() -> None:
    module = ObjectSceneRegistrationModule(
        localization="lidar", min_frame_brightness=0.06, min_frame_sharpness=0.12
    )
    try:
        assert module._frame_quality_ok(_textured_image(2)) is True  # bright + sharp -> keep
        assert module._frame_quality_ok(_flat_image(2)) is False  # near-black -> dark reject
        assert module._frame_quality_ok(_flat_image(200)) is False  # bright but flat -> blur reject
        assert module._frames_seen == 3
        assert module._frames_rejected == 2
    finally:
        module.stop()


def test_frame_quality_gate_disabled_by_default() -> None:
    module = ObjectSceneRegistrationModule(localization="lidar")
    try:
        # Both thresholds default to 0.0: even an all-black frame passes, and
        # the gate short-circuits before it even starts counting frames.
        assert module._frame_quality_ok(_flat_image(0)) is True
        assert module._frames_seen == 0
    finally:
        module.stop()


def test_update_object_keeps_sharpest_appearance_crop() -> None:
    """A blurry re-sighting must not overwrite a sharp stored crop: the VLM
    labels from obj.image, so we keep the clearest frame (and its matching
    bbox), while the 3D geometry still tracks the latest detection."""
    sharp = _make_object("cup", Vector3(1.0, 2.0, 0.5))
    sharp.image = _textured_image(0)
    sharp.bbox = (10.0, 10.0, 20.0, 20.0)

    blurry = _make_object("cup", Vector3(1.1, 2.1, 0.5))  # object moved slightly in frame
    blurry.image = _flat_image()
    blurry.bbox = (30.0, 30.0, 40.0, 40.0)
    assert sharp.image.sharpness > blurry.image.sharpness

    sharp.update_object(blurry)

    # Geometry follows the latest (blurry) detection...
    assert sharp.center.x == pytest.approx(1.1)
    assert sharp.detections_count == 2
    # ...but the labeling appearance (image + its bbox) stays the sharp one.
    assert sharp.bbox == (10.0, 10.0, 20.0, 20.0)
    assert sharp.image.sharpness > _flat_image().sharpness


def test_update_object_adopts_sharper_new_appearance() -> None:
    """The reverse: a sharp new frame replaces a blurry stored crop, bbox and all."""
    blurry = _make_object("cup", Vector3(1.0, 2.0, 0.5))
    blurry.image = _flat_image()
    blurry.bbox = (30.0, 30.0, 40.0, 40.0)

    sharp = _make_object("cup", Vector3(1.0, 2.0, 0.5))
    sharp.image = _textured_image(1)
    sharp.bbox = (10.0, 10.0, 20.0, 20.0)

    blurry.update_object(sharp)

    assert blurry.bbox == (10.0, 10.0, 20.0, 20.0)  # adopted the sharp frame's bbox
    assert blurry.image.sharpness > _flat_image().sharpness
