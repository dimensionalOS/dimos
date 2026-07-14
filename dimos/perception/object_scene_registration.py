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

from typing import Any, Literal

import cv2
import numpy as np
from numpy.typing import NDArray
import open3d as o3d  # type: ignore[import-untyped]

from dimos.agents.annotation import skill
from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.stream import In, Out
from dimos.models.vl.create import create as create_vl_model
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.std_msgs.Header import Header
from dimos.msgs.vision_msgs.Detection2DArray import Detection2DArray
from dimos.msgs.vision_msgs.Detection3DArray import Detection3DArray
from dimos.perception.contextual_labeling import (
    DEFAULT_CONFIDENCE_THRESHOLD,
    contextual_label,
    is_ambiguous,
)
from dimos.perception.detection.detectors.yoloe import Yoloe2DDetector, YoloePromptMode
from dimos.perception.detection.objectDB import ObjectDB
from dimos.perception.detection.type.detection2d.imageDetections2D import ImageDetections2D
from dimos.perception.detection.type.detection3d.object import (
    Object,
    Object as DetObject,
    aggregate_pointclouds,
    to_detection3d_array,
)
from dimos.perception.spatial_memory_spec import SceneMemorySpec
from dimos.types.robot_location import RobotLocation
from dimos.types.timestamped import align_timestamped
from dimos.utils.logging_config import setup_logger
from dimos.utils.reactive import backpressure

logger = setup_logger()


class ObjectSceneRegistrationModule(Module):
    """Builds a live, world-located catalog of everything the robot has seen.

    As the robot moves through an environment, this module continuously:
    optionally skips camera frames too dark or blurry to be worth detecting on
    (the frame-quality gate, ``min_frame_brightness``/``min_frame_sharpness``;
    dark frames especially yield junk detections and embed close to
    everything), detects objects in each surviving frame (YOLO-E,
    open-vocabulary — no prompt needed), localizes each detection in the world
    frame (RGBD back-projection when ``localization="depth"``, or projection of
    a world-frame lidar cloud when ``localization="lidar"`` for robots without
    a depth camera, e.g. the Go2), and deduplicates/accumulates repeat
    sightings of the same physical object into a persistent `ObjectDB` (a
    "cup" seen 40 times is one database row, promoted to permanent once
    corroborated by enough detections; the sharpest of those sightings is kept
    as the crop a VLM later labels from).

    Once the robot has navigated a space, `catalog_scene()` (or the faster
    `list_observed_items()`) answers "what's here and where is it?" as a
    prompt-free inventory: name, object_id, world position, physical size,
    detection confidence, sighting count, and — once refined with a
    vision-language model (`identify_object`, `refine_observed_labels`,
    or automatically via `catalog_scene`) — a plain-language description.
    `detect(...)` additionally supports directed search for specific named
    items, via a second, lazily-created promptable detector run on-demand
    against the latest cached frame -- the always-on background detector
    stays on its prompt-free model throughout, so directed search and
    automatic cataloging both work without disrupting each other. Replay-
    validated end-to-end on a recorded Go2 session; see
    `dimos/perception/test_object_scene_registration_replay.py` and wishlist
    item 2 in `dimos/skills/SKILLS_WISHLIST.md`.
    """

    color_image: In[Image]
    depth_image: In[Image]
    lidar: In[PointCloud2]
    camera_info: In[CameraInfo]

    detections_2d: Out[Detection2DArray]
    detections_3d: Out[Detection3DArray]
    objects: Out[list[DetObject]]
    pointcloud: Out[PointCloud2]

    _detector: Yoloe2DDetector | None = None
    # Lazily-created second detector, prompt-mode, used only by detect() for
    # on-demand directed search. Kept separate from _detector (the
    # open-vocab, prompt-free model that drives continuous background
    # cataloging) because Ultralytics prompt-free checkpoints structurally
    # reject set_classes() ("Prompt-free model does not support setting
    # classes") -- so calling detect() never disrupts the background catalog.
    _prompt_detector: Yoloe2DDetector | None = None
    _camera_info: CameraInfo | None = None
    _object_db: ObjectDB
    _latest_depth_image: Image | None = None
    _latest_camera_transform: Any = None
    # Latest processed frame + resolved localization inputs, cached by the
    # background pipeline on every successfully-processed frame. detect()
    # reuses these for its on-demand localization pass instead of waiting on
    # (or duplicating) the live subscription.
    _latest_color_image: Image | None = None
    _latest_lidar: PointCloud2 | None = None
    _latest_world_to_optical: Any = None
    _vl_model: Any = None

    # Optional link to spatial memory. When a SpatialMemory module is present in
    # the same graph it is auto-injected; otherwise this stays None and the
    # catalog degrades to objects-only. Lets catalog_scene fold the robot's
    # named/tagged places into the inventory and tag each object with the
    # nearest place.
    _spatial_memory: SceneMemorySpec | None = None

    def __init__(
        self,
        target_frame: str = "map",
        prompt_mode: YoloePromptMode = YoloePromptMode.LRPC,
        # Localization source: "depth" (RGBD back-projection) or "lidar"
        # (project a world-frame lidar cloud into detections). "depth" is the
        # default so existing depth-camera robots (e.g. xarm) are unchanged.
        localization: Literal["depth", "lidar"] = "depth",
        # Camera optical frame used for the lidar projection tf lookup.
        camera_optical_frame: str = "camera_optical",
        # Max camera/lidar timestamp gap for a frame pair to be processed in
        # lidar mode. 0.25s suits a live Go2 (camera ~14Hz, lidar ~7Hz:
        # partners are always close). Sparse recordings need more: e.g. the
        # full china-office .rrd logs camera at ~0.5Hz and lidar at ~0.85Hz,
        # where 0.25s leaves 60% of camera frames partnerless (never
        # processed at all). Widening is geometrically safe for accumulated
        # world-frame clouds -- they change slowly, so a cloud up to ~1s away
        # still projects correctly -- but raise it only as far as needed.
        lidar_match_tolerance: float = 0.25,
        # Pointcloud noise filters applied per detection in lidar mode.
        # "default" (None -> Detection3DPC.from_2d's stack: raycast +
        # radius_outlier(20 neighbors/0.3m) + statistical(40 neighbors)) is
        # tuned for dense accumulated clouds. "sparse" keeps the raycast
        # occlusion rejection but relaxes the density demands
        # (radius_outlier(4 neighbors/0.5m), no statistical) -- for sparse
        # scans/windows where the default kills most real clusters (measured
        # on the china-office recording: 12/31 detections localized with
        # "default" vs 22/31 with "sparse").
        lidar_filter_preset: Literal["default", "sparse"] = "default",
        # ObjectDB tuning
        distance_threshold: float = 0.2,
        min_detections_for_permanent: int = 6,
        # Object 3D reconstruction tuning
        max_distance: float = 0.0,
        use_aabb: bool = False,
        max_obstacle_width: float = 0.0,
        # Max horizontal distance (m) at which a tagged place is attached to an
        # object as its "near <place>" context in catalog_scene.
        place_radius: float = 5.0,
        # Min CLIP similarity (1 - cosine distance) for a spatial-memory frame
        # to count as a match in locate()'s semantic fallback.
        semantic_locate_threshold: float = 0.23,
        # Frame quality gate: skip running the detector (and updating the
        # cached "latest frame") on camera frames that are too dark or too
        # blurry to yield useful detections. Dark frames in particular are
        # both useless AND embed semantically close to everything (false
        # positives downstream). Both thresholds are in [0, 1] against
        # Image.brightness / Image.sharpness; 0.0 disables that gate (the
        # default -> unchanged behavior for existing depth robots). Opt in
        # per-blueprint on sparse/real recordings; a conservative pair is
        # ~(0.06, 0.12).
        min_frame_brightness: float = 0.0,
        min_frame_sharpness: float = 0.0,
        **kwargs: Any,
    ) -> None:
        super().__init__(**kwargs)
        self._target_frame = target_frame
        self._prompt_mode = prompt_mode
        self._localization = localization
        self._camera_optical_frame = camera_optical_frame
        self._lidar_match_tolerance = lidar_match_tolerance
        self._lidar_filter_preset = lidar_filter_preset
        self._object_db = ObjectDB(
            distance_threshold=distance_threshold,
            min_detections_for_permanent=min_detections_for_permanent,
        )
        self._max_distance = max_distance
        self._use_aabb = use_aabb
        self._max_obstacle_width = max_obstacle_width
        self._place_radius = place_radius
        self._semantic_locate_threshold = semantic_locate_threshold
        self._min_frame_brightness = min_frame_brightness
        self._min_frame_sharpness = min_frame_sharpness
        # Frame-quality-gate observability (see _frame_quality_ok).
        self._frames_seen = 0
        self._frames_rejected = 0

    @rpc
    def start(self) -> None:
        super().start()

        if self._prompt_mode == YoloePromptMode.LRPC:
            model_name = "yoloe-11l-seg-pf.pt"
        else:
            model_name = "yoloe-11l-seg.pt"

        self._detector = Yoloe2DDetector(
            model_name=model_name,
            prompt_mode=self._prompt_mode,
        )

        self.camera_info.subscribe(lambda msg: setattr(self, "_camera_info", msg))

        if self._localization == "lidar":
            aligned_frames = align_timestamped(
                self.color_image.observable(),
                self.lidar.observable(),
                buffer_size=20.0,
                match_tolerance=self._lidar_match_tolerance,
            )
            backpressure(aligned_frames).subscribe(self._on_lidar_frames)
        else:
            aligned_frames = align_timestamped(
                self.color_image.observable(),
                self.depth_image.observable(),
                buffer_size=2.0,
                match_tolerance=0.1,
            )
            backpressure(aligned_frames).subscribe(self._on_aligned_frames)

    @rpc
    def stop(self) -> None:
        """Stop the module and clean up resources."""

        if self._detector:
            self._detector.stop()
            self._detector = None

        if self._prompt_detector:
            self._prompt_detector.stop()
            self._prompt_detector = None

        if self._vl_model is not None:
            self._vl_model.stop()
            self._vl_model = None

        self._object_db.clear()

        logger.info("ObjectSceneRegistrationModule stopped")
        super().stop()

    @rpc
    def set_prompts(
        self,
        text: list[str] | None = None,
        bboxes: NDArray[np.float64] | None = None,
    ) -> None:
        """Set prompts for detection. Provide either text or bboxes, not both."""
        if self._detector is not None:
            self._detector.set_prompts(text=text, bboxes=bboxes)

    @rpc
    def select_object(self, track_id: int) -> dict[str, Any] | None:
        """Get object data by track_id and promote to permanent."""
        for obj in self._object_db.get_all_objects():
            if obj.track_id == track_id:
                self._object_db.promote(obj.object_id)
                return obj.to_dict()
        return None

    @rpc
    def get_object_track_ids(self) -> list[int]:
        """Get track_ids of all permanent objects."""
        return [obj.track_id for obj in self._object_db.get_all_objects()]

    @rpc
    def get_detected_objects(self) -> list[dict[str, Any]]:
        """Get all detected objects with object_id (UUID) and name."""
        now = self._object_db.now
        return [obj.agent_encode(now=now) for obj in self._object_db.get_all_objects()]

    @rpc
    def get_located_objects(self, include_pending: bool = False) -> list[dict[str, Any]]:
        """Get observed objects with world position, size and confidence.

        Args:
            include_pending: Also include freshly-seen (not yet stable) objects.
        """
        objs = (
            self._object_db.get_all_objects() if include_pending else self._object_db.get_objects()
        )
        now = self._object_db.now
        return [obj.locate_encode(now=now) for obj in objs]

    @rpc
    def get_object_pointcloud_by_name(self, name: str) -> PointCloud2 | None:
        """Get pointcloud for an object by class name."""
        objects = self._object_db.find_by_name(name)
        return objects[0].pointcloud if objects else None

    @rpc
    def get_object_pointcloud_by_object_id(self, object_id: str) -> PointCloud2 | None:
        """Get pointcloud for an object by its stable object_id (searches all objects)."""
        obj = self._object_db.find_by_object_id(object_id)
        if obj is None:
            logger.warning(f"No object found with object_id='{object_id}'")
            return None
        pc = obj.pointcloud
        num_points = len(pc.pointcloud.points) if pc else 0
        logger.info(f"Found object '{object_id}' ({obj.name}) with {num_points} points")
        return pc

    def _get_object_mask(self, object_id: str) -> NDArray[np.uint8] | None:
        """Get dilated mask for an object by ID."""
        for obj in self._object_db.get_all_objects():
            if obj.object_id != object_id:
                continue
            if obj.mask is None:
                return None

            mask = obj.mask.astype(np.uint8)
            if mask.max() == 1:
                mask = (mask * 255).astype(np.uint8)

            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (15, 15))
            return cv2.dilate(mask, kernel).astype(np.uint8)

        return None

    @rpc
    def get_full_scene_pointcloud(
        self,
        exclude_object_id: str | None = None,
        depth_trunc: float = 2.0,
        voxel_size: float = 0.01,
    ) -> PointCloud2 | None:
        """Get full scene pointcloud from depth, including table/surfaces for collision filtering."""
        if self._latest_depth_image is None or self._camera_info is None:
            return None

        depth_cv = self._latest_depth_image.to_opencv()
        h, w = depth_cv.shape[:2]

        # Zero out excluded object's depth
        if exclude_object_id:
            exclude_mask = self._get_object_mask(exclude_object_id)
            if exclude_mask is not None:
                depth_cv = depth_cv.copy()
                depth_cv[exclude_mask > 0] = 0

        # Build pointcloud from depth
        fx, fy = self._camera_info.K[0], self._camera_info.K[4]
        cx, cy = self._camera_info.K[2], self._camera_info.K[5]
        intrinsic = o3d.camera.PinholeCameraIntrinsic(w, h, fx, fy, cx, cy)

        depth_o3d = o3d.geometry.Image(depth_cv.astype(np.float32))
        pcd = o3d.geometry.PointCloud.create_from_depth_image(
            depth_o3d, intrinsic, depth_scale=1.0, depth_trunc=depth_trunc
        )

        if len(pcd.points) < 100:
            return None

        pcd = pcd.voxel_down_sample(voxel_size)

        pc = PointCloud2(
            pcd,
            frame_id=self._latest_depth_image.frame_id,
            ts=self._latest_depth_image.ts,
        )

        if self._latest_camera_transform is not None:
            pc = pc.transform(self._latest_camera_transform)

        return pc

    def _get_prompt_detector(self) -> Yoloe2DDetector:
        """Lazily create the promptable detector used by detect(). Separate
        model instance from the background prompt-free one (see the
        _prompt_detector field comment) -- costs extra GPU memory once
        detect() is first called, in exchange for directed search actually
        working without touching the always-on background catalog."""
        if self._prompt_detector is None:
            self._prompt_detector = Yoloe2DDetector(
                model_name="yoloe-11l-seg.pt",
                prompt_mode=YoloePromptMode.PROMPT,
            )
        return self._prompt_detector

    def _lidar_filters(self) -> list[Any] | None:
        """Pointcloud filter stack for the configured preset (see __init__)."""
        if self._lidar_filter_preset == "sparse":
            from dimos.perception.detection.type.detection3d.pointcloud_filters import (
                radius_outlier,
                raycast,
            )

            return [raycast(), radius_outlier(min_neighbors=4, radius=0.5)]
        return None  # Detection3DPC.from_2d applies its dense-cloud defaults

    def _localize_detections(
        self, detections_2d: ImageDetections2D[Any], color_image: Image
    ) -> list[DetObject] | None:
        """2D detections -> world-frame Objects for the *cached* latest
        frame, via whichever localization source this module is configured
        with. Used by detect()'s on-demand pass; the continuous background
        pipeline (_process_lidar / _process_3d_detections) has its own
        per-frame version since it runs on every frame and already has
        these values in hand without needing a cache.
        """
        if self._camera_info is None:
            return None
        if self._localization == "lidar":
            if self._latest_lidar is None or self._latest_world_to_optical is None:
                return None
            return Object.from_2d_to_list_lidar(
                detections_2d=detections_2d,
                world_pointcloud=self._latest_lidar,
                camera_info=self._camera_info,
                world_to_optical_transform=self._latest_world_to_optical,
                filters=self._lidar_filters(),
                use_aabb=self._use_aabb,
                max_distance=self._max_distance,
                max_obstacle_width=self._max_obstacle_width,
            )
        if self._latest_depth_image is None:
            return None
        return Object.from_2d_to_list(
            detections_2d=detections_2d,
            color_image=color_image,
            depth_image=self._latest_depth_image,
            camera_info=self._camera_info,
            camera_transform=self._latest_camera_transform,
            max_distance=self._max_distance,
            use_aabb=self._use_aabb,
            max_obstacle_width=self._max_obstacle_width,
        )

    @skill
    def detect(self, prompts: list[str]) -> str:
        """Directed, text-prompted search for specific named items.

        Runs alongside (not instead of) the automatic open-vocabulary
        cataloging that's always running in the background -- this uses a
        separate promptable detector on the most recent camera frame, so
        calling it never disrupts list_observed_items/catalog_scene. Any
        match is localized in the world frame and folded into the same
        catalog those tools read from.

        Do NOT call this tool multiple times for one query. Pass all objects in a single call.
        For example, to detect a cup and mouse, call detect(["cup", "mouse"]) not detect(["cup"]) then detect(["mouse"]).

        Args:
            prompts (list[str]): Text descriptions of objects to detect (e.g., ["person", "car", "dog"])

        Returns:
            str: Detected objects with their object_id (stable UUID) and name.

        Example:
            detect(["person", "car", "dog"])
            detect(["cup"])
        """
        if not prompts:
            return "No prompts provided."
        if self._latest_color_image is None:
            return "No camera frame available yet."

        detector = self._get_prompt_detector()
        detector.set_prompts(text=list(prompts))
        detections_2d = detector.process_image(self._latest_color_image)
        if not detections_2d.detections:
            return "No objects detected."

        objects = self._localize_detections(detections_2d, self._latest_color_image)
        if not objects:
            return "Detected objects but could not localize them in the world frame."

        ingested = self._ingest_and_publish(objects)

        obj_list = [f"  - {obj.display_name} (object_id='{obj.object_id}')" for obj in ingested]
        return f"Detected {len(ingested)} object(s):\n" + "\n".join(obj_list)

    @skill
    def select(self, track_id: int) -> str:
        """Select an object by track_id and promote it to permanent.

        Example:
            select(5)
        """
        result = self.select_object(track_id)
        if result is None:
            return f"No object found with track_id {track_id}."
        return f"Selected object {track_id}: {result['name']}"

    @skill
    def list_observed_items(self) -> str:
        """Catalog every object the robot currently observes in its environment.

        Reports each item's world position (meters, in the map/world frame),
        physical size, detection confidence, and — when a closer look has been
        taken (see identify_object / refine_observed_labels) — a one-sentence
        description. Uses whatever the robot has already seen; no prompt or
        target object needed. Use this to inventory the scene or to find where
        a specific item is. This is the fast, no-VLM-calls view; use
        catalog_scene instead for a one-call inventory with descriptions
        filled in for any item the detector could only label ambiguously.

        Returns:
            str: A catalog entry per item: name, object_id, world (x, y, z)
            position, size, confidence, observation count, and description
            when available.
        """
        located = self.get_located_objects()
        if not located:
            return "No objects observed yet."

        lines = self._format_located_objects(located, places=None)
        return f"Observing {len(located)} item(s):\n" + "\n".join(lines)

    def _fetch_tagged_places(self) -> list[RobotLocation]:
        """Fetch the robot's named/tagged places from spatial memory, if wired.

        Returns an empty list when no SpatialMemory is in the graph (the
        optional dependency resolves to None) or the call fails, so the catalog
        degrades gracefully to objects-only.
        """
        if self._spatial_memory is None:
            return []
        try:
            places = self._spatial_memory.get_tagged_locations()
        except Exception:
            logger.info("Failed to fetch tagged places from spatial memory", exc_info=True)
            return []
        return places or []

    def _nearest_place(
        self, x: float, y: float, places: list[RobotLocation]
    ) -> tuple[str, float] | None:
        """Nearest tagged place (name, horizontal distance) within place_radius."""
        best: tuple[str, float] | None = None
        for place in places:
            px, py = place.position[0], place.position[1]
            dist = ((x - px) ** 2 + (y - py) ** 2) ** 0.5
            if best is None or dist < best[1]:
                best = (place.name, dist)
        if best is None or best[1] > self._place_radius:
            return None
        return best

    def _format_located_objects(
        self, located: list[dict[str, Any]], places: list[RobotLocation] | None
    ) -> list[str]:
        """Render located objects as catalog lines, tagging each with the
        nearest known place when ``places`` is provided."""
        lines = []
        for obj in located:
            pos = obj["position"]
            size = obj["size"]
            line = (
                f"  - {obj['name']} (object_id='{obj['object_id']}'): "
                f"pos=({pos['x']:.2f}, {pos['y']:.2f}, {pos['z']:.2f})m "
                f"size=({size['x']:.2f}x{size['y']:.2f}x{size['z']:.2f})m "
                f"conf={obj['confidence']:.2f} "
                f"seen {obj['detections']}x, last {obj['last_seen']}"
            )
            if places:
                near = self._nearest_place(pos["x"], pos["y"], places)
                if near is not None:
                    line += f" — near '{near[0]}' ({near[1]:.1f}m)"
            lines.append(line)
            if obj.get("description"):
                lines.append(f"      {obj['description']}")
        return lines

    @skill
    def catalog_scene(self) -> str:
        """Build the most complete catalog of the space the robot has explored.

        The one-call way to answer "what have you seen, where is it, and
        what is it?" after exploring an environment — drive/patrol around
        first so the detector and depth/lidar localizer can build up
        observations, then call this. It fuses everything the robot knows
        about the space:

        - Detected objects: every stable (multiply-observed) item's name,
          object_id, world-frame (x, y, z) position, physical size, detection
          confidence, sighting count, and — after a vision-language model
          makes an educated guess on any ambiguously-labeled item — a
          one-sentence description.
        - Known places: named locations the robot has been told about via
          spatial memory (e.g. "kitchen", "charging dock"), which object
          detection alone can't produce. Each detected object is additionally
          tagged with the nearest such place for spatial context.

        The places section only appears when a spatial-memory module is
        running alongside this one and locations have been tagged; otherwise
        this returns the objects-only inventory (same as list_observed_items
        after refinement).

        Use list_observed_items instead for the fast objects-only view without
        VLM refinement, or identify_object / refine_observed_labels to control
        which items get a closer look.

        Returns:
            str: A "Known places" section (when available) followed by the
            full object inventory, each object annotated with its nearest place.
        """
        self.refine_object_labels(only_ambiguous=True)

        located = self.get_located_objects()
        places = self._fetch_tagged_places()

        if not located and not places:
            return "No objects observed yet."

        sections = []
        if places:
            place_lines = [
                f"  - {p.name}: ({p.position[0]:.2f}, {p.position[1]:.2f})m" for p in places
            ]
            sections.append(f"Known places ({len(places)}):\n" + "\n".join(place_lines))
        if located:
            obj_lines = self._format_located_objects(located, places=places)
            sections.append(f"Observing {len(located)} item(s):\n" + "\n".join(obj_lines))
        else:
            sections.append("No objects observed yet.")

        return "\n\n".join(sections)

    def _match_located_objects(self, query: str) -> list[dict[str, Any]]:
        """Located objects whose name matches the query (case-insensitive,
        substring either direction so "the cup" matches "cup")."""
        q = query.strip().lower()
        matches = []
        for obj in self.get_located_objects():
            name = str(obj.get("name", "")).strip().lower()
            if name and (q in name or name in q):
                matches.append(obj)
        return matches

    @skill
    def locate(self, query: str) -> str:
        """Locate a single thing by name or description and report where it is.

        Resolves the query against everything the robot knows, in order of
        precision:

        1. Catalogued objects — a detected item whose name matches (e.g.
           "backpack"); reports its world position, and its nearest known
           place when spatial memory is available.
        2. Tagged places — a named location the robot was told about (e.g.
           "kitchen", "charging dock").
        3. Semantic recall — if nothing above matches, falls back to spatial
           memory's visual memory of the space and returns the position of the
           spot that best matches the description, when the match is confident
           enough.

        Ask for one subject at a time. Use list_observed_items / catalog_scene
        to see the full inventory instead of a single lookup.

        Args:
            query: Name or description of the thing to locate (e.g. "the sofa").

        Returns:
            str: Where the thing is (world x, y, z in meters), or a message
            that it could not be located.
        """
        if not query or not query.strip():
            return "No query provided."

        # 1. Catalogued objects.
        matches = self._match_located_objects(query)
        if matches:
            places = self._fetch_tagged_places()
            lines = self._format_located_objects(matches, places=places)
            header = (
                f"Located '{query}' — {len(matches)} matching item(s):"
                if len(matches) > 1
                else f"Located '{query}':"
            )
            return header + "\n" + "\n".join(lines)

        # 2. Tagged places.
        if self._spatial_memory is not None:
            try:
                place = self._spatial_memory.query_tagged_location(query)
            except Exception:
                logger.info("query_tagged_location failed", exc_info=True)
                place = None
            if place is not None:
                x, y, z = place.position
                return f"'{query}' is the tagged place '{place.name}' at ({x:.2f}, {y:.2f}, {z:.2f})m."

            # 3. Semantic recall over remembered views.
            recalled = self._locate_from_memory(query)
            if recalled is not None:
                x, y, similarity = recalled
                return (
                    f"No catalogued '{query}', but from what I've seen it best matches "
                    f"a spot near ({x:.2f}, {y:.2f})m (similarity {similarity:.2f})."
                )

        return (
            f"Could not locate '{query}'. It isn't among the catalogued objects"
            + (
                " or anywhere I remember visiting."
                if self._spatial_memory is not None
                else "; no spatial memory is available for a broader search."
            )
        )

    def _locate_from_memory(self, query: str) -> tuple[float, float, float] | None:
        """Best (x, y, similarity) from spatial memory's frame embeddings for a
        text query, or None if no confident-enough match."""
        if self._spatial_memory is None:
            return None
        try:
            results = self._spatial_memory.query_by_text(query, limit=1)
        except Exception:
            logger.info("query_by_text failed", exc_info=True)
            return None
        if not results:
            return None

        best = results[0]
        distance = best.get("distance")
        similarity = 1.0 - (distance if distance is not None else 1.0)
        if similarity < self._semantic_locate_threshold:
            return None

        metadata = best.get("metadata")
        if isinstance(metadata, list):
            metadata = metadata[0] if metadata else None
        if not isinstance(metadata, dict):
            return None
        return float(metadata.get("pos_x", 0.0)), float(metadata.get("pos_y", 0.0)), similarity

    def _get_vl_model(self) -> Any:
        """Lazily create and start the VLM used for contextual labeling."""
        if self._vl_model is None:
            self._vl_model = create_vl_model(self.config.g.detection_model)
            self._vl_model.start()
        return self._vl_model

    def _refine_object(self, obj: DetObject) -> dict[str, Any] | None:
        """Run contextual labeling on one object and store the result on it."""
        scene = self._object_db.get_all_objects()
        result = contextual_label(self._get_vl_model(), obj, scene)
        if result is None:
            return None
        obj.refined_name = result.label
        obj.refined_description = result.description
        return {
            "object_id": obj.object_id,
            "detector_name": obj.name,
            "refined_name": result.label,
            "description": result.description,
            "confidence": round(result.confidence, 2),
        }

    @rpc
    def refine_object_labels(
        self,
        object_ids: list[str] | None = None,
        only_ambiguous: bool = True,
        confidence_threshold: float = DEFAULT_CONFIDENCE_THRESHOLD,
    ) -> list[dict[str, Any]]:
        """Refine object labels with a VLM educated guess using scene context.

        Args:
            object_ids: Specific objects to refine; None means all permanent objects.
            only_ambiguous: Skip objects the detector already identified confidently.
            confidence_threshold: Detector confidence below which a label counts
                as ambiguous.

        Returns:
            One dict per refined object with detector vs refined name.
        """
        if object_ids is not None:
            targets = [
                obj
                for oid in object_ids
                if (obj := self._object_db.find_by_object_id(oid)) is not None
            ]
        else:
            targets = self._object_db.get_objects()
            if only_ambiguous:
                targets = [obj for obj in targets if is_ambiguous(obj, confidence_threshold)]

        results = []
        for obj in targets:
            refined = self._refine_object(obj)
            if refined is not None:
                results.append(refined)
        return results

    @skill
    def identify_object(self, object_id: str) -> str:
        """Take a closer look at one observed object and make an educated guess
        about what it really is.

        Uses a vision-language model on the object's image together with scene
        context (the detector's guess, the object's real-world size, and what
        else is nearby) to assign a richer, more specific label. Use this when
        a detection's name looks wrong, generic, or low-confidence.

        Args:
            object_id: The object's id as reported by list_observed_items.

        Returns:
            str: The refined label and a one-sentence description.
        """
        obj = self._object_db.find_by_object_id(object_id)
        if obj is None:
            return f"No object found with object_id='{object_id}'."

        refined = self._refine_object(obj)
        if refined is None:
            return f"Could not identify object '{object_id}' ({obj.name}) — no usable image or VLM response."
        return (
            f"Object '{object_id}': detector said \"{refined['detector_name']}\", "
            f'educated guess is "{refined["refined_name"]}" '
            f"(confidence {refined['confidence']:.2f}). {refined['description']}"
        )

    @skill
    def refine_observed_labels(self) -> str:
        """Improve the labels of all ambiguously-identified observed objects.

        Goes through every observed object whose label is low-confidence or
        generic and asks a vision-language model for an educated guess, using
        each object's image, real-world size, and nearby objects as context.
        Refined names then appear in list_observed_items.

        Returns:
            str: One line per relabeled object (detector name -> refined name).
        """
        refined = self.refine_object_labels()
        if not refined:
            return "No ambiguous objects to relabel."

        lines = [
            f"  - {r['detector_name']} -> {r['refined_name']} "
            f"(conf={r['confidence']:.2f}): {r['description']}"
            for r in refined
        ]
        return f"Relabeled {len(refined)} object(s):\n" + "\n".join(lines)

    def _frame_quality_ok(self, color_image: Image) -> bool:
        """Whether a camera frame is worth running the detector on.

        Rejects frames darker than ``min_frame_brightness`` or blurrier than
        ``min_frame_sharpness`` (both disabled at 0.0). Rejected frames are
        neither detected on nor cached as the "latest" frame, so detect()'s
        on-demand pass also reuses only a good frame. Counts are logged
        periodically for observability.
        """
        if self._min_frame_brightness <= 0.0 and self._min_frame_sharpness <= 0.0:
            return True

        self._frames_seen += 1
        reason = None
        if self._min_frame_brightness > 0.0 and color_image.brightness < self._min_frame_brightness:
            reason = f"dark (brightness {color_image.brightness:.3f}<{self._min_frame_brightness})"
        elif self._min_frame_sharpness > 0.0 and color_image.sharpness < self._min_frame_sharpness:
            reason = f"blurry (sharpness {color_image.sharpness:.3f}<{self._min_frame_sharpness})"

        if reason is not None:
            self._frames_rejected += 1
            if self._frames_rejected % 25 == 1:
                logger.info(
                    f"Frame-quality gate: skipped {self._frames_rejected}/{self._frames_seen} "
                    f"frames so far (latest: {reason})"
                )
            return False
        return True

    def _on_aligned_frames(self, frames) -> None:  # type: ignore[no-untyped-def]
        color_msg, depth_msg = frames
        self._process_images(color_msg, depth_msg)

    def _process_images(self, color_msg: Image, depth_msg: Image) -> None:
        """Process synchronized color and depth images (runs in background thread)."""
        if not self._detector or not self._camera_info:
            return
        if not self._frame_quality_ok(color_msg):
            return

        color_image = color_msg
        # Convert depth to meters (float32)
        depth_cv = depth_msg.to_opencv()
        if depth_msg.format == ImageFormat.DEPTH16:
            depth_cv = depth_cv.astype(np.float32) / 1000.0
        elif depth_cv.dtype != np.float32:
            depth_cv = depth_cv.astype(np.float32)
        depth_image = Image(
            data=depth_cv, format=ImageFormat.DEPTH, frame_id=depth_msg.frame_id, ts=depth_msg.ts
        )

        # Run 2D detection
        detections_2d = self._detect_and_publish_2d(color_image)

        # Process 3D detections
        self._process_3d_detections(detections_2d, color_image, depth_image)

    def _detect_and_publish_2d(self, color_image: Image) -> ImageDetections2D[Any]:
        """Run the 2D detector on a color image and publish the ROS message."""
        assert self._detector is not None
        detections_2d: ImageDetections2D[Any] = self._detector.process_image(color_image)

        detections_2d_msg = Detection2DArray(
            detections_length=len(detections_2d.detections),
            header=Header(color_image.ts, color_image.frame_id or ""),
            detections=[det.to_ros_detection2d() for det in detections_2d.detections],
        )
        self.detections_2d.publish(detections_2d_msg)
        return detections_2d

    def _on_lidar_frames(self, frames) -> None:  # type: ignore[no-untyped-def]
        color_msg, lidar_msg = frames
        self._process_lidar(color_msg, lidar_msg)

    def _process_lidar(self, color_image: Image, lidar_msg: PointCloud2) -> None:
        """Locate detected objects by projecting a world-frame lidar cloud.

        Sensor-alternative to the depth path for robots with lidar but no depth
        camera (e.g. Go2). Feeds the same ObjectDB as the depth path.
        """
        if not self._detector or self._camera_info is None:
            return
        if not self._frame_quality_ok(color_image):
            return

        detections_2d = self._detect_and_publish_2d(color_image)

        # Transform from the lidar/world frame into the camera optical frame,
        # used as extrinsics to project points into the image. Resolved (and
        # cached below) regardless of whether the background prompt-free
        # detector found anything in this frame, so detect()'s on-demand
        # pass always has the truly latest frame -- not just the latest one
        # the background detector happened to catch something in.
        world_to_optical = self.tf.get(
            self._camera_optical_frame,
            lidar_msg.frame_id,
            color_image.ts,
            0.25,
        )
        if world_to_optical is None:
            logger.info("Failed to lookup transform from lidar frame to camera optical frame")
            return

        self._latest_color_image = color_image
        self._latest_lidar = lidar_msg
        self._latest_world_to_optical = world_to_optical

        if not detections_2d.detections:
            return

        objects = Object.from_2d_to_list_lidar(
            detections_2d=detections_2d,
            world_pointcloud=lidar_msg,
            camera_info=self._camera_info,
            world_to_optical_transform=world_to_optical,
            filters=self._lidar_filters(),
            use_aabb=self._use_aabb,
            max_distance=self._max_distance,
            max_obstacle_width=self._max_obstacle_width,
        )
        if not objects:
            return

        self._ingest_and_publish(objects)

    def _ingest_and_publish(self, objects: list[DetObject]) -> list[DetObject]:
        """Deduplicate objects into the DB and publish the full permanent set.

        Publishes ALL permanent objects so downstream consumers get the full
        scene, not just this frame's batch. Returns the input objects as
        updated/created in the DB (new object_ids, matched/merged state) for
        callers that need to report on exactly what they just ingested (e.g.
        detect()).
        """
        ingested = self._object_db.add_objects(objects)

        all_permanent = self._object_db.get_objects()

        detections_3d = to_detection3d_array(all_permanent)
        self.detections_3d.publish(detections_3d)
        self.objects.publish(all_permanent)

        aggregated_pc = aggregate_pointclouds(all_permanent)
        self.pointcloud.publish(aggregated_pc)

        return ingested

    def _process_3d_detections(
        self,
        detections_2d: ImageDetections2D[Any],
        color_image: Image,
        depth_image: Image,
    ) -> None:
        """Convert 2D detections to 3D and publish."""
        if self._camera_info is None:
            return

        # Cache depth image for full scene pointcloud generation
        self._latest_depth_image = depth_image

        # Look up transform from camera frame to target frame (e.g., map)
        camera_transform = None
        if self._target_frame != color_image.frame_id:
            camera_transform = self.tf.get(
                self._target_frame,
                color_image.frame_id,
                color_image.ts,
                0.1,
            )
            if camera_transform is None:
                logger.info("Failed to lookup transform from camera frame to target frame")
                return

        # Cache camera transform for full scene pointcloud
        self._latest_camera_transform = camera_transform
        # Cache the frame + resolved transform for detect()'s on-demand pass.
        self._latest_color_image = color_image

        objects = Object.from_2d_to_list(
            detections_2d=detections_2d,
            color_image=color_image,
            depth_image=depth_image,
            camera_info=self._camera_info,
            camera_transform=camera_transform,
            max_distance=self._max_distance,
            use_aabb=self._use_aabb,
            max_obstacle_width=self._max_obstacle_width,
        )
        if not objects:
            return

        self._ingest_and_publish(objects)
