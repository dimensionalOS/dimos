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

"""Perception localize: live object memory over a camera rig's ports.

The module consumes ``color_image``, ``camera_info``, ``tf`` and the geometry
its ``geometry`` config names: ``depth_image`` aligned to the colour camera,
or ``lidar``, a point cloud in the world frame or a frame tf reaches. It keeps
bounded memory, embeds the colour feed, and answers ``localize`` from what it
has embedded. It opens no file and does not know whether the ports carry a
robot or a replay.

Memory layout: the raw colour and geometry feeds are kept for ``feed_frames``
frames, only as long as the embed tail and the pairing need them. What
``localize`` reads is at index rate: the embedded frames, stored as JPEG, and
the depth frame or lidar scan paired with each of them, stored lz4. Both roll
over ``horizon_s`` seconds. A lidar call lifts through its window's scans
merged.

Agents calling ``state`` and ``localize`` over MCP: ``SKILL.md`` in this
package covers windows, watching labels, narrowing a position and params.
"""

from __future__ import annotations

from collections.abc import Iterator
from dataclasses import replace
import json
import math
import threading
import time
from typing import Any, Literal

from dimos_lcm.geometry_msgs import Pose
from dimos_lcm.vision_msgs import (
    BoundingBox2D,
    BoundingBox3D,
    ObjectHypothesis,
    ObjectHypothesisWithPose,
    Point2D,
    Pose2D,
)
import numpy as np
from numpy.typing import NDArray

from dimos.agents.annotation import skill
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.memory.blobstore.memory import MemoryBlobStore
from dimos.memory.observationstore.memory import ListObservationStore
from dimos.memory.store.memory import MemoryStore
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.std_msgs.Header import Header
from dimos.msgs.std_msgs.String import String
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.msgs.vision_msgs.Detection2D import Detection2D
from dimos.msgs.vision_msgs.Detection2DArray import Detection2DArray
from dimos.msgs.vision_msgs.Detection3D import Detection3D
from dimos.msgs.vision_msgs.Detection3DArray import Detection3DArray
from dimos.perception.localize.association import Groups
from dimos.perception.localize.dandetect import DanDetector
from dimos.perception.localize.rig import (
    DEPTH_TOLERANCE,
    EMBED_HZ,
    WALK_EMBED_HZ,
    Rig,
    RigCamera,
)
from dimos.perception.localize.types import Localization, LocalizeParams
from dimos.protocol.tf.tf import TF
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

INDEX_STREAM = "color_image_embedded"
GEOMETRY_STREAM = "geometry_memory"
COLOR_CODEC = "jpeg"
GEOMETRY_CODEC = "lz4+lcm"
LIDAR_TOLERANCE = 0.2  # s - an embedded frame to its lidar scan
SCAN_WINDOW_S = 3.0  # s - what scan_scene reads: what the camera sees now, three frames at EMBED_HZ
WATCH_WINDOW_S = (
    1.0  # s - a watch pass reads the newest frame: 1.7 s on an Orin NX, any label count
)
WATCH_PAUSE_S = 0.5  # s - between passes, so the GPU is not held all the time
WATCH_FORGET_S = 15.0  # s - an object not seen this long leaves the watched list


class PerceptionLocalizeConfig(ModuleConfig):
    world_frame: str = "world"
    # what lifts a detection: the aligned depth_image, or the lidar
    geometry: Literal["depth", "lidar"] = "depth"
    # the camera rides a walking base: the index runs at WALK_EMBED_HZ
    mobile: bool = False
    horizon_s: float = 600.0
    # raw frames kept for the embed tail and the pairing
    feed_frames: int = 300
    # each a dandetect registry name or a module:attr path
    embedder: str = "siglip"
    detector: str = "owlv2"
    segmenter: str = "edgetam"
    # the rig's params, set on the blueprint; localize() can override fields per call
    params: LocalizeParams = LocalizeParams()
    # labels looked for all the time, until watch_labels replaces them; empty = only when asked
    watch: list[str] = []


class PerceptionLocalize(Module):
    """Embed the colour feed and answer ``localize`` from the paired depth or lidar.

    ``camera_info`` must be stamped in the colour frame, and ``tf`` must reach
    that frame from ``world_frame`` at every colour timestamp; frames without a
    pose are never embedded. Depth must be aligned to colour.
    """

    # Its models hold the GIL for seconds at a time. Sharing a worker with ManipulationModule, the
    # weights never finished loading (minutes, GPU idle, against 13 s alone).
    dedicated_worker = True

    config: PerceptionLocalizeConfig

    color_image: In[Image]
    depth_image: In[Image]
    lidar: In[PointCloud2]
    camera_info: In[CameraInfo]
    tf: In[TFMessage]
    # comma-separated labels to watch for; empty goes back to the config's
    watch_labels: In[String]

    detections: Out[Detection3DArray]
    # what the watch has in view, as boxes in colour pixels
    detections_2d: Out[Detection2DArray]

    @rpc
    def start(self) -> None:
        super().start()
        self._stop = threading.Event()
        self._ready = threading.Event()
        self._camera_seen = threading.Event()
        self._camera: CameraInfo | None = None
        self._stage = "waiting for camera_info"
        self._groups: dict[str, Groups] = {}
        self._scanned: dict[str, tuple[str, Localization]] = {}
        self._detecting = threading.Lock()
        self._watch = list(self.config.watch)
        self._watched: dict[str, tuple[str, Localization]] = {}

        embed_hz = WALK_EMBED_HZ if self.config.mobile else EMBED_HZ
        memory_frames = int(self.config.horizon_s * embed_hz)
        feed_frames = self.config.feed_frames
        geometry_type = Image if self.config.geometry == "depth" else PointCloud2
        self._memory = self.register_disposable(MemoryStore())
        self._color_feed = self._memory.stream(
            "color_feed",
            Image,
            observation_store=ListObservationStore(name="color_feed", max_size=feed_frames),
        )
        self._geometry_feed = self._memory.stream(
            "geometry_feed",
            geometry_type,
            observation_store=ListObservationStore(name="geometry_feed", max_size=feed_frames),
        )
        self.index = self._memory.stream(
            INDEX_STREAM,
            Image,
            codec=COLOR_CODEC,
            blob_store=MemoryBlobStore(max_items=memory_frames),
            observation_store=ListObservationStore(name=INDEX_STREAM, max_size=memory_frames),
        )
        self._geometry_memory = self._memory.stream(
            GEOMETRY_STREAM,
            geometry_type,
            codec=GEOMETRY_CODEC,
            blob_store=MemoryBlobStore(max_items=memory_frames),
            observation_store=ListObservationStore(name=GEOMETRY_STREAM, max_size=memory_frames),
        )
        self._tf_buffer = TF(self.tf, buffer_size=self.config.horizon_s)

        def on_camera_info(info: CameraInfo) -> None:
            if self._camera is None:
                self._camera = info
                self._camera_seen.set()

        def on_geometry(msg: Image | PointCloud2) -> None:
            self._geometry_feed.append(msg, ts=msg.ts)

        geometry = self.depth_image if self.config.geometry == "depth" else self.lidar
        self._unsubs = [
            self.color_image.subscribe(lambda img: self._color_feed.append(img, ts=img.ts)),
            geometry.subscribe(on_geometry),
            self.camera_info.subscribe(on_camera_info),
            self.watch_labels.subscribe(self._on_watch_labels),
        ]
        self._thread = threading.Thread(target=self._warm, name="localize-warmup", daemon=True)
        self._thread.start()
        self._watch_thread = threading.Thread(
            target=self._watch_loop, name="localize-watch", daemon=True
        )
        self._watch_thread.start()

    @rpc
    def stop(self) -> None:
        self._stop.set()
        self._ready.clear()
        self._thread.join(timeout=5.0)
        self._watch_thread.join(timeout=5.0)
        for unsubscribe in self._unsubs:
            unsubscribe()
        self._tf_buffer.dispose()
        super().stop()

    def _pair_geometry(self, upstream: Iterator[Any]) -> Iterator[Any]:
        """Move the depth frame or lidar scan of each embedded frame from the feed into memory."""
        tolerance = DEPTH_TOLERANCE if self.config.geometry == "depth" else LIDAR_TOLERANCE
        for obs in upstream:
            try:
                paired = self._geometry_feed.at(obs.ts, tolerance).first()
            except LookupError:
                yield obs
                continue
            self._geometry_memory.append(paired.data, ts=paired.ts)
            yield obs

    def _warm(self) -> None:
        stages = (self.config.embedder, self.config.detector, self.config.segmenter)
        self._stage = f"loading {', '.join(stages)} weights"
        logger.info(f"localize: {self._stage}")
        self.detector = self.register_disposable(DanDetector(*stages))
        self.detector.start()

        self._stage = "waiting for camera_info"
        logger.info(f"localize: {self._stage}")
        while not self._camera_seen.wait(0.5):
            if self._stop.is_set():
                return
        camera = self._camera
        assert camera is not None
        depth = self._geometry_memory if self.config.geometry == "depth" else None
        self.rig = Rig(
            cameras={camera.frame_id: RigCamera(info=camera, color=self.index, depth=depth)},
            tf=self._tf_buffer,
            world_frame=self.config.world_frame,
            lidar=None if depth is not None else self._geometry_memory,
            mobile=self.config.mobile,
        )
        self.detector.embed_live(self.rig, self._color_feed, self.index)
        self.register_disposable(self.index.live().transform(self._pair_geometry).drain_thread())

        self._stage = "waiting for the first posed frame of the feed"
        logger.info(f"localize: {self._stage}")
        while self.index.count() == 0:
            if self._stop.is_set():
                return
            time.sleep(0.5)

        self._stage = "ready"
        self._ready.set()
        logger.info(f"localize: ready on {self.index.count()} embedded frames")

    @skill
    def state(self) -> str:
        """Whether localize can answer yet, and what it is doing if not."""
        if self._ready.is_set():
            return f"ready: {self.index.count()} frames embedded, localize will answer"
        return f"not ready: {self._stage}"

    def _localize_window(
        self, objects: str, start: float, duration: float, params: str
    ) -> tuple[list[str], list[list[Localization]], Any, Rig, tuple[float, float, float]]:
        """The call behind ``localize`` and ``localize_boxes``: the window resolved, the
        detector run, the result published; returns the queries, their hits, the index
        window, the rig, and (first, head, lo) of the feed."""
        queries = [q.strip() for q in objects.split(",") if q.strip()]
        first, head = self.index.get_time_range()
        lo = max(first, head + start if start < 0 else first + start)
        overrides = json.loads(params) if params.strip() else None
        results, index, rig = self._detect(queries, lo, duration, overrides, self._groups)
        self.detections.publish(as_detection_array(queries, results, rig.world_frame))
        return queries, results, index, rig, (first, head, lo)

    def _detect(
        self,
        queries: list[str],
        lo: float,
        duration: float,
        overrides: dict[str, Any] | None,
        groups: dict[str, Groups],
    ) -> tuple[list[list[Localization]], Any, Rig]:
        """The detector over a window of the index, one call at a time: ``groups`` is not
        safe to share and the watch runs beside the RPCs."""
        index = self.index.time_range(lo, lo + duration)
        rig = self.rig
        if rig.lidar is not None:
            rig = replace(rig, lidar=rig.lidar.time_range(lo, lo + duration))
        with self._detecting:
            results = self.detector.localize(
                queries,
                index=index,
                rig=rig,
                params=self.config.params,
                fields=overrides,
                groups=groups,
            )
        return results, index, rig

    def _on_watch_labels(self, message: String) -> None:
        labels = [label.strip() for label in message.data.split(",") if label.strip()]
        self._watch = labels or list(self.config.watch)
        logger.info(f"localize: watching {self._watch}")

    def _watch_loop(self) -> None:
        while not self._stop.wait(WATCH_PAUSE_S):
            labels = self._watch
            if not labels or not self._ready.is_set():
                continue
            try:
                self._watch_pass(labels)
            except Exception:
                logger.exception("localize: a watch pass failed")

    def _watch_pass(self, labels: list[str]) -> None:
        """Look at the newest frame for ``labels`` and publish everything seen lately.

        A pass starts from nothing, so each instance's cloud is this frame's: one folded over
        many frames would keep an object a missed grasp nudged where it stood. One frame is one
        view, so ``min_views`` cannot apply. An object keeps its id from pass to pass by where
        it is, and stays listed through a frame that missed it.
        """
        first, head = self.index.get_time_range()
        results, _index, rig = self._detect(
            labels, max(first, head - WATCH_WINDOW_S), WATCH_WINDOW_S, {"min_views": 1}, {}
        )
        radius = self.config.params.cluster_radius_m
        kept = {
            known: entry
            for known, entry in self._watched.items()
            if head - entry[1].last_seen_timestamp <= WATCH_FORGET_S
        }
        seen: dict[str, tuple[str, Localization]] = {}
        for label, hit in _one_per_place(labels, results, radius):
            same = next((k for k, (_, old) in kept.items() if _apart(old, hit) <= radius), None)
            if same is not None:
                del kept[same]
                hit = replace(hit, instance_id=same)
            seen[hit.instance_id] = (label, hit)
        self._watched = {**kept, **seen}

        listed = sorted(self._watched.values(), key=lambda entry: -entry[1].semantic_score)
        self.detections.publish(
            as_detection_array(
                [label for label, _ in listed], [[hit] for _, hit in listed], rig.world_frame
            )
        )
        camera = self._camera
        assert camera is not None
        world_to_optical = rig.world_to_optical(head, camera.frame_id)
        if world_to_optical is not None:
            self.detections_2d.publish(
                as_detection_2d_array(listed, head, world_to_optical.to_matrix(), camera)
            )

    @skill
    def localize(
        self, objects: str, start: float = -10.0, duration: float = 10.0, params: str = ""
    ) -> str:
        """Locate objects in a window of the robot's memory.

        ``objects`` is one label, or several separated by commas.

        ``start`` and ``duration`` are seconds and name the window. A positive
        ``start`` counts forward from the beginning of the feed; a negative one
        counts back from the newest frame, so the default reads the last ten
        seconds. What earlier calls proved is remembered and still answered.

        ``params`` is a JSON object of LocalizeParams field overrides for this
        call only, e.g. '{"accept_score": 0.45, "verify_radius_m": 2.0}'.
        The next call without ``params`` is back on the module's params.

        Usage guide: dimos/perception/localize/SKILL.md.
        """
        if not self._ready.is_set():
            return f"localize cannot answer yet: {self._stage}. Poll state() until it reads ready."

        queries, results, index, _rig, (first, head, lo) = self._localize_window(
            objects, start, duration, params
        )
        lines: list[str] = [
            f"window {lo - first:.1f}s to {lo + duration - first:.1f}s of "
            f"{head - first:.1f}s of feed, {index.count()} frames"
        ]
        for query, hits in zip(queries, results, strict=True):
            if not hits:
                lines.append(f"no verified detection of {query!r}")
            for hit in hits:
                x, y, z = hit.position_world_xyz
                lines.append(
                    f"{query!r} at ({x:.2f}, {y:.2f}, {z:.2f}) in {hit.frame_id} "
                    f"score={hit.semantic_score:.2f} views={hit.n_views} id={hit.instance_id} "
                    f"seen {max(0.0, head - hit.last_seen_timestamp):.0f}s ago"
                )
        return "\n".join(lines)

    @rpc
    def localize_boxes(
        self, objects: str, start: float = -10.0, duration: float = 10.0, params: str = ""
    ) -> list[dict[str, Any]]:
        """``localize`` for a UI: one JSON box per verified instance instead of prose.

        Same arguments. Each box: ``id``, ``label`` (the query it answered), ``score``,
        ``views``, ``center`` and ``size`` (the instance's cloud extents, metres in
        ``frame``), ``position`` (the latest sighting), ``seen_s_ago``. Raises while the
        module is not ready, with its stage.
        """
        if not self._ready.is_set():
            raise RuntimeError(f"localize cannot answer yet: {self._stage}")
        queries, results, _index, _rig, (_first, head, _lo) = self._localize_window(
            objects, start, duration, params
        )
        boxes: list[dict[str, Any]] = []
        for query, hits in zip(queries, results, strict=True):
            for hit in hits:
                center, size = _extents(hit)
                boxes.append(
                    {
                        "id": hit.instance_id,
                        "label": query,
                        "score": float(hit.semantic_score),
                        "views": int(hit.n_views),
                        "center": center,
                        "size": size,
                        "position": [float(v) for v in hit.position_world_xyz],
                        "frame": hit.frame_id,
                        "seen_s_ago": float(max(0.0, head - hit.last_seen_timestamp)),
                    }
                )
        return boxes

    @rpc
    def scan_scene(self, text: list[str] | None = None) -> Detection3DArray:
        """``localize`` for a pick: one instance per object ``text`` names in the last moments.

        A scan reads afresh: what earlier calls proved of these labels is forgotten, or an
        object already picked would answer from where it stood. Each instance is kept by id
        until the next scan. Raises while the module is not ready, with its stage.
        """
        if not self._ready.is_set():
            raise RuntimeError(f"localize cannot answer yet: {self._stage}")
        labels = [label.strip() for label in text or [] if label.strip()]
        for label in labels:
            self._groups.pop(label, None)
        queries, results, _index, rig, _times = self._localize_window(
            ",".join(labels), -SCAN_WINDOW_S, SCAN_WINDOW_S, ""
        )
        self._scanned = {
            hit.instance_id: (query, hit)
            for query, hits in zip(queries, results, strict=True)
            for hit in hits
        }
        return as_detection_array(queries, results, rig.world_frame)

    @rpc
    def get_detected_objects(self) -> list[dict[str, Any]]:
        """The latest ``scan_scene`` for a UI: ``object_id``, ``name`` (the query it answered),
        ``confidence``, ``center`` and ``size`` (metres in ``frame_id``), ``detections`` (views)
        and ``last_seen``, as the scene registration's objects encode them."""
        objects = []
        for name, hit in self._scanned.values():
            center, size = _extents(hit)
            objects.append(
                {
                    "object_id": hit.instance_id,
                    "name": name,
                    "confidence": float(hit.semantic_score),
                    "center": center,
                    "size": size,
                    "frame_id": hit.frame_id,
                    "detections": int(hit.n_views),
                    "last_seen": f"{round(time.time() - hit.last_seen_timestamp)}s ago",
                }
            )
        return objects

    @rpc
    def get_object_pointcloud_by_object_id(self, object_id: str) -> PointCloud2 | None:
        """An instance's cloud from the latest ``scan_scene`` or the watch, in the world frame."""
        scanned = self._scanned.get(object_id) or self._watched.get(object_id)
        return None if scanned is None else scanned[1].point_cloud

    @rpc
    def get_full_scene_pointcloud(
        self,
        exclude_object_id: str | None = None,
        depth_trunc: float = 2.0,
        voxel_size: float = 0.01,
    ) -> PointCloud2 | None:
        """Localize registers no scene, so a pick has nothing to check its grasps against."""
        return None


def _extents(hit: Localization) -> tuple[list[float], list[float]]:
    """The centre and size of an instance's cloud, metres in its frame."""
    points = hit.point_cloud.as_numpy()[0]
    low, high = points.min(axis=0), points.max(axis=0)
    return [float(v) for v in (low + high) / 2], [max(float(v), 1e-3) for v in high - low]


def _apart(a: Localization, b: Localization) -> float:
    return math.dist(a.position_world_xyz, b.position_world_xyz)


def _one_per_place(
    queries: list[str], results: list[list[Localization]], radius: float
) -> list[tuple[str, Localization]]:
    """Each object once, best score first, under the label that scored it best: labels are not
    compared, so "box" and "block" both answer for one carton."""
    ranked = sorted(
        ((query, hit) for query, hits in zip(queries, results, strict=True) for hit in hits),
        key=lambda found: -found[1].semantic_score,
    )
    places: list[tuple[str, Localization]] = []
    for query, hit in ranked:
        if all(_apart(hit, other) > radius for _, other in places):
            places.append((query, hit))
    return places


def _image_box(
    points: NDArray[Any], world_to_optical: NDArray[Any], camera: CameraInfo
) -> tuple[NDArray[Any], NDArray[Any]] | None:
    """The (low, high) pixel corners around ``points``, or None when none land in the image."""
    optical = points @ world_to_optical[:3, :3].T + world_to_optical[:3, 3]
    optical = optical[optical[:, 2] > 0]
    if not len(optical):
        return None
    pixels = optical @ camera.get_K_matrix().T
    pixels = pixels[:, :2] / pixels[:, 2:]
    low = np.maximum(pixels.min(axis=0), 0.0)
    high = np.minimum(pixels.max(axis=0), (camera.width, camera.height))
    return None if (high <= low).any() else (low, high)


def as_detection_2d_array(
    listed: list[tuple[str, Localization]],
    ts: float,
    world_to_optical: NDArray[Any],
    camera: CameraInfo,
) -> Detection2DArray:
    """One labelled box per instance in view: its cloud as the camera sees it now, so the
    box stays right for an instance this frame missed."""
    header = Header(ts, camera.frame_id)
    boxes = []
    for label, hit in listed:
        box = _image_box(hit.point_cloud.as_numpy()[0], world_to_optical, camera)
        if box is None:
            continue
        (left, top), (right, bottom) = box
        boxes.append(
            Detection2D(
                header=header,
                id=hit.instance_id,
                results=[
                    ObjectHypothesisWithPose(
                        hypothesis=ObjectHypothesis(class_id=label, score=hit.semantic_score)
                    )
                ],
                results_length=1,
                bbox=BoundingBox2D(
                    center=Pose2D(
                        position=Point2D(x=float(left + right) / 2, y=float(top + bottom) / 2),
                        theta=0.0,
                    ),
                    size_x=float(right - left),
                    size_y=float(bottom - top),
                ),
            )
        )
    return Detection2DArray(detections_length=len(boxes), header=header, detections=boxes)


def as_detection_array(
    queries: list[str], results: list[list[Localization]], frame_id: str
) -> Detection3DArray:
    """One labelled box per verified instance, for the rerun bridge."""
    boxes = []
    latest = 0.0
    for query, hits in zip(queries, results, strict=True):
        for hit in hits:
            points = hit.point_cloud.as_numpy()[0]
            low, high = points.min(axis=0), points.max(axis=0)
            middle, extent = (low + high) / 2, high - low
            center = Vector3(*(float(v) for v in middle))
            size = Vector3(*(max(float(v), 1e-3) for v in extent))
            latest = max(latest, hit.last_seen_timestamp)
            boxes.append(
                Detection3D(
                    header=Header(hit.last_seen_timestamp, frame_id),
                    id=hit.instance_id,
                    results=[
                        ObjectHypothesisWithPose(
                            hypothesis=ObjectHypothesis(class_id=query, score=hit.semantic_score)
                        )
                    ],
                    results_length=1,
                    bbox=BoundingBox3D(
                        center=Pose(position=center, orientation=Quaternion(0.0, 0.0, 0.0, 1.0)),
                        size=size,
                    ),
                )
            )
    return Detection3DArray(
        detections_length=len(boxes),
        header=Header(latest, frame_id),
        detections=boxes,
    )
