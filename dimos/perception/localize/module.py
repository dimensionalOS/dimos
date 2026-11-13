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
import threading
import time
from typing import Any, Literal

import cv2
from dimos_lcm.geometry_msgs import Pose
from dimos_lcm.vision_msgs import BoundingBox3D, ObjectHypothesis, ObjectHypothesisWithPose
import numpy as np
from reactivex.disposable import CompositeDisposable, Disposable

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
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.msgs.vision_msgs.Detection3D import Detection3D
from dimos.msgs.vision_msgs.Detection3DArray import Detection3DArray
from dimos.perception.localize.dandetect import DanDetector
from dimos.perception.localize.localize import LocalizeTrace
from dimos.perception.localize.memento import Memento
from dimos.perception.localize.rig import (
    DEPTH_TOLERANCE,
    EMBED_HZ,
    WALK_EMBED_HZ,
    Rig,
    RigCamera,
    camera_hz,
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
MASK_ALPHA = 0.45
LABEL_COLORS: tuple[tuple[int, int, int], ...] = (
    (31, 119, 180),
    (255, 127, 14),
    (44, 160, 44),
    (214, 39, 40),
    (148, 103, 189),
    (140, 86, 75),
    (227, 119, 194),
    (188, 189, 34),
    (23, 190, 207),
    (127, 127, 127),
)


class PerceptionLocalizeConfig(ModuleConfig):
    world_frame: str = "world"
    # what lifts a detection: the aligned depth_image, or the lidar
    geometry: Literal["depth", "lidar"] = "depth"
    # uint16 depth units in meters (0.001 m for D435/D455, 0.0001 m for D405)
    depth_scale: float = 0.001
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


class PerceptionLocalize(Module):
    """Embed the colour feed and answer ``localize`` from the paired depth or lidar.

    ``camera_info`` must be stamped in the colour frame, and ``tf`` must reach
    that frame from ``world_frame`` at every colour timestamp; frames without a
    pose are never embedded. Depth must be aligned to colour.
    """

    config: PerceptionLocalizeConfig

    color_image: In[Image]
    depth_image: In[Image]
    lidar: In[PointCloud2]
    camera_info: In[CameraInfo]
    tf: In[TFMessage]

    detections: Out[Detection3DArray]
    segmented_image: Out[Image]
    detected_pointcloud: Out[PointCloud2]

    _thread: threading.Thread | None = None
    detector: DanDetector | None = None

    def _ensure_detector(self) -> DanDetector:
        if self.detector is None:
            stages = (self.config.embedder, self.config.detector, self.config.segmenter)
            self._stage = f"loading {', '.join(stages)} weights"
            logger.info(f"localize: {self._stage}")
            self.detector = self.register_disposable(DanDetector(*stages))
            self.detector.start()
            if hasattr(self.detector.embedder, "_model"):
                self.detector.embedder.embed_text("warmup")
        return self.detector

    @rpc
    def build(self) -> None:
        super().build()
        self._ensure_detector()

    @rpc
    def start(self) -> None:
        super().start()
        self._stop = threading.Event()
        self._ready = threading.Event()
        self._camera_seen = threading.Event()
        self._camera: CameraInfo | None = None
        self._stage = "waiting for camera_info"
        # Unsubscribe inputs before closing the stores they write to.
        subscriptions = self.register_disposable(CompositeDisposable())
        self._memento = Memento()
        self.register_disposable(Disposable(self._memento.close))

        embed_hz = (
            WALK_EMBED_HZ
            if self.config.mobile and self.config.params.embed_hz == EMBED_HZ
            else self.config.params.embed_hz
        )
        self._embed_hz = embed_hz
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
        subscriptions.add(Disposable(self._tf_buffer.dispose))

        def on_camera_info(info: CameraInfo) -> None:
            if self._camera is None:
                self._camera = info
                self._camera_seen.set()

        def on_geometry(msg: Image | PointCloud2) -> None:
            self._geometry_feed.append(msg, ts=msg.ts)

        geometry = self.depth_image if self.config.geometry == "depth" else self.lidar
        subscriptions.add(
            Disposable(
                self.color_image.subscribe(lambda img: self._color_feed.append(img, ts=img.ts))
            )
        )
        subscriptions.add(Disposable(geometry.subscribe(on_geometry)))
        subscriptions.add(Disposable(self.camera_info.subscribe(on_camera_info)))
        self._thread = threading.Thread(target=self._warm, name="localize-warmup", daemon=True)
        self._thread.start()

    @rpc
    def stop(self) -> None:
        if self._thread is not None:
            self._stop.set()
            self._ready.clear()
            if self._thread.is_alive():
                self._thread.join(timeout=5.0)
        self.detector = None
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
        detector = self._ensure_detector()

        self._stage = "waiting for camera_info"
        logger.info(f"localize: {self._stage}")
        while not self._camera_seen.wait(0.5):
            if self._stop.is_set():
                return
        camera = self._camera
        assert camera is not None
        depth = self._geometry_memory if self.config.geometry == "depth" else None
        self.rig = Rig(
            cameras={
                camera.frame_id: RigCamera(
                    info=camera,
                    color=self.index,
                    depth=depth,
                    hz=camera_hz(self._color_feed),
                    depth_scale=self.config.depth_scale,
                )
            },
            tf=self._tf_buffer,
            world_frame=self.config.world_frame,
            lidar=None if depth is not None else self._geometry_memory,
            mobile=self.config.mobile,
            embed_hz=self._embed_hz,
        )
        detector.embed_live(self.rig, self._color_feed, self.index, embed_hz=self.rig.embed_hz)
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

    @rpc
    def index_span(self) -> tuple[float, float, int] | None:
        """First stamp, newest stamp and count of embedded frames; None until ready."""
        if not self._ready.is_set():
            return None
        count = self.index.count()
        if count == 0:
            return None
        first, head = self.index.get_time_range()
        return float(first), float(head), count

    @skill
    def localize(
        self, objects: str, start: float = -10.0, duration: float = 10.0, params: str = ""
    ) -> str:
        """Locate objects in a window of the robot's memory.

        ``objects`` is one label, or several separated by commas.

        ``start`` and ``duration`` are seconds and name the window. A positive
        ``start`` counts forward from the beginning of the feed; a negative one
        counts back from the newest frame, so the default reads the last ten
        seconds. The answer holds only what was seen inside the window. A frame
        an earlier call detected for a label is answered from memory, not
        detected again, so an overlapping window detects only what is new.

        ``params`` is a JSON object of LocalizeParams field overrides for this
        call only, e.g. '{"accept_score": 0.45, "verify_radius_m": 2.0}'.
        The next call without ``params`` is back on the module's params.

        Usage guide: dimos/perception/localize/SKILL.md.
        """
        if not self._ready.is_set():
            return f"localize cannot answer yet: {self._stage}. Poll state() until it reads ready."

        queries = [q.strip() for q in objects.split(",") if q.strip()]
        overrides = json.loads(params) if params.strip() else None
        results, (lo, hi, first, head, frames) = self._localize(queries, start, duration, overrides)

        lines: list[str] = [
            f"window {lo - first:.1f}s to {hi - first:.1f}s of "
            f"{head - first:.1f}s of feed, {frames} frames"
        ]
        for query, hits in zip(queries, results, strict=True):
            if not hits:
                lines.append(f"no verified detection of {query!r}")
            for hit in hits:
                x, y, z = hit.position_world_xyz
                lines.append(
                    f"{query!r} at ({x:.2f}, {y:.2f}, {z:.2f}) in {hit.frame_id} "
                    f"score={hit.semantic_score:.2f} views={hit.n_views} id={hit.instance_id} "
                    f"seen {max(0.0, head - hit.last_seen_timestamp):.0f}s ago, "
                    f"first {max(0.0, head - hit.first_seen_timestamp):.0f}s ago"
                )
        return "\n".join(lines)

    def _localize(
        self,
        queries: list[str],
        start: float,
        duration: float,
        overrides: dict[str, Any] | None,
    ) -> tuple[list[list[Localization]], tuple[float, float, float, float, int]]:
        """One localize call: the instances per query, and the window it read."""
        first, head = self.index.get_time_range()
        lo = max(first, head + start if start < 0 else first + start)
        hi = lo + duration
        # frames older than the feed's memory can no longer be asked for
        self._memento.forget(first)
        index = self.index.time_range(lo, hi)
        for cam in self.rig.cameras.values():
            if cam.hz is None:
                cam.hz = camera_hz(self._color_feed)
        rig = self.rig
        if rig.lidar is not None:
            rig = replace(rig, lidar=rig.lidar.time_range(lo, hi))
        assert self.detector is not None
        traces = [LocalizeTrace() for _ in queries]
        results = self.detector.localize(
            queries,
            index=index,
            window=(lo, hi),
            head=head,
            rig=rig,
            params=self.config.params,
            fields=overrides,
            memento=self._memento,
            traces=traces,
        )
        results = [
            [h for h in hits if h.n_views >= 0.25 * max(x.n_views for x in hits)] if hits else []
            for hits in results
        ]
        self.detections.publish(as_detection_array(queries, results, rig.world_frame))
        self._publish_visualizations(queries, results, traces, rig.world_frame)
        return results, (lo, hi, first, head, index.count())

    def _publish_visualizations(
        self,
        queries: list[str],
        results: list[list[Localization]],
        traces: list[LocalizeTrace],
        world_frame: str,
    ) -> None:
        """Publish 2D segmented overlay image and 3D detected object point clouds for Rerun."""
        colors = {query: LABEL_COLORS[k % len(LABEL_COLORS)] for k, query in enumerate(queries)}
        shots: dict[tuple[str, float], tuple[Image, list[Any]]] = {}
        for trace in traces:
            for obs in trace.detection_frames:
                image = obs.data.image
                shots.setdefault((image.frame_id, obs.ts), (image, []))[1].extend(
                    obs.data.detections
                )
        for (_camera, _ts), (image, dets) in sorted(shots.items(), key=lambda shot: shot[0][1]):
            if dets:
                self.segmented_image.publish(_overlay_detections(image, dets, colors))

        cloud_parts: list[np.ndarray] = []
        latest_ts = 0.0
        for hits in results:
            for hit in hits:
                pts = hit.point_cloud.points_f32()
                if len(pts):
                    cloud_parts.append(pts)
                    latest_ts = max(latest_ts, hit.last_seen_timestamp)
        if cloud_parts:
            merged_cloud = PointCloud2.from_numpy(
                np.vstack(cloud_parts),
                frame_id=world_frame,
                timestamp=latest_ts or time.time(),
            )
            self.detected_pointcloud.publish(merged_cloud)


def _overlay_detections(
    image: Image,
    detections: list[Any],
    colors: dict[str, tuple[int, int, int]],
) -> Image:
    """Render each detection's segmentation mask, contour, box, and label onto ``image``."""
    best_by_box: dict[tuple[float, ...], Any] = {}
    for det in detections:
        key = tuple(round(float(v), 1) for v in det.bbox)
        prev = best_by_box.get(key)
        if prev is None or det.confidence > prev.confidence:
            best_by_box[key] = det
    selected = list(best_by_box.values())

    canvas = image.to_opencv().copy()
    tint = canvas.copy()
    for det in selected:
        rgb = colors.get(det.name, LABEL_COLORS[0])
        mask = getattr(det, "mask", None)
        if mask is not None:
            tint[mask > 0] = rgb[::-1]
    canvas = cv2.addWeighted(tint, MASK_ALPHA, canvas, 1.0 - MASK_ALPHA, 0.0)
    for det in selected:
        rgb = colors.get(det.name, LABEL_COLORS[0])
        bgr = rgb[::-1]
        mask = getattr(det, "mask", None)
        if mask is not None:
            contours, _ = cv2.findContours(
                (mask > 0).astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )
            cv2.drawContours(canvas, contours, -1, bgr, 2)
        x1, y1, x2, y2 = (int(v) for v in det.bbox)
        cv2.rectangle(canvas, (x1, y1), (x2, y2), bgr, 1)
        label = f"{det.name} {det.confidence:.2f}"
        (width, height), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
        cv2.rectangle(canvas, (x1, y1 - height - 6), (x1 + width + 4, y1), (0, 0, 0), -1)
        cv2.rectangle(canvas, (x1, y1 - height - 6), (x1 + width + 4, y1), bgr, 1)
        cv2.putText(
            canvas, label, (x1 + 2, y1 - 4), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1
        )
    return Image.from_opencv(canvas, frame_id=image.frame_id, ts=image.ts)


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
