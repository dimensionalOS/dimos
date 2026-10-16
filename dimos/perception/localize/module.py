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
from pydantic import Field

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
from dimos.perception.localize.association import M_CX, M_CZ, M_KX, M_KZ, M_SCORE, M_TS, Groups
from dimos.perception.localize.dandetect import DanDetector
from dimos.perception.localize.rig import DEPTH_TOLERANCE, EMBED_HZ, Rig, RigCamera
from dimos.perception.localize.types import Localization
from dimos.protocol.tf.tf import TF
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

INDEX_STREAM = "color_image_embedded"
GEOMETRY_STREAM = "geometry_memory"
COLOR_CODEC = "jpeg"
GEOMETRY_CODEC = "lz4+lcm"
LIDAR_TOLERANCE = 0.2  # s - an embedded frame to its lidar scan


class PerceptionLocalizeConfig(ModuleConfig):
    world_frame: str = "world"
    # what lifts a detection: the aligned depth_image, or the lidar
    geometry: Literal["depth", "lidar"] = "depth"
    horizon_s: float = 600.0
    # raw frames kept for the embed tail and the pairing
    feed_frames: int = 300
    # each a dandetect registry name or a module:attr path
    embedder: str = "siglip"
    detector: str = "owlv2"
    segmenter: str = "edgetam"
    # LocalizePolicy overrides on the models' shifts, tuned offline with tools/tool_localize
    policy: dict[str, Any] = Field(default_factory=dict)


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

    @rpc
    def start(self) -> None:
        super().start()
        self._stop = threading.Event()
        self._ready = threading.Event()
        self._camera_seen = threading.Event()
        self._camera: CameraInfo | None = None
        self._stage = "waiting for camera_info"
        self._groups: dict[str, Groups] = {}
        self._clouds: dict[str, PointCloud2] = {}  # by instance id, as last reported

        memory_frames = int(self.config.horizon_s * EMBED_HZ)
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
        ]
        self._thread = threading.Thread(target=self._warm, name="localize-warmup", daemon=True)
        self._thread.start()

    @rpc
    def stop(self) -> None:
        self._stop.set()
        self._ready.clear()
        self._thread.join(timeout=5.0)
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

    @skill
    def localize(
        self,
        objects: str | list[str],
        start: float = -10.0,
        duration: float = 10.0,
        policy: str = "",
    ) -> str:
        """Locate objects in a window of the robot's memory.

        ``objects`` is one label, several separated by commas, or a list of labels.
        Labels that describe how an object looks (colour and shape, e.g. "brown square
        object") match better than object names.

        ``start`` and ``duration`` are seconds and name the window. A positive
        ``start`` counts forward from the beginning of the feed; a negative one
        counts back from the newest frame, so the default reads the last ten
        seconds. What earlier calls proved is remembered and still answered.

        ``policy`` is optional: a JSON object of LocalizePolicy field overrides,
        e.g. '{"accept_score": 0.4, "verify_radius_m": 2.0}'. Leave it empty for the defaults.
        """
        if not self._ready.is_set():
            return f"localize cannot answer yet: {self._stage}. Poll state() until it reads ready."

        labels = objects.split(",") if isinstance(objects, str) else objects
        queries = [q.strip() for q in labels if q.strip()]
        try:
            overrides = json.loads(policy) if policy else {}
            if not isinstance(overrides, dict):
                raise ValueError
            tuning = self.detector.policy({**self.config.policy, **overrides})
        except (ValueError, TypeError):
            return (
                f"policy {policy!r} is not a JSON object of LocalizePolicy fields; "
                "pass e.g. '{\"min_views\": 1}', or leave it empty for the defaults."
            )
        first, head = self.index.get_time_range()
        lo = max(first, head + start if start < 0 else first + start)
        index = self.index.time_range(lo, lo + duration)
        rig = self.rig
        if rig.lidar is not None:
            rig = replace(rig, lidar=rig.lidar.time_range(lo, lo + duration))
        results = self.detector.localize(
            queries, index=index, rig=rig, policy=tuning, groups=self._groups
        )
        self.detections.publish(as_detection_array(queries, results, rig.world_frame))

        lines: list[str] = [
            f"window {lo - first:.1f}s to {lo + duration - first:.1f}s of "
            f"{head - first:.1f}s of feed, {index.count()} frames"
        ]
        for query, hits in zip(queries, results, strict=True):
            if not hits:
                line = f"no verified detection of {query!r}"
                near = best_candidate(self._groups.get(query), lo, lo + duration)
                if near is not None:
                    score, views, (x, y, z) = near
                    line += (
                        f"; best candidate score={score:.2f} views={views} at "
                        f"({x:.2f}, {y:.2f}, {z:.2f}), needs accept_score={tuning.accept_score:.2f} "
                        f"min_views={tuning.min_views}"
                    )
                lines.append(line)
            for hit in hits:
                self._clouds[hit.instance_id] = hit.point_cloud
                x, y, z = hit.position_world_xyz
                line = f"{query!r} at ({x:.2f}, {y:.2f}, {z:.2f}) in {hit.frame_id} "
                box = footprint(hit.point_cloud.as_numpy()[0])
                if box is not None:
                    (cx, cy), (length, width), yaw, (bottom, top) = box
                    line += (
                        f"footprint {length:.3f} x {width:.3f} m centred ({cx:.3f}, {cy:.3f}) "
                        f"long side yaw={yaw:.0f}deg, seen surface z {bottom:.3f} to {top:.3f} "
                    )
                lines.append(
                    line + f"score={hit.semantic_score:.2f} views={hit.n_views} "
                    f"id={hit.instance_id} seen {max(0.0, head - hit.last_seen_timestamp):.0f}s ago"
                )
        return "\n".join(lines)

    @rpc
    def object_cloud(self, object_id: str) -> PointCloud2 | None:
        """The world-frame point cloud of an object localize reported, by its id; None if no
        call has reported that id."""
        return self._clouds.get(object_id)


def footprint(
    points: np.ndarray,
) -> tuple[tuple[float, float], tuple[float, float], float, tuple[float, float]] | None:
    """Grasp geometry of a lifted cloud: the smallest rectangle around its XY points (centre,
    long and short side in metres, long side's yaw in degrees in [-90, 90)) and the z range of
    the surfaces the camera saw. The outer 2% each way is trimmed as depth noise. Faces the
    camera never saw are missing, so the bottom is only as low as what was in view."""
    if len(points) < 3:
        return None
    lo, hi = np.quantile(points, [0.02, 0.98], axis=0)
    kept = points[np.all((points >= lo) & (points <= hi), axis=1)]
    if len(kept) < 3:
        return None
    rect = cv2.minAreaRect(kept[:, :2].astype(np.float32))
    corners = cv2.boxPoints(rect)
    edges = [corners[1] - corners[0], corners[2] - corners[1]]
    lengths = [float(np.hypot(*e)) for e in edges]
    long_edge = edges[int(np.argmax(lengths))]
    yaw = (np.degrees(np.arctan2(long_edge[1], long_edge[0])) + 90.0) % 180.0 - 90.0
    centre = (float(rect[0][0]), float(rect[0][1]))
    return centre, (max(lengths), min(lengths)), float(yaw), (float(lo[2]), float(hi[2]))


def best_candidate(
    groups: Groups | None, t0: float, t1: float
) -> tuple[float, int, tuple[float, float, float]] | None:
    """The highest-scoring group a call left unverified, from its sightings in ``[t0, t1]``:
    best score, distinct views and newest position. None when nothing was lifted there."""
    if groups is None:
        return None
    best = None
    for i in range(len(groups.centers)):
        rows = groups.rows(i)
        rows = rows[(rows[:, M_TS] >= t0) & (rows[:, M_TS] <= t1)]
        if not len(rows):
            continue
        score = float(rows[:, M_SCORE].max())
        if best is None or score > best[0]:
            views = len(np.unique(np.round(rows[:, M_KX : M_KZ + 1], 2), axis=0))
            newest = rows[rows[:, M_TS].argmax(), M_CX : M_CZ + 1]
            best = (score, views, (float(newest[0]), float(newest[1]), float(newest[2])))
    return best


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
