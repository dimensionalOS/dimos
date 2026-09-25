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

"""Live object memory over a robot's ports.

The module consumes colour, depth, ``camera_info`` and ``tf`` ports, keeps a
bounded in-memory memory, embeds the colour feed as it arrives, and answers
``localize`` from what it has embedded. It opens no file; recording is the
Recorder's job. It does not know whether the ports carry a robot or a replay.

Memory layout: the raw colour and depth feeds are kept for a few hundred
frames, only as long as the embed tail and the depth pairing need them. The
memory that ``localize`` reads is at index rate: the embedded frames, stored
as JPEG, and the depth frame paired with each of them, stored lz4. Both roll
over ``horizon_s`` seconds. The tf buffer holds the same horizon.
"""

from __future__ import annotations

from collections.abc import Iterator
from dataclasses import replace
import json
import math
import threading
from typing import Any, cast

from dimos_lcm.geometry_msgs import Pose
from dimos_lcm.vision_msgs import BoundingBox3D, ObjectHypothesis, ObjectHypothesisWithPose
import numpy as np
from pydantic import Field
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
from dimos.perception.localize.localize import Groups
from dimos.perception.localize.rig import DEPTH_TOLERANCE, EMBED_HZ, WALK_EMBED_HZ, Rig
from dimos.perception.localize.spec import LocalizationSpec
from dimos.perception.localize.types import Localization
from dimos.protocol.tf.tf import TF
from dimos.types.timestamped import align_timestamped
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

INDEX_STREAM = "color_image_embedded"
DEPTH_STREAM = "depth_memory"
COLOR_CODEC = "jpeg"
DEPTH_CODEC = "lz4+lcm"


class LiveLocalizeModuleConfig(ModuleConfig):
    world_frame: str = "world"
    # Empty selects the first calibration. Robot blueprints select their camera explicitly.
    optical_frame: str = ""
    mobile: bool = False
    horizon_s: float = Field(default=600.0, gt=0.0)
    # raw frames kept for the embed tail and the depth pairing
    feed_frames: int = Field(default=300, ge=1)
    tf_tolerance: float = Field(default=0.12, ge=0.0)
    policy: dict[str, Any] = Field(default_factory=dict)


class LiveLocalizeModule(Module, LocalizationSpec):
    """Embed the colour feed, pair depth to it, answer ``localize`` from memory.

    ``camera_info`` must be stamped in the colour frame, and ``tf`` must reach
    that frame from ``world_frame`` at every colour timestamp; frames without a
    pose are never embedded.
    """

    config: LiveLocalizeModuleConfig

    color_image: In[Image]
    depth_image: In[Image]
    camera_info: In[CameraInfo]
    tf: In[TFMessage]

    detections: Out[Detection3DArray]
    hit_points: Out[PointCloud2]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._stop = threading.Event()
        self._ready = threading.Event()
        self._camera_seen = threading.Event()
        self._camera: CameraInfo | None = None
        self._stage = "not started"
        self._groups: dict[str, Groups] = {}
        self._inference_lock = threading.RLock()
        self._thread: threading.Thread | None = None

    @rpc
    def start(self) -> None:
        super().start()
        self._stop.clear()
        self._ready.clear()
        self._camera_seen.clear()
        self._camera = None
        self._stage = "waiting for camera_info"
        self._groups.clear()

        embed_hz = WALK_EMBED_HZ if self.config.mobile else EMBED_HZ
        memory_frames = max(1, math.ceil(self.config.horizon_s * embed_hz))
        feed_frames = self.config.feed_frames
        # Stop model/pairing workers before closing the streams they consume.
        self._workers = self.register_disposable(CompositeDisposable())
        self._memory = self.register_disposable(MemoryStore())
        self._color_feed = self._memory.stream(
            "color_feed",
            Image,
            observation_store=ListObservationStore(name="color_feed", max_size=feed_frames),
        )
        self._depth_feed = self._memory.stream(
            "depth_feed",
            Image,
            observation_store=ListObservationStore(name="depth_feed", max_size=feed_frames),
        )
        self.index = self._memory.stream(
            INDEX_STREAM,
            Image,
            codec=COLOR_CODEC,
            blob_store=MemoryBlobStore(max_items=memory_frames),
            observation_store=ListObservationStore(name=INDEX_STREAM, max_size=memory_frames),
        )
        self._depth_memory = self._memory.stream(
            DEPTH_STREAM,
            Image,
            codec=DEPTH_CODEC,
            blob_store=MemoryBlobStore(max_items=memory_frames),
            observation_store=ListObservationStore(name=DEPTH_STREAM, max_size=memory_frames),
        )
        self._tf_buffer = TF(self.tf, buffer_size=self.config.horizon_s)
        self.register_disposable(Disposable(self._tf_buffer.dispose))
        self.register_disposable(Disposable(self.camera_info.subscribe(self._on_camera_info)))
        self.register_disposable(
            align_timestamped(
                self.color_image.observable(),
                self.depth_image.observable(),
                buffer_size=2.0,
                match_tolerance=DEPTH_TOLERANCE,
            ).subscribe(self._on_frames)
        )
        self._thread = threading.Thread(target=self._warm, name="localize-warmup", daemon=True)
        self._thread.start()

    @rpc
    def stop(self) -> None:
        self._stop.set()
        self._ready.clear()
        if self._thread is not None:
            self._thread.join()
            self._thread = None
        with self._inference_lock:
            self._groups.clear()
            self._stage = "stopped"
            super().stop()

    def _on_camera_info(self, info: CameraInfo) -> None:
        if self.config.optical_frame and info.frame_id != self.config.optical_frame:
            return
        if self._camera is None:
            self._camera = info
            self._camera_seen.set()

    def _on_frames(self, frames: tuple[Image, ...]) -> None:
        color, depth = frames
        camera = self._camera
        if self._stop.is_set() or camera is None or color.frame_id != camera.frame_id:
            return
        # Inputs must be depth registered to the selected colour camera. Save
        # the pair before embedding can consume colour, keyed by colour time.
        self._depth_feed.append(depth, ts=color.ts)
        self._color_feed.append(color, ts=color.ts)

    def _pair_depth(self, upstream: Iterator[Any]) -> Iterator[Any]:
        """Move the depth frame of each embedded frame from the feed into memory."""
        for obs in upstream:
            try:
                depth = self._depth_feed.at(obs.ts, 0.0).first()
            except LookupError:
                yield obs
                continue
            self._depth_memory.append(depth.data, ts=obs.ts)
            yield obs

    def _warm(self) -> None:
        try:
            self._initialize()
        except Exception as exc:
            self._stage = f"initialization failed: {exc}"
            logger.exception("localize initialization failed")

    def _initialize(self) -> None:
        self._stage = "loading SigLIP, OWLv2 and EdgeTAM weights"
        logger.info(f"localize: {self._stage}")
        self.detector = DanDetector()
        self._workers.add(self.detector)
        self.detector.start()
        if self._stop.is_set():
            return

        self._stage = "waiting for camera_info"
        logger.info(f"localize: {self._stage}")
        while not self._camera_seen.wait(0.5):
            if self._stop.is_set():
                return
        camera = self._camera
        assert camera is not None
        self.rig = Rig(
            cameras={camera.frame_id: camera},
            color=self.index,
            world_frame=self.config.world_frame,
            tf=self._tf_buffer,
            depth=self._depth_memory,
            embed_hz=WALK_EMBED_HZ if self.config.mobile else EMBED_HZ,
            mobile=self.config.mobile,
            tf_tolerance=self.config.tf_tolerance,
        )
        # Subscribe first: the first embedded frame must retain its paired depth.
        self._workers.add(self.index.live().transform(self._pair_depth).drain_thread())
        self.detector.embed_live(self._memory, rig=self.rig, source=self._color_feed)

        self._stage = "waiting for the first posed frame of the feed"
        logger.info(f"localize: {self._stage}")
        while self._depth_memory.count() == 0:
            if self._stop.wait(0.1):
                return

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
    def localize_objects(
        self,
        prompts: list[str],
        start: float = -10.0,
        duration: float = 10.0,
        policy: str = "",
        max_age: float | None = None,
    ) -> list[list[Localization]]:
        """All verified instances per prompt, with cumulative evidence and exact clouds.

        The window selects new evidence; previously verified objects remain
        answerable. max_age optionally filters last-seen time against the newest
        RGB observation (the sensor clock, also valid for replay).
        """
        if not prompts or any(not p or p.strip() != p for p in prompts):
            raise ValueError("Provide non-empty, trimmed object prompts")
        if len(set(prompts)) != len(prompts):
            raise ValueError("Object prompts must be unique")
        if not math.isfinite(start) or not math.isfinite(duration) or duration <= 0:
            raise ValueError("Window start must be finite and duration must be positive")
        if max_age is not None and (not math.isfinite(max_age) or max_age < 0):
            raise ValueError("max_age must be finite and non-negative")
        with self._inference_lock:
            if not self._ready.is_set():
                raise RuntimeError(f"localize cannot answer yet: {self._stage}. Poll state().")
            overrides = json.loads(policy) if policy else {}
            if not isinstance(overrides, dict):
                raise ValueError("policy must be a JSON object")
            try:
                tuning = replace(
                    self.rig.default_localize_policy(), **(self.config.policy | overrides)
                )
            except TypeError as exc:
                raise ValueError(f"Invalid localization policy: {exc}") from exc
            first, head = self.index.get_time_range()
            lo = max(first, head + start if start < 0 else first + start)
            index = self.index.time_range(lo, lo + duration).materialize()
            results = cast(
                "list[list[Localization]]",
                self.detector.localize(
                    self._memory,
                    prompts,
                    index=index,
                    rig=self.rig,
                    policy=tuning,
                    groups=self._groups,
                    require_pose=True,
                ),
            )
            if len(results) != len(prompts):
                raise RuntimeError("Detector returned an invalid batch result")
            newest = self._color_feed.last().ts
            filtered: list[list[Localization]] = []
            for hits in results:
                accepted = []
                for hit in hits:
                    if hit.point_cloud is None or hit.position_world_xyz is None:
                        raise RuntimeError("Localization is missing its geometry")
                    if (
                        hit.frame_id != self.config.world_frame
                        or hit.point_cloud.frame_id != self.config.world_frame
                    ):
                        raise RuntimeError("Localization frame mismatch")
                    if max_age is None or newest - hit.last_seen_timestamp <= max_age:
                        accepted.append(hit)
                filtered.append(accepted)
            self.detections.publish(as_detection_array(prompts, filtered, self.rig.world_frame))
            self.hit_points.publish(as_cloud(filtered, self.rig.world_frame))
            return filtered

    @skill
    def localize(
        self,
        objects: str,
        start: float = -10.0,
        duration: float = 10.0,
        policy: str = "",
        max_age: float | None = None,
    ) -> str:
        """Locate objects in a window of the robot's memory.

        ``objects`` is one label, or several separated by commas.

        ``start`` and ``duration`` are seconds and name the window. A positive
        ``start`` counts forward from the beginning of the feed; a negative one
        counts back from the newest frame, so the default reads the last ten
        seconds. What earlier calls proved is remembered and still answered.

        ``policy`` is a JSON object of LocalizePolicy field overrides,
        e.g. '{"accept_score": 0.4, "verify_radius_m": 2.0}'.
        ``max_age`` optionally limits last-seen age in seconds on the sensor clock.
        For an eye-in-hand camera, move the arm and let it settle at different
        viewpoints to build evidence; the default requires two camera positions.
        """
        if not self._ready.is_set():
            return f"localize cannot answer yet: {self._stage}. Poll state() until it reads ready."

        queries = [q.strip() for q in objects.split(",") if q.strip()]
        try:
            results = self.localize_objects(queries, start, duration, policy, max_age)
        except (ValueError, RuntimeError) as exc:
            return str(exc)
        lines: list[str] = []
        for query, hits in zip(queries, results, strict=True):
            if not hits:
                lines.append(f"no verified detection of {query!r}")
            for hit in hits:
                assert hit.position_world_xyz is not None
                x, y, z = hit.position_world_xyz
                lines.append(
                    f"{query!r} at ({x:.2f}, {y:.2f}, {z:.2f}) in {hit.frame_id} "
                    f"score={hit.semantic_score:.2f} views={hit.n_views} "
                    f"last_seen={hit.last_seen_timestamp:.3f} "
                    f"ambiguity_margin={hit.ambiguity_margin:.2f} reason={hit.reason}"
                )
        return "\n".join(lines)


def as_cloud(results: list[list[Localization]], frame_id: str) -> PointCloud2:
    """Visualize fused geometry without retaining per-sighting images in traces."""
    clouds = [hit.point_cloud for hits in results for hit in hits if hit.point_cloud is not None]
    points = np.vstack([cloud.points_f32() for cloud in clouds]) if clouds else np.empty((0, 3))
    return PointCloud2.from_numpy(
        points, frame_id=frame_id, timestamp=max((cloud.ts for cloud in clouds), default=0.0)
    )


def as_detection_array(
    queries: list[str], results: list[list[Localization]], frame_id: str
) -> Detection3DArray:
    """One labelled box per verified instance, for the rerun bridge."""
    boxes = []
    latest = 0.0
    for query, hits in zip(queries, results, strict=True):
        for instance, hit in enumerate(hits):
            if hit.point_cloud is None:
                continue
            points = hit.point_cloud.as_numpy()[0]
            low, high = points.min(axis=0), points.max(axis=0)
            middle, extent = (low + high) / 2, high - low
            center = Vector3(*(float(v) for v in middle))
            size = Vector3(*(max(float(v), 1e-3) for v in extent))
            latest = max(latest, hit.last_seen_timestamp)
            boxes.append(
                Detection3D(
                    header=Header(hit.last_seen_timestamp, frame_id),
                    id=f"{query}:{instance}",
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
