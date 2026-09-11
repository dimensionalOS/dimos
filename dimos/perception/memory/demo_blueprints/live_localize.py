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
import threading
import time
from typing import Any

from dimos_lcm.geometry_msgs import Pose
from dimos_lcm.vision_msgs import BoundingBox3D, ObjectHypothesis, ObjectHypothesisWithPose
import numpy as np

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
from dimos.perception.detection.type.detection3d.pointcloud import Detection3DPC
from dimos.perception.memory.dandetect import DanDetector
from dimos.perception.memory.localize import Groups, LocalizeTrace
from dimos.perception.memory.rig import DEPTH_TOLERANCE, EMBED_HZ, WALK_EMBED_HZ, Rig
from dimos.protocol.tf.tf import TF
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

INDEX_STREAM = "color_image_embedded"
DEPTH_STREAM = "depth_memory"
COLOR_CODEC = "jpeg"
DEPTH_CODEC = "lz4+lcm"


class LiveLocalizeModuleConfig(ModuleConfig):
    world_frame: str = "world"
    mobile: bool = False
    horizon_s: float = 600.0
    # raw frames kept for the embed tail and the depth pairing
    feed_frames: int = 300


class LiveLocalizeModule(Module):
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

    @rpc
    def start(self) -> None:
        super().start()
        self._stop = threading.Event()
        self._ready = threading.Event()
        self._camera_seen = threading.Event()
        self._camera: CameraInfo | None = None
        self._stage = "waiting for camera_info"
        self._groups: dict[str, Groups] = {}

        embed_hz = WALK_EMBED_HZ if self.config.mobile else EMBED_HZ
        memory_frames = int(self.config.horizon_s * embed_hz)
        feed_frames = self.config.feed_frames
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

        def on_camera_info(info: CameraInfo) -> None:
            if self._camera is None:
                self._camera = info
                self._camera_seen.set()

        self._unsubs = [
            self.color_image.subscribe(lambda img: self._color_feed.append(img, ts=img.ts)),
            self.depth_image.subscribe(lambda img: self._depth_feed.append(img, ts=img.ts)),
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

    def _pair_depth(self, upstream: Iterator[Any]) -> Iterator[Any]:
        """Move the depth frame of each embedded frame from the feed into memory."""
        for obs in upstream:
            try:
                depth = self._depth_feed.at(obs.ts, DEPTH_TOLERANCE).first()
            except LookupError:
                yield obs
                continue
            self._depth_memory.append(depth.data, ts=depth.ts)
            yield obs

    def _warm(self) -> None:
        self._stage = "loading SigLIP, OWLv2 and EdgeTAM weights"
        logger.info(f"localize: {self._stage}")
        self.detector = self.register_disposable(DanDetector())
        self.detector.start()

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
        )
        self.detector.embed_live(self._memory, rig=self.rig, source=self._color_feed)
        self.register_disposable(self.index.live().transform(self._pair_depth).drain_thread())

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
        self, objects: str, start: float = -10.0, duration: float = 10.0, policy: str = ""
    ) -> str:
        """Locate objects in a window of the robot's memory.

        ``objects`` is one label, or several separated by commas.

        ``start`` and ``duration`` are seconds and name the window. A positive
        ``start`` counts forward from the beginning of the feed; a negative one
        counts back from the newest frame, so the default reads the last ten
        seconds. What earlier calls proved is remembered and still answered.

        ``policy`` is a JSON object of LocalizePolicy field overrides,
        e.g. '{"accept_score": 0.4, "verify_radius_m": 2.0}'.
        """
        if not self._ready.is_set():
            return f"localize cannot answer yet: {self._stage}. Poll state() until it reads ready."

        queries = [q.strip() for q in objects.split(",") if q.strip()]
        first, head = self.index.get_time_range()
        lo = max(first, head + start if start < 0 else first + start)
        index = self.index.time_range(lo, lo + duration)
        tuning = self.rig.default_localize_policy()
        if policy:
            tuning = replace(tuning, **json.loads(policy))
        traces = [LocalizeTrace() for _ in queries]
        results: Any = self.detector.localize(
            self._memory,
            queries,
            index=index,
            rig=self.rig,
            policy=tuning,
            groups=self._groups,
            trace=traces,
        )
        self.detections.publish(as_detection_array(queries, results, self.rig.world_frame))
        camera = self.rig.cameras[self.rig.optical_frame]
        self.hit_points.publish(as_textured_cloud(traces, camera, self.rig.world_frame))

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
                    f"score={hit.semantic_score:.2f} views={hit.n_views}"
                )
        return "\n".join(lines)


def _image_colors(det: Detection3DPC, camera: CameraInfo) -> np.ndarray:
    """Each cloud point's colour sampled from the sighting's own image at its reprojection."""
    points = det.pointcloud.points_f32()
    matrix = det.transform.to_matrix()
    in_camera = points @ matrix[:3, :3].T + matrix[:3, 3]
    pixels = Detection3DPC.project_pixels(in_camera, camera)
    cols = np.round(pixels[:, 0]).astype(int)
    rows = np.round(pixels[:, 1]).astype(int)
    rgb = det.image.to_rgb().data
    height, width = rgb.shape[:2]
    sampled: np.ndarray = rgb[np.clip(rows, 0, height - 1), np.clip(cols, 0, width - 1)]
    return sampled.astype(np.float32) / 255.0


def as_textured_cloud(
    traces: list[LocalizeTrace], camera: CameraInfo, frame_id: str
) -> PointCloud2:
    """Every verified instance's sightings as one cloud, each point wearing its own image's pixel."""
    import open3d as o3d
    import open3d.core as o3c

    members = [det for trace in traces for answer in trace.answers for det in answer]
    pcd = o3d.t.geometry.PointCloud()
    if members:
        positions = np.vstack([det.pointcloud.points_f32() for det in members])
        colors = np.vstack([_image_colors(det, camera) for det in members])
        pcd.point["positions"] = o3c.Tensor(positions, dtype=o3c.float32)
        pcd.point["colors"] = o3c.Tensor(colors, dtype=o3c.float32)
        latest = max(det.ts for det in members)
    else:
        pcd.point["positions"] = o3c.Tensor(np.zeros((0, 3), dtype=np.float32), dtype=o3c.float32)
        latest = 0.0
    return PointCloud2(pointcloud=pcd, frame_id=frame_id, ts=latest)


def as_detection_array(queries: list[str], results: list[Any], frame_id: str) -> Detection3DArray:
    """One labelled box per verified instance, for the rerun bridge."""
    boxes = []
    latest = 0.0
    for query, hits in zip(queries, results, strict=True):
        for hit in hits:
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
