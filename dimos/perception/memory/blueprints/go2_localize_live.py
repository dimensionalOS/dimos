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

"""A go2 recording looped as a live feed, with localize behind an agent skill.

This is the deployment shape of the perception memory stack: one module owns
a store and the models, ``DanDetector.embed(live=True)`` keeps a background
tail filling the index while the robot runs, and a ``@skill`` answers
``localize`` calls against whatever has been embedded so far.

:class:`LoopFeeder` stands in for the sensors and the recorder. It replays a
recording into a fresh store forever, adding the recording's own span to
every timestamp each lap, so the store's clock only moves forward while the
scene repeats. The message stamp, the observation stamp and every tf
transform shift by the same lap offset, so pose interpolation, cloud
accumulation and the index window all see one continuous take. It writes the
tf chain and the camera intrinsics too, which is what lets
:class:`LocalizeModule` resolve its rig from the live store with no manifest.

The canonical recording is opened read-only and never written.
"""

from __future__ import annotations

from dataclasses import replace
import heapq
import json
from pathlib import Path
import threading
import time
from typing import TYPE_CHECKING, Any
import zlib

from dimos_lcm.geometry_msgs import Pose
from dimos_lcm.vision_msgs import BoundingBox3D, ObjectHypothesis, ObjectHypothesisWithPose
import numpy as np

from dimos.agents.annotation import skill
from dimos.agents.mcp.mcp_server import McpServer
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.core import rpc
from dimos.core.global_config import global_config
from dimos.core.stream import Out
from dimos.mapping.voxels.module import VoxelGridMapper
from dimos.memory.module import MemoryModule, MemoryModuleConfig
from dimos.memory.replay import resolve_db_path
from dimos.memory.store.sqlite import SqliteStore
from dimos.memory.tf import StreamTF
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.std_msgs.Header import Header
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.msgs.vision_msgs.Detection3D import Detection3D
from dimos.msgs.vision_msgs.Detection3DArray import Detection3DArray
from dimos.perception.detection.type.detection3d.pointcloud import (
    Detection3DPC,
    lattice_quantum,
)
from dimos.perception.memory.dandetect import DanDetector
from dimos.perception.memory.identity_store import IdentityStore
from dimos.perception.memory.rig import Rig
from dimos.robot.unitree.go2.connection import BASE_TO_OPTICAL, GO2Connection
from dimos.utils.logging_config import setup_logger
from dimos.visualization.vis_module import vis_module

if TYPE_CHECKING:
    from collections.abc import Iterator

logger = setup_logger()

LAP_GAP_S = 0.2  # dead time between laps so consecutive laps stay disjoint
WARMUP_WINDOW_S = 5.0  # feed the warmup query a short window, only to load weights
UNSEEN_GREY = 0.25  # voxels outside the camera frustum carry no pixel
# The map is drawn achromatic so anything coloured - the camera texture, a
# detection box - reads as foreground against it. Lightness carries height,
# dark at the floor, the way shaded relief is read.
MAP_INK = (45, 215)
# Cube edge as a fraction of the source's own lattice pitch. Full pitch
# tiles into a solid block that hides the shape behind it; a gap between
# cubes reads as structure while the voxel count stays the same.
CUBE_FILL = 0.55


class LoopFeederConfig(MemoryModuleConfig):
    dataset: str = "go2_short"
    db_path: str | Path = "recording_go2_live.db"


class LoopFeeder(MemoryModule):
    """Replay ``dataset`` into this module's store forever, one lap at a time.

    Roles are read straight off the recording; tf and camera_info are derived
    the way :class:`GO2Connection` derives them live, from each odom pose and
    the static front-camera calibration.
    """

    config: LoopFeederConfig

    color_image: Out[Image]
    lidar: Out[PointCloud2]
    odom: Out[PoseStamped]
    camera_info: Out[CameraInfo]
    tf: Out[TFMessage]

    ROLES = ("color_image", "lidar", "odom", "tf", "camera_info", "color_image_embedded")

    @rpc
    def start(self) -> None:
        super().start()
        # Drop what a previous run left, in place: the localize module holds
        # the same file open, so the streams go rather than the inode.
        for name in self.ROLES:
            self.store.delete_stream(name)

        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._feed, name="loop-feeder", daemon=True)
        self._thread.start()

    @rpc
    def stop(self) -> None:
        self._stop.set()
        self._thread.join(timeout=5.0)
        super().stop()

    @staticmethod
    def _stamped(stream: Any, name: str) -> Iterator[tuple[float, str, Any]]:
        """The stream's observations tagged with their role, for the lap merge."""
        for obs in stream:
            yield (obs.ts, name, obs)

    def _feed(self) -> None:
        source = SqliteStore(path=str(resolve_db_path(self.config.dataset)), must_exist=True)
        source.start()
        lo, hi = source.stream("color_image").get_time_range()
        span = (hi - lo) + LAP_GAP_S

        live = self.store
        targets: dict[str, Any] = {
            name: live.stream(name, source.stream(name).data_type)
            for name in ("color_image", "lidar", "odom")
        }
        tf_stream = live.stream("tf", TFMessage)
        intrinsics = GO2Connection.camera_info_static
        live.stream("camera_info", CameraInfo).append(intrinsics, ts=lo, pose=None)

        ports: dict[str, Any] = {
            "color_image": self.color_image,
            "lidar": self.lidar,
            "odom": self.odom,
        }

        # The embed tail runs here, in the process that writes the frames.
        # SubjectNotifier fans out in-process only, so a tail subscribed from
        # another worker backfills once and then never sees another append.
        self._embedder = self.register_disposable(DanDetector())
        self._embedder.start()
        self._embedder.embed(live, live=True, rig=_live_rig(Rig.from_store(source), live))
        logger.info(
            f"loop feeder: {self.config.dataset} ({span - LAP_GAP_S:.1f}s) -> {self.config.db_path}"
        )

        lap = 0
        while not self._stop.is_set():
            offset = lap * span
            wall_start = time.time()
            self.camera_info.publish(intrinsics)
            schedule: Any = heapq.merge(
                *(self._stamped(source.stream(name), name) for name in targets),
                key=lambda item: item[0],
            )
            image: Image | None = None
            base: PoseStamped | None = None
            for ts, name, obs in schedule:
                if self._stop.is_set():
                    return
                self._stop.wait(max(0.0, (ts - lo) - (time.time() - wall_start)))
                data = obs.data
                data.ts = ts + offset
                if name == "odom":
                    data.frame_id = "world"
                    base = data
                    transforms = GO2Connection._odom_to_tf(data)
                    tf_stream.append(TFMessage(*transforms), ts=data.ts, pose=None)
                    self.tf.publish(TFMessage(*transforms))
                elif name == "color_image":
                    image = data
                targets[name].append(data, ts=data.ts, pose=obs.pose)
                # The store keeps the raw scan; only the viewer sees the texture.
                if name == "lidar" and image is not None and base is not None:
                    self.lidar.publish(_textured(data, image, base))
                else:
                    ports[name].publish(data)
            lap += 1
            logger.info(f"loop feeder: lap {lap} done, feed clock at +{lap * span:.0f}s")


def _textured(cloud: PointCloud2, image: Image, pose: PoseStamped) -> PointCloud2:
    """The scan re-emitted with every voxel carrying the pixel it projects onto.

    The scan is already in the world frame, so it goes through the camera the
    way a detection's cloud does: world to optical through the base pose and
    the static mount, then the camera's own distortion model. Voxels behind
    the camera or outside the image keep a neutral grey.
    """
    import open3d as o3d
    import open3d.core as o3c

    points = cloud.points_f32()
    matrix = (-(Transform.from_pose("base_link", pose) + BASE_TO_OPTICAL)).to_matrix()
    camera = points @ matrix[:3, :3].T + matrix[:3, 3]
    rgb = image.to_rgb().data
    height, width = rgb.shape[:2]

    colors = np.full((len(points), 3), UNSEEN_GREY, dtype=np.float32)
    ahead = np.flatnonzero(camera[:, 2] > 0)
    if len(ahead):
        pixels = Detection3DPC.project_pixels(camera[ahead], GO2Connection.camera_info_static)
        cols = np.round(pixels[:, 0]).astype(int)
        rows = np.round(pixels[:, 1]).astype(int)
        inside = (cols >= 0) & (cols < width) & (rows >= 0) & (rows < height)
        colors[ahead[inside]] = rgb[rows[inside], cols[inside]] / 255.0

    pcd = o3d.t.geometry.PointCloud()
    pcd.point["positions"] = o3c.Tensor(points, dtype=o3c.float32)
    pcd.point["colors"] = o3c.Tensor(colors, dtype=o3c.float32)
    return PointCloud2(pointcloud=pcd, frame_id=cloud.frame_id, ts=cloud.ts)


class LocalizeModuleConfig(MemoryModuleConfig):
    dataset: str = "go2_short"
    db_path: str | Path = "recording_go2_live.db"
    warmup_query: str = "zijnh"


def _live_rig(source: Rig, live: Any) -> Rig:
    """The recording's rig, reading the live store's streams instead.

    A robot knows its rig from calibration; it does not rediscover it at
    runtime. Taking the shape from the recording is also what lets startup
    skip waiting for enough recorded motion for the mobile gate to settle,
    and carries the measured color delay over instead of re-estimating it.
    """
    return Rig(
        camera_info=source.camera_info,
        color=live.stream("color_image", source.color.data_type),
        world_frame=source.world_frame,
        optical_frame=source.optical_frame,
        tf=StreamTF(live.stream("tf", TFMessage)) if source.tf is not None else None,
        base_to_optical=source.base_to_optical,
        poses=live.stream("odom", source.poses.data_type) if source.poses is not None else None,
        cloud=live.stream("lidar", source.cloud.data_type) if source.cloud is not None else None,
        tf_tolerance=source.tf_tolerance,
        cloud_accum_s=source.cloud_accum_s,
        speed_max=source.speed_max,
        color_delay=source.color_delay,
        scene_gate=source.scene_gate,
        embed_hz=source.embed_hz,
        mobile=source.mobile,
    )


class LocalizeModule(MemoryModule):
    """Live object memory: a background embed tail plus a localize skill.

    Startup loads every weight against the recording itself, so it overlaps
    the feed filling instead of following it, then opens the live tail and
    answers as soon as the first frames are embedded. ``state`` reports where
    it is; ``localize`` names the same stage when asked too early.

    Answers are published as a :class:`Detection3DArray`, which the rerun
    bridge draws as labelled boxes. Nothing is written back to the recording.
    """

    config: LocalizeModuleConfig

    detections: Out[Detection3DArray]

    @rpc
    def start(self) -> None:
        super().start()
        self._ready = threading.Event()
        self._stage = "starting"
        self._identity = IdentityStore()
        self._thread = threading.Thread(target=self._warm, name="localize-warmup", daemon=True)
        self._thread.start()

    @rpc
    def stop(self) -> None:
        self._ready.clear()
        super().stop()

    def _warm(self) -> None:
        source = SqliteStore(path=str(resolve_db_path(self.config.dataset)), must_exist=True)
        source.start()

        self._stage = "loading SigLIP, OWLv2 and EdgeTAM weights"
        logger.info(f"localize: {self._stage}")
        recorded = Rig.from_store(source)
        self.detector = self.register_disposable(DanDetector())
        self.detector.start()

        # Every model loads here, on this thread, against the recording, so
        # none of it waits on the feed. DanDetector.start() only constructs
        # the wrappers; the models themselves arrive on first use, and the
        # tail would otherwise construct SigLIP at the same instant as the
        # first query, leaving one of the two threads a half-loaded copy.
        lo, _ = recorded.color.get_time_range()
        warm = self.detector.embed(source, lo, lo + WARMUP_WINDOW_S, rig=recorded)
        self.detector.localize(source, self.config.warmup_query, index=warm, rig=recorded)

        self._stage = "waiting for the first frames of feed"
        logger.info(f"localize: {self._stage}")
        self.rig = _live_rig(recorded, self.store)
        # The feeder's process owns the embed tail; this one only reads it.
        self.index = self.store.stream("color_image_embedded", Image)
        while self.index.count() == 0:
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

        ``start`` and ``duration`` are seconds and name the window, the way
        ``--from`` and ``--duration`` do on tool_localize. A positive
        ``start`` counts forward from the beginning of the feed; a negative
        one counts back from the newest frame, like a negative Python index,
        so the default reads the last ten seconds. The window is what this
        call examines; what earlier calls proved is remembered and still
        answered, so coverage accumulates while the cost per call does not.

        ``policy`` is a JSON object of LocalizePolicy field overrides,
        e.g. '{"accept_score": 0.4, "verify_radius_m": 2.0}'.
        """
        if not self._ready.is_set():
            return f"localize cannot answer yet: {self._stage}. Poll state() until it reads ready."

        queries = [q.strip() for q in objects.split(",") if q.strip()]
        first, head = self.index.get_time_range()
        # A window reaching past the start of the feed begins at the start.
        lo = max(first, head + start if start < 0 else first + start)
        index = self.index.time_range(lo, lo + duration)
        tuning = self.rig.default_localize_policy()
        if policy:
            tuning = replace(tuning, **json.loads(policy))
        results: Any = self.detector.localize(
            self.store,
            queries,
            index=index,
            rig=self.rig,
            policy=tuning,
            identity_store=self._identity,
        )
        self.detections.publish(_as_detection_array(queries, results, self.rig.world_frame))

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


def _as_detection_array(queries: list[str], results: list[Any], frame_id: str) -> Detection3DArray:
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


def _static_robot_body(rr: Any) -> list[Any]:
    return [
        rr.Boxes3D(half_sizes=[0.35, 0.155, 0.2], colors=[(0, 255, 127)]),
        rr.Transform3D(parent_frame="tf#/base_link"),
    ]


def _cube_size(points: np.ndarray) -> dict[str, float]:
    """``voxel_size`` for a grid source, or nothing for a continuous one."""
    quantum = lattice_quantum(points)
    return {"voxel_size": quantum * CUBE_FILL} if quantum else {}


def _convert_global_map(grid: Any) -> Any:
    """The accumulated lidar as cubes shaded dark-to-light by height.

    Whole map, floor included - a height cutoff at the world origin would
    drop it, since the world frame is anchored at the robot's start pose and
    the ground sits below it. Height is normalized over the map's own extent,
    which only grows, so the ramp settles as the map fills.
    """
    points = grid.points_f32()
    if not len(points):
        return grid.to_rerun(mode="boxes")
    z = points[:, 2]
    low, high = MAP_INK
    lightness = low + (z - z.min()) / (z.max() - z.min() + 1e-8) * (high - low)
    grey = np.repeat(lightness.astype(np.uint8)[:, None], 3, axis=1)
    return grid.to_rerun(mode="boxes", colors=grey, **_cube_size(points))


def _label_colour(label: str) -> tuple[int, int, int]:
    """A saturated colour that follows the label, not the call.

    Derived from the text so one label keeps its colour across queries and
    windows; two labels in view are two colours without reading either.
    """
    import colorsys

    hue = (zlib.crc32(label.encode()) % 3600) / 3600.0
    return tuple(int(c * 255) for c in colorsys.hsv_to_rgb(hue, 0.75, 1.0))  # type: ignore[return-value]


def _detection_entities(array: Any) -> Any:
    """One rerun entity per label, so each owns a row on the timeline."""
    import rerun as rr

    grouped: dict[str, list[Any]] = {}
    for detection in array.detections[: array.detections_length]:
        label = str(detection.results[0].hypothesis.class_id)
        grouped.setdefault(label, []).append(detection)

    entities = []
    for label, members in grouped.items():
        boxes = [d.bbox for d in members]
        entities.append(
            (
                f"world/detections/{label.replace(' ', '_')}",
                rr.Boxes3D(
                    centers=[
                        (b.center.position.x, b.center.position.y, b.center.position.z)
                        for b in boxes
                    ],
                    half_sizes=[(b.size.x / 2, b.size.y / 2, b.size.z / 2) for b in boxes],
                    labels=[label] * len(boxes),
                    colors=[_label_colour(label)] * len(boxes),
                ),
            )
        )
    return entities


def _lidar_cubes(cloud: Any) -> Any:
    """The lidar lattice as solid voxel cubes wearing the camera's pixels."""
    _, colors = cloud.as_numpy()
    rgb = (colors * 255).astype(np.uint8) if colors is not None else None
    return cloud.to_rerun(mode="boxes", colors=rgb, **_cube_size(cloud.points_f32()))


def _convert_camera_info(camera_info: Any) -> Any:
    return camera_info.to_rerun(
        image_topic="/world/color_image",
        optical_frame="camera_optical",
    )


def _rerun_blueprint() -> Any:
    """Camera feed, 3D world, and the localize answers on top of both."""
    import rerun as rr
    import rerun.blueprint as rrb

    return rrb.Blueprint(
        rrb.Horizontal(
            rrb.Spatial2DView(origin="world/color_image", name="Camera"),
            rrb.Spatial3DView(
                origin="world",
                name="3D",
                background=rrb.Background(kind="SolidColor", color=[0, 0, 0]),
                line_grid=rrb.LineGrid3D(plane=rr.components.Plane3D.XY.with_distance(0.5)),
            ),
            column_shares=[1, 2],
        ),
        rrb.TimePanel(state="expanded"),
        rrb.SelectionPanel(state="hidden"),
    )


rerun_config: dict[str, Any] = {
    "blueprint": _rerun_blueprint,
    "visual_override": {
        "world/camera_info": _convert_camera_info,
        "world/lidar": _lidar_cubes,
        "world/global_map": _convert_global_map,
        "world/detections": _detection_entities,
    },
    # Rerun keeps every frame it is logged, so an unthrottled entity is an
    # unbounded allocation: the bridge grows with the feed, not with the
    # scene. The map is a backdrop that only has to be current, not live.
    "max_hz": {"world/color_image": 10, "world/lidar": 1, "world/global_map": 0.2},
    # A share of the machine rather than a number, and small enough that the
    # viewer drops its own history long before the host runs out.
    "memory_limit": "5%",
    "tf_axes": 0.5,
    "static": {"world/robot_body": _static_robot_body},
}


go2_localize_live = autoconnect(
    vis_module(viewer_backend=global_config.viewer, rerun_config=rerun_config),
    LoopFeeder.blueprint(),
    VoxelGridMapper.blueprint(emit_every=5),
    LocalizeModule.blueprint(),
    McpServer.blueprint(),
).global_config(n_workers=7, robot_model="unitree_go2")
