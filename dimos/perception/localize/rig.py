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

"""The camera rig behind a run: calibrated cameras, their tf poses, aligned depth or a lidar."""

from __future__ import annotations

from dataclasses import dataclass, field
import math
from typing import TYPE_CHECKING, Any

from dimos_generated.geometry_msgs.msg import PoseStamped, TransformStamped as Transform
from dimos_generated.sensor_msgs.msg import CameraInfo, Image, PointCloud2
from dimos_generated.std_msgs.msg import Header
import numpy as np

from dimos.memory.tf import StreamTF
from dimos.msgs.geometry import inverse_transform, pose_from_transform
from dimos.msgs.image import image_view
from dimos.msgs.pointcloud import (
    pointcloud_from_rgbd,
    pointcloud_from_xyz,
    pointcloud_xyz,
    transform_cloud,
)
from dimos.msgs.time import time_from_seconds
from dimos.perception.detection.type.detection3d.imageDetections3DPC import ImageDetections3DPC
from dimos.perception.detection.type.detection3d.pointcloud_filters import (
    range_cluster,
    raycast,
    statistical,
)
from dimos.protocol.tf.tf import MultiTBuffer
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from dimos.memory.type.observation import Observation
    from dimos.perception.detection.type.detection2d.imageDetections2D import ImageDetections2D
    from dimos.perception.localize.support_plane import SupportPlane
    from dimos.protocol.tf.tf import TFLookup

logger = setup_logger()

WORLD_FRAME = "world"
TF_TOLERANCE = 0.12  # s
DEPTH_TOLERANCE = 0.06  # s - temporal join color->depth
LIDAR_CELL = 0.05  # m - the merged lidar map keeps one point per cell
FRAME_TOLERANCE = 0.02  # s - an index timestamp back to its own color frame
EMBED_HZ = 1.0  # index density for a camera parked over a workspace
TF_ROOT_WINDOW = 2.5  # s - tf read either side of a recording's middle to find its root
# a lidar map holds what the camera cannot see: keep the visible, split off the background, trim
LIDAR_FILTERS = [raycast(), range_cluster(), statistical(nb_neighbors=12, std_ratio=2.0)]


def _tf_root(store: Any, camera: str) -> str:
    """The top of ``camera``'s tf chain in the middle of the recording, the frame its
    poses are taken in; an edge tf only stamps at the ends of a run does not count."""
    stream = store.stream("tf")
    t0, t1 = stream.get_time_range()
    middle = (t0 + t1) / 2
    parent = {
        t.child_frame_id: t.header.frame_id
        for obs in stream.after(middle - TF_ROOT_WINDOW).before(middle + TF_ROOT_WINDOW)
        for t in obs.data.transforms
    }
    chain = [camera]
    while chain[-1] in parent and parent[chain[-1]] not in chain:
        chain.append(parent[chain[-1]])
    return chain[-1] if len(chain) > 1 else WORLD_FRAME


def window_tf(store: Any, t0: float, t1: float) -> MultiTBuffer:
    """A recording's tf over ``[t0, t1]``, read once; lookups in the window match StreamTF's."""
    tf = MultiTBuffer(buffer_size=math.inf)
    for obs in store.stream("tf").after(t0 - 2 * TF_TOLERANCE).before(t1 + 2 * TF_TOLERANCE):
        tf.receive_transform(*obs.data.transforms)
    return tf


@dataclass
class RigCamera:
    """One camera: intrinsics, its colour feed, and the depth aligned to it."""

    info: CameraInfo
    color: Any  # color_image stream
    depth: Any  # aligned depth stream; None lifts through the rig's lidar


@dataclass
class Rig:
    """Cameras keyed by optical frame; tf gives each one's world pose.

    A camera lifts through its aligned depth, or without one through every scan
    of ``lidar`` merged, a point cloud stream in the world frame or in a frame
    tf reaches.
    """

    cameras: dict[str, RigCamera]
    tf: TFLookup
    world_frame: str = WORLD_FRAME
    lidar: Any = None
    plane_cache: dict[tuple[str, int, int], SupportPlane] = field(default_factory=dict, repr=False)
    _lidar_map: tuple[int, PointCloud2 | None] | None = field(default=None, init=False, repr=False)

    @classmethod
    def from_store(cls, store: Any) -> Rig:
        """Every calibrated optical frame that records colour, with aligned depth or
        beside the store's one point cloud stream, is a camera, posed in the root of
        its tf chain."""
        names = store.list_streams()
        types = {name: store.stream(name).data_type for name in names}

        infos: dict[str, CameraInfo] = {}
        for name in sorted(n for n in names if types[n] is CameraInfo):
            if store.stream(name).count():
                info = store.stream(name).first().data
                infos.setdefault(
                    info.header.frame_id, info
                )  # sorted: a colour info beats its depth twin

        images: dict[str, dict[str, str]] = {"color": {}, "depth": {}}
        for name in names:
            if types[name] is not Image or not store.stream(name).count():
                continue
            image = store.stream(name).first().data
            pixels = image_view(image)
            if pixels.dtype == np.uint16 or pixels.dtype.kind == "f":
                kind = "depth"
            elif pixels.ndim == 3:
                kind = "color"
            else:
                continue
            # a derived copy, such as an embedding index, holds at most its feed's frames
            held = images[kind].get(image.header.frame_id)
            if held is None or store.stream(name).count() > store.stream(held).count():
                images[kind][image.header.frame_id] = name

        color, depth = images["color"], images["depth"]
        clouds = [n for n in names if types[n] is PointCloud2 and store.stream(n).count()]
        if len(clouds) > 1 and any(f in color and f not in depth for f in infos):
            raise ValueError(f"several point cloud streams for a camera without depth: {clouds}")
        lidar = clouds[0] if clouds else None
        cameras = {
            frame: RigCamera(
                info=info,
                color=store.stream(color[frame]),
                depth=store.stream(depth[frame]) if frame in depth else None,
            )
            for frame, info in infos.items()
            if frame in color and (frame in depth or lidar is not None)
        }
        if not cameras:
            raise ValueError(
                f"no calibrated camera with colour and aligned depth or a lidar among {names}"
            )
        tf = StreamTF.from_store(store)
        if tf is None:
            raise ValueError(f"no tf stream among {names}")
        if all(camera.depth is not None for camera in cameras.values()):
            lidar = None
        world = _tf_root(store, next(iter(cameras)))
        logger.info(f"rig: cameras={list(cameras)} lidar={lidar!r} world={world!r}")
        return cls(
            cameras=cameras,
            tf=tf,
            world_frame=world,
            lidar=store.stream(lidar) if lidar else None,
        )

    def time_range(self) -> tuple[float, float]:
        """Earliest and latest colour stamp over every camera."""
        ranges = [camera.color.get_time_range() for camera in self.cameras.values()]
        return min(lo for lo, _ in ranges), max(hi for _, hi in ranges)

    def world_to_optical(self, ts: float, frame: str) -> Transform | None:
        return self.tf.get(frame, self.world_frame, ts, TF_TOLERANCE)

    def camera_pose(self, ts: float, frame: str) -> PoseStamped | None:
        """World pose of a camera's optical frame at ts."""
        transform = self.world_to_optical(ts, frame)
        return pose_from_transform(inverse_transform(transform)) if transform is not None else None

    def frame_at(self, obs: Observation[Any]) -> Image:
        """The colour frame an index observation refers to.

        A live index carries the image itself; a replay index carries only what
        frame selection needs, so its pixels come back from its camera's colour
        stream.
        """

        if isinstance(obs.data, Image):
            return obs.data
        color = self.cameras[obs.data.frame_id].color
        image: Image = color.at(obs.ts, FRAME_TOLERANCE).first().data
        return image

    def depth_at(self, ts: float, frame: str) -> Image | None:
        """Temporal join: a camera's aligned depth frame for one of its colour timestamps."""
        try:
            depth: Image = self.cameras[frame].depth.at(ts, DEPTH_TOLERANCE).first().data
        except LookupError:
            return None
        return depth

    def lidar_map(self) -> PointCloud2 | None:
        """Every scan of ``lidar`` in the world frame, one point per cell, built once.

        The scene is taken as static, so a frame lifts through what the lidar
        recorded while the robot was near an object, not only at that frame.
        Bound ``lidar`` to the window first; whatever moved during it is in the
        map wherever it was.
        """
        if self._lidar_map is not None and self._lidar_map[0] == id(self.lidar):
            return self._lidar_map[1]
        parts: list[np.ndarray] = []
        newest = 0.0
        for obs in self.lidar:
            cloud: PointCloud2 = obs.data
            if cloud.header.frame_id != self.world_frame:
                transform = self.tf.get(
                    self.world_frame, cloud.header.frame_id, obs.ts, TF_TOLERANCE
                )
                if transform is None:
                    continue
                cloud = transform_cloud(cloud, transform)
            parts.append(pointcloud_xyz(cloud))
            newest = max(newest, obs.ts)
        merged = None
        if parts:
            points = np.vstack(parts)
            cells = np.floor(points / LIDAR_CELL).astype(np.int64) + (1 << 20)
            keys = (cells[:, 0] << 42) | (cells[:, 1] << 21) | cells[:, 2]
            _, first = np.unique(keys, return_index=True)
            merged = pointcloud_from_xyz(
                points[first],
                header=Header(frame_id=self.world_frame, stamp=time_from_seconds(newest)),
            )
        self._lidar_map = (id(self.lidar), merged)
        return merged

    def lift(
        self, detections: ImageDetections2D[Any], transform: Transform
    ) -> ImageDetections3DPC | None:
        """A frame's detections as world clouds, through its camera's depth or the lidar."""
        frame = detections.image.header.frame_id
        camera = self.cameras[frame]
        if camera.depth is not None:
            depth = self.depth_at(detections.ts, frame)
            if depth is None:
                return None
            return ImageDetections3DPC.from_depth(detections, depth, camera.info, transform)
        cloud = self.lidar_map()
        if cloud is None:
            return None
        return ImageDetections3DPC.from_2d(detections, cloud, camera.info, transform, LIDAR_FILTERS)

    def backdrop(self, ts: float, frame: str, depth_trunc: float = 1.5) -> PointCloud2 | None:
        """A camera's own RGBD frame at ts as a world-frame cloud, for plane fits and rendering;
        the lidar map for a camera without depth."""
        if self.cameras[frame].depth is None:
            return self.lidar_map()
        depth = self.depth_at(ts, frame)
        if depth is None:
            return None
        try:
            color = self.cameras[frame].color.at(ts, 0.1).first().data
        except LookupError:
            return None
        transform = self.world_to_optical(ts, frame)
        if transform is None:
            return None
        return transform_cloud(
            pointcloud_from_rgbd(
                color, depth, self.cameras[frame].info, depth_scale=0.001, depth_trunc=depth_trunc
            ),
            inverse_transform(transform),
        )
