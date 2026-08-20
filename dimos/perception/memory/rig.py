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

"""The sensor rig behind a recording: intrinsics, poses, and 3D geometry.

Two independent axes generalize the perception stack beyond one robot:

* **Pose source.** The world-to-optical transform comes from a recorded
  ``tf`` stream, or - for recordings without one - from the world pose
  stamped on each observation plus a static base-to-optical mount.
* **Geometry source.** 3D geometry comes from an aligned ``depth`` stream,
  unprojected per detection mask, or from a world-frame pointcloud stream
  (a registered lidar), projected through the camera per detection mask.
  Registered scans are sparse, so the cloud at a timestamp is the
  concatenation of the scans in a short window around it - the scene is
  static in world frame, which is what makes accumulation valid.

``Rig.from_store`` recognizes both recording shapes; every field can also
be supplied directly for live stores whose streams are still filling.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Any, cast

import numpy as np

from dimos.memory.tf import StreamTF
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.perception.detection.project import sees as project_sees
from dimos.perception.detection.type.detection3d.imageDetections3DPC import ImageDetections3DPC
from dimos.perception.detection.type.detection3d.pointcloud_filters import (
    range_cluster,
    statistical,
)
from dimos.perception.memory import gates
from dimos.perception.memory.gates import SPEED_MAX, STILL_ENVELOPE, TF_TOLERANCE
from dimos.perception.memory.types import InventoryPolicy, LocalizePolicy

if TYPE_CHECKING:
    from collections.abc import Callable

    from dimos_lcm.sensor_msgs import CameraInfo

    from dimos.memory.type.observation import Observation
    from dimos.msgs.geometry_msgs.Pose import Pose
    from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
    from dimos.msgs.sensor_msgs.Image import Image
    from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
    from dimos.perception.detection.type.detection2d.imageDetections2D import ImageDetections2D
    from dimos.protocol.tf.tf import TFLookup

# Pose-stamped rigs ride per-frame odometry, so walking does not stale the
# projection the way a sweeping wrist stales interpolated tf; the gate only
# drops speed glitches and sprints.
WALK_SPEED_MAX = 1.5

DEPTH_TOLERANCE = 0.06  # s - temporal join color->depth
CLOUD_ACCUM_S = 2.0  # s - scans within this of a frame form its geometry

EMBED_HZ = 1.0  # index density for a wrist camera parked over a workspace
# A walking robot changes viewpoint every frame and its frames blur
# unevenly, so the index must sample denser for retrieval to catch the
# sharp sightings.
WALK_EMBED_HZ = 3.0

# Sparse projected clouds: split off background seen through the mask, then a
# loose outlier trim. The dense-cloud defaults (raycast + radius) assume a
# density registered lidar does not have.
_CLOUD_LIFT_FILTERS = [range_cluster(), statistical(nb_neighbors=12, std_ratio=2.0)]

# Room-scale policies for mobile-robot rigs: objects are furniture-sized,
# viewpoints meters apart, odometry drifts centimeters between passes, and
# registered lidar is noisier and sparser than wrist-camera depth.
ROOM_LOCALIZE_POLICY = LocalizePolicy(
    candidate_floor=0.18,
    accept_score=0.32,
    retrieval_frames=20,
    cluster_radius_m=0.30,
    min_depth_points=30,
    max_object_extent_m=2.0,
    min_camera_range_m=0.5,
    surface_patch_max_rise_m=0.08,
    surface_patch_min_drop_m=-0.06,
    verify_radius_m=5.0,
)
ROOM_INVENTORY_POLICY = InventoryPolicy(
    keyframe_stride_s=1.25,
    min_mask_area_px=900,
    min_depth_points=30,
    max_object_extent_m=2.0,
    min_height_above_plane_m=0.08,
    band_above_plane_m=(-0.05, 1.5),
    min_camera_range_m=0.5,
    envelope_pad_m=0.12,
    search_radius_m=0.8,
    size_gap_max_m=0.8,
    support_explained=0.25,
    name_attach_iou=0.20,
    same_frame_merge_gap_m=0.10,
    split_extent_m=1.2,
    split_height_m=0.9,
    split_eps_m=0.10,
)


@dataclass
class Rig:
    """Everything the stack needs to go from a 2D mask to world geometry.

    Exactly one of ``tf`` / (``base_to_optical`` + ``poses``) provides the
    world-to-optical transform, and exactly one of ``depth`` / ``cloud``
    provides 3D geometry.
    """

    camera_info: CameraInfo
    color: Any  # color_image stream
    world_frame: str
    optical_frame: str
    tf: TFLookup | None = None
    base_to_optical: Transform | None = None
    poses: Any = None  # stream carrying world base poses (e.g. odom)
    depth: Any = None  # aligned depth stream, lifted via from_depth
    cloud: Any = None  # world-frame pointcloud stream, lifted via from_2d
    tf_tolerance: float = TF_TOLERANCE
    cloud_accum_s: float = CLOUD_ACCUM_S
    speed_max: float = SPEED_MAX
    scene_gate: bool = True
    embed_hz: float = EMBED_HZ
    _cloud_memo: tuple[float, PointCloud2] | None = field(default=None, repr=False, init=False)

    @classmethod
    def from_store(cls, store: Any) -> Rig:
        """Recognize the store's shape.

        A ``tf`` stream wins as pose source; without one the observations'
        stamped poses are used with the Go2 front-camera mount. ``depth_image``
        wins as geometry source; a ``lidar`` stream is next, registered in
        whatever world frame its scans carry; a store with neither can still
        embed and retrieve, just never lift. Intrinsics and the optical frame
        name come from the ``camera_info`` stream, or - for stores that carry
        none - the static Go2 front-camera calibration. Stream contents are
        only read where a name must be sniffed, so a live store whose streams
        are still empty resolves too.
        """
        streams = store.list_streams()
        color = store.streams.color_image
        tf = StreamTF.from_store(store)

        depth = store.streams.depth_image if "depth_image" in streams else None
        cloud = None
        world_frame = gates.WORLD_FRAME
        if depth is None and "lidar" in streams:
            cloud = store.streams.lidar
            try:
                world_frame = cloud.first().data.frame_id
            except LookupError:
                pass  # live store, nothing recorded yet

        if "camera_info" in streams:
            camera_info = store.streams.camera_info.first().data
            optical_frame = camera_info.frame_id
        else:
            from dimos.robot.unitree.go2.connection import GO2Connection

            camera_info = GO2Connection.camera_info_static
            # With a tf tree the optical name must match that tree; the Go2
            # calibration names apply only to the tf-less Go2 shape.
            optical_frame = camera_info.frame_id if tf is None else gates.OPTICAL_FRAME

        base_to_optical = None
        poses = None
        if tf is None:
            from dimos.robot.unitree.go2.connection import BASE_TO_OPTICAL

            base_to_optical = BASE_TO_OPTICAL
            poses = store.streams.odom

        mobile = cloud is not None
        return cls(
            camera_info=camera_info,
            color=color,
            world_frame=world_frame,
            optical_frame=optical_frame,
            tf=tf,
            base_to_optical=base_to_optical,
            poses=poses,
            depth=depth,
            cloud=cloud,
            speed_max=WALK_SPEED_MAX if mobile else SPEED_MAX,
            scene_gate=not mobile,
            embed_hz=WALK_EMBED_HZ if mobile else EMBED_HZ,
        )

    # pose 

    def world_to_optical(self, ts: float) -> Transform | None:
        if self.tf is not None:
            return self.tf.get(self.optical_frame, self.world_frame, ts, self.tf_tolerance)
        pose = self.pose_at(ts)
        if pose is None:
            return None
        mount = cast("Transform", self.base_to_optical)
        return -(Transform.from_pose("base_link", pose) + mount)

    def pose_at(self, ts: float) -> PoseStamped | None:
        """World base pose nearest ts, from the poses stream.

        ``at().first()`` is window-earliest, not window-nearest; a walking
        robot covers centimeters per pose period, so the nearest pose in the
        window is what keeps the projection aligned.
        """
        candidates = list(self.poses.at(ts, self.tf_tolerance))
        if not candidates:
            return None
        obs = min(candidates, key=lambda o: abs(o.ts - ts))
        pose: PoseStamped | None = obs.pose_stamped
        return pose

    def camera_pose(self, ts: float) -> PoseStamped | None:
        """World pose of the camera optical frame at ts."""
        transform = self.world_to_optical(ts)
        return (-transform).to_pose() if transform is not None else None

    def index_pose(self, obs: Observation[Any]) -> Pose | PoseStamped | None:
        """The pose an embedded index observation carries.

        tf rigs stamp the derived optical pose; pose-stamped rigs keep the
        recorded base pose, which is what ``sees`` expects to find on an
        observation when it resolves the transform through the mount.
        """
        if self.tf is not None:
            return self.camera_pose(obs.ts)
        return obs.pose

    def camera_speed(self, ts: float, dt: float = 0.06) -> float | None:
        """Linear camera speed (m/s) around ts, from pose differencing."""
        a = self.camera_pose(ts - dt)
        b = self.camera_pose(ts + dt)
        if a is None or b is None:
            return None
        return float((b.position - a.position).magnitude() / (2 * dt))

    def camera_still(self, ts: float, envelope: float = STILL_ENVELOPE) -> bool:
        """Camera below the rig's speed gate over the whole capture envelope."""
        for offset in (-envelope, 0.0, envelope):
            speed = self.camera_speed(ts + offset)
            if speed is None or speed > self.speed_max:
                return False
        return True

    def still_intervals(self, t0: float, t1: float) -> list[tuple[float, float]]:
        """Maximal camera-still intervals inside [t0, t1], sampled at 0.25 s."""
        step = 0.25
        times = np.arange(t0, t1 + step, step)
        intervals: list[tuple[float, float]] = []
        run_start: float | None = None
        for t in times:
            speed = self.camera_speed(float(t))
            still = speed is not None and speed <= self.speed_max
            if still and run_start is None:
                run_start = float(t)
            elif not still and run_start is not None:
                intervals.append((run_start, float(t) - step))
                run_start = None
        if run_start is not None:
            intervals.append((run_start, float(times[-1])))
        return [(a, b) for a, b in intervals if b >= a]

    # geometry 

    def depth_at(self, ts: float) -> Image | None:
        """Temporal join: aligned depth frame for a color timestamp."""
        try:
            depth: Image = self.depth.at(ts, DEPTH_TOLERANCE).first().data
        except LookupError:
            return None
        return depth

    def cloud_at(self, ts: float) -> PointCloud2 | None:
        """World-frame geometry at ts: the scans accumulated around it."""
        if self._cloud_memo is not None and self._cloud_memo[0] == ts:
            return self._cloud_memo[1]
        from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2

        scans = list(self.cloud.after(ts - self.cloud_accum_s).before(ts + self.cloud_accum_s))
        if not scans:
            return None
        merged: PointCloud2
        if len(scans) == 1:
            merged = scans[0].data
        else:
            # A grid-quantized source (the Go2 occupancy stream) repeats the
            # same cell in every snapshot it persists through; accumulation
            # must not count one voxel once per snapshot.
            points = np.unique(np.vstack([scan.data.as_numpy()[0] for scan in scans]), axis=0)
            merged = PointCloud2.from_numpy(points, frame_id=self.world_frame, timestamp=ts)
        self._cloud_memo = (ts, merged)
        return merged

    def lift(self, detections: ImageDetections2D) -> ImageDetections3DPC | None:
        """2D detections to world-frame 3D clouds, or None without geometry/pose."""
        transform = self.world_to_optical(detections.ts)
        if transform is None:
            return None
        if self.depth is not None:
            depth = self.depth_at(detections.ts)
            if depth is None:
                return None
            return ImageDetections3DPC.from_depth(detections, depth, self.camera_info, transform)
        cloud = self.cloud_at(detections.ts)
        if cloud is None:
            return None
        return ImageDetections3DPC.from_2d(
            detections, cloud, self.camera_info, transform, _CLOUD_LIFT_FILTERS
        )

    def backdrop(self, ts: float) -> PointCloud2 | None:
        """World-frame scene cloud around ts, for plane fits and rendering."""
        if self.depth is None:
            return self.cloud_at(ts)
        from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2

        depth = self.depth_at(ts)
        transform = self.world_to_optical(ts)
        if depth is None or transform is None:
            return None
        try:
            color = self.color.at(ts, 0.1).first().data
        except LookupError:
            return None
        return PointCloud2.from_rgbd(
            color, depth, self.camera_info, depth_scale=0.001, depth_trunc=1.5
        ).transform(-transform)

    # predicates 

    def sees(
        self,
        point: Any,
        *,
        extent: Any | None = None,
        min_fraction: float = 1.0,
        max_range: float | None = None,
    ) -> Callable[[Observation[Any]], bool]:
        """Predicate: does an observation's camera see the world point.

        Occlusion checking through measured depth only exists on depth rigs;
        projected-cloud rigs rely on the geometric visibility test alone.
        """
        return project_sees(
            point,
            self.camera_info,
            tf=self.tf,
            base_to_optical=self.base_to_optical,
            world_frame=self.world_frame,
            optical_frame=self.optical_frame,
            time_tolerance=self.tf_tolerance,
            extent=extent,
            min_fraction=min_fraction,
            max_range=max_range,
            depth=(lambda obs: self.depth_at(obs.ts)) if self.depth is not None else None,
        )

    def keyframes(
        self,
        t0: float,
        t1: float,
        stride: float,
        motion_threshold: float = gates.MOTION_THRESHOLD,
    ) -> list[Observation[Image]]:
        """Camera-still (and, on scene-gated rigs, scene-still) frames on a grid.

        For each grid point the nearest passing frame within half a stride is
        selected, so a grid point landing mid-sweep snaps to the neighboring
        pause instead of being lost.
        """
        intervals = self.still_intervals(t0, t1) if self.scene_gate else []
        gray: dict[float, np.ndarray | None] = {}
        selected: list[Observation[Image]] = []
        seen: set[float] = set()
        offsets = [0.0]
        probe = 0.35
        while probe <= stride / 2:
            offsets.extend([probe, -probe])
            probe += 0.35

        t = t0 + 0.5
        while t < t1:
            for offset in offsets:
                ts = t + offset
                if ts < t0 or ts > t1:
                    continue
                if not self.camera_still(ts):
                    continue
                if self.scene_gate and not gates.scene_still(
                    self.color, ts, intervals, gray, motion_threshold
                ):
                    continue
                try:
                    obs = self.color.at(ts, 0.1).first()
                except LookupError:
                    continue
                if obs.ts in seen:
                    break
                if self.world_to_optical(obs.ts) is None:
                    continue
                seen.add(obs.ts)
                selected.append(obs)
                break
            t += stride
        return selected

    def default_localize_policy(self) -> LocalizePolicy:
        return LocalizePolicy() if self.depth is not None else ROOM_LOCALIZE_POLICY

    def default_inventory_policy(self) -> InventoryPolicy:
        return InventoryPolicy() if self.depth is not None else ROOM_INVENTORY_POLICY
