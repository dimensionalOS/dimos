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

from collections import OrderedDict
from dataclasses import dataclass, field
import json
from pathlib import Path
from typing import TYPE_CHECKING, Any, cast

import numpy as np

from dimos.memory.tf import StreamTF
from dimos.memory.transform import Transformer
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
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from collections.abc import Callable, Iterator

    from dimos_lcm.sensor_msgs import CameraInfo

    from dimos.memory.type.observation import Observation
    from dimos.msgs.geometry_msgs.Pose import Pose
    from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
    from dimos.msgs.sensor_msgs.Image import Image
    from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
    from dimos.perception.detection.type.detection2d.imageDetections2D import ImageDetections2D
    from dimos.perception.memory.support_plane import SupportPlane
    from dimos.protocol.tf.tf import TFLookup

logger = setup_logger()

# Pose-stamped rigs ride per-frame odometry, so walking does not stale the
# projection the way a sweeping wrist stales interpolated tf; the gate only
# drops speed glitches and sprints.
WALK_SPEED_MAX = 1.5

DEPTH_TOLERANCE = 0.06  # s - temporal join color->depth
# Scans within this of a frame form its geometry. Wide enough that a
# spinning lidar's near-floor blind ring is filled by scans taken from
# earlier and later poses - the scene is static in world frame.
CLOUD_ACCUM_S = 4.0

_SCAN_CACHE_MAX = 256  # registered scans held per rig; a window needs a few dozen
_CELL_OFFSET = 1 << 20  # shifts lattice cell indices positive for 21-bit key packing

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
    cluster_radius_m=0.30,
    min_depth_points=30,
    max_object_extent_m=2.0,
    min_camera_range_m=0.5,
    surface_patch_max_rise_m=0.08,
    surface_patch_min_drop_m=-0.06,
    verify_radius_m=5.0,
)


MOBILE_SPAN_M = 3.0  # camera translation beyond this means a mobile base


def _tf_root(store: Any, tf_name: str) -> str | None:
    """The tf tree's root frame: a parent that is never a child."""
    parents: set[str] = set()
    children: set[str] = set()
    for obs in store.stream(tf_name).limit(500):
        for transform in obs.data.transforms:
            parents.add(transform.frame_id)
            children.add(transform.child_frame_id)
    roots = parents - children
    return roots.pop() if len(roots) == 1 else None


def _stream_rate(stream: Any) -> float:
    count: int = stream.count()
    if count < 2:
        return float(count)
    t0, t1 = stream.get_time_range()
    return count / max(float(t1 - t0), 1e-6)


def _camera_span(rig: Rig) -> float:
    """Diagonal of the camera positions' bounding box over the recording."""
    if rig.tf is None and rig.poses is None:
        return 0.0  # no pose source at all: an embed-only store being seeded
    try:
        t0, t1 = rig.color.get_time_range()
    except LookupError:
        return 0.0  # live store, nothing recorded yet
    positions = []
    for k in range(12):
        pose = rig.camera_pose(t0 + (t1 - t0) * k / 11)
        if pose is not None:
            positions.append([pose.position.x, pose.position.y, pose.position.z])
    if len(positions) < 2:
        return 0.0
    spread = np.array(positions)
    return float(np.linalg.norm(spread.max(axis=0) - spread.min(axis=0)))


def _lattice_quantum(points: np.ndarray) -> float | None:
    """The grid pitch when coordinates lie on a lattice; None for continuous scans.

    Grid-quantized sources (an occupancy map streamed as clouds) repeat the
    same cell across snapshots and dedup by cell key; continuous scans never
    collide and skip dedup entirely.
    """
    sample = points[:2048]
    x = np.unique(sample[:, 0])
    if len(x) < 8:
        return None
    diffs = np.diff(x)
    diffs = diffs[diffs > 1e-9]
    if len(diffs) == 0:
        return None
    quantum = float(diffs.min())
    if quantum < 1e-4:
        return None
    scaled = sample / quantum
    if float(np.abs(scaled - np.round(scaled)).max()) > 0.01:
        return None
    return quantum


class RegisterScans(Transformer["PointCloud2", "PointCloud2"]):
    """Map sensor-frame scans into a world frame through tf, one transform per scan.

    A plain stream transformer, so the registered cloud is a derived stream:
    ``scans.transform(RegisterScans(tf, world))`` yields world-frame clouds,
    ``.save(...)`` persists them, and a live pipeline can tail-register the
    same way the embed pipeline does. Scans already in the world frame pass
    through untouched; scans with no transform at their time are dropped.
    """

    def __init__(self, tf: TFLookup, world_frame: str, tolerance: float = TF_TOLERANCE) -> None:
        self.tf = tf
        self.world_frame = world_frame
        self.tolerance = tolerance

    def __call__(
        self, upstream: Iterator[Observation[PointCloud2]]
    ) -> Iterator[Observation[PointCloud2]]:
        from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2

        for obs in upstream:
            if obs.data.frame_id == self.world_frame:
                yield obs
                continue
            transform = self.tf.get(self.world_frame, obs.data.frame_id, obs.ts, self.tolerance)
            if transform is None:
                continue
            matrix = transform.to_matrix()
            points = obs.data.as_numpy()[0] @ matrix[:3, :3].T + matrix[:3, 3]
            yield obs.derive(
                data=PointCloud2.from_numpy(points, frame_id=self.world_frame, timestamp=obs.ts)
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
    cloud: Any = None  # pointcloud stream, lifted via from_2d after registration
    tf_tolerance: float = TF_TOLERANCE
    cloud_accum_s: float = CLOUD_ACCUM_S
    speed_max: float = SPEED_MAX
    scene_gate: bool = True
    embed_hz: float = EMBED_HZ
    mobile: bool = False  # camera rides a mobile base: room-scale policies
    _cloud_memo: tuple[float, PointCloud2] | None = field(default=None, repr=False, init=False)
    _scan_cache: OrderedDict[float, np.ndarray | None] = field(
        default_factory=OrderedDict, repr=False, init=False
    )
    _plane_cache: dict[tuple[int, int], SupportPlane] = field(
        default_factory=dict, repr=False, init=False
    )
    _quantum: float | None = field(default=None, repr=False, init=False)
    _quantum_known: bool = field(default=False, repr=False, init=False)

    @classmethod
    def from_store(
        cls,
        store: Any,
        manifest: dict[str, Any] | None = None,
        overrides: dict[str, str] | None = None,
    ) -> Rig:
        """Recognize the store's shape without depending on stream names.

        Resolution order per role: ``overrides`` (explicit stream names from
        a caller or CLI), then ``manifest`` (passed in, or the ``<db>.rig.json``
        sidecar next to the recording), then discovery by stream data type
        and content: TFMessage-typed stream as tf, Image streams classified
        by their frames (uint16/float is depth, distinct-channel uint8 is
        color; a lossy-coded gray stream is neither), a PointCloud2 stream as
        geometry when there is no metric depth, and - when tf is absent - the
        highest-rate pose-stamped stream as pose source. On tf rigs the world
        frame is the tf tree's root; ambiguity or missing calibration raises
        with the candidates rather than guessing.

        The manifest carries roles as stream names plus, for recordings whose
        calibration was never recorded, an inline ``camera_info`` dict and a
        ``base_to_optical`` mount.
        """
        from dimos.msgs.geometry_msgs.Quaternion import Quaternion
        from dimos.msgs.geometry_msgs.Vector3 import Vector3
        from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo as CameraInfoMsg
        from dimos.msgs.sensor_msgs.Image import Image as ImageMsg
        from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2 as PointCloudMsg
        from dimos.msgs.tf2_msgs.TFMessage import TFMessage

        names = store.list_streams()
        types = {name: store.stream(name).data_type for name in names}

        if manifest is None:
            path = getattr(store.config, "path", None)
            if path:
                sidecar = Path(f"{path}.rig.json")
                if sidecar.exists():
                    manifest = json.loads(sidecar.read_text())
                    logger.info(f"rig: manifest {sidecar}")
        roles: dict[str, Any] = dict(manifest or {})
        roles.update(overrides or {})

        claimed = {value for value in roles.values() if isinstance(value, str)}
        color_name: str | None = roles.get("color")
        depth_name: str | None = roles.get("depth")
        cloud_name: str | None = roles.get("cloud")
        poses_name: str | None = roles.get("poses")

        tf_names = [n for n in names if types[n] is TFMessage]
        tf_name = tf_names[0] if len(tf_names) == 1 else ("tf" if "tf" in tf_names else None)
        tf = StreamTF.from_store(store, tf_name) if tf_name is not None else None

        # image streams classify by content: metric depth or genuine color
        image_names = [n for n in names if types[n] is ImageMsg and n not in claimed]
        color_candidates: list[str] = []
        depth_candidates: list[str] = []
        empty_images: list[str] = []
        for name in image_names:
            stream = store.stream(name)
            if stream.count() == 0:
                empty_images.append(name)
                continue
            frame = stream.first().data.data
            if frame.dtype == np.uint16 or frame.dtype.kind == "f":
                depth_candidates.append(name)
            elif (
                frame.ndim == 3
                and frame.shape[2] == 3
                and not np.array_equal(frame[..., 0], frame[..., 1])
            ):
                color_candidates.append(name)
            else:
                logger.info(f"rig: image stream {name!r} is neither color nor metric depth")
        if color_name is None:
            if len(color_candidates) > 1:
                color_name = max(color_candidates, key=lambda n: _stream_rate(store.stream(n)))
                logger.info(f"rig: several color streams, using highest-rate {color_name!r}")
            elif color_candidates:
                color_name = color_candidates[0]
            elif len(empty_images) == 1:
                color_name = empty_images[0]  # a live store's not-yet-filled feed
        if color_name is None:
            raise ValueError(f"no color image stream among {names}; pass a manifest or --color")
        claimed.add(color_name)
        if depth_name is None:
            if len(depth_candidates) > 1:
                raise ValueError(f"several depth streams {depth_candidates}; pass --depth")
            depth_name = depth_candidates[0] if depth_candidates else None

        if cloud_name is None and depth_name is None:
            cloud_names = [n for n in names if types[n] is PointCloudMsg and n not in claimed]
            if len(cloud_names) > 1:
                raise ValueError(f"several pointcloud streams {cloud_names}; pass --cloud")
            cloud_name = cloud_names[0] if cloud_names else None

        color = store.stream(color_name)
        depth = store.stream(depth_name) if depth_name is not None else None
        cloud = store.stream(cloud_name) if cloud_name is not None and depth is None else None
        if cloud_name is not None:
            claimed.add(cloud_name)

        world_frame = gates.WORLD_FRAME
        if tf is not None:
            world_frame = _tf_root(store, cast("str", tf_name)) or world_frame
        elif cloud is not None:
            try:
                world_frame = cloud.first().data.frame_id
            except LookupError:
                pass  # live store, nothing recorded yet

        # intrinsics: inline manifest dict, named stream, or discovery by
        # type with the color camera's frame deciding among several
        camera_info = None
        camera_info_role = roles.get("camera_info")
        if isinstance(camera_info_role, dict):
            camera_info = CameraInfoMsg(
                height=camera_info_role["height"],
                width=camera_info_role["width"],
                distortion_model=camera_info_role.get("distortion_model", ""),
                D=camera_info_role.get("D"),
                K=camera_info_role["K"],
                R=camera_info_role.get("R"),
                P=camera_info_role.get("P"),
                frame_id=camera_info_role["frame_id"],
            )
        else:
            ci_name = camera_info_role if isinstance(camera_info_role, str) else None
            if ci_name is None:
                candidates = [
                    n for n in names if types[n] is CameraInfoMsg and store.stream(n).count()
                ]
                try:
                    color_frame = color.first().data.frame_id
                except LookupError:
                    color_frame = None
                matching = [
                    n for n in candidates if store.stream(n).first().data.frame_id == color_frame
                ]
                if matching:
                    ci_name = sorted(matching)[0]
                elif len(candidates) == 1:
                    ci_name = candidates[0]
                elif len(candidates) > 1:
                    raise ValueError(f"several camera_info streams {candidates}; pass a manifest")
            if ci_name is not None:
                camera_info = store.stream(ci_name).first().data

        base_to_optical = None
        mount = roles.get("base_to_optical")
        if isinstance(mount, dict):
            base_to_optical = Transform(
                translation=Vector3(*mount["translation"]),
                rotation=Quaternion(*mount["rotation"]),
                frame_id="base_link",
                child_frame_id=camera_info.frame_id if camera_info else "camera_optical",
            )

        poses = None
        if tf is None:
            if poses_name is None:
                posed = [
                    n
                    for n in names
                    if n not in claimed
                    and n != color_name
                    and store.stream(n).count()
                    and store.stream(n).first().pose_tuple is not None
                ]
                if posed:
                    poses_name = max(posed, key=lambda n: _stream_rate(store.stream(n)))
            poses = store.stream(poses_name) if poses_name is not None else None

        if depth is not None or cloud is not None:
            if camera_info is None:
                raise ValueError(
                    "store has 3D geometry but no camera calibration; add a CameraInfo "
                    "stream role or an inline camera_info to the <db>.rig.json manifest"
                )
            if tf is None and (poses is None or base_to_optical is None):
                raise ValueError(
                    "store has no tf; a pose-stamped rig needs a poses stream and a "
                    "base_to_optical mount in the <db>.rig.json manifest"
                )

        # embed-only stores (no geometry) may carry no calibration at all;
        # every geometry API raises on use, embedding never touches it
        optical_frame = camera_info.frame_id if camera_info is not None else gates.OPTICAL_FRAME
        rig = cls(
            camera_info=cast("CameraInfo", camera_info),
            color=color,
            world_frame=world_frame,
            optical_frame=optical_frame,
            tf=tf,
            base_to_optical=base_to_optical,
            poses=poses,
            depth=depth,
            cloud=cloud,
        )

        span = _camera_span(rig)
        rig.mobile = span > MOBILE_SPAN_M
        if rig.mobile:
            rig.speed_max = WALK_SPEED_MAX
            rig.scene_gate = False
            rig.embed_hz = WALK_EMBED_HZ
        logger.info(
            f"rig: color={color_name!r} depth={depth_name!r} cloud={cloud_name!r} "
            f"tf={tf_name!r} world={world_frame!r} span={span:.1f}m mobile={rig.mobile}"
        )
        return rig

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

    def registered_scan(self, scan: Observation[PointCloud2]) -> np.ndarray | None:
        """A scan's points in the world frame, registered via tf when needed.

        Decode and registration run once per scan per run: accumulation
        windows of neighboring frames overlap almost entirely, and every
        query of a multi-label run shares every window.
        """
        key = scan.ts
        if key in self._scan_cache:
            self._scan_cache.move_to_end(key)
            return self._scan_cache[key]
        points: np.ndarray | None = scan.data.as_numpy()[0]
        frame = scan.data.frame_id
        if frame != self.world_frame:
            transform = (
                self.tf.get(self.world_frame, frame, scan.ts, self.tf_tolerance)
                if self.tf is not None
                else None
            )
            if transform is None:
                points = None
            else:
                matrix = transform.to_matrix()
                points = points @ matrix[:3, :3].T + matrix[:3, 3]
        self._scan_cache[key] = points
        if len(self._scan_cache) > _SCAN_CACHE_MAX:
            self._scan_cache.popitem(last=False)
        return points

    def _cloud_quantum(self, points: np.ndarray) -> float | None:
        if not self._quantum_known:
            self._quantum = _lattice_quantum(points)
            self._quantum_known = True
        return self._quantum

    def cloud_at(self, ts: float) -> PointCloud2 | None:
        """World-frame geometry at ts: the scans accumulated around it."""
        if self._cloud_memo is not None and self._cloud_memo[0] == ts:
            return self._cloud_memo[1]
        from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2

        scans = self.cloud.after(ts - self.cloud_accum_s).before(ts + self.cloud_accum_s)
        parts = [p for p in (self.registered_scan(scan) for scan in scans) if p is not None]
        if not parts:
            return None
        if len(parts) == 1:
            points = parts[0]
        else:
            stacked = np.vstack(parts)
            quantum = self._cloud_quantum(parts[0])
            if quantum is None:
                points = stacked
            else:
                # A grid-quantized source repeats the same cell in every
                # snapshot it persists through; accumulation must not count
                # one voxel once per snapshot. Cell keys dedup in one pass.
                cells = np.round(stacked / quantum).astype(np.int64) + _CELL_OFFSET
                keys = (cells[:, 0] << 42) | (cells[:, 1] << 21) | cells[:, 2]
                _, index = np.unique(keys, return_index=True)
                points = stacked[index]
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
        return ROOM_LOCALIZE_POLICY if self.mobile else LocalizePolicy()

    def default_inventory_policy(self) -> InventoryPolicy:
        return ROOM_INVENTORY_POLICY if self.mobile else InventoryPolicy()
