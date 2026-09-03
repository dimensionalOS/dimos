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

"""The sensor rig behind a run: intrinsics, poses, and 3D geometry.

Three independent axes generalize the stack beyond one robot: pose from a
``tf`` stream or from stamped world poses plus a static mount; geometry from
an aligned ``depth`` stream or a world-frame pointcloud; and intrinsics and
mounts keyed by optical frame, each lookup taking the frame off the image it
resolves rather than off a single rig-wide field.

:meth:`Rig.from_store` recognizes a recording's shape and self-calibrates what
it omits; a robot that knows its calibration builds a ``Rig`` field by field
and never touches it.
"""

from __future__ import annotations

from collections import OrderedDict
from dataclasses import dataclass, field, replace
import json
from pathlib import Path
from typing import TYPE_CHECKING, Any, cast

import numpy as np

from dimos.memory.tf import StreamTF
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.perception.detection.type.detection3d.imageDetections3DPC import ImageDetections3DPC
from dimos.perception.detection.type.detection3d.pointcloud import lattice_quantum
from dimos.perception.detection.type.detection3d.pointcloud_filters import (
    range_cluster,
    statistical,
)
from dimos.perception.memory.support_plane import PLANE_DISTANCE_CLOUD
from dimos.perception.memory.types import LocalizePolicy
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
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

OPTICAL_FRAME = "camera_color_optical_frame"  # only when a store carries no calibration
WORLD_FRAME = "world"
TF_TOLERANCE = 0.12  # s - one world-pose period plus margin

DEPTH_TOLERANCE = 0.06  # s - temporal join color->depth
# Scans within this of a frame form its geometry. Wide enough that a spinning
# lidar's near-floor blind ring is filled by scans taken from earlier and
# later poses - the scene is static in world frame.
CLOUD_ACCUM_S = 4.0
FRAME_TOLERANCE = 0.02  # s - an index timestamp back to its own color frame

_SCAN_CACHE_MAX = 256  # registered scans held per rig; a window needs a few dozen
_CLOUD_CACHE_MAX = 8  # merged windows held; adjacent frames re-ask for the same one

EMBED_HZ = 1.0  # index density for a camera parked over a workspace
WALK_EMBED_HZ = 3.0  # a walking robot changes viewpoint every frame
MOBILE_SPAN_M = 3.0  # camera translation beyond this means a mobile base

# Sparse projected clouds: split off background seen through the mask, then a
# loose outlier trim. The dense-cloud defaults assume a density a registered
# lidar does not have.
_CLOUD_TRIM_NEIGHBORS = 12
_CLOUD_LIFT_FILTERS = [
    range_cluster(),
    statistical(nb_neighbors=_CLOUD_TRIM_NEIGHBORS, std_ratio=2.0),
]
# A projected lift's evidence floor is the smallest cloud the trim can vet,
# its own neighborhood: lattice-cell counts and depth-pixel counts do not
# compare, so a depth-scale floor does not apply to one.
CLOUD_MIN_POINTS = _CLOUD_TRIM_NEIGHBORS + 1

# Room-scale defaults for mobile rigs: objects are furniture-sized, viewpoints
# meters apart, odometry drifts centimeters, and registered lidar is noisier
# and sparser than wrist-camera depth.
ROOM_LOCALIZE_POLICY = LocalizePolicy(
    candidate_floor=0.18,
    accept_score=0.32,
    cluster_radius_m=0.30,
    verify_radius_m=5.0,
    max_object_extent_m=2.0,
    surface_patch_max_rise_m=0.08,
    surface_patch_min_drop_m=-0.06,
    min_points=30,
    min_camera_range_m=0.5,
    fuse_voxel_m=0.03,
)


def _column_keys(points: np.ndarray, quantum: float, anchor: np.ndarray) -> np.ndarray:
    """Packed XY lattice-cell key per point, anchored so any grid phase maps exactly."""
    cells = np.round((points[:, :2] - anchor) / quantum).astype(np.int64) + (1 << 20)
    keys: np.ndarray = (cells[:, 0] << 21) | cells[:, 1]
    return keys


ROOT_PROBES = 24  # instants a frame is probed at before it counts as unreachable
# Color timestamps stamped at receive lag the pose source by a constant the
# recording reveals: the lag that best correlates optical-flow yaw rate with
# the camera's heading rate. The grid is the searched span and its resolution;
# a true delay outside it lands on an edge and is refused rather than clamped.
DELAY_LAGS = np.arange(-0.5, 0.5001, 0.01)
DELAY_WINDOWS = 12  # flow is sampled in short windows spread over the recording
DELAY_WINDOW_S = 2.5


def _rate(stream: Any) -> float:
    count: int = stream.count()
    if count < 2:
        return float(count)
    t0, t1 = stream.get_time_range()
    return count / max(float(t1 - t0), 1e-6)


def _tf_root(store: Any, tf_name: str, tf: StreamTF, optical: str) -> str | None:
    """The tf root: a parent that is never a child, among reachable frames.

    A recording can carry an anchor edge published once at each end of the run,
    above the frame everything else is stamped in. Taking it strands every pose
    lookup, so frames no probe resolves are dropped first; probes sit at bin
    midpoints, where an anchor stamped at the ends cannot answer.
    """
    edges = {
        (t.frame_id, t.child_frame_id) for obs in store.stream(tf_name) for t in obs.data.transforms
    }
    if not edges:
        return None  # live store, nothing recorded yet
    frames = {frame for edge in edges for frame in edge}
    t0, t1 = store.stream(tf_name).get_time_range()
    reached = {optical}
    for k in range(ROOT_PROBES):
        ts = t0 + (t1 - t0) * (k + 0.5) / ROOT_PROBES
        for frame in frames - reached:
            if tf.get(optical, frame, ts, TF_TOLERANCE, warn=False) is not None:
                reached.add(frame)
    linked = [(p, c) for p, c in edges if p in reached and c in reached]
    roots = {p for p, _ in linked} - {c for _, c in linked}
    return roots.pop() if len(roots) == 1 else None


def _camera_span(rig: Rig) -> float:
    """Diagonal of the camera positions' bounding box over the recording."""
    if rig.tf is None and rig.poses is None:
        return 0.0
    try:
        t0, t1 = rig.color.get_time_range()
    except LookupError:
        return 0.0  # live store, nothing recorded yet
    seen = [rig.camera_pose(t0 + (t1 - t0) * k / 11) for k in range(12)]
    positions = [[p.position.x, p.position.y, p.position.z] for p in seen if p is not None]
    if len(positions) < 2:
        return 0.0
    spread = np.array(positions)
    return float(np.linalg.norm(spread.max(axis=0) - spread.min(axis=0)))


def _heading_series(rig: Rig, spans: list[tuple[float, float]]) -> tuple[np.ndarray, np.ndarray]:
    """Camera heading rate over the given spans: (rate midpoints, rates).

    A rigid mount adds a constant offset, so a poses stream's base yaw rate is
    the camera heading rate. A tf rig has no pose stream; the optical axis is
    sampled through tf on each span's frame-period grid instead.
    """
    times: list[float] = []
    headings: list[float] = []
    if rig.poses is not None:
        for obs in rig.poses.after(spans[0][0]).before(spans[-1][1]):
            q = obs.pose_stamped.orientation
            times.append(obs.ts)
            headings.append(
                np.arctan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            )
    else:
        for lo, hi in spans:
            frame_ts = [obs.ts for obs in rig.color.after(lo).before(hi)]
            if len(frame_ts) < 2:
                continue
            step = float(np.median(np.diff(frame_ts)))
            for t in np.arange(lo, hi, step):
                transform = rig.world_to_optical(float(t))
                if transform is None:
                    continue
                axis = (-transform).to_matrix()[:3, 2]
                times.append(float(t))
                headings.append(np.arctan2(axis[1], axis[0]))
    if len(times) < 3:
        return np.empty(0), np.empty(0)
    stamps = np.array(times)
    yaw = np.unwrap(np.array(headings))
    return (stamps[1:] + stamps[:-1]) / 2, np.diff(yaw) / np.diff(stamps)


def estimate_color_delay(rig: Rig) -> float:
    """Constant lag of color timestamps behind the pose source, in seconds.

    Optical-flow yaw rate is correlated against camera heading rate over a grid
    of candidate lags; the strongest is the delay. The estimate validates
    itself: the full sample and its two interleaved halves must agree to within
    one frame period - the measurement's own resolution - and every peak must
    be interior to the searched span. Without usable rotation it keeps 0.
    """
    import cv2

    try:
        t0, t1 = rig.color.get_time_range()
    except LookupError:
        return 0.0

    fx = rig.cameras[rig.optical_frame].K[0]
    flow_ts: list[float] = []
    flow_rate: list[float] = []
    flow_dt: list[float] = []
    span = max(t1 - t0 - DELAY_WINDOW_S, 0.0)
    starts = [t0 + span * k / max(DELAY_WINDOWS - 1, 1) for k in range(DELAY_WINDOWS)]
    windows = [(lo, lo + DELAY_WINDOW_S) for lo in starts]
    for lo, hi in windows:
        previous: np.ndarray | None = None
        previous_ts = 0.0
        for obs in rig.color.after(lo).before(hi):
            frame = obs.data.to_opencv()
            scale = 640 / frame.shape[1]
            gray = cv2.resize(
                cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY), (640, int(frame.shape[0] * scale))
            )
            if previous is not None and obs.ts > previous_ts:
                corners = cv2.goodFeaturesToTrack(previous, 150, 0.01, 12)
                if corners is not None:
                    moved, status, _err = cv2.calcOpticalFlowPyrLK(previous, gray, corners, None)  # type: ignore[call-overload]
                    tracked = status.ravel().astype(bool)
                    if tracked.any():
                        dx = float(np.median(moved[tracked, 0, 0] - corners[tracked, 0, 0])) / scale
                        flow_ts.append(0.5 * (obs.ts + previous_ts))
                        flow_rate.append(dx / fx / (obs.ts - previous_ts))
                        flow_dt.append(obs.ts - previous_ts)
            previous, previous_ts = gray, obs.ts
    if len(flow_ts) < 4:
        return 0.0  # each interleaved half needs a correlation of its own

    margin = float(DELAY_LAGS[-1])
    rate_t, rates = _heading_series(rig, [(lo - margin, hi + margin) for lo, hi in windows])
    if len(rates) == 0:
        return 0.0

    flow_t = np.array(flow_ts)
    flow = np.array(flow_rate)

    def lag_of(sel: np.ndarray) -> float | None:
        strength = np.abs(
            np.array(
                [
                    np.corrcoef(flow[sel], np.interp(flow_t[sel] - lag, rate_t, rates))[0, 1]
                    for lag in DELAY_LAGS
                ]
            )
        )
        if not np.isfinite(strength).all():
            return None
        peak = int(strength.argmax())
        if peak == 0 or peak == len(DELAY_LAGS) - 1:
            return None  # the true lag is outside the searched span
        lag = float(DELAY_LAGS[peak])
        a, b, c = strength[peak - 1], strength[peak], strength[peak + 1]
        denominator = a - 2 * b + c
        if denominator < 0:
            lag += 0.5 * (a - c) / denominator * float(DELAY_LAGS[1] - DELAY_LAGS[0])
        return lag

    everything = np.arange(len(flow))
    lags = [lag_of(everything), lag_of(everything[0::2]), lag_of(everything[1::2])]
    if any(lag is None for lag in lags):
        return 0.0
    estimates = cast("list[float]", lags)
    if max(estimates) - min(estimates) > float(np.median(flow_dt)):
        return 0.0
    return estimates[0]


def _images(store: Any, names: list[str]) -> tuple[dict[str, str], list[tuple[str, str]]]:
    """Color stream per optical frame, and every metric-depth stream as (frame, name).

    Several color streams in one frame (a recording that also stored a derived
    feed) resolve to the highest-rate one.
    """
    color: dict[str, str] = {}
    depth: list[tuple[str, str]] = []
    for name in names:
        stream = store.stream(name)
        if stream.count() == 0:
            continue
        image = stream.first().data
        frame, data = image.frame_id, image.data
        if data.dtype == np.uint16 or data.dtype.kind == "f":
            depth.append((frame, name))
        elif (
            data.ndim == 3
            and data.shape[2] == 3
            and not np.array_equal(data[..., 0], data[..., 1])
        ):
            held = color.get(frame)
            if held is None or _rate(stream) > _rate(store.stream(held)):
                color[frame] = name
        else:
            logger.info(f"rig: image stream {name!r} is neither color nor metric depth")
    return color, depth


def _cameras(store: Any, roles: dict[str, Any], names: list[str], types: dict[str, type]):
    """Intrinsics per optical frame: an inline manifest dict, or every
    CameraInfo stream keyed by the frame it calibrates.

    Several infos in one frame (a colour/depth pair) resolve by name order, so
    the colour one wins; infos in different frames are different cameras.
    """
    from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo as CameraInfoMsg

    role = roles.get("camera_info")
    if isinstance(role, dict):
        info = CameraInfoMsg(
            height=role["height"],
            width=role["width"],
            distortion_model=role.get("distortion_model", ""),
            D=role.get("D"),
            K=role["K"],
            R=role.get("R"),
            P=role.get("P"),
            frame_id=role["frame_id"],
        )
        return {info.frame_id: info}
    found = (
        [role]
        if isinstance(role, str)
        else sorted(n for n in names if types[n] is CameraInfoMsg and store.stream(n).count())
    )
    cameras = {}
    for name in found:  # sorted, so a color info wins over its depth twin
        info = store.stream(name).first().data
        cameras.setdefault(info.frame_id, info)
    return cameras


@dataclass
class Rig:
    """2D mask to world geometry.

    Exactly one of ``tf`` / (``mounts`` + ``poses``) gives the world-to-optical
    transform, and one of ``depth`` / ``cloud`` gives geometry. ``cameras`` and
    ``mounts`` key on optical frame id, resolved per image by
    :meth:`camera_frame`. ``color`` is one stream, so a rig with several
    cameras needs their frames registered here and their feeds interleaved
    into it by whoever builds the rig.
    """

    cameras: dict[str, CameraInfo]
    color: Any  # color_image stream
    world_frame: str
    tf: TFLookup | None = None
    mounts: dict[str, Transform] = field(default_factory=dict)
    poses: Any = None  # stream carrying world base poses (e.g. odom)
    depth: Any = None  # aligned depth stream, lifted via from_depth
    cloud: Any = None  # pointcloud stream, lifted via from_2d after registration
    tf_tolerance: float = TF_TOLERANCE
    cloud_accum_s: float = CLOUD_ACCUM_S
    color_delay: float = 0.0  # s - color timestamps lag the pose stream by this
    embed_hz: float = EMBED_HZ
    mobile: bool = False  # camera rides a mobile base: room-scale defaults
    plane_cache: dict[tuple[int, int], SupportPlane] = field(default_factory=dict, repr=False)
    # ts -> (merged cloud, the rolling map's pitch; None for scan sources)
    _clouds: OrderedDict[float, tuple[PointCloud2, float | None]] = field(
        default_factory=OrderedDict, repr=False, init=False
    )
    # (ts, plane) -> per-column floor table of that frame's merged cloud
    _shells: OrderedDict[tuple[float, int], tuple[np.ndarray, np.ndarray, float, np.ndarray]] = (
        field(default_factory=OrderedDict, repr=False, init=False)
    )
    _scans: OrderedDict[float, np.ndarray | None] = field(
        default_factory=OrderedDict, repr=False, init=False
    )

    @classmethod
    def from_store(
        cls,
        store: Any,
        manifest: dict[str, Any] | None = None,
        overrides: dict[str, str] | None = None,
    ) -> Rig:
        """Recognize the store's shape without depending on stream names.

        Per role: ``overrides``, then ``manifest`` (passed in, or the
        ``<db>.rig.json`` sidecar), then discovery. Cameras key on the optical
        frame they calibrate and their colour and depth streams are the Image
        streams stamped in that frame, so resolution is frame-driven rather than a
        cascade of name tie-breaks. Ambiguity the frames cannot settle, and
        missing calibration, raise with the candidates.
        """
        from dimos.msgs.sensor_msgs.Image import Image as ImageMsg
        from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2 as PointCloudMsg
        from dimos.msgs.tf2_msgs.TFMessage import TFMessage

        names = store.list_streams()
        types = {name: store.stream(name).data_type for name in names}

        if manifest is None:
            path = getattr(store.config, "path", None)
            sidecar = Path(f"{path}.rig.json") if path else None
            if sidecar is not None and sidecar.exists():
                manifest = json.loads(sidecar.read_text())
                logger.info(f"rig: manifest {sidecar}")
        roles: dict[str, Any] = {**(manifest or {}), **(overrides or {})}
        claimed = {v for v in roles.values() if isinstance(v, str)}

        tf_names = [n for n in names if types[n] is TFMessage]
        tf_name = tf_names[0] if len(tf_names) == 1 else ("tf" if "tf" in tf_names else None)
        tf = StreamTF.from_store(store, tf_name) if tf_name is not None else None

        cameras = _cameras(store, roles, names, types)
        image_names = [n for n in names if types[n] is ImageMsg and n not in claimed]
        by_frame_color, depth_streams = _images(store, image_names)

        color_name = roles.get("color")
        if color_name is None:
            # a camera's own frame first, then any colour feed, then a live store's
            # not-yet-filled one
            ranked = [by_frame_color[f] for f in cameras if f in by_frame_color]
            ranked += [n for n in by_frame_color.values() if n not in ranked]
            ranked += [n for n in image_names if store.stream(n).count() == 0]
            color_name = next(iter(ranked), None)
        if color_name is None:
            raise ValueError(f"no color image stream among {names}; pass a manifest or --color")
        claimed.add(color_name)

        color = store.stream(color_name)
        color_frame = color.first().data.frame_id if color.count() else None
        depth_name = roles.get("depth")
        if depth_name is None:
            # aligned depth shares the colour frame; otherwise the only depth stream
            # stands, and a choice between several is the caller's
            aligned = sorted(n for f, n in depth_streams if f == color_frame)
            elsewhere = [n for f, n in depth_streams if f != color_frame]
            if aligned:
                depth_name = aligned[0]
            elif len(elsewhere) > 1:
                raise ValueError(f"several depth streams {elsewhere}; pass --depth")
            elif elsewhere:
                depth_name = elsewhere[0]

        cloud_name = roles.get("cloud")
        if cloud_name is None and depth_name is None:
            cloud_names = [n for n in names if types[n] is PointCloudMsg and n not in claimed]
            if len(cloud_names) > 1:
                raise ValueError(f"several pointcloud streams {cloud_names}; pass --cloud")
            cloud_name = cloud_names[0] if cloud_names else None

        depth = store.stream(depth_name) if depth_name is not None else None
        cloud = store.stream(cloud_name) if cloud_name is not None and depth is None else None
        if cloud_name is not None:
            claimed.add(cloud_name)

        if not cameras:
            # embed-only stores may carry no calibration; geometry raises on use
            cameras = {OPTICAL_FRAME: cast("CameraInfo", None)}
        if color_frame in cameras:
            optical = color_frame
        elif len(cameras) == 1:
            optical = next(iter(cameras))  # images stamped in a frame calibration never names
        else:
            raise ValueError(
                f"color stream {color_name!r} is stamped {color_frame!r}, which is none of the "
                f"cameras {sorted(cameras)}; name the right camera_info in the manifest"
            )
        cameras = {optical: cameras[optical], **cameras}  # the colour camera answers first

        world_frame = WORLD_FRAME
        if tf is not None:
            world_frame = _tf_root(store, cast("str", tf_name), tf, optical) or world_frame
        elif cloud is not None:
            try:
                world_frame = cloud.first().data.frame_id
            except LookupError:
                pass  # live store, nothing recorded yet

        mounts: dict[str, Transform] = {}
        if isinstance(mount := roles.get("base_to_optical"), dict):
            mounts[optical] = Transform(
                translation=Vector3(*mount["translation"]),
                rotation=Quaternion(*mount["rotation"]),
                frame_id=mount.get("frame_id", "base_link"),
                child_frame_id=optical,
            )

        poses = None
        if tf is None:
            poses_name = roles.get("poses")
            if poses_name is None:
                posed = [
                    n
                    for n in names
                    if n not in claimed
                    and store.stream(n).count()
                    and store.stream(n).first().pose_tuple is not None
                ]
                poses_name = max(posed, key=lambda n: _rate(store.stream(n))) if posed else None
            poses = store.stream(poses_name) if poses_name is not None else None

        if depth is not None or cloud is not None:
            if cameras[optical] is None:
                raise ValueError(
                    "store has 3D geometry but no camera calibration; add a CameraInfo stream "
                    "role or an inline camera_info to the <db>.rig.json manifest"
                )
            if tf is None and (poses is None or not mounts):
                raise ValueError(
                    "store has no tf; a pose-stamped rig needs a poses stream and a "
                    "base_to_optical mount in the <db>.rig.json manifest"
                )

        rig = cls(
            cameras=cameras,
            color=color,
            world_frame=world_frame,
            tf=tf,
            mounts=mounts,
            poses=poses,
            depth=depth,
            cloud=cloud,
        )

        span = _camera_span(rig)
        rig.mobile = span > MOBILE_SPAN_M
        if rig.mobile:
            rig.embed_hz = WALK_EMBED_HZ
            if cameras[optical] is not None:
                rig.color_delay = estimate_color_delay(rig)
        logger.info(
            f"rig: color={color_name!r} depth={depth_name!r} cloud={cloud_name!r} tf={tf_name!r} "
            f"world={world_frame!r} span={span:.1f}m mobile={rig.mobile} "
            f"color_delay={rig.color_delay * 1000:.0f}ms"
        )
        return rig

    @property
    def optical_frame(self) -> str:
        """The default camera: the one the rig's ``color`` stream feeds.

        Discovery registers it first; a hand-built rig orders its own dict.
        """
        return next(iter(self.cameras))

    def camera_frame(self, frame: str | None) -> str:
        """Which camera resolves a lookup: the image's own, if the rig has it.

        A recording can stamp its images in a frame its calibration never
        names, so on a one-camera rig any frame falls to that camera. With
        several, nothing says which one took the image, and folding would
        silently project through the wrong model, so it is an error.
        """
        if frame is None or frame in self.cameras:
            return frame or self.optical_frame
        if len(self.cameras) > 1:
            raise KeyError(
                f"image frame {frame!r} is none of the rig's cameras {sorted(self.cameras)}"
            )
        return self.optical_frame

    # pose

    def world_to_optical(self, ts: float, frame: str | None = None) -> Transform | None:
        frame = self.camera_frame(frame)
        ts -= self.color_delay  # color stamps lag; the capture instant is earlier
        if self.tf is not None:
            return self.tf.get(frame, self.world_frame, ts, self.tf_tolerance)
        pose = self.pose_at(ts)
        if pose is None:
            return None
        mount = self.mounts[frame]
        return -(Transform.from_pose(mount.frame_id, pose) + mount)

    def pose_at(self, ts: float) -> PoseStamped | None:
        """World base pose at ts, interpolated between bracketing samples.

        A walking robot covers centimeters per pose period, so snapping to a
        sample misplaces the projection. Outside the bracketed span the
        nearest sample in the tolerance window stands.
        """
        from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped

        candidates = list(self.poses.at(ts, self.tf_tolerance))
        if not candidates:
            return None
        earlier = [o for o in candidates if o.ts <= ts]
        later = [o for o in candidates if o.ts > ts]
        if not earlier or not later:
            pose: PoseStamped | None = min(candidates, key=lambda o: abs(o.ts - ts)).pose_stamped
            return pose
        a = max(earlier, key=lambda o: o.ts).pose_stamped
        b = min(later, key=lambda o: o.ts).pose_stamped
        alpha = (ts - a.ts) / (b.ts - a.ts)
        qa = np.array([a.orientation.x, a.orientation.y, a.orientation.z, a.orientation.w])
        qb = np.array([b.orientation.x, b.orientation.y, b.orientation.z, b.orientation.w])
        if float(qa @ qb) < 0:
            qb = -qb
        q = (1 - alpha) * qa + alpha * qb
        q /= np.linalg.norm(q)
        return PoseStamped(
            ts=ts,
            frame_id=a.frame_id,
            position=a.position + (b.position - a.position) * alpha,
            orientation=(float(q[0]), float(q[1]), float(q[2]), float(q[3])),
        )

    def camera_pose(self, ts: float, frame: str | None = None) -> PoseStamped | None:
        """World pose of a camera's optical frame at ts."""
        transform = self.world_to_optical(ts, frame)
        return (-transform).to_pose() if transform is not None else None

    def index_pose(self, obs: Observation[Any]) -> Pose | PoseStamped | None:
        """tf rigs stamp the optical pose; pose-stamped rigs keep the base one."""
        if self.tf is not None:
            return self.camera_pose(obs.ts, obs.data.frame_id)
        return obs.pose

    # geometry

    def frame_at(self, obs: Observation[Any]) -> Image:
        """The colour frame an index observation refers to.

        A live index carries the image itself; a replay index carries only what
        frame selection needs, so its pixels come back from the colour stream,
        which must still hold the frame the index was built from.
        """
        from dimos.msgs.sensor_msgs.Image import Image

        if isinstance(obs.data, Image):
            return obs.data
        image: Image = self.color.at(obs.ts, FRAME_TOLERANCE).first().data
        return image

    def depth_at(self, ts: float) -> Image | None:
        """Temporal join: aligned depth frame for a color timestamp."""
        try:
            depth: Image = self.depth.at(ts, DEPTH_TOLERANCE).first().data
        except LookupError:
            return None
        return depth

    def registered_scan(self, scan: Observation[PointCloud2]) -> np.ndarray | None:
        """A scan's world-frame points, registered via tf, decoded once per run."""
        key = scan.ts
        if key in self._scans:
            self._scans.move_to_end(key)
            return self._scans[key]
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
        self._scans[key] = points
        if len(self._scans) > _SCAN_CACHE_MAX:
            self._scans.popitem(last=False)
        return points

    def cloud_at(self, ts: float) -> PointCloud2 | None:
        """World-frame geometry at ts: the window's scans merged.

        A stream whose next snapshot re-reports the majority of the nearest
        one's exact points is a rolling occupancy map, already integrated over
        the area it covers and cleared of what moved away: those merge
        nearest-ts first, a farther snapshot contributing only cells outside
        every nearer one's XY coverage, since plain accumulation would
        resurrect each moved object's trail. Scans that never repeat are fresh
        samples of a static scene and accumulate whole; quantization alone
        proves nothing, a mm-integer wire format grids a scan without making
        it a map.
        """
        held = self._clouds.get(ts)
        if held is not None:
            self._clouds.move_to_end(ts)
            return held[0]
        from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2

        scans = self.cloud.after(ts - self.cloud_accum_s).before(ts + self.cloud_accum_s)
        pairs = [
            (scan.ts, points)
            for scan in scans
            if (points := self.registered_scan(scan)) is not None
        ]
        if not pairs:
            return None
        ordered = sorted(pairs, key=lambda pair: abs(pair[0] - ts))
        nearest = ordered[0][1]
        quantum = None
        if len(ordered) > 1:
            rows = np.dtype((np.void, nearest.dtype.itemsize * nearest.shape[1]))
            near_rows = np.ascontiguousarray(nearest).view(rows).ravel()
            next_rows = np.ascontiguousarray(ordered[1][1]).view(rows).ravel()
            if np.isin(next_rows, near_rows).mean() > 0.5:
                quantum = lattice_quantum(nearest)
        if len(ordered) == 1:
            points = nearest
        elif quantum is None:
            points = np.vstack([p for _t, p in pairs])
        else:
            anchor = nearest[0, :2]
            cells = [
                np.round((part[:, :2] - anchor) / quantum).astype(np.int64) for _t, part in ordered
            ]
            mins = [c.min(axis=0) for c in cells]
            maxs = [c.max(axis=0) for c in cells]
            base = np.minimum.reduce(mins)
            covered = np.zeros(tuple(np.maximum.reduce(maxs) - base + 1), dtype=bool)
            kept: list[np.ndarray] = []
            for (_t, part), c, lo, hi in zip(ordered, cells, mins, maxs, strict=True):
                fresh = ~covered[c[:, 0] - base[0], c[:, 1] - base[1]]
                if fresh.any():
                    kept.append(part[fresh])
                covered[
                    lo[0] - base[0] : hi[0] + 1 - base[0], lo[1] - base[1] : hi[1] + 1 - base[1]
                ] = True
            points = np.vstack(kept)
        merged = PointCloud2.from_numpy(points, frame_id=self.world_frame, timestamp=ts)
        self._clouds[ts] = (merged, quantum)
        if len(self._clouds) > _CLOUD_CACHE_MAX:
            self._clouds.popitem(last=False)
        return merged

    def _quantum(self, ts: float) -> float | None:
        """The lattice pitch of the merged cloud at ts; None for scan sources."""
        self.cloud_at(ts)
        return self._clouds[ts][1]

    def _shell_table(
        self, ts: float, plane: SupportPlane
    ) -> tuple[np.ndarray, np.ndarray, float, np.ndarray] | None:
        """Per-column floor of the merged cloud; None for continuous sources.

        A rolling map registers each snapshot with its own odometry error, so
        the support sits at a different absolute level per region; a point's
        own XY column is the local reference one global plane cannot be.
        """
        cloud = self.cloud_at(ts)
        quantum = self._clouds[ts][1] if cloud is not None else None
        if quantum is None:
            return None
        key = (ts, id(plane))
        held = self._shells.get(key)
        if held is not None:
            self._shells.move_to_end(key)
            return held
        points = cloud.as_numpy()[0]  # type: ignore[union-attr]
        anchor = points[0, :2]
        keys = _column_keys(points, quantum, anchor)
        heights = plane.height_above(points)
        order = np.argsort(keys)
        keys_sorted = keys[order]
        starts = np.nonzero(np.concatenate(([True], np.diff(keys_sorted) != 0)))[0]
        table = (keys_sorted[starts], np.minimum.reduceat(heights[order], starts), quantum, anchor)
        self._shells[key] = table
        if len(self._shells) > _CLOUD_CACHE_MAX:
            self._shells.popitem(last=False)
        return table

    def support_heights(self, ts: float, plane: SupportPlane, points: np.ndarray) -> np.ndarray:
        """Signed heights above the frame's support shell.

        Depth and continuous-scan sources measure against the fitted plane; a
        rolling map measures against its own column floor, so ``points`` must
        come from that frame's merged cloud, which every projected lift is.
        """
        heights = plane.height_above(points)
        if self.cloud is None:
            return heights
        table = self._shell_table(ts, plane)
        if table is None:
            return heights
        col_keys, col_floor, quantum, anchor = table
        idx = np.searchsorted(col_keys, _column_keys(points, quantum, anchor))
        local: np.ndarray = heights - col_floor[idx]
        return local

    def _support_strip(self, ts: float, plane: SupportPlane, shell: float, gap: float = 0.3) -> Any:
        """Drop support-shell points outside the object's own stance.

        A misaligned mask row collects the support along the whole view ray;
        shell points survive only within the camera-range span of the
        detection's above-shell structure - under the object, not along the
        approach. Points below the shell never survive; a detection with no
        structure above it passes untouched.
        """

        def filter_func(det: Any, pc: Any, ci: Any, tf: Any) -> Any:
            points, _ = pc.as_numpy()
            local = self.support_heights(ts, plane, points)
            above = local > shell
            if not above.any():
                return pc
            camera = tf.inverse().translation.to_numpy()
            ranges = np.linalg.norm(points - camera, axis=1)
            stance = np.sort(ranges[above])
            splits = np.nonzero(np.diff(stance) > gap)[0]
            starts = np.concatenate(([0], splits + 1))
            ends = np.concatenate((splits + 1, [len(stance)]))
            median_idx = np.searchsorted(stance, np.median(stance))
            start, end = next(
                (s, e) for s, e in zip(starts, ends, strict=True) if s <= median_idx < e
            )
            in_stance = (ranges >= stance[start]) & (ranges <= stance[end - 1])
            keep = above | ((local >= -shell) & in_stance)
            from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2

            return PointCloud2.from_numpy(points[keep], frame_id=pc.frame_id, timestamp=pc.ts)

        return filter_func

    def lift(
        self, detections: ImageDetections2D, plane: SupportPlane | None = None
    ) -> ImageDetections3DPC | None:
        """2D detections to world-frame clouds, or None without geometry/pose.

        The shell is the support's own occupied band: a plane crossing a
        lattice straddles at most two levels, so a rolling map's shell ends
        halfway to the third, and a continuous source's is the fit's inlier
        distance.
        """
        frame = self.camera_frame(detections.image.frame_id)
        transform = self.world_to_optical(detections.ts, frame)
        if transform is None:
            return None
        camera_info = self.cameras[frame]
        if self.depth is not None:
            depth = self.depth_at(detections.ts)
            if depth is None:
                return None
            return ImageDetections3DPC.from_depth(detections, depth, camera_info, transform)
        cloud = self.cloud_at(detections.ts)
        if cloud is None:
            return None
        filters = _CLOUD_LIFT_FILTERS
        if plane is not None:
            quantum = self._clouds[detections.ts][1]
            shell = 1.5 * quantum if quantum is not None else PLANE_DISTANCE_CLOUD
            filters = [self._support_strip(detections.ts, plane, shell), *_CLOUD_LIFT_FILTERS]
        return ImageDetections3DPC.from_2d(detections, cloud, camera_info, transform, filters)

    def backdrop(self, ts: float, depth_trunc: float = 1.5) -> PointCloud2 | None:
        """World-frame scene cloud around ts, for plane fits and rendering."""
        if self.depth is None:
            return self.cloud_at(ts)
        from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2

        depth = self.depth_at(ts)
        if depth is None:
            return None
        try:
            color = self.color.at(ts, 0.1).first().data
        except LookupError:
            return None
        frame = self.camera_frame(color.frame_id)
        transform = self.world_to_optical(ts, frame)
        if transform is None:
            return None
        return PointCloud2.from_rgbd(
            color, depth, self.cameras[frame], depth_scale=0.001, depth_trunc=depth_trunc
        ).transform(-transform)

    def default_localize_policy(self) -> LocalizePolicy:
        base = ROOM_LOCALIZE_POLICY if self.mobile else LocalizePolicy()
        if self.cloud is None:
            return base
        return replace(base, min_points=CLOUD_MIN_POINTS)
