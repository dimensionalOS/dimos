# Copyright 2025-2026 Dimensional Inc.
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

"""ArUco / AprilTag detection as memory2 transforms.

Wraps :func:`dimos.perception.fiducial.marker_detect.detect_markers_in_image`,
emitting one :class:`Detection3DMarker` per marker with ``.pose`` composed into
world frame from the upstream camera pose. Also holds the smoothing helpers,
``MarkersPerFrame`` (collapses marker fan-out back to one ``Detection3DArray``
per source image) and :class:`AggregateTagBursts` (the per-glimpse quality gates plus
one robust aggregated pose per tag visit). :class:`MarkerTfModule` handles live TF
publication.

The gate + aggregation core AggregateTagBursts drives originates in the jnav
apriltag aggregation, dimos PR #2587.

Frames with no upstream ``.pose`` are skipped (debug log): without a camera-in-
world pose we can't honor the always-world-frame output contract.
"""

from __future__ import annotations

from collections.abc import Callable
import dataclasses
import math
from typing import TYPE_CHECKING, Any, cast

import numpy as np

from dimos.memory2.transform import Transformer
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseWithCovariance import PoseWithCovariance
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.vision_msgs.Detection3DArray import Detection3DArray
from dimos.perception.detection.type.detection3d.imageDetections3D import ImageDetections3D
from dimos.perception.detection.type.detection3d.marker import Detection3DMarker
from dimos.perception.fiducial.apriltag_aggregation import (
    AggregationConfig,
    TagAggregator,
    TagEstimate,
    TagObservation,
    matrix_from_pose7,
    pose7_from_matrix,
    tag_covariance,
    tag_noise_scale,
    tag_side_px,
    view_quality,
)
from dimos.perception.fiducial.marker_detect import (
    detect_markers_in_image as _detect_markers_in_image,
)
from dimos.perception.fiducial.marker_pose import (
    camera_info_to_cv_matrices,
    camera_optical_frame_id,
    create_aruco_detector,
)
from dimos.types.timestamped import TimestampedBufferCollection
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from collections.abc import Iterator

    from dimos.memory2.type.observation import Observation

logger = setup_logger()


CameraInfoSource = CameraInfo | Callable[[], CameraInfo | None] | None


def _camera_info_key(info: CameraInfo) -> tuple[Any, ...]:
    """Return an intrinsics-only key for cached OpenCV calibration state."""
    return (
        info.width,
        info.height,
        info.distortion_model,
        tuple(info.K),
        tuple(info.D),
    )


def _pose_tuple_to_transform(
    pose: tuple[float, float, float, float, float, float, float],
    *,
    frame_id: str,
    child_frame_id: str,
    ts: float,
) -> Transform:
    x, y, z, qx, qy, qz, qw = pose
    return Transform(
        translation=Vector3(x, y, z),
        rotation=Quaternion(qx, qy, qz, qw),
        frame_id=frame_id,
        child_frame_id=child_frame_id,
        ts=ts,
    )


def _average_marker_pose(
    buffer: TimestampedBufferCollection[Detection3DMarker],
) -> tuple[Vector3, Quaternion]:
    """Mean translation; sign-aligned linear quaternion mean (small-angle
    approximation of the Markley eigenvector average).
    https://ntrs.nasa.gov/citations/20070017872

    q and -q are the same rotation, so naive averaging can cancel: flip any
    sample whose dot against the first (hemisphere reference) is negative. For
    closely-spaced rotations in a short window this matches a SLERP average.
    """
    items = list(buffer)
    n = len(items)
    cx = sum(d.center.x for d in items) / n
    cy = sum(d.center.y for d in items) / n
    cz = sum(d.center.z for d in items) / n

    ref = items[0].orientation
    qsx = qsy = qsz = qsw = 0.0
    for d in items:
        q = d.orientation
        s = -1.0 if (q.x * ref.x + q.y * ref.y + q.z * ref.z + q.w * ref.w) < 0 else 1.0
        qsx += s * q.x
        qsy += s * q.y
        qsz += s * q.z
        qsw += s * q.w
    norm = math.sqrt(qsx * qsx + qsy * qsy + qsz * qsz + qsw * qsw)
    if norm < 1e-12:
        # Signs cancelled exactly — pathological, fall back to the hemisphere ref.
        return (Vector3(cx, cy, cz), ref)
    return (
        Vector3(cx, cy, cz),
        Quaternion(qsx / norm, qsy / norm, qsz / norm, qsw / norm),
    )


class DetectMarkers(Transformer[Image, Detection3DMarker]):
    """Detect fiducial markers and emit one world-pose observation per marker."""

    def __init__(
        self,
        camera_info: CameraInfoSource,
        marker_length_m: float,
        aruco_dictionary: str = "DICT_APRILTAG_36h11",
        world_frame: str = "world",
        smoothing_window: float = 0.0,
        emit_empty_frames: bool = False,
        ambiguity_ratio_min: float = 1.0,
    ) -> None:
        if marker_length_m <= 0:
            raise ValueError(f"marker_length_m must be > 0, got {marker_length_m}")
        if smoothing_window < 0:
            raise ValueError(f"smoothing_window must be >= 0, got {smoothing_window}")
        self.camera_info = camera_info
        self.marker_length_m = marker_length_m
        self.aruco_dictionary = aruco_dictionary
        self.world_frame = world_frame
        self.smoothing_window = smoothing_window
        self.emit_empty_frames = emit_empty_frames
        self.ambiguity_ratio_min = ambiguity_ratio_min
        self._detector = create_aruco_detector(aruco_dictionary)
        self._camera_info_key: tuple[Any, ...] | None = None
        self._cam_mtx: np.ndarray | None = None
        self._dist: np.ndarray | None = None
        # Per marker_id sliding-window buffer of raw detections, used to emit
        # smoothed pose updates when ``smoothing_window > 0``.
        self._buffers: dict[int, TimestampedBufferCollection[Detection3DMarker]] = {}
        # Tracking (smoothing only): if a marker_id reappears after a gap
        # larger than the buffer window, treat it as a new track. track_id
        # increments monotonically across the whole stream so it's unique.
        self._marker_to_track: dict[int, int] = {}
        self._next_track_id = 0

    def _resolve_camera_info(self) -> CameraInfo | None:
        source = self.camera_info
        info = source() if callable(source) else source
        if info is None:
            return None

        key = _camera_info_key(info)
        if key != self._camera_info_key:
            self._cam_mtx, self._dist = camera_info_to_cv_matrices(info)
            self._detector = create_aruco_detector(self.aruco_dictionary)
            self._camera_info_key = key
        return info

    def __call__(
        self, upstream: Iterator[Observation[Image]]
    ) -> Iterator[Observation[Detection3DMarker]]:
        for obs in upstream:
            pose_tuple = obs.pose_tuple
            if pose_tuple is None:
                logger.debug("marker frame skipped: no camera-in-world pose", obs_id=obs.id)
                continue

            info = self._resolve_camera_info()
            if info is None:
                logger.debug("marker frame skipped: no CameraInfo", obs_id=obs.id)
                continue
            assert self._cam_mtx is not None, "camera matrix unset despite resolved CameraInfo"
            assert self._dist is not None, "dist coeffs unset despite resolved CameraInfo"

            image = obs.data
            image_size_mismatch = (
                info.width
                and info.height
                and (image.width != info.width or image.height != info.height)
            )
            if image_size_mismatch:
                logger.debug(
                    "marker frame skipped: image size != CameraInfo",
                    img_w=image.width,
                    img_h=image.height,
                    info_w=info.width,
                    info_h=info.height,
                )
                continue

            optical_frame = camera_optical_frame_id(image, info)
            t_world_optical = _pose_tuple_to_transform(
                pose_tuple,
                frame_id=self.world_frame,
                child_frame_id=optical_frame,
                ts=obs.ts,
            )

            detections = _detect_markers_in_image(
                image,
                camera_info=info,
                world_T_optical=t_world_optical,
                marker_length_m=self.marker_length_m,
                aruco_dictionary=self.aruco_dictionary,
                world_frame=self.world_frame,
                ambiguity_ratio_min=self.ambiguity_ratio_min,
                detector=self._detector,
                camera_matrix=self._cam_mtx,
                dist_coeffs=self._dist,
            )

            if not detections:
                if self.emit_empty_frames:
                    yield cast(
                        "Observation[Detection3DMarker]",
                        obs.derive(data=None).tag(
                            marker_frame_image=image,
                            marker_frame_count=0,
                        ),
                    )
                continue

            marker_count = len(detections)
            for marker_index, det in enumerate(detections):
                mid = det.marker_id
                # track_id is only for smoothing / mem2 tags; marker identity is marker_id.
                # Without smoothing, use -1 (no temporal track), same as untracked 2D detections.
                if self.smoothing_window > 0:
                    prior_buf = self._buffers.get(mid)
                    prior_last = prior_buf.last() if prior_buf is not None else None
                    if prior_last is None or (obs.ts - prior_last.ts) > self.smoothing_window:
                        self._next_track_id += 1
                        self._marker_to_track[mid] = self._next_track_id
                    track_id = self._marker_to_track[mid]
                else:
                    track_id = -1

                det = dataclasses.replace(det, track_id=track_id)
                yielded_pose = Transform(
                    translation=det.center,
                    rotation=det.orientation,
                    frame_id=self.world_frame,
                    child_frame_id=f"marker_{mid}",
                    ts=obs.ts,
                )

                yielded_det = det
                if self.smoothing_window > 0:
                    # Emit the windowed-mean pose so each detection refines the
                    # same marker's estimate rather than a fresh observation.
                    buf = self._buffers.setdefault(
                        mid, TimestampedBufferCollection(self.smoothing_window)
                    )
                    buf.add(det)
                    avg_center, avg_orient = _average_marker_pose(buf)
                    # Drop `transform` (camera-in-world): the averaged pose spans
                    # many frames, so any single camera transform is inconsistent.
                    yielded_det = dataclasses.replace(
                        det, center=avg_center, orientation=avg_orient, transform=None
                    )
                    yielded_pose = Transform(
                        translation=avg_center,
                        rotation=avg_orient,
                        frame_id=self.world_frame,
                        child_frame_id=f"marker_{mid}",
                        ts=obs.ts,
                    )

                yield obs.derive(data=yielded_det, pose=yielded_pose).tag(
                    marker_id=mid,
                    track_id=track_id,
                    marker_frame_image=image,
                    marker_frame_count=marker_count,
                    marker_frame_index=marker_index,
                )


class MarkersPerFrame(Transformer[Detection3DMarker | None, Detection3DArray]):
    """Collapse marker fan-out back into one Detection3DArray per image frame.

    ``DetectMarkers`` emits one observation per decoded marker, but live LCM
    consumers need one array per processed image (empty ones included).
    ``DetectMarkers(emit_empty_frames=True)`` supplies a ``None`` sentinel for
    empty frames and tags each marker with the source image + frame marker
    count, so this transformer emits without waiting for a later timestamp.
    """

    def __init__(self, frame_id: str = "world") -> None:
        self.frame_id = frame_id

    def __call__(
        self, upstream: Iterator[Observation[Detection3DMarker | None]]
    ) -> Iterator[Observation[Detection3DArray]]:
        pending: list[Detection3DMarker] = []
        pending_obs: Observation[Detection3DMarker | None] | None = None
        pending_ts: float | None = None

        def flush() -> Observation[Detection3DArray] | None:
            nonlocal pending, pending_obs, pending_ts
            if pending_obs is None:
                return None
            result = self._to_array_observation(pending_obs, pending)
            pending = []
            pending_obs = None
            pending_ts = None
            return result

        for obs in upstream:
            det = obs.data
            if det is None:
                flushed = flush()
                if flushed is not None:
                    yield flushed
                yield self._to_array_observation(obs, [])
                continue

            if pending_ts is not None and obs.ts != pending_ts:
                flushed = flush()
                if flushed is not None:
                    yield flushed

            if pending_obs is None:
                pending_obs = obs
                pending_ts = obs.ts
            pending.append(det)

            expected_count = obs.tags.get("marker_frame_count")
            if (
                isinstance(expected_count, int)
                and expected_count > 0
                and len(pending) >= expected_count
            ):
                flushed = flush()
                if flushed is not None:
                    yield flushed

        flushed = flush()
        if flushed is not None:
            yield flushed

    def _to_array_observation(
        self,
        obs: Observation[Detection3DMarker | None],
        detections: list[Detection3DMarker],
    ) -> Observation[Detection3DArray]:
        image = self._source_image(obs, detections)
        msg = ImageDetections3D(image, detections).to_ros_detection3d_array(frame_id=self.frame_id)
        pose = self._source_pose(obs, detections)
        return obs.derive(data=msg, pose=pose).tag(detections_length=len(detections))

    @staticmethod
    def _source_image(
        obs: Observation[Detection3DMarker | None],
        detections: list[Detection3DMarker],
    ) -> Image:
        if detections:
            return detections[0].image
        image = obs.tags.get("marker_frame_image")
        if isinstance(image, Image):
            return image
        raise ValueError("MarkersPerFrame requires marker_frame_image for empty frames")

    @staticmethod
    def _source_pose(
        obs: Observation[Detection3DMarker | None],
        detections: list[Detection3DMarker],
    ) -> Transform | Pose | None:
        # Both branches are re-coerced to the storage 7-tuple by ``derive(pose=)``
        # via observation._to_tuple, which maps Transform.translation/rotation
        # and Pose.position/orientation to (x, y, z, qx, qy, qz, qw).
        if detections and detections[0].transform is not None:
            return detections[0].transform
        return obs.pose


class AggregateTagBursts:
    """Gate each tag glimpse, then publish ONE robustly-aggregated pose per tag VISIT.

    A ``Stream.tap`` callable sitting between ``DetectMarkers`` and
    ``MarkersPerFrame``: it reads every ``Detection3DMarker`` and yields nothing, so
    the per-frame ``detections`` array downstream stays byte-identical. This is the
    only place the ``AggregationConfig`` gates CAN run live -- ``corners_px``,
    ``reprojection_error`` and the camera transform all exist here and every one of
    them is dropped by the wire ``Detection3DArray`` encoding a step later.

    Emission is EDGE-triggered per (marker, visit): the glimpse that first brings a
    marker's window to ``min_observations`` publishes, later glimpses of the same
    burst do not. Upstream ``QualityWindow`` passes at most one frame per
    ``quality_window_s`` (0.5 s default), so a level-triggered publish would fire
    ~2x/s for one unchanged fix -- and at that rate ``min_observations=3`` means a
    tag must hold view ~1.0 s before its first aggregated pose ships.

    ``ambiguity_ratio_min`` is NOT part of this chain: the detector already gates the
    IPPE mirror flip per view, upstream of everything here.
    """

    def __init__(
        self,
        publish: Callable[[Detection3DArray], Any],
        config: AggregationConfig,
        world_frame: str = "world",
    ) -> None:
        self._publish = publish
        self._world_frame = world_frame
        self._aggregator = TagAggregator(config)
        # Markers whose CURRENT burst has already published. A marker leaves the set
        # once TagAggregator's window thins back under min_observations (it left view
        # for longer than time_window_s), which re-arms the edge for its next visit.
        self._burst_counted: set[int] = set()

    def __call__(self, obs: Observation[Detection3DMarker | None]) -> None:
        det = obs.data
        if det is None:
            return  # DetectMarkers(emit_empty_frames=True) sentinel: a tag-free frame

        world_T_marker = (
            det.center.x,
            det.center.y,
            det.center.z,
            det.orientation.x,
            det.orientation.y,
            det.orientation.z,
            det.orientation.w,
        )
        distance_m: float | None = None
        view_angle_deg: float | None = None
        if det.transform is not None:
            # det.transform IS world_T_optical; this inverse is the exact undo of the
            # compose that built world_T_marker, not a second pose estimate.
            optical_T_marker = np.linalg.inv(det.transform.to_matrix()) @ matrix_from_pose7(
                world_T_marker
            )
            distance_m, view_angle_deg = view_quality(pose7_from_matrix(optical_T_marker))
        # else: smoothing_window > 0 emits an averaged pose with transform=None (it
        # spans many frames), so the two camera-relative gates go quiet by design and
        # only the reproj / tag_px gates bite. Blueprint default is 0.0 (off).

        reason = self._aggregator.observe(
            TagObservation(
                ts=obs.ts,
                marker_id=det.marker_id,
                pose=world_T_marker,
                distance_m=distance_m,
                view_angle_deg=view_angle_deg,
                reproj_px=det.reprojection_error,
                tag_px=tag_side_px(det.corners_px),
            )
        )
        if reason is not None:
            return
        estimate = self._aggregator.robust_estimate(det.marker_id)
        if estimate is None:
            self._burst_counted.discard(det.marker_id)  # window thinned -> re-arm the edge
            return
        if det.marker_id in self._burst_counted:
            return  # same burst still satisfied: publishing per frame would fire ~2x/s
        self._burst_counted.add(det.marker_id)
        self._publish(self._aggregated_array(det, estimate))

    def _aggregated_array(self, det: Detection3DMarker, estimate: TagEstimate) -> Detection3DArray:
        """One-detection array carrying this tag's aggregated world_T_marker plus its
        health signal: the covariance and the score are the same number twice, so
        they cannot disagree."""
        scale = tag_noise_scale(estimate.distance_m, estimate.reproj_px)
        x, y, z, qx, qy, qz, qw = estimate.pose
        aggregated = dataclasses.replace(
            det,
            center=Vector3(x, y, z),
            orientation=Quaternion(qx, qy, qz, qw),
            transform=None,  # aggregated across many frames: no ONE camera transform holds
            confidence=min(1.0, 1.0 / scale),  # -> hypothesis.score, dimensionless 0-1
        )
        msg = ImageDetections3D(det.image, [aggregated]).to_ros_detection3d_array(
            frame_id=self._world_frame
        )
        # ASSIGN, never mutate in place: the generated LCM constructor shares ONE
        # default PoseWithCovariance across every ObjectHypothesisWithPose, so writing
        # .covariance would stamp this onto every detection in the process. Same
        # hazard the bbox assignment in detection3d/bbox.py documents.
        msg.detections[0].results[0].pose = PoseWithCovariance(
            Pose(position=aggregated.center, orientation=aggregated.orientation),
            tag_covariance(estimate.pose, scale),
        )
        # n_observations stays OFF the wire and the covariance is NOT divided by n:
        # PnP errors within one visit share a viewpoint and are strongly correlated,
        # so 1/n would read optimistic to any downstream landmark factor.
        logger.info(
            "tag burst aggregated",
            marker_id=det.marker_id,
            n_obs=estimate.n_observations,
            distance_m=None if estimate.distance_m is None else round(estimate.distance_m, 2),
            reproj_px=None if estimate.reproj_px is None else round(estimate.reproj_px, 2),
            score=round(aggregated.confidence, 2),
        )
        return msg
