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

"""ArUco / AprilTag detection as a memory2 transformer.

Wraps the pure helpers in :mod:`dimos.perception.fiducial.marker_pose`
and emits one :class:`Detection3DMarker` observation per detected marker, with
``.pose`` composed into world frame from the upstream observation's
camera-in-world pose. The companion module :class:`MarkerTfModule` remains
the right choice for live TF publication; this transformer is for offline /
mem2-stream composition.

Skips frames where the upstream observation has no ``.pose`` (debug log):
without a camera-in-world pose, we can't honor the "always world-frame"
output contract.
"""

from __future__ import annotations

import dataclasses
import math
from typing import TYPE_CHECKING, Any

import numpy as np

from dimos.memory2.transform import Transformer
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image
from dimos.perception.detection.type.detection3d.marker import Detection3DMarker
from dimos.perception.fiducial.marker_pose import (
    camera_info_to_cv_matrices,
    create_aruco_detector,
    estimate_marker_pose,
    marker_corners_to_bbox,
    marker_reprojection_error,
    rvec_tvec_to_transform,
)
from dimos.types.timestamped import TimestampedBufferCollection
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from collections.abc import Iterator

    from dimos.memory2.type.observation import Observation

logger = setup_logger()


def _average_marker_pose(
    buffer: TimestampedBufferCollection[Detection3DMarker],
) -> tuple[Vector3, Quaternion]:
    """Mean translation; quaternion mean with hemisphere alignment.

    Quaternions q and -q encode the same rotation, so naive averaging can
    cancel. We pick the first sample as a hemisphere reference and flip the
    sign of any sample whose dot product against it is negative before
    summing. For closely-spaced rotations within a short window this is
    indistinguishable from a proper SLERP-style average.
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


def detect_markers_in_image(
    image: Image,
    *,
    camera_info: CameraInfo,
    world_T_optical: Transform,
    marker_length_m: float,
    aruco_dictionary: str,
    world_frame: str = "world",
    detector: Any | None = None,
    camera_matrix: np.ndarray | None = None,
    dist_coeffs: np.ndarray | None = None,
) -> list[Detection3DMarker]:
    """Detect markers in one image and return rich world-frame 3D detections.
    """
    if marker_length_m <= 0:
        raise ValueError(f"marker_length_m must be > 0, got {marker_length_m}")
    if (
        camera_info.width
        and camera_info.height
        and (image.width != camera_info.width or image.height != camera_info.height)
    ):
        return []

    if detector is None:
        detector = create_aruco_detector(aruco_dictionary)
    if (camera_matrix is None) != (dist_coeffs is None):
        raise ValueError("camera_matrix and dist_coeffs must be provided together")
    if camera_matrix is None or dist_coeffs is None:
        camera_matrix, dist_coeffs = camera_info_to_cv_matrices(camera_info)

    gray = image.to_grayscale().as_numpy()
    corners, ids, _ = detector.detectMarkers(gray)
    if ids is None or len(ids) == 0:
        return []

    optical_frame = world_T_optical.child_frame_id or "optical"
    t_world_optical = Transform(
        translation=world_T_optical.translation,
        rotation=world_T_optical.rotation,
        frame_id=world_frame,
        child_frame_id=optical_frame,
        ts=image.ts,
    )
    marker_size = Vector3(marker_length_m, marker_length_m, 0.0)
    detections: list[Detection3DMarker] = []

    for corner_set, mid_arr in zip(corners, ids, strict=True):
        mid = int(mid_arr[0])
        pose = estimate_marker_pose(
            corner_set,
            marker_length_m,
            camera_matrix,
            dist_coeffs,
            distortion_model=camera_info.distortion_model,
        )
        if pose is None:
            continue

        rvec, tvec = pose
        t_optical_marker = rvec_tvec_to_transform(
            rvec,
            tvec,
            frame_id=optical_frame,
            child_frame_id=f"marker_{mid}",
            ts=image.ts,
        )
        t_world_marker = t_world_optical + t_optical_marker

        corners_2d = corner_set.reshape(4, 2).astype(np.float32)
        bbox = marker_corners_to_bbox(corners_2d)
        reprojection_error = marker_reprojection_error(
            corners_2d,
            marker_length_m,
            camera_matrix,
            dist_coeffs,
            rvec,
            tvec,
            distortion_model=camera_info.distortion_model,
        )

        detections.append(
            Detection3DMarker(
                bbox=bbox,
                track_id=-1,
                class_id=mid,
                confidence=1.0,
                name="",  # set in Detection3DMarker.__post_init__ to marker_label
                ts=image.ts,
                image=image,
                center=t_world_marker.translation,
                size=marker_size,
                transform=t_world_optical,
                frame_id=world_frame,
                orientation=t_world_marker.rotation,
                marker_id=mid,
                corners_px=corners_2d,
                dictionary=aruco_dictionary,
                reprojection_error=reprojection_error,
            )
        )

    return detections


class DetectMarkers(Transformer[Image, Detection3DMarker]):
    """Detect fiducial markers and emit one world-pose observation per marker."""

    def __init__(
        self,
        camera_info: CameraInfo,
        marker_length_m: float,
        aruco_dictionary: str = "DICT_APRILTAG_36h11",
        world_frame: str = "world",
        smoothing_window: float = 0.0,
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
        self._detector = create_aruco_detector(aruco_dictionary)
        self._cam_mtx, self._dist = camera_info_to_cv_matrices(camera_info)
        # Per marker_id sliding-window buffer of raw detections, used to emit
        # smoothed pose updates when ``smoothing_window > 0``.
        self._buffers: dict[int, TimestampedBufferCollection[Detection3DMarker]] = {}
        # Tracking (smoothing only): if a marker_id reappears after a gap
        # larger than the buffer window, treat it as a new track. track_id
        # increments monotonically across the whole stream so it's unique.
        self._marker_to_track: dict[int, int] = {}
        self._next_track_id = 0

    def __call__(
        self, upstream: Iterator[Observation[Image]]
    ) -> Iterator[Observation[Detection3DMarker]]:
        info = self.camera_info

        for obs in upstream:
            pose = obs.pose
            if pose is None:
                logger.debug("DetectMarkers: obs %s has no .pose; skipping", obs.id)
                continue

            image = obs.data
            image_size_mismatch = (
                info.width
                and info.height
                and (image.width != info.width or image.height != info.height)
            )
            if image_size_mismatch:
                logger.debug(
                    "DetectMarkers: image %sx%s != CameraInfo %sx%s; skip",
                    image.width,
                    image.height,
                    info.width,
                    info.height,
                )
                continue

            t_world_optical = Transform(
                translation=pose.position,
                rotation=pose.orientation,
                frame_id=self.world_frame,
                child_frame_id="optical",
                ts=obs.ts,
            )

            detections = detect_markers_in_image(
                image,
                camera_info=info,
                world_T_optical=t_world_optical,
                marker_length_m=self.marker_length_m,
                aruco_dictionary=self.aruco_dictionary,
                world_frame=self.world_frame,
                detector=self._detector,
                camera_matrix=self._cam_mtx,
                dist_coeffs=self._dist,
            )
            for det in detections:
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
                    # Buffer raw detections per marker_id over a sliding
                    # window; emit the windowed-mean pose so each successive
                    # detection refines the same marker's estimate instead
                    # of producing a fresh independent observation.
                    buf = self._buffers.setdefault(
                        mid, TimestampedBufferCollection(self.smoothing_window)
                    )
                    buf.add(det)
                    avg_center, avg_orient = _average_marker_pose(buf)
                    # Drop `transform` (camera-in-world): the averaged pose is
                    # built from many frames, so any single camera transform is
                    # inconsistent with center/orientation.
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
                    marker_id=mid, track_id=track_id
                )
