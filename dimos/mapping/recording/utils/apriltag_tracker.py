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

"""Incremental AprilTag tracking — the streaming counterpart of `apriltags.py`.

`apriltags.detect_apriltags` is batch: it loads a whole image stream, gates,
clusters, and writes once. This module processes one image at a time, keeping a
rolling window of recent detections per tag id and emitting a `finished_tags`
estimate when a tag has not been seen for longer than the window (no fresher
observation can revise it). Detection math (solvePnP, reprojection error,
medoid) is shared with `apriltags.py`.
"""

from __future__ import annotations

from copy import copy
from dataclasses import dataclass, field
import time
from typing import Any

import cv2
import numpy as np
from scipy.spatial.transform import Rotation

from dimos.mapping.recording.utils.apriltags import (
    DEFAULT_CLUSTER_GAP_SEC,
    DEFAULT_MAX_DISTANCE_M,
    DEFAULT_MAX_REPROJ_PX,
    DEFAULT_ROTATION_WEIGHT_M_PER_RAD,
    _pose_distance,
    estimate_marker_pose,
    make_detector,
    reprojection_error_px,
)

_DEFAULT_MARKER_LENGTH = 0.10
_DEFAULT_DICTIONARY = "DICT_APRILTAG_36h11"


@dataclass
class TagInfo:
    tag_number: float
    confidence: float
    pose: list[float]  # [x, y, z, qx, qy, qz, qw] relative to camera
    ts: float
    camera_frame_id: str = ""


@dataclass
class GroupingOptions:
    tag_ids_to_ignore: list[float] = field(default_factory=list)
    time_window: float = DEFAULT_CLUSTER_GAP_SEC
    max_distance: float = DEFAULT_MAX_DISTANCE_M
    min_confidence: float = 0.0


class AprilTagger:
    def __init__(
        self,
        camera_intrinsics: Any,
        grouping_options: Any,
        tags_ids_to_ignore: list[float],
    ) -> None:
        self.camera_intrinsics = camera_intrinsics
        self.grouping_options = grouping_options
        self.tags_ids_to_ignore = tags_ids_to_ignore

    def get_tag(self, img: Any) -> list[TagInfo]:
        return pure_get_tags(img, self.camera_intrinsics)

    def iter_april_tag_groups(
        self,
        next_observation: Any,
        options: Any = None,
        state: dict[str, Any] | None = None,
    ) -> tuple[list[TagInfo], dict[float, TagInfo], dict[float, TagInfo]]:
        if state is None:
            state = {}
        if options is None:
            options = self.grouping_options
        recent_tags, tag_estimates, finished_tags = pure_iter_april_tag_groups(
            next_observation,
            self.camera_intrinsics,
            options,
            state.get("recent_tags", []),
            state.get("tag_estimates", {}),
        )
        state["recent_tags"] = recent_tags
        state["tag_estimates"] = tag_estimates
        return recent_tags, tag_estimates, finished_tags


def pure_get_tags(img: Any, camera_intrinsics: Any) -> list[TagInfo]:
    """Detect AprilTags in an image and return relative poses via solvePnP.

    img: ndarray or an Image msg (frame_id/ts are carried onto each TagInfo).
    camera_intrinsics: dict with "intrinsics" (3x3 ndarray), and optionally
    "distortion", "marker_length", "dictionary".
    """
    intrinsics = camera_intrinsics["intrinsics"]
    distortion = camera_intrinsics.get("distortion", np.zeros(5))
    marker_length = camera_intrinsics.get("marker_length", _DEFAULT_MARKER_LENGTH)
    dictionary = camera_intrinsics.get("dictionary", _DEFAULT_DICTIONARY)

    camera_frame_id = getattr(img, "frame_id", "")
    ts = getattr(img, "ts", None)
    if ts is None:
        ts = time.time()

    if isinstance(img, np.ndarray):
        bgr = img
    elif hasattr(img, "numpy"):
        bgr = img.numpy()
    elif hasattr(img, "data"):
        bgr = np.asarray(img.data)
    else:
        bgr = np.asarray(img)
    if bgr.ndim == 2:
        bgr = cv2.cvtColor(bgr, cv2.COLOR_GRAY2BGR)

    detector = make_detector(dictionary)
    all_corners, marker_ids, _ = detector.detectMarkers(bgr)
    if marker_ids is None:
        return []

    tags: list[TagInfo] = []
    for corners, marker_id in zip(all_corners, marker_ids.flatten(), strict=False):
        pose_result = estimate_marker_pose(corners, marker_length, intrinsics, distortion)
        if pose_result is None:
            continue
        rotation_vector, translation_vector = pose_result
        quaternion = Rotation.from_rotvec(rotation_vector.reshape(3)).as_quat()
        translation = translation_vector.reshape(3)
        pose = [
            float(translation[0]),
            float(translation[1]),
            float(translation[2]),
            float(quaternion[0]),
            float(quaternion[1]),
            float(quaternion[2]),
            float(quaternion[3]),
        ]
        reproj = reprojection_error_px(
            corners,
            rotation_vector,
            translation_vector,
            marker_length,
            intrinsics,
            distortion,
        )
        confidence = max(0.0, 1.0 - reproj / DEFAULT_MAX_REPROJ_PX)
        tags.append(
            TagInfo(
                tag_number=float(marker_id),
                confidence=confidence,
                pose=pose,
                ts=float(ts),
                camera_frame_id=camera_frame_id,
            )
        )
    return tags


def pure_iter_april_tag_groups(
    next_observation: Any,
    camera_intrinsics: Any,
    options: Any,
    recent_tags: list[TagInfo],
    tag_estimates: dict[float, TagInfo],
) -> tuple[list[TagInfo], dict[float, TagInfo], dict[float, TagInfo]]:
    """Returns (recent_tags, tag_estimates, finished_tags).

    finished_tags holds estimates whose tag id aged out of the time window this
    iteration — no fresher observation can revise them, so they are final."""
    tag_estimates = copy(tag_estimates)
    recent_tags = list(recent_tags)

    tags = pure_get_tags(next_observation.color_image, camera_intrinsics)
    recent_tags.extend(tags)

    tag_ids_to_ignore = set(getattr(options, "tag_ids_to_ignore", []))
    time_window = getattr(options, "time_window", DEFAULT_CLUSTER_GAP_SEC)
    max_distance = getattr(options, "max_distance", DEFAULT_MAX_DISTANCE_M)
    min_confidence = getattr(options, "min_confidence", 0.0)

    # Purge tags outside the time window
    if recent_tags:
        latest_ts = max(tag.ts for tag in recent_tags)
        recent_tags = [tag for tag in recent_tags if latest_ts - tag.ts <= time_window]

    all_recent_tag_ids = set(
        tag.tag_number for tag in recent_tags if tag.tag_number not in tag_ids_to_ignore
    )

    # Tag ids with an estimate but no surviving observations are finalized
    finished_tags = {
        tag_id: estimate
        for tag_id, estimate in tag_estimates.items()
        if tag_id not in all_recent_tag_ids
    }
    for tag_id in finished_tags:
        del tag_estimates[tag_id]

    for each_tag_id in all_recent_tag_ids:
        tags_for_id = [tag for tag in recent_tags if tag.tag_number == each_tag_id]

        # Filter by distance and confidence
        filtered = [
            tag
            for tag in tags_for_id
            if tag.confidence >= min_confidence
            and float(np.linalg.norm(tag.pose[:3])) <= max_distance
        ]
        if not filtered:
            continue

        # Pick the medoid (most central observation by pose distance)
        if len(filtered) == 1:
            tag_estimates[each_tag_id] = filtered[0]
        else:
            best_index = 0
            best_cost = float("inf")
            for i, tag_i in enumerate(filtered):
                cost = sum(
                    _pose_distance(tag_i.pose, tag_j.pose, DEFAULT_ROTATION_WEIGHT_M_PER_RAD)
                    for j, tag_j in enumerate(filtered)
                    if j != i
                )
                if cost < best_cost:
                    best_cost = cost
                    best_index = i
            tag_estimates[each_tag_id] = filtered[best_index]

    return recent_tags, tag_estimates, finished_tags
