# Copyright 2025 Dimensional Inc.
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

from collections import deque
from collections.abc import Sequence

import numpy as np


def compute_iou(bbox1, bbox2):  # type: ignore[no-untyped-def]
    """
    Compute Intersection over Union (IoU) of two bounding boxes.
    Each bbox is [x1, y1, x2, y2].
    """
    x1 = max(bbox1[0], bbox2[0])
    y1 = max(bbox1[1], bbox2[1])
    x2 = min(bbox1[2], bbox2[2])
    y2 = min(bbox1[3], bbox2[3])

    inter_area = max(0, x2 - x1) * max(0, y2 - y1)
    area1 = (bbox1[2] - bbox1[0]) * (bbox1[3] - bbox1[1])
    area2 = (bbox2[2] - bbox2[0]) * (bbox2[3] - bbox2[1])

    union_area = area1 + area2 - inter_area
    if union_area == 0:
        return 0
    return inter_area / union_area


def get_tracked_results(tracked_targets):  # type: ignore[no-untyped-def]
    """
    Extract tracked results from a list of target2d objects.

    Args:
        tracked_targets (list[target2d]): List of target2d objects (published targets)
                                        returned by the tracker's update() function.

    Returns:
        tuple: (tracked_masks, tracked_bboxes, tracked_track_ids, tracked_probs, tracked_names)
            where each is a list of the corresponding attribute from each target.
    """
    tracked_masks = []
    tracked_bboxes = []
    tracked_track_ids = []
    tracked_probs = []
    tracked_names = []

    for target in tracked_targets:
        # Extract the latest values stored in each target.
        tracked_masks.append(target.latest_mask)
        tracked_bboxes.append(target.latest_bbox)
        # Here we use the most recent detection's track ID.
        tracked_track_ids.append(target.target_id)
        # Use the latest probability from the history.
        tracked_probs.append(target.score)
        # Use the stored name (if any). If not available, you can use a default value.
        tracked_names.append(target.name)

    return tracked_masks, tracked_bboxes, tracked_track_ids, tracked_probs, tracked_names


class target2d:
    """
    Represents a tracked 2D target.
    Stores the latest bounding box and mask along with a short history of track IDs,
    detection probabilities, and computed texture values.
    """

    def __init__(  # type: ignore[no-untyped-def]
        self,
        initial_mask,
        initial_bbox,
        track_id,
        prob: float,
        name: str,
        texture_value,
        target_id,
        history_size: int = 10,
    ) -> None:
        """
        Args:
            initial_mask (torch.Tensor): Latest segmentation mask.
            initial_bbox (list): Bounding box in [x1, y1, x2, y2] format.
            track_id (int): Detection’s track ID (may be -1 if not provided).
            prob (float): Detection probability.
            name (str): Object class name.
            texture_value (float): Computed average texture value for this detection.
            target_id (int): Unique identifier assigned by the tracker.
            history_size (int): Maximum number of frames to keep in the history.
        """
        self.target_id = target_id
        self.latest_mask = initial_mask
        self.latest_bbox = initial_bbox
        self.name = name
        self.score = 1.0

        self.track_id = track_id
        self.probs_history = deque(maxlen=history_size)  # type: ignore[var-annotated]
        self.texture_history = deque(maxlen=history_size)  # type: ignore[var-annotated]

        self.frame_count = deque(maxlen=history_size)  # type: ignore[var-annotated]  # Total frames this target has been seen.
        self.missed_frames = 0  # Consecutive frames when no detection was assigned.
        self.history_size = history_size

    def update(self, mask, bbox, track_id, prob: float, name: str, texture_value) -> None:  # type: ignore[no-untyped-def]
        """
        Update the target with a new detection.
        """
        self.latest_mask = mask
        self.latest_bbox = bbox
        self.name = name

        self.track_id = track_id
        self.probs_history.append(prob)
        self.texture_history.append(texture_value)

        self.frame_count.append(1)
        self.missed_frames = 0

    def mark_missed(self) -> None:
        """
        Increment the count of consecutive frames where this target was not updated.
        """
        self.missed_frames += 1
        self.frame_count.append(0)

    def compute_score(  # type: ignore[no-untyped-def]
        self,
        frame_shape,
        min_area_ratio,
        max_area_ratio,
        texture_range=(0.0, 1.0),
        border_safe_distance: int = 50,
        weights=None,
    ):
        """
        Compute a combined score for the target based on several factors.

        Factors:
          - **Detection probability:** Average over recent frames.
          - **Temporal stability:** How consistently the target has appeared.
          - **Texture quality:** Normalized using the provided min and max values.
          - **Border proximity:** Computed from the minimum distance from the bbox to the frame edges.
          - **Size:** How the object's area (relative to the frame) compares to acceptable bounds.

        Args:
            frame_shape (tuple): (height, width) of the frame.
            min_area_ratio (float): Minimum acceptable ratio (bbox area / frame area).
            max_area_ratio (float): Maximum acceptable ratio.
            texture_range (tuple): (min_texture, max_texture) expected values.
            border_safe_distance (float): Distance (in pixels) considered safe from the border.
            weights (dict): Weights for each component. Expected keys:
                            'prob', 'temporal', 'texture', 'border', and 'size'.

        Returns:
            float: The combined (normalized) score in the range [0, 1].
        """
        # Default weights if none provided.
        if weights is None:
            weights = {"prob": 1.0, "temporal": 1.0, "texture": 1.0, "border": 1.0, "size": 1.0}

        h, w = frame_shape
        x1, y1, x2, y2 = self.latest_bbox
        bbox_area = (x2 - x1) * (y2 - y1)
        frame_area = w * h
        area_ratio = bbox_area / frame_area

        # Detection probability factor.
        avg_prob = np.mean(self.probs_history)
        # Temporal stability factor: normalized by history size.
        temporal_stability = np.mean(self.frame_count)
        # Texture factor: normalize average texture using the provided range.
        avg_texture = np.mean(self.texture_history) if self.texture_history else 0.0
        min_texture, max_texture = texture_range
        if max_texture == min_texture:
            normalized_texture = avg_texture
        else:
            normalized_texture = (avg_texture - min_texture) / (max_texture - min_texture)
            normalized_texture = max(0.0, min(normalized_texture, 1.0))

        # Border factor: compute the minimum distance from the bbox to any frame edge.
        left_dist = x1
        top_dist = y1
        right_dist = w - x2
        min_border_dist = min(left_dist, top_dist, right_dist)
        # Normalize the border distance: full score (1.0) if at least border_safe_distance away.
        border_factor = min(1.0, min_border_dist / border_safe_distance)

        # Size factor: penalize objects that are too small or too big.
        if area_ratio < min_area_ratio:
            size_factor = area_ratio / min_area_ratio
        elif area_ratio > max_area_ratio:
            # Here we compute a linear penalty if the area exceeds max_area_ratio.
            if 1 - max_area_ratio > 0:
                size_factor = max(0, (1 - area_ratio) / (1 - max_area_ratio))
            else:
                size_factor = 0.0
        else:
            size_factor = 1.0

        # Combine factors using a weighted sum (each factor is assumed in [0, 1]).
        w_prob = weights.get("prob", 1.0)
        w_temporal = weights.get("temporal", 1.0)
        w_texture = weights.get("texture", 1.0)
        w_border = weights.get("border", 1.0)
        w_size = weights.get("size", 1.0)
        total_weight = w_prob + w_temporal + w_texture + w_border + w_size

        # print(f"track_id: {self.target_id}, avg_prob: {avg_prob:.2f}, temporal_stability: {temporal_stability:.2f}, normalized_texture: {normalized_texture:.2f}, border_factor: {border_factor:.2f}, size_factor: {size_factor:.2f}")

        final_score = (
            w_prob * avg_prob
            + w_temporal * temporal_stability
            + w_texture * normalized_texture
            + w_border * border_factor
            + w_size * size_factor
        ) / total_weight

        self.score = final_score

        return final_score


class target2dTracker:
    """
    Tracker that maintains a history of targets across frames.
    New segmentation detections (frame, masks, bboxes, track_ids, probabilities,
    and computed texture values) are matched to existing targets or used to create new ones.

    The tracker uses a scoring system that incorporates:
      - **Detection probability**
      - **Temporal stability**
      - **Texture quality** (normalized within a specified range)
      - **Proximity to image borders** (a continuous penalty based on the distance)
      - **Object size** relative to the frame

    Targets are published if their score exceeds the start threshold and are removed if their score
    falls below the stop threshold or if they are missed for too many consecutive frames.
    """

    def __init__(  # type: ignore[no-untyped-def]
        self,
        history_size: int = 10,
        score_threshold_start: float = 0.5,
        score_threshold_stop: float = 0.3,
        min_frame_count: int = 10,
        max_missed_frames: int = 3,
        min_area_ratio: float = 0.001,
        max_area_ratio: float = 0.1,
        texture_range=(0.0, 1.0),
        border_safe_distance: int = 50,
        weights=None,
    ) -> None:
        """
        Args:
            history_size (int): Maximum history length (number of frames) per target.
            score_threshold_start (float): Minimum score for a target to be published.
            score_threshold_stop (float): If a target’s score falls below this, it is removed.
            min_frame_count (int): Minimum number of frames a target must be seen to be published.
            max_missed_frames (int): Maximum consecutive frames a target can be missing before deletion.
            min_area_ratio (float): Minimum acceptable bbox area relative to the frame.
            max_area_ratio (float): Maximum acceptable bbox area relative to the frame.
            texture_range (tuple): (min_texture, max_texture) expected values.
            border_safe_distance (float): Distance (in pixels) considered safe from the border.
            weights (dict): Weights for the scoring components (keys: 'prob', 'temporal',
                            'texture', 'border', 'size').
        """
        self.history_size = history_size
        self.score_threshold_start = score_threshold_start
        self.score_threshold_stop = score_threshold_stop
        self.min_frame_count = min_frame_count
        self.max_missed_frames = max_missed_frames
        self.min_area_ratio = min_area_ratio
        self.max_area_ratio = max_area_ratio
        self.texture_range = texture_range
        self.border_safe_distance = border_safe_distance
        # Default weights if none are provided.
        if weights is None:
            weights = {"prob": 1.0, "temporal": 1.0, "texture": 1.0, "border": 1.0, "size": 1.0}
        self.weights = weights

        self.targets = {}  # type: ignore[var-annotated]  # Dictionary mapping target_id -> target2d instance.
        self.next_target_id = 0

    def update(  # type: ignore[no-untyped-def]
        self,
        frame,
        masks,
        bboxes,
        track_ids,
        probs: Sequence[float],
        names: Sequence[str],
        texture_values,
    ):
        """
        Update the tracker with new detections from the current frame.

        Args:
            frame (np.ndarray): Current BGR frame.
            masks (list[torch.Tensor]): List of segmentation masks.
            bboxes (list): List of bounding boxes [x1, y1, x2, y2].
            track_ids (list): List of detection track IDs.
            probs (list): List of detection probabilities.
            names (list): List of class names.
            texture_values (list): List of computed texture values.

        Returns:
            published_targets (list[target2d]): Targets that are active and have scores above
                                                the start threshold.
        """
        updated_target_ids = set()
        frame_shape = frame.shape[:2]  # (height, width)

        # For each detection, try to match with an existing target.
        for mask, bbox, det_tid, prob, name, texture in zip(
            masks, bboxes, track_ids, probs, names, texture_values, strict=False
        ):
            matched_target = None

            # First, try matching by detection track ID if valid.
            if det_tid != -1:
                for target in self.targets.values():
                    if target.track_id == det_tid:
                        matched_target = target
                        break

            # Otherwise, try matching using IoU.
            if matched_target is None:
                best_iou = 0
                for target in self.targets.values():
                    iou = compute_iou(bbox, target.latest_bbox)  # type: ignore[no-untyped-call]
                    if iou > 0.5 and iou > best_iou:
                        best_iou = iou
                        matched_target = target

            # Update existing target or create a new one.
            if matched_target is not None:
                matched_target.update(mask, bbox, det_tid, prob, name, texture)
                updated_target_ids.add(matched_target.target_id)
            else:
                new_target = target2d(
                    mask, bbox, det_tid, prob, name, texture, self.next_target_id, self.history_size
                )
                self.targets[self.next_target_id] = new_target
                updated_target_ids.add(self.next_target_id)
                self.next_target_id += 1

        # Mark targets that were not updated.
        for target_id, target in list(self.targets.items()):
            if target_id not in updated_target_ids:
                target.mark_missed()
                if target.missed_frames > self.max_missed_frames:
                    del self.targets[target_id]
                    continue  # Skip further checks for this target.
            # Remove targets whose score falls below the stop threshold.
            score = target.compute_score(
                frame_shape,
                self.min_area_ratio,
                self.max_area_ratio,
                texture_range=self.texture_range,
                border_safe_distance=self.border_safe_distance,
                weights=self.weights,
            )
            if score < self.score_threshold_stop:
                del self.targets[target_id]

        # Publish targets with scores above the start threshold.
        published_targets = []
        for target in self.targets.values():
            score = target.compute_score(
                frame_shape,
                self.min_area_ratio,
                self.max_area_ratio,
                texture_range=self.texture_range,
                border_safe_distance=self.border_safe_distance,
                weights=self.weights,
            )
            if (
                score >= self.score_threshold_start
                and sum(target.frame_count) >= self.min_frame_count
                and target.missed_frames <= 5
            ):
                published_targets.append(target)

        return published_targets
