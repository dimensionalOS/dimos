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

import numpy as np
from typing import Dict, Any, Optional, List, Tuple, Union
from dataclasses import dataclass

from dimos.msgs.geometry_msgs import Pose, Vector3, Quaternion


@dataclass
class ObjectMatchResult:
    """Result of object matching with confidence metrics."""
    matched_object: Optional[Dict[str, Any]]
    confidence: float
    distance: float
    size_similarity: float
    is_valid_match: bool


def calculate_object_similarity(
    target_obj: Dict[str, Any], 
    candidate_obj: Dict[str, Any],
    distance_weight: float = 0.6,
    size_weight: float = 0.4
) -> Tuple[float, float, float]:
    """
    Calculate comprehensive similarity between two objects.
    
    Args:
        target_obj: Target object with 'position' and optionally 'size'
        candidate_obj: Candidate object with 'position' and optionally 'size'
        distance_weight: Weight for distance component (0-1)
        size_weight: Weight for size component (0-1)
    
    Returns:
        Tuple of (total_similarity, distance_m, size_similarity)
    """
    # Extract positions
    target_pos = target_obj.get("position", {})
    candidate_pos = candidate_obj.get("position", {})
    
    if isinstance(target_pos, Vector3):
        target_xyz = np.array([target_pos.x, target_pos.y, target_pos.z])
    else:
        target_xyz = np.array([target_pos.get("x", 0), target_pos.get("y", 0), target_pos.get("z", 0)])
    
    if isinstance(candidate_pos, Vector3):
        candidate_xyz = np.array([candidate_pos.x, candidate_pos.y, candidate_pos.z])
    else:
        candidate_xyz = np.array([candidate_pos.get("x", 0), candidate_pos.get("y", 0), candidate_pos.get("z", 0)])
    
    # Calculate Euclidean distance
    distance = np.linalg.norm(target_xyz - candidate_xyz)
    distance_similarity = 1.0 / (1.0 + distance)  # Exponential decay
    
    # Calculate size similarity by comparing each dimension individually
    size_similarity = 1.0  # Default if no size info
    target_size = target_obj.get("size", {})
    candidate_size = candidate_obj.get("size", {})
    
    if target_size and candidate_size:
        # Extract dimensions with defaults
        target_dims = [
            target_size.get("width", 0.0), 
            target_size.get("height", 0.0), 
            target_size.get("depth", 0.0)
        ]
        candidate_dims = [
            candidate_size.get("width", 0.0), 
            candidate_size.get("height", 0.0), 
            candidate_size.get("depth", 0.0)
        ]
        
        # Calculate similarity for each dimension pair
        dim_similarities = []
        for target_dim, candidate_dim in zip(target_dims, candidate_dims):
            if target_dim == 0.0 and candidate_dim == 0.0:
                dim_similarities.append(1.0)  # Both dimensions are zero
            elif target_dim == 0.0 or candidate_dim == 0.0:
                dim_similarities.append(0.0)  # One dimension is zero, other is not
            else:
                # Calculate similarity as min/max ratio
                max_dim = max(target_dim, candidate_dim)
                min_dim = min(target_dim, candidate_dim)
                dim_similarity = min_dim / max_dim if max_dim > 0 else 0.0
                dim_similarities.append(dim_similarity)
        
        # Return average similarity across all dimensions
        size_similarity = np.mean(dim_similarities) if dim_similarities else 0.0
    
    # Weighted combination
    total_similarity = distance_weight * distance_similarity + size_weight * size_similarity
    
    return total_similarity, distance, size_similarity


def find_best_object_match(
    target_obj: Dict[str, Any],
    candidates: List[Dict[str, Any]],
    max_distance: float = 0.1,
    min_size_similarity: float = 0.4,
    distance_weight: float = 0.7,
    size_weight: float = 0.3
) -> ObjectMatchResult:
    """
    Find the best matching object from candidates using distance and size criteria.
    
    Args:
        target_obj: Target object to match against
        candidates: List of candidate objects
        max_distance: Maximum allowed distance for valid match (meters)
        min_size_similarity: Minimum size similarity for valid match (0-1)
        distance_weight: Weight for distance in similarity calculation
        size_weight: Weight for size in similarity calculation
    
    Returns:
        ObjectMatchResult with best match and confidence metrics
    """
    if not candidates or not target_obj.get("position"):
        return ObjectMatchResult(None, 0.0, float('inf'), 0.0, False)
    
    best_match = None
    best_confidence = 0.0
    best_distance = float('inf')
    best_size_sim = 0.0
    
    for candidate in candidates:
        if not candidate.get("position"):
            continue
            
        similarity, distance, size_sim = calculate_object_similarity(
            target_obj, candidate, distance_weight, size_weight
        )
        
        # Check validity constraints
        is_valid = distance <= max_distance and size_sim >= min_size_similarity
        
        if is_valid and similarity > best_confidence:
            best_match = candidate
            best_confidence = similarity
            best_distance = distance
            best_size_sim = size_sim
    
    return ObjectMatchResult(
        matched_object=best_match,
        confidence=best_confidence,
        distance=best_distance,
        size_similarity=best_size_sim,
        is_valid_match=best_match is not None
    )


def parse_zed_pose(zed_pose_data: Dict[str, Any]) -> Optional[Pose]:
    """
    Parse ZED pose data dictionary into a Pose object.

    Args:
        zed_pose_data: Dictionary from ZEDCamera.get_pose() containing:
            - position: [x, y, z] in meters
            - rotation: [x, y, z, w] quaternion
            - euler_angles: [roll, pitch, yaw] in radians
            - valid: Whether pose is valid

    Returns:
        Pose object with position and orientation, or None if invalid
    """
    if not zed_pose_data or not zed_pose_data.get("valid", False):
        return None

    # Extract position
    position = zed_pose_data.get("position", [0, 0, 0])
    pos_vector = Vector3(position[0], position[1], position[2])

    quat = zed_pose_data["rotation"]
    orientation = Quaternion(quat[0], quat[1], quat[2], quat[3])
    return Pose(pos_vector, orientation)


def estimate_object_depth(
    depth_image: np.ndarray, segmentation_mask: Optional[np.ndarray], bbox: List[float]
) -> float:
    """
    Estimate object depth dimension using segmentation mask and depth data.
    Optimized for real-time performance.

    Args:
        depth_image: Depth image in meters
        segmentation_mask: Binary segmentation mask for the object
        bbox: Bounding box [x1, y1, x2, y2]

    Returns:
        Estimated object depth in meters
    """
    x1, y1, x2, y2 = int(bbox[0]), int(bbox[1]), int(bbox[2]), int(bbox[3])

    # Extract depth ROI once
    roi_depth = depth_image[y1:y2, x1:x2]

    if segmentation_mask is not None and segmentation_mask.size > 0:
        # Extract mask ROI efficiently
        mask_roi = (
            segmentation_mask[y1:y2, x1:x2]
            if segmentation_mask.shape != roi_depth.shape
            else segmentation_mask
        )

        # Fast mask application using boolean indexing
        valid_mask = mask_roi > 0
        if np.sum(valid_mask) > 10:  # Early exit if not enough points
            masked_depths = roi_depth[valid_mask]

            # Fast percentile calculation using numpy's optimized functions
            depth_90 = np.percentile(masked_depths, 90)
            depth_10 = np.percentile(masked_depths, 10)
            depth_range = depth_90 - depth_10

            # Clamp to reasonable bounds with single operation
            return np.clip(depth_range, 0.02, 0.5)

    # Fast fallback using area calculation
    bbox_area = (x2 - x1) * (y2 - y1)

    # Vectorized area-based estimation
    if bbox_area > 10000:
        return 0.15
    elif bbox_area > 5000:
        return 0.10
    else:
        return 0.05
