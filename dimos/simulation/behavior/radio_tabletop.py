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

"""Evaluator-only support/release verification for the ordered SDK radio demo."""

from collections.abc import Mapping, Sequence
import math
from typing import Any

import numpy as np
from scipy.spatial.transform import Rotation


def placement_stable(
    samples: Sequence[Mapping[str, Any]], table_name: str, *, released: bool
) -> bool:
    """Require real table contacts and stable poses; assistance is not table support."""
    if len(samples) < 5 or len({v["step"] for v in samples}) < 5:
        return False
    if samples[-1]["observed_at_monotonic"] - samples[0]["observed_at_monotonic"] < 0.3:
        return False
    initial = np.asarray(samples[0]["radio_pose"], dtype=float)
    if not _valid_pose(initial):
        return False
    for value in samples:
        if value["episode"] != samples[0]["episode"]:
            return False
        current_pairs = value.get("all_radio_contact_pairs") or []
        supported = any(f"/{table_name}/" in pair["other_link"] for pair in current_pairs)
        if not supported:
            body = value.get("evaluator_radio_body") or {}
            cached_pairs = value.get("sleep_aware_radio_contact_pairs") or []
            supported = (
                body.get("is_asleep") is True
                and body.get("rigid_body_enabled") is True
                and body.get("kinematic_enabled") is False
                and body.get("gravity_disabled") is False
                and any(f"/{table_name}/" in pair["other_link"] for pair in cached_pairs)
            )
        if not supported:
            return False
        pose = np.asarray(value["radio_pose"], dtype=float)
        if not _valid_pose(pose):
            return False
        delta = np.linalg.inv(initial) @ pose
        if (
            np.linalg.norm(delta[:3, 3]) > 0.002
            or Rotation.from_matrix(delta[:3, :3]).magnitude() > 0.01
        ):
            return False
        if released:
            opening = value.get("measured_gripper")
            if (
                not assisted_release_complete(value)
                or opening is None
                or not math.isfinite(opening)
                or not 0.97 <= opening <= 1.0
            ):
                return False
    return True


def _valid_pose(pose: np.ndarray) -> bool:
    return bool(
        pose.shape == (4, 4)
        and np.isfinite(pose).all()
        and np.allclose(pose[3], [0, 0, 0, 1], atol=1e-6)
        and np.allclose(pose[:3, :3].T @ pose[:3, :3], np.eye(3), atol=1e-6)
        and abs(np.linalg.det(pose[:3, :3]) - 1) <= 1e-6
    )


def assisted_release_complete(value: Mapping[str, Any]) -> bool:
    """Attachment lifecycle and contact check; not a joint-limit/pose admission."""
    state = (value.get("evaluator_assisted_grasp") or {}).get("right", {})
    return bool(
        state.get("candidate_in_hand") is False
        and state.get("constraint_valid") is False
        and state.get("constraint_path") is None
        and state.get("release_counter") is None
        and not any((value.get("finger_contacts") or {}).values())
    )
