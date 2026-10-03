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

"""Serialize evaluator-only rigid contact pairs for a development radio grasp."""

from collections.abc import Callable, Iterable, Mapping, Sequence
import math
from typing import Any, cast

import numpy as np
from scipy.spatial.transform import Rotation


def radio_overlap_finger_hits(
    query: Callable[..., Any], radius: float, center: list[float], eligible: set[str]
) -> list[str]:
    """Read every eligible body from a passive PhysX sphere query; no state writes."""
    hits: list[str] = []

    def record(hit: Any) -> bool:
        if hit.rigid_body in eligible:
            hits.append(str(hit.rigid_body))
        return True

    query(radius=radius, pos=center, reportFn=record)
    return list(dict.fromkeys(hits))


def radio_contact_pairs(pairs: Iterable[tuple[str, str]]) -> list[dict[str, str]]:
    """Keep link identity; rigid pair membership alone gives no force or normal."""
    return [
        {"robot_link": robot_link, "radio_link": radio_link}
        for robot_link, radio_link in sorted(set((str(a), str(b)) for a, b in pairs))
    ]


def radio_stage_displacement(before: np.ndarray, current: np.ndarray) -> dict[str, object]:
    """Evaluator-only displacement relative to a stage's measured starting pose."""
    delta = np.linalg.inv(before) @ current
    return {
        "translation_world_m": (current[:3, 3] - before[:3, 3]).tolist(),
        "translation_initial_radio_m": delta[:3, 3].tolist(),
        "translation_norm_m": float(np.linalg.norm(delta[:3, 3])),
        "rotation_angle_rad": float(Rotation.from_matrix(delta[:3, :3]).magnitude()),
    }


def radio_stage_timeline(samples: Sequence[Mapping[str, Any]]) -> dict[str, Any]:
    """Locate observed displacement and contacts without inferring causal forces.

    Threshold crossings are diagnostic markers, not task success or failure.
    Current contacts and sleep-aware cached pairs retain separate provenance.
    """
    if not samples:
        return {"samples": 0, "timeline": []}
    initial = np.asarray(samples[0]["radio_pose"], dtype=float)
    timeline = []
    first_motion = None
    for value in samples:
        displacement = radio_stage_displacement(initial, np.asarray(value["radio_pose"]))
        marker = {
            "step": value["step"],
            "observed_at_monotonic": value["observed_at_monotonic"],
            "radio_pose": value["radio_pose"],
            "displacement": displacement,
            "current_contacts": value.get("all_radio_contact_pairs"),
            "sleep_aware_contacts": value.get("sleep_aware_radio_contact_pairs"),
            "body": value.get("evaluator_radio_body"),
            "assisted_grasp": value.get("evaluator_assisted_grasp"),
            "press_geometry": value.get("candidate_press_geometry"),
        }
        timeline.append(marker)
        if first_motion is None and (
            cast("float", displacement["translation_norm_m"]) > 0.002
            or cast("float", displacement["rotation_angle_rad"]) > 0.01
        ):
            first_motion = marker
    return {
        "samples": len(samples),
        "first_observed_motion_over_2mm_or_0_01rad": first_motion,
        "maximum_translation_m": max(v["displacement"]["translation_norm_m"] for v in timeline),
        "maximum_rotation_rad": max(v["displacement"]["rotation_angle_rad"] for v in timeline),
        "timeline": timeline,
    }


def radio_finger_contacts(pairs: Iterable[tuple[str, str]]) -> dict[str, bool]:
    fingers = {
        f"{side}_gripper_finger_link{i}": False for side in ("left", "right") for i in (1, 2)
    }
    for robot_link, _ in pairs:
        name = str(robot_link).rsplit("/", 1)[-1]
        if name in fingers:
            fingers[name] = True
    return fingers


def radio_interaction_evidence(
    before: dict[str, Any], current: dict[str, Any], contact: dict[str, Any] | None = None
) -> dict[str, Any]:
    """Separate carried-object motion from slip; never infer forces from pairs.

    The contact plane is the operator's calibrated candidate, not a measured
    collision normal. All input poses must share one snapshot and world frame.
    """
    radio = np.asarray(current["radio_pose"])
    initial_radio = np.asarray(before["radio_pose"])
    result: dict[str, Any] = {
        "radio_stage_displacement": radio_stage_displacement(initial_radio, radio)
    }
    if "evaluator_gripper_pose" in before and "evaluator_gripper_pose" in current:
        initial_attachment = np.linalg.inv(before["evaluator_gripper_pose"]) @ initial_radio
        attachment = np.linalg.inv(current["evaluator_gripper_pose"]) @ radio
        result["physical_grasp_relative_displacement"] = radio_stage_displacement(
            initial_attachment, attachment
        )
    hand = (
        "evaluator_gripper_pose"
        if contact is not None and "pad_in_gripper" in contact
        else "evaluator_left_gripper_pose"
    )
    if contact is not None and hand in current:
        pad = np.asarray(current[hand]) @ np.append(
            contact["pad_in_gripper"]
            if "pad_in_gripper" in contact
            else contact["pad_in_left_gripper"],
            1,
        )
        pad_in_radio = (np.linalg.inv(radio) @ pad)[:3]
        delta = pad_in_radio - np.asarray(contact["surface_in_radio"])
        result["candidate_press_geometry"] = {
            "pad_in_radio_m": pad_in_radio.tolist(),
            "signed_outward_gap_m": float(delta[0]),
            "tangential_offset_m": float(np.linalg.norm(delta[1:])),
            "candidate_outward_normal_world": radio[:3, 0].tolist(),
            "source": "Operator-calibrated radio +X plane; not a measured contact normal",
        }
    return result


class AssistedRadioRetention:
    """Task-local official assisted holding plus measured rigid retention.

    Requires opposing contacts initially. Continuing assistance is not evidence
    of a friction-only grasp, and never substitutes for measured pose checks.
    """

    def __init__(self) -> None:
        self.attachment: np.ndarray | None = None
        self.constraint_path: str | None = None

    def check(self, value: Mapping[str, Any]) -> None:
        assistance = (value.get("evaluator_assisted_grasp") or {}).get("right", {})
        if not (
            assistance.get("mode") == "assisted"
            and assistance.get("candidate_is_grasping") == "1"
            and assistance.get("candidate_in_hand") is True
            and assistance.get("release_counter") is None
            and assistance.get("constraint_valid") is True
            and assistance.get("constraint_path")
        ):
            raise RuntimeError("Official assisted radio grasp is absent or releasing")
        gripper = value.get("measured_gripper")
        if gripper is None or not math.isfinite(gripper) or not 0.10 < gripper < 0.95:
            raise RuntimeError("Radio grasp lacks measured blocked closure")
        poses = [
            np.asarray(value[k], dtype=float) for k in ("radio_pose", "evaluator_gripper_pose")
        ]
        for pose in poses:
            if (
                pose.shape != (4, 4)
                or not np.isfinite(pose).all()
                or not np.allclose(pose[3], [0, 0, 0, 1])
                or not np.allclose(pose[:3, :3].T @ pose[:3, :3], np.eye(3), atol=1e-7)
                or not np.isclose(np.linalg.det(pose[:3, :3]), 1, atol=1e-7)
            ):
                raise RuntimeError("Radio retention requires finite rigid measured poses")
        attachment = np.linalg.inv(poses[1]) @ poses[0]
        if self.attachment is None:
            contacts = value.get("finger_contacts") or {}
            if not all(contacts.get(f"right_gripper_finger_link{i}") for i in (1, 2)):
                raise RuntimeError("Initial radio grasp requires opposing finger contacts")
            self.attachment = attachment.copy()
            self.constraint_path = assistance["constraint_path"]
        elif self.constraint_path != assistance["constraint_path"]:
            raise RuntimeError("Radio assisted attachment changed")
        delta = np.linalg.inv(self.attachment) @ attachment
        if (
            np.linalg.norm(delta[:3, 3]) > 0.004
            or Rotation.from_matrix(delta[:3, :3]).magnitude() > 0.03
        ):
            raise RuntimeError("Radio slipped relative to physical holding gripper")
