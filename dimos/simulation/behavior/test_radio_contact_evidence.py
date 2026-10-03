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

"""Contact provenance and pose deltas must survive nontrivial world frames."""

import copy
from types import SimpleNamespace

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from dimos.simulation.behavior.radio_contact_evidence import (
    AssistedRadioRetention,
    radio_contact_pairs,
    radio_interaction_evidence,
    radio_overlap_finger_hits,
    radio_stage_displacement,
    radio_stage_timeline,
)


def test_stage_timeline_preserves_contact_provenance_at_first_displacement():
    pose = np.eye(4)
    moved = pose.copy()
    moved[0, 3] = 0.003
    samples = [
        {"step": 12, "observed_at_monotonic": 1.0, "radio_pose": pose.tolist()},
        {
            "step": 13,
            "observed_at_monotonic": 1.1,
            "radio_pose": moved.tolist(),
            "all_radio_contact_pairs": [],
            "sleep_aware_radio_contact_pairs": [{"other_link": "/table/base"}],
            "evaluator_radio_body": {"is_asleep": True},
        },
    ]
    original = copy.deepcopy(samples)
    result = radio_stage_timeline(samples)
    marker = result["first_observed_motion_over_2mm_or_0_01rad"]
    assert marker["step"] == 13
    assert marker["current_contacts"] == []
    assert marker["sleep_aware_contacts"] == [{"other_link": "/table/base"}]
    assert result["maximum_translation_m"] == pytest.approx(0.003)
    assert samples == original


def test_stage_timeline_detects_rotation_without_translation_and_empty_input():
    initial = np.eye(4)
    rotated = initial.copy()
    rotated[:3, :3] = Rotation.from_euler("z", 0.02).as_matrix()
    values = [
        {"step": i, "observed_at_monotonic": float(i), "radio_pose": p.tolist()}
        for i, p in enumerate((initial, rotated))
    ]
    result = radio_stage_timeline(values)
    assert result["first_observed_motion_over_2mm_or_0_01rad"]["step"] == 1
    assert result["maximum_rotation_rad"] == pytest.approx(0.02)
    assert radio_stage_timeline([]) == {"samples": 0, "timeline": []}


def test_passive_overlap_query_keeps_all_finger_hits_and_never_stops_after_first():
    continued = []

    def query(*, radius, pos, reportFn):
        assert radius == 0.022
        assert pos == [1, 2, 3]
        for body in ("/left/finger1", "/radio/body", "/left/finger2", "/left/finger1"):
            continued.append(reportFn(SimpleNamespace(rigid_body=body)))

    hits = radio_overlap_finger_hits(query, 0.022, [1, 2, 3], {"/left/finger1", "/left/finger2"})
    assert hits == ["/left/finger1", "/left/finger2"]
    assert continued == [True, True, True, True]


def test_pairs_preserve_full_paths_and_do_not_invent_contact_normals():
    pairs = iter(
        [
            ("/r1/left_gripper_finger_link1", "/radio/base_link"),
            ("/r1/left_gripper_link", "/radio/base_link"),
            ("/r1/left_gripper_finger_link1", "/radio/base_link"),
        ]
    )
    assert radio_contact_pairs(pairs) == [
        {"robot_link": "/r1/left_gripper_finger_link1", "radio_link": "/radio/base_link"},
        {"robot_link": "/r1/left_gripper_link", "radio_link": "/radio/base_link"},
    ]


def test_displacement_distinguishes_world_and_original_radio_frame():
    before = np.eye(4)
    before[:3, :3] = Rotation.from_euler("z", 90, degrees=True).as_matrix()
    before[:3, 3] = [3.6, 4.15, 0.6]
    relative = np.eye(4)
    relative[:3, 3] = [0.01, 0, 0]
    relative[:3, :3] = Rotation.from_euler("x", 180, degrees=True).as_matrix()
    result = radio_stage_displacement(before, before @ relative)
    assert result["translation_world_m"] == pytest.approx([0, 0.01, 0])
    assert result["translation_initial_radio_m"] == pytest.approx([0.01, 0, 0])
    assert result["translation_norm_m"] == pytest.approx(0.01)
    assert result["rotation_angle_rad"] == pytest.approx(np.pi)


def test_rigidly_carried_rotation_is_not_grasp_slip():
    radio = np.eye(4)
    radio[:3, 3] = [0.1, 0, 0]
    motion = np.eye(4)
    motion[:3, :3] = Rotation.from_euler("y", 180, degrees=True).as_matrix()
    motion[:3, 3] = [1, 2, 3]
    before = {"radio_pose": radio, "evaluator_gripper_pose": np.eye(4)}
    current = {"radio_pose": motion @ radio, "evaluator_gripper_pose": motion}
    result = radio_interaction_evidence(before, current)
    assert result["radio_stage_displacement"]["rotation_angle_rad"] == pytest.approx(np.pi)
    assert result["physical_grasp_relative_displacement"]["translation_norm_m"] == pytest.approx(0)
    assert result["physical_grasp_relative_displacement"]["rotation_angle_rad"] == pytest.approx(0)


def test_press_gap_uses_updated_radio_frame_and_preserves_real_slip():
    radio = np.eye(4)
    radio[:3, :3] = Rotation.from_euler("z", 90, degrees=True).as_matrix()
    radio[:3, 3] = [3, 4, 0.6]
    hand = radio.copy()
    hand[:3, 3] += radio[:3, :3] @ [0.012, 0.005, 0]
    initial_radio = radio.copy()
    initial_radio[:3, 3] -= radio[:3, :3] @ [0.003, 0, 0]
    result = radio_interaction_evidence(
        {"radio_pose": initial_radio, "evaluator_gripper_pose": radio},
        {"radio_pose": radio, "evaluator_gripper_pose": radio, "evaluator_left_gripper_pose": hand},
        {"surface_in_radio": [0, 0, 0], "pad_in_left_gripper": [0, 0, 0]},
    )
    assert result["physical_grasp_relative_displacement"]["translation_norm_m"] == pytest.approx(
        0.003
    )
    geometry = result["candidate_press_geometry"]
    assert geometry["signed_outward_gap_m"] == pytest.approx(0.012)
    assert geometry["tangential_offset_m"] == pytest.approx(0.005)
    assert geometry["candidate_outward_normal_world"] == pytest.approx([0, 1, 0])


@pytest.fixture
def assisted_observation():
    return {
        "evaluator_assisted_grasp": {
            "right": {
                "mode": "assisted",
                "candidate_is_grasping": "1",
                "candidate_in_hand": True,
                "release_counter": None,
                "constraint_valid": True,
                "constraint_path": "/right/ag_constraint",
            }
        },
        "measured_gripper": 0.75,
        "finger_contacts": {"right_gripper_finger_link1": True, "right_gripper_finger_link2": True},
        "radio_pose": np.eye(4).tolist(),
        "evaluator_gripper_pose": np.eye(4).tolist(),
    }


def test_assisted_continuation_preserves_rigid_hold_despite_contact_sample_loss(
    assisted_observation,
):
    retention = AssistedRadioRetention()
    retention.check(assisted_observation)
    observation = copy.deepcopy(assisted_observation)
    observation["finger_contacts"]["right_gripper_finger_link2"] = False
    motion = np.eye(4)
    motion[:3, :3] = Rotation.from_euler("z", 180, degrees=True).as_matrix()
    motion[:3, 3] = [1, 2, 3]
    observation["radio_pose"] = motion.tolist()
    observation["evaluator_gripper_pose"] = motion.tolist()
    retention.check(observation)
    assert observation["finger_contacts"]["right_gripper_finger_link2"] is False
    assert retention.attachment == pytest.approx(np.eye(4))


@pytest.mark.parametrize(
    "field,value",
    [
        ("candidate_in_hand", False),
        ("candidate_is_grasping", "-1"),
        ("constraint_valid", False),
        ("constraint_path", None),
        ("constraint_path", "/replacement/ag_constraint"),
        ("release_counter", 0),
        ("mode", "physical"),
    ],
)
def test_lost_or_replaced_assistance_rejects_even_unchanged_pose(
    assisted_observation, field, value
):
    retention = AssistedRadioRetention()
    retention.check(assisted_observation)
    assisted_observation["evaluator_assisted_grasp"]["right"][field] = value
    with pytest.raises(RuntimeError, match="assisted|attachment"):
        retention.check(assisted_observation)


@pytest.mark.parametrize(
    "failure", ["translation", "rotation", "nonfinite_pose", "nonfinite_closure"]
)
def test_assistance_alone_cannot_hide_measured_failure(assisted_observation, failure):
    retention = AssistedRadioRetention()
    retention.check(assisted_observation)
    pose = np.eye(4)
    if failure == "translation":
        pose[0, 3] = 0.0041
    elif failure == "rotation":
        pose[:3, :3] = Rotation.from_euler("z", 0.031).as_matrix()
    elif failure == "nonfinite_pose":
        pose[0, 3] = np.nan
    else:
        assisted_observation["measured_gripper"] = np.nan
    assisted_observation["radio_pose"] = pose.tolist()
    with pytest.raises(RuntimeError, match="slipped|finite|closure"):
        retention.check(assisted_observation)


def test_assistance_does_not_establish_grasp_without_initial_opposing_contacts(
    assisted_observation,
):
    retention = AssistedRadioRetention()
    assisted_observation["finger_contacts"]["right_gripper_finger_link2"] = False
    with pytest.raises(RuntimeError, match="Initial"):
        retention.check(assisted_observation)
    assert retention.attachment is None
