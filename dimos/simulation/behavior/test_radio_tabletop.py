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

"""Table support cannot be inferred from a stable assisted attachment."""

import copy

import numpy as np
import pytest

from dimos.simulation.behavior.radio_tabletop import assisted_release_complete, placement_stable


@pytest.fixture
def released_samples():
    return [
        {
            "step": i,
            "episode": "same",
            "observed_at_monotonic": i * 0.1,
            "radio_pose": np.eye(4).tolist(),
            "all_radio_contact_pairs": [{"radio_link": "/radio/base", "other_link": "/table/base"}],
            "finger_contacts": {"right_gripper_finger_link1": False},
            "measured_gripper": 0.99,
            "evaluator_assisted_grasp": {
                "right": {
                    "candidate_in_hand": False,
                    "constraint_valid": False,
                    "constraint_path": None,
                    "release_counter": None,
                }
            },
        }
        for i in range(5)
    ]


def test_real_support_and_confirmed_release_are_stable(released_samples):
    assert placement_stable(released_samples, "table", released=True)


@pytest.mark.parametrize(
    "failure",
    [
        "no_table",
        "wrong_table",
        "attached",
        "releasing",
        "closed",
        "finger_contact",
        "translation",
        "rotation",
        "nonfinite",
        "singular",
        "reflection",
        "episode",
        "duplicate_step",
    ],
)
def test_support_or_release_failure_never_admits_following_press(released_samples, failure):
    samples = copy.deepcopy(released_samples)
    v = samples[-1]
    if failure == "no_table":
        v["all_radio_contact_pairs"] = []
    elif failure == "wrong_table":
        v["all_radio_contact_pairs"][0]["other_link"] = "/other_table/base"
    elif failure == "attached":
        v["evaluator_assisted_grasp"]["right"]["constraint_valid"] = True
    elif failure == "releasing":
        v["evaluator_assisted_grasp"]["right"]["release_counter"] = 0
    elif failure == "closed":
        v["measured_gripper"] = 0.75
    elif failure == "finger_contact":
        v["finger_contacts"]["right_gripper_finger_link1"] = True
    elif failure == "translation":
        v["radio_pose"][0][3] = 0.003
    elif failure == "rotation":
        v["radio_pose"][:2] = [
            [np.cos(0.02), -np.sin(0.02), 0, 0],
            [np.sin(0.02), np.cos(0.02), 0, 0],
        ]
    elif failure == "nonfinite":
        v["radio_pose"][0][3] = np.nan
    elif failure == "singular":
        v["radio_pose"][0][0] = 0
    elif failure == "reflection":
        v["radio_pose"][0][0] = -1
    elif failure == "episode":
        v["episode"] = "changed"
    else:
        v["step"] = samples[0]["step"]
    assert not placement_stable(samples, "table", released=True)


def test_open_joint_feedback_does_not_imply_assisted_release(released_samples):
    value = released_samples[-1]
    value["measured_gripper"] = 1.0
    value["evaluator_assisted_grasp"]["right"].update(
        candidate_in_hand=True, constraint_valid=True, constraint_path="/constraint"
    )
    assert not assisted_release_complete(value)


def test_release_window_must_finish_before_retreat(released_samples):
    value = released_samples[-1]
    value["evaluator_assisted_grasp"]["right"]["release_counter"] = 5
    assert not assisted_release_complete(value)
    value["evaluator_assisted_grasp"]["right"]["release_counter"] = None
    assert assisted_release_complete(value)


@pytest.mark.parametrize(
    "failure",
    [None, "awake", "kinematic", "gravity_disabled", "disabled", "unknown", "wrong_table"],
)
def test_sleep_aware_support_requires_verified_dynamic_body(released_samples, failure):
    for value in released_samples:
        value["all_radio_contact_pairs"] = []
        value["sleep_aware_radio_contact_pairs"] = [
            {"radio_link": "/radio/base", "other_link": "/table/base"}
        ]
        value["evaluator_radio_body"] = {
            "is_asleep": True,
            "rigid_body_enabled": True,
            "kinematic_enabled": False,
            "gravity_disabled": False,
        }
    last = released_samples[-1]
    if failure == "awake":
        last["evaluator_radio_body"]["is_asleep"] = False
    elif failure == "kinematic":
        last["evaluator_radio_body"]["kinematic_enabled"] = True
    elif failure == "gravity_disabled":
        last["evaluator_radio_body"]["gravity_disabled"] = True
    elif failure == "disabled":
        last["evaluator_radio_body"]["rigid_body_enabled"] = False
    elif failure == "unknown":
        last["evaluator_radio_body"] = {}
    elif failure == "wrong_table":
        last["sleep_aware_radio_contact_pairs"][0]["other_link"] = "/other/base"
    assert placement_stable(released_samples, "table", released=True) == (failure is None)
