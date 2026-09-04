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

import pytest

from dimos.imitation.profile import ImageSource
from dimos.imitation.workflows import (
    COLLECTION_WORKFLOWS,
    ROLLOUT_WORKFLOWS,
    get_collection_workflow,
    get_rollout_workflow,
)
from dimos.robot.manipulators.dual_openyam.config import DUAL_OPENYAM_JOINTS
from dimos.robot.manipulators.dual_openyam.learning import ABC_JOINTS


def test_collection_and_rollout_catalogs_are_independent() -> None:
    assert list(COLLECTION_WORKFLOWS) == [
        "openyam-teach",
        "openyam-quest",
        "dual-openyam-quest",
    ]
    assert list(ROLLOUT_WORKFLOWS) == ["openyam-lerobot", "dual-openyam-abc"]


def test_dual_collection_uses_two_wrist_cameras_and_canonical_joint_order() -> None:
    profile = get_collection_workflow("dual-openyam-quest").load_profile()
    camera_streams = {
        source.stream for source in profile.observations.values() if isinstance(source, ImageSource)
    }

    assert camera_streams == {"left_wrist_image", "right_wrist_image"}
    assert profile.action.demonstration.joints == tuple(DUAL_OPENYAM_JOINTS)


def test_abc_rollout_has_real_top_camera_and_released_joint_order() -> None:
    profile = get_rollout_workflow("dual-openyam-abc").load_profile()

    assert profile.observations["top"].stream == "top_image"
    assert profile.action.demonstration.joints == ABC_JOINTS
    assert profile.sync.tolerance_ms == 20.0


def test_unknown_workflows_list_only_the_relevant_catalog() -> None:
    with pytest.raises(ValueError, match="dual-openyam-quest"):
        get_collection_workflow("missing")
    with pytest.raises(ValueError, match="dual-openyam-abc"):
        get_rollout_workflow("missing")
