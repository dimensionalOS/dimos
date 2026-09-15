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

from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.galaxea.r1pro.config import R1PRO_MODEL
from dimos.robot.galaxea.r1pro.connection import (
    R1PRO_UPPER_BODY_JOINTS,
    ArticulatedTf,
    R1ProConnectionConfig,
)

HEAD = "camera_head_left_link"


@pytest.fixture(scope="module")
def config() -> R1ProConnectionConfig:
    return R1ProConnectionConfig()


@pytest.fixture(scope="module")
def fk(config: R1ProConnectionConfig) -> ArticulatedTf:
    return ArticulatedTf(R1PRO_MODEL, config.articulated_frame_ids, root_link=config.frame_id)


def joint_state(**positions: float) -> JointState:
    """motor_states as the connection publishes it, with named joints bent."""
    values = dict.fromkeys(R1PRO_UPPER_BODY_JOINTS, 0.0)
    for joint, angle in positions.items():
        values[f"r1pro/{joint}"] = angle
    return JointState(
        ts=1.0,
        frame_id="base_link",
        name=R1PRO_UPPER_BODY_JOINTS,
        position=list(values.values()),
    )


def test_motor_states_joint_names_all_exist_in_the_model(fk: ArticulatedTf) -> None:
    """The URDF renames every joint to `r1pro/<joint>`; motor_states must match.

    A mismatch is silent: unknown joints are skipped, FK falls back to the
    neutral pose, and the camera transform looks plausible but never moves.
    """
    fk.transforms(joint_state())

    assert fk._unknown_joints == set()


def test_head_camera_moves_when_the_torso_bends(fk: ArticulatedTf) -> None:
    """The head sits past four revolute torso joints, so it cannot be static."""
    upright = {t.child_frame_id: t.translation for t in fk.transforms(joint_state())}
    bent = {t.child_frame_id: t.translation for t in fk.transforms(joint_state(torso_joint2=0.5))}

    assert bent[HEAD].x - upright[HEAD].x == pytest.approx(0.436, abs=0.01)
    assert bent[HEAD].z - upright[HEAD].z == pytest.approx(-0.144, abs=0.01)


def test_transforms_hang_off_the_configured_root(
    fk: ArticulatedTf, config: R1ProConnectionConfig
) -> None:
    transforms = fk.transforms(joint_state())

    assert {t.frame_id for t in transforms} == {config.frame_id}
    assert [t.child_frame_id for t in transforms] == list(config.articulated_frame_ids)
    assert all(t.ts == 1.0 for t in transforms)


def test_unknown_links_are_rejected_at_construction() -> None:
    with pytest.raises(ValueError, match="no_such_link"):
        ArticulatedTf(R1PRO_MODEL, ("no_such_link",))
