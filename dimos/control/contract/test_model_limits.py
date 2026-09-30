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

"""Tests for reading joint limits out of a URDF.

Two models are used. A small one written here covers the three kinds of joint
that matter -- one with proper limits, one that spins freely, and one missing
its limits altogether -- because no real robot has all three at once.

Then the G1 humanoid included in this repo, to check the reader against 29
joints nobody here wrote. Its expected numbers are typed out in the test
rather than read back from the same file, so if the model changes the test
fails instead of quietly agreeing with it.
"""

from __future__ import annotations

from pathlib import Path

import pytest

from dimos.control.contract.description import Limits
from dimos.control.contract.model_limits import limits_from_urdf
from dimos.control.contract.presets import manipulator_description, pd_joint_description
from dimos.control.contract.validate import Rejected, validate_command
from dimos.msgs.control_msgs.ControlValues import ControlValues

#: The G1 shipped in this repo. Not the planner's xacro: that one is fetched
#: from a vendor git remote at first use, so a test could not count on it.
G1_URDF = Path(__file__).resolve().parents[2] / "robot" / "unitree" / "g1" / "g1.urdf"

TOY_URDF = """<?xml version="1.0"?>
<robot name="toy">
  <link name="base"/>
  <link name="l1"/>
  <link name="l2"/>
  <link name="l3"/>
  <joint name="bounded_joint" type="revolute">
    <parent link="base"/>
    <child link="l1"/>
    <limit lower="-1.5" upper="2.5" velocity="3.0" effort="40.0"/>
  </joint>
  <joint name="spinner_joint" type="continuous">
    <parent link="l1"/>
    <child link="l2"/>
    <limit velocity="10.0" effort="5.0"/>
  </joint>
  <joint name="bare_joint" type="revolute">
    <parent link="l2"/>
    <child link="l3"/>
  </joint>
</robot>
"""


def test_a_bounded_joint_gives_all_three_interfaces() -> None:
    limits = limits_from_urdf(TOY_URDF, {"arm/j1": "bounded_joint"})
    assert limits == {
        "arm/j1/position": Limits(-1.5, 2.5),
        # URDF states one magnitude for velocity and effort; the range is
        # symmetric around zero.
        "arm/j1/velocity": Limits(-3.0, 3.0),
        "arm/j1/effort": Limits(-40.0, 40.0),
    }


def test_keys_are_full_three_segment_control_keys() -> None:
    limits = limits_from_urdf(TOY_URDF, {"arm/j1": "bounded_joint"})
    for key in limits:
        assert key.count("/") == 2
        assert key.startswith("arm/j1/")


def test_the_mapping_reconciles_the_two_naming_schemes() -> None:
    # The URDF name and the canonical name are allowed to look nothing alike;
    # that is what the mapping is for.
    limits = limits_from_urdf(
        TOY_URDF, {"left_arm/shoulder": "bounded_joint"}, velocity=False, effort=False
    )
    assert limits == {"left_arm/shoulder/position": Limits(-1.5, 2.5)}


def test_a_joint_that_spins_freely_gets_no_position_limit() -> None:
    limits = limits_from_urdf(TOY_URDF, {"arm/spin": "spinner_joint"})
    # It still has a velocity and an effort limit, and those are real.
    assert limits == {
        "arm/spin/velocity": Limits(-10.0, 10.0),
        "arm/spin/effort": Limits(-5.0, 5.0),
    }


def test_a_half_written_position_range_raises() -> None:
    # The dangerous one: dropping the side the model does declare would leave
    # the joint unlimited, the description would still pass its checks, and
    # any position at all would then be accepted.
    only_lower = TOY_URDF.replace(
        '<limit lower="-1.5" upper="2.5" velocity="3.0" effort="40.0"/>',
        '<limit lower="-1.5" velocity="3.0" effort="40.0"/>',
    )
    with pytest.raises(ValueError, match="declares a lower position bound but no upper"):
        limits_from_urdf(only_lower, {"arm/j1": "bounded_joint"})

    only_upper = TOY_URDF.replace(
        '<limit lower="-1.5" upper="2.5" velocity="3.0" effort="40.0"/>',
        '<limit upper="2.5" velocity="3.0" effort="40.0"/>',
    )
    with pytest.raises(ValueError, match="declares a upper position bound but no lower"):
        limits_from_urdf(only_upper, {"arm/j1": "bounded_joint"})


def test_only_a_joint_that_spins_freely_may_go_without_a_position_limit() -> None:
    # Same empty <limit> as the spinner, but typed revolute: that is a broken
    # model, not a free spinner, and guessing "unbounded" would take a real
    # arm's limits away.
    mistyped = TOY_URDF.replace(
        '<joint name="spinner_joint" type="continuous">',
        '<joint name="spinner_joint" type="revolute">',
    )
    with pytest.raises(ValueError, match="is revolute but declares no position bounds"):
        limits_from_urdf(mistyped, {"arm/spin": "spinner_joint"})


def test_a_half_written_range_is_still_caught_when_position_is_wanted_alone() -> None:
    only_lower = TOY_URDF.replace(
        '<limit lower="-1.5" upper="2.5" velocity="3.0" effort="40.0"/>',
        '<limit lower="-1.5" velocity="3.0" effort="40.0"/>',
    )
    with pytest.raises(ValueError, match="no upper"):
        limits_from_urdf(only_lower, {"arm/j1": "bounded_joint"}, velocity=False, effort=False)
    # Asking for no position at all is the one way past it, and then the
    # velocity and effort bounds are still real.
    assert limits_from_urdf(only_lower, {"arm/j1": "bounded_joint"}, position=False) == {
        "arm/j1/velocity": Limits(-3.0, 3.0),
        "arm/j1/effort": Limits(-40.0, 40.0),
    }


def test_a_joint_with_no_limit_element_raises() -> None:
    # A wrong limits table stops a real arm halfway through a motion, so this
    # never returns a quietly smaller table.
    with pytest.raises(ValueError, match="bare_joint.*no <limit>"):
        limits_from_urdf(TOY_URDF, {"arm/bare": "bare_joint"})


def test_a_mapped_joint_missing_from_the_urdf_raises() -> None:
    with pytest.raises(ValueError, match="'nope'.*not in the URDF"):
        limits_from_urdf(TOY_URDF, {"arm/j1": "nope"})


def test_a_canonical_name_that_is_not_source_slash_resource_raises() -> None:
    with pytest.raises(ValueError, match="not '<source>/<resource>'"):
        limits_from_urdf(TOY_URDF, {"joint1": "bounded_joint"})
    with pytest.raises(ValueError, match="not '<source>/<resource>'"):
        limits_from_urdf(TOY_URDF, {"arm/j1/position": "bounded_joint"})


def test_velocity_and_effort_can_be_switched_off() -> None:
    limits = limits_from_urdf(TOY_URDF, {"arm/j1": "bounded_joint"}, velocity=False, effort=False)
    assert set(limits) == {"arm/j1/position"}


def test_position_can_be_switched_off() -> None:
    limits = limits_from_urdf(TOY_URDF, {"arm/j1": "bounded_joint"}, position=False)
    assert set(limits) == {"arm/j1/velocity", "arm/j1/effort"}


def test_asking_for_nothing_still_checks_the_table() -> None:
    assert (
        limits_from_urdf(
            TOY_URDF, {"arm/j1": "bounded_joint"}, position=False, velocity=False, effort=False
        )
        == {}
    )
    # Even then a joint that is not there is a broken table, not an empty one.
    with pytest.raises(ValueError, match="not in the URDF"):
        limits_from_urdf(TOY_URDF, {"arm/j1": "nope"}, position=False, velocity=False, effort=False)


def test_limits_read_from_a_model_refuse_rather_than_clamp() -> None:
    limits = limits_from_urdf(TOY_URDF, {"arm/j1": "bounded_joint"})
    assert not any(limit.clamp for limit in limits.values())


def test_the_urdf_can_be_a_path_as_well_as_text(tmp_path: Path) -> None:
    written = tmp_path / "toy.urdf"
    written.write_text(TOY_URDF)
    from_text = limits_from_urdf(TOY_URDF, {"arm/j1": "bounded_joint"})
    assert limits_from_urdf(written, {"arm/j1": "bounded_joint"}) == from_text
    # A path given as a plain string works too: only the leading '<' tells the
    # two apart.
    assert limits_from_urdf(str(written), {"arm/j1": "bounded_joint"}) == from_text


def test_a_non_numeric_limit_raises() -> None:
    broken = TOY_URDF.replace('velocity="3.0"', 'velocity="fast"')
    with pytest.raises(ValueError, match="velocity='fast' is not a number"):
        limits_from_urdf(broken, {"arm/j1": "bounded_joint"})


def test_the_in_repo_g1_urdf_is_there() -> None:
    assert G1_URDF.is_file()


def test_known_g1_joint_bounds_come_back_verbatim() -> None:
    # Spelled out from g1.urdf rather than re-read from it, so a change to the
    # model is a failing test and not a silently different table.
    limits = limits_from_urdf(
        G1_URDF,
        {
            "g1/left_hip_pitch": "left_hip_pitch_joint",
            "g1/left_knee": "left_knee_joint",
            "g1/waist_yaw": "waist_yaw_joint",
        },
    )
    assert limits["g1/left_hip_pitch/position"] == Limits(-2.5307, 2.8798)
    assert limits["g1/left_hip_pitch/velocity"] == Limits(-32.0, 32.0)
    assert limits["g1/left_hip_pitch/effort"] == Limits(-88.0, 88.0)
    # An asymmetric range, so nothing here is quietly symmetrizing position.
    assert limits["g1/left_knee/position"] == Limits(-0.087267, 2.8798)
    assert limits["g1/waist_yaw/position"] == Limits(-2.618, 2.618)


def test_the_whole_g1_maps_in_one_call() -> None:
    joints = (
        "left_hip_pitch left_hip_roll left_hip_yaw left_knee left_ankle_pitch left_ankle_roll "
        "right_hip_pitch right_hip_roll right_hip_yaw right_knee right_ankle_pitch "
        "right_ankle_roll waist_yaw waist_roll waist_pitch left_shoulder_pitch "
        "left_shoulder_roll left_shoulder_yaw left_elbow left_wrist_roll left_wrist_pitch "
        "left_wrist_yaw right_shoulder_pitch right_shoulder_roll right_shoulder_yaw "
        "right_elbow right_wrist_roll right_wrist_pitch right_wrist_yaw"
    ).split()
    assert len(joints) == 29
    limits = limits_from_urdf(G1_URDF, {f"g1/{j}": f"{j}_joint" for j in joints})
    assert len(limits) == 29 * 3
    # Every G1 joint is bounded: the whole body is revolute, nothing spins free.
    assert all(
        limit.lo is not None and limit.hi is not None and limit.lo < limit.hi
        for limit in limits.values()
    )


G1_LEG = ("left_hip_pitch", "left_hip_roll", "left_knee")


def test_an_arm_is_described_from_its_model_in_one_call() -> None:
    # The point of the whole file: this table is what a preset is handed.
    arm = manipulator_description(
        "g1", G1_LEG, limits=limits_from_urdf(G1_URDF, {f"g1/{j}": f"{j}_joint" for j in G1_LEG})
    )
    assert arm.limits["g1/left_knee/position"] == Limits(-0.087267, 2.8798)
    assert arm.limits["g1/left_knee/velocity"] == Limits(-20.0, 20.0)
    # The arm is not told an effort, so the model's effort limit has no place.
    assert "g1/left_knee/effort" not in arm.limits


def test_an_arm_described_from_its_model_refuses_a_command_past_a_limit() -> None:
    arm = manipulator_description(
        "g1", G1_LEG, limits=limits_from_urdf(G1_URDF, {f"g1/{j}": f"{j}_joint" for j in G1_LEG})
    )

    def command(knee: float) -> ControlValues:
        return ControlValues(
            source="coordinator",
            epoch=0,
            sequence=1,
            interface_names=["g1/left_knee/position"],
            values=[knee],
        )

    assert not isinstance(validate_command(arm, command(1.0), last_sequence=None), Rejected)
    refused = validate_command(arm, command(3.0), last_sequence=None)
    assert isinstance(refused, Rejected) and refused.reason == "limit"


def test_a_body_is_described_from_its_model_in_one_call() -> None:
    body = pd_joint_description(
        "g1", G1_LEG, limits=limits_from_urdf(G1_URDF, {f"g1/{j}": f"{j}_joint" for j in G1_LEG})
    )
    # A body held by stiffness and damping is told an effort, so that limit stays.
    assert body.limits["g1/left_knee/effort"] == Limits(-139.0, 139.0)
