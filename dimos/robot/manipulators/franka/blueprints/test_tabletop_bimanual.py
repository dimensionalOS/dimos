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

import platform
import sys

import pytest

# The tests extra excludes yourdfpy on Linux ARM because embreex has no wheel.
if sys.platform == "linux" and platform.machine() == "aarch64":
    pytest.importorskip(
        "yourdfpy", reason="yourdfpy is unavailable in the Linux ARM test environment"
    )

from dimos.core.coordination.blueprints import Blueprint
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.robot.manipulators.franka.blueprints.tabletop_bimanual import (
    DEFAULT_CAMERA_SERIALS,
    DEFAULT_IPS,
    FrankaTabletopCoordinator,
    fr3_tabletop_bimanual,
)
from dimos.robot.manipulators.franka.connection import FrankaConnection
from dimos.robot.manipulators.franka.separation_guard import SeparationGuard

SIDES = ("right", "left")


def _topics(blueprint: Blueprint) -> dict[tuple[str, str], str]:
    """The name each module instance's stream connects on, keyed by (instance, stream)."""
    return {
        (atom.name, stream.name): blueprint.remapping_map.get((atom.name, stream.name), stream.name)
        for atom in blueprint.active_blueprints
        for stream in atom.streams
    }


@pytest.fixture(scope="module")
def solid() -> Blueprint:
    return fr3_tabletop_bimanual.solidify()


def _atom(solid: Blueprint, name: str):  # type: ignore[no-untyped-def]
    return next(a for a in solid.active_blueprints if a.name == name)


def test_each_arm_gets_its_own_connection_at_its_own_address(solid: Blueprint) -> None:
    for side in SIDES:
        atom = _atom(solid, f"{side}_arm")
        assert atom.module is FrankaConnection
        assert atom.kwargs["source"] == side
        assert atom.kwargs["ip"] == DEFAULT_IPS[side]
    assert DEFAULT_IPS["right"] != DEFAULT_IPS["left"]


@pytest.mark.parametrize("side", SIDES)
def test_every_command_reaches_the_arm_through_the_guard(solid: Blueprint, side: str) -> None:
    topics = _topics(solid)
    guard = next(a.name for a in solid.active_blueprints if a.module is SeparationGuard)
    from_coordinator = topics["ControlCoordinator", f"{side}_position_command"]
    into_arm = topics[f"{side}_arm", "position_command"]
    assert topics[guard, f"{side}_request"] == from_coordinator
    assert topics[guard, f"{side}_command"] == into_arm
    assert from_coordinator != into_arm
    # Nothing but the guard publishes onto the arm's command topic.
    writers = [
        (name, stream)
        for (name, stream), topic in topics.items()
        if topic == into_arm and (name, stream) != (f"{side}_arm", "position_command")
    ]
    assert writers == [(guard, f"{side}_command")]


@pytest.mark.parametrize("side", SIDES)
def test_each_arms_joints_reach_the_coordinator_and_the_guard(solid: Blueprint, side: str) -> None:
    topics = _topics(solid)
    guard = next(a.name for a in solid.active_blueprints if a.module is SeparationGuard)
    state = topics[f"{side}_arm", "joint_state"]
    assert topics["ControlCoordinator", f"{side}_joint_state"] == state
    assert topics[guard, f"{side}_joint_state"] == state
    other = "left" if side == "right" else "right"
    assert topics[f"{other}_arm", "joint_state"] != state


def test_the_coordinator_drives_both_arms(solid: Blueprint) -> None:
    atom = _atom(solid, "ControlCoordinator")
    assert atom.module is FrankaTabletopCoordinator
    assert {c.source for c in atom.kwargs["connections"]} == set(SIDES)


def test_each_camera_has_its_own_serial_and_streams(solid: Blueprint) -> None:
    topics = _topics(solid)
    images = set()
    for camera, serial in DEFAULT_CAMERA_SERIALS.items():
        atom = _atom(solid, camera)
        assert atom.kwargs["serial_number"] == serial
        assert atom.kwargs["frame_id"] == f"{camera}_camera_link"
        images.add(topics[camera, "color_image"])
        assert topics[camera, "color_image"] == f"{camera}_color_image"
    assert len(images) == 3


def test_the_planner_places_all_three_cameras(solid: Blueprint) -> None:
    atom = next(a for a in solid.active_blueprints if a.module is ManipulationModule)
    children = {t.child_frame_id for t in atom.kwargs["static_transforms"]}
    assert children == {f"{camera}_camera_link" for camera in DEFAULT_CAMERA_SERIALS}
    # The wrist cameras hang off the Hands, which the planner must publish on tf.
    assert {"right/fr3_hand", "left/fr3_hand"} <= set(atom.kwargs["model"].tf_extra_links)
