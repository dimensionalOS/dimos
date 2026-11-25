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

import subprocess
import sys

import pytest

from dimos.core.coordination.blueprints import Blueprint
from dimos.core.hardware_module import HardwareSpec
from dimos.robot.assets.model import UsdModel
from dimos.robot.assets.usd_utils.frames import prefix_usd_frames, usd_frame_names
from dimos.sim2.assets.logistics import LOGISTICS
from dimos.sim2.assets.toy import toy_robot
from dimos.sim2.mujoco.sim import MujocoSim
from dimos.sim2.sim import SimulatedRobotSpec, compose_usds, meeting, sim
from dimos.sim2.spec import SimulatedRobot, validate

pytest.importorskip("pxr")


def _room() -> str:
    return (
        UsdModel.from_file(LOGISTICS).with_frames({"dog": ("logistics", (0, 0, 0), (0, 0, 0))}).usd
    )


def _dog() -> str:
    return prefix_usd_frames(toy_robot().usd, "dog")


def test_each_hardware_is_grafted_onto_the_frame_it_is_mounted_on() -> None:
    world = compose_usds([(_room(), None), (_dog(), "dog")])
    frames = usd_frame_names(world.usd)
    assert "dog" in frames
    assert any(frame.startswith("dog/") for frame in frames)


def test_a_world_needs_exactly_one_unmounted_usd() -> None:
    with pytest.raises(ValueError, match="exactly one unmounted"):
        compose_usds([(_dog(), "dog")])
    with pytest.raises(ValueError, match="exactly one unmounted"):
        compose_usds([(_room(), None), (_room(), None)])


def test_a_mount_frame_nothing_has_is_refused() -> None:
    with pytest.raises(ValueError, match="mount frames \\['cat'\\]"):
        compose_usds([(_room(), None), (_dog(), "cat")])


class _Running:
    """A running module as ``on_system_modules`` hands it over: what it answers is its ``rpcs``."""

    def __init__(self, *rpcs: str) -> None:
        self.rpcs = frozenset(rpcs)


def test_the_world_finds_its_hardware_and_robots_by_what_they_answer() -> None:
    room = _Running("get_usd", "mounted_on", "hardware_namespace", "start")
    dog = _Running("get_usd", "mounted_on", "hardware_namespace", "simulated_robot")
    other = _Running("get_usd", "describe")
    assert meeting(HardwareSpec, [room, dog, other]) == [room, dog]
    assert meeting(SimulatedRobotSpec, [room, dog, other]) == [dog]


def test_the_sim_choice_picks_a_backend_by_name() -> None:
    choice = sim(
        {"mujoco": MujocoSim.blueprint(), "other": MujocoSim.blueprint()}, default="mujoco"
    )
    assert choice.name == "sim"
    assert choice.default == "mujoco"
    assert set(choice.choices) == {"mujoco", "other"}
    assert all(isinstance(option, Blueprint) for option in choice.choices.values())


def test_robots_are_named_once_and_by_their_namespace() -> None:
    with pytest.raises(ValueError, match="duplicate"):
        validate([SimulatedRobot("dog", toy_robot()), SimulatedRobot("dog", toy_robot())])
    with pytest.raises(ValueError, match="namespace"):
        validate([SimulatedRobot("", toy_robot())])


def test_the_sim_protocol_imports_no_engine() -> None:
    code = (
        "import sys; import dimos.sim2.sim;"
        " assert not [m for m in sys.modules if m.startswith('mujoco')], sorted(sys.modules)"
    )
    subprocess.run([sys.executable, "-c", code], check=True)
