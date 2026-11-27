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
from dimos.robot.assets.usd_utils.frames import usd_frame_names
from dimos.robot.assets.usd_utils.layers import usd_under_namespace
from dimos.sim2.assets.logistics import LOGISTICS
from dimos.sim2.assets.toy import toy_robot
from dimos.sim2.sim import SimulatedRobotSpec, SimWorld, compose_usds, meeting, sim
from dimos.sim2.spec import SimulatedRobot, validate

pytest.importorskip("pxr")


def _room() -> str:
    return (
        UsdModel.from_file(LOGISTICS).with_frames({"dog": ("logistics", (0, 0, 0), (0, 0, 0))}).usd
    )


# The logistics room's frame a dog mounts on, and so the dog's namespace.
_SPOT = "logistics/dog"


def _dog() -> str:
    """The dog's USD as it serves it: its prims under its namespace."""
    return usd_under_namespace(toy_robot().usd, _SPOT)


def test_each_hardware_is_overlaid_at_the_frame_it_is_mounted_on() -> None:
    world = compose_usds([(_room(), None), (_dog(), _SPOT)])
    frames = usd_frame_names(world.usd)
    assert _SPOT in frames
    assert f"{_SPOT}/{toy_robot().root_link}" in frames


def test_a_world_needs_exactly_one_unmounted_usd() -> None:
    with pytest.raises(ValueError, match="exactly one unmounted"):
        compose_usds([(_dog(), _SPOT)])
    with pytest.raises(ValueError, match="exactly one unmounted"):
        compose_usds([(_room(), None), (_room(), None)])


def test_a_mount_frame_nothing_has_is_refused() -> None:
    with pytest.raises(ValueError, match="mount frames \\['cat'\\]"):
        compose_usds([(_room(), None), (_dog(), "cat")])


def test_a_mount_frame_only_the_mounted_usd_itself_holds_is_refused() -> None:
    """A mounted USD holds its mount's path as its namespace, which is no frame anything else offers."""
    stray = usd_under_namespace(toy_robot().usd, "logistics/cat")
    with pytest.raises(ValueError, match="mount frames \\['logistics/cat'\\]"):
        compose_usds([(_room(), None), (stray, "logistics/cat")])


def test_the_world_keeps_one_physics_scene_and_welds_each_part_to_its_mount() -> None:
    from pxr import Sdf, Usd, UsdPhysics

    layer = Sdf.Layer.CreateAnonymous(".usda")
    assert layer.ImportFromString(compose_usds([(_room(), None), (_dog(), _SPOT)]).usd)
    stage = Usd.Stage.Open(layer)
    scenes = [prim for prim in stage.Traverse() if prim.IsA(UsdPhysics.Scene)]
    assert [str(prim.GetPath()) for prim in scenes] == ["/PhysicsScene"]
    welds = [prim for prim in stage.Traverse() if prim.GetTypeName() == "PhysicsJoint"]
    assert [
        str(t) for weld in welds for t in weld.GetRelationship("physics:body0").GetTargets()
    ] == [f"/{_SPOT}"]


class _Running:
    """A running module as ``on_system_modules`` hands it over: what it answers is its ``rpcs``."""

    def __init__(self, *rpcs: str) -> None:
        self.rpcs = frozenset(rpcs)


def test_the_world_finds_its_hardware_and_robots_by_what_they_answer() -> None:
    room = _Running("get_usd", "attached_to", "get_hardware_transform_ids", "start")
    dog = _Running("get_usd", "attached_to", "get_hardware_transform_ids", "simulated_robot")
    other = _Running("get_usd", "describe")
    assert meeting(HardwareSpec, [room, dog, other]) == [room, dog]
    assert meeting(SimulatedRobotSpec, [room, dog, other]) == [dog]


def test_the_sim_choice_picks_a_backend_by_name() -> None:
    choice = sim({"mujoco": SimWorld.blueprint(), "other": SimWorld.blueprint()}, default="mujoco")
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


def test_on_mount_shares_the_sim_streams_its_blueprint_has_and_namespaces_the_rest() -> None:
    from dimos.core.coordination.blueprints import autoconnect
    from dimos.core.hardware_module import Mount
    from dimos.sim2.assets.logistics import Logistics
    from dimos.sim2.sim import _stream_names, on_mount

    mounted = on_mount(autoconnect(Logistics.blueprint()), Mount("room/spot")).solidify()

    assert _stream_names(mounted) == {
        "room/spot/hardware_file_request",
        "room/spot/hardware_file_response",
        "tf",
        "tf_static",
        "world_observations",
    }
