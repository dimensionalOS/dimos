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

"""Real CPU planning-world registration with a hermetic robot model."""

import copy

import pytest

from dimos.manipulation.planning.groups.models import PlanningGroupDefinition
from dimos.manipulation.planning.monitor.world_monitor import WorldMonitor
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.robot.assets.model import RobotModel
from dimos.simulation.behavior.radio_motion import RadioManipulationModule

pytestmark = pytest.mark.self_hosted

# The optional backend is absent from lightweight environments. No simulator is started.
pytest.importorskip("roboplan")
from dimos.manipulation.planning.world import roboplan_world


@pytest.fixture
def radio_runtime(tmp_path):
    # Registration ownership/rollback does not depend on R1 kinematics. Keep
    # the real native world, but require no licensed simulator asset checkout.
    path = tmp_path / "registration.urdf"
    path.write_text("""<robot name="registration">
      <link name="base"/><link name="tip">
        <collision><geometry><sphere radius="0.01"/></geometry></collision>
      </link>
      <joint name="slide" type="prismatic">
        <parent link="base"/><child link="tip"/><axis xyz="1 0 0"/>
        <limit lower="-1" upper="1" effort="10" velocity="1" acceleration="2"/>
      </joint></robot>""")
    model = RobotModelConfig(
        model=RobotModel.from_file(path),
        joint_names=["slide"],
        base_link="base",
        planning_groups=[PlanningGroupDefinition("right_arm", ("slide",), "base", "tip")],
    )
    # Native integration tests reload bindings. Resolve the current class at
    # setup, rather than retaining its identity from test collection.
    world = roboplan_world.RoboPlanWorld()
    monitor = WorldMonitor(world)
    runtime = RadioManipulationModule(model=model)
    try:
        monitor.load_model(model)
        monitor.finalize()
        runtime._world_monitor = monitor
        yield runtime, world
    finally:
        monitor.stop_all_monitors()
        runtime.stop()


@pytest.fixture
def geometry():
    return [
        {
            "name": "radio",
            "position": [2, 0, 1],
            "orientation": [0, 0, 0, 1],
            "extent": [0.2, 0.1, 0.1],
        },
        {
            "name": "table",
            "position": [2, 0, 0.5],
            "orientation": [0, 0, 0, 1],
            "extent": [1, 1, 0.1],
        },
    ]


def test_repeated_registration_preserves_real_world_and_snapshots(radio_runtime, geometry):
    runtime, world = radio_runtime
    source = {"calibration": [0, 0, 0.0053]}
    reference = {"radio": {"position": [2, 0, 1]}}
    runtime.configure_development_collision_scene(geometry, source, reference)
    snapshot = runtime.get_development_collision_scene()
    runtime.configure_development_collision_scene(list(reversed(geometry)), source, reference)
    assert {o.name for o in world.get_obstacles()} == {"radio", "table"}
    assert runtime.get_development_collision_scene() == snapshot
    geometry[0]["position"][0] = 99
    snapshot["objects"][0]["extent"][0] = 99
    assert runtime.get_development_collision_scene()["objects"][0]["position"] == [2, 0, 1]
    assert runtime.get_development_collision_scene()["objects"][0]["extent"] == [0.2, 0.1, 0.1]


@pytest.mark.parametrize("field", ["position", "extent", "orientation"])
def test_conflicting_geometry_rejected_without_mutation(radio_runtime, geometry, field):
    runtime, world = radio_runtime
    runtime.configure_development_collision_scene(geometry)
    before = runtime.get_development_collision_scene()
    changed = copy.deepcopy(geometry)
    changed[0][field][0] += 0.01
    with pytest.raises(ValueError, match="Conflicting"):
        runtime.configure_development_collision_scene(changed)
    assert runtime.get_development_collision_scene() == before
    assert {o.name for o in world.get_obstacles()} == {"radio", "table"}


def test_conflicting_owner_rejected_and_partial_registration_rolls_back(
    radio_runtime, geometry, mocker
):
    runtime, world = radio_runtime
    original_add = runtime._world_monitor.add_obstacle

    def fail_support(obstacle):
        return original_add(obstacle) if obstacle.name == "radio" else ""

    mocker.patch.object(runtime._world_monitor, "add_obstacle", side_effect=fail_support)
    remove = mocker.spy(runtime._world_monitor, "remove_obstacle")
    with pytest.raises(RuntimeError, match="not registered"):
        runtime.configure_development_collision_scene(geometry)
    remove.assert_called_once_with("radio")
    assert runtime.get_development_collision_scene() is None
    assert world.get_obstacles() == []


def test_changed_source_or_reference_rejected(radio_runtime, geometry):
    runtime, _ = radio_runtime
    runtime.configure_development_collision_scene(
        geometry, {"calibration": [0, 0, 0]}, {"episode": "a"}
    )
    for source, reference in (
        ({"calibration": [0, 0, 0.001]}, {"episode": "a"}),
        ({"calibration": [0, 0, 0]}, {"episode": "b"}),
    ):
        with pytest.raises(ValueError, match="Conflicting"):
            runtime.configure_development_collision_scene(geometry, source, reference)


def test_external_scene_mutation_is_not_hidden_by_same_registration(radio_runtime, geometry):
    runtime, world = radio_runtime
    runtime.configure_development_collision_scene(geometry)
    assert world.update_obstacle_pose("radio", PoseStamped(frame_id="world", position=[3, 0, 1]))
    with pytest.raises(ValueError, match="was changed"):
        runtime.configure_development_collision_scene(geometry)


def test_existing_unowned_names_are_rejected(radio_runtime, geometry):
    runtime, world = radio_runtime
    runtime.configure_development_collision_scene(geometry)
    # A new module on the same external world has no right to adopt its named objects.
    another = RadioManipulationModule(model=runtime.config.model)
    another._world_monitor = runtime._world_monitor
    try:
        with pytest.raises(ValueError, match="another owner"):
            another.configure_development_collision_scene(geometry)
        assert another.get_development_collision_scene() is None
        assert {o.name for o in world.get_obstacles()} == {"radio", "table"}
    finally:
        another.stop()
