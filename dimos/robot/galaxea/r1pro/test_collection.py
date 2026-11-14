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

"""R1 episode capture -> validated data -> training action layout."""

import h5py
import numpy as np
import pytest

from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.core.coordination.module_coordinator import resolve_module_refs
from dimos.imitation.dataprep.build import inspect_recording, run_dataprep
from dimos.imitation.dataprep.schema import OutputConfig, RecordingSchema
from dimos.robot.assets.model import UsdModel
from dimos.robot.galaxea.r1pro._collection_test_fakes import write_collection
from dimos.robot.galaxea.r1pro.blueprints.teleop.r1pro_joylo import r1pro_openarm_joylo_sim2
from dimos.robot.galaxea.r1pro.blueprints.teleop.r1pro_joylo_desktop import (
    r1pro_openarm_joylo_real,
)
from dimos.robot.galaxea.r1pro.collection import CAMERAS, r1pro_collection
from dimos.robot.galaxea.r1pro.sim.config import JOINTS
from dimos.robot.galaxea.r1pro.sim.office import SCENE as OFFICE_SCENE
from dimos.robot.usd_manager import UsdManagerConfig
from dimos.sim2.scenes.compose import blueprint_robot

# What stands for the robot and its cameras in each blueprint; nothing else may differ.
HARDWARE = {
    "r1proconnection",
    "headleftcamera",
    "headrightcamera",
    "headcamerainfo",
    "r1spatialsensors",
}
SIMULATION = {
    "simtfmodule",
    "simmodule",
    "simjointencodermodule",
    "r1prosimcontrol",
    "r1prosimconnection",
    "head_camera",
    "left_wrist_camera",
    "right_wrist_camera",
    "camerarendermodule",
}


def _publishers(blueprint, name, kind):
    """The modules that write one stream, after remapping."""
    return [
        atom.name
        for atom in blueprint.active_blueprints
        for output in atom.streams
        if output.direction == "out"
        and output.type is kind
        and blueprint.remapping_map.get((atom.name, output.name), output.name) == name
    ]


def test_recording_exports_measured_state_compressed_cameras_and_causal_actions(tmp_path):
    directory = write_collection(tmp_path / "recording")
    schema = RecordingSchema.read(directory)
    config = schema.dataprep_config(
        directory, OutputConfig(format="hdf5", path=tmp_path / "dataset")
    )
    report = inspect_recording(directory / schema.payload, config=config)
    assert len(report["incomplete_episodes"]) == 1
    assert report["quality"][0]["valid"]
    dataset = run_dataprep(config)
    with h5py.File(dataset) as file:
        assert len(file["episodes"]) == 1  # Discarded and interrupted takes excluded.
        episode = file["episodes/episode_000000"]
        actions = episode["action/action"][:]
        assert actions.shape == (5, 23)
        np.testing.assert_allclose(actions[:, :20], 0.5)
        np.testing.assert_allclose(actions[:, 20], [0.2, 0.4, 0.4, 0.4, 0.4])
        np.testing.assert_allclose(episode["observation/observation.state"][0], np.arange(20) / 100)
        for camera, (_, shape) in CAMERAS.items():
            assert episode[f"observation/observation.images.{camera}"].shape == (5, *shape)


@pytest.mark.parametrize("fault", ["camera", "stale_commands"])
def test_incomplete_sources_cannot_silently_become_training_data(tmp_path, fault):
    directory = write_collection(
        tmp_path / "recording",
        missing_camera="left_wrist" if fault == "camera" else None,
        stale_commands=fault == "stale_commands",
    )
    schema = RecordingSchema.read(directory)
    config = schema.dataprep_config(
        directory, OutputConfig(format="hdf5", path=tmp_path / "dataset")
    )
    with pytest.raises(RuntimeError, match="failed dataset validation"):
        run_dataprep(config)
    assert not (tmp_path / "dataset.hdf5").exists()


def test_collection_contract_has_explicit_selection_and_rpc_episode_controls():
    graph = r1pro_collection(cameras=("head", "right_wrist"))
    recorder, episodes = graph.active_blueprints
    assert recorder.name == "recorder"
    assert episodes.name == "episodes"
    assert episodes.kwargs["button_map"] == {}
    assert "left_wrist" not in recorder.kwargs["input_types"]
    assert "motion_settings" in recorder.kwargs["input_types"]
    assert recorder.kwargs["recording_schema"].action_vector == ("position", "body_velocity")
    with pytest.raises(ValueError, match="unknown"):
        r1pro_collection(cameras=("missing",))


def test_hardware_run_records_robot_streams_with_robot_side_producers():
    blueprint = r1pro_openarm_joylo_real
    modules = {atom.name: atom for atom in blueprint.active_blueprints}
    recorder = modules["recorder"]
    required_inputs = set(recorder.kwargs["input_types"]) | {"status"}
    robot = next(p for p in blueprint.hosted_placements if p.tags == {"r1pro": True})
    assert robot.module_names[0] == "recorder"
    assert "episodes" in robot.module_names
    for port in recorder.streams:
        if port.name not in required_inputs:
            continue
        topic = blueprint.remapping_map.get((recorder.name, port.name), port.name)
        publishers = _publishers(blueprint, topic, port.type)
        assert len(publishers) == 1, (port.name, publishers)
        assert publishers[0] in robot.module_names
    schema = recorder.kwargs["recording_schema"]
    assert {feature.stream for feature in schema.action.values()} == {
        "coordinator_joint_command",
        "coordinator_joint_velocity",
    }
    refs = resolve_module_refs(blueprint)
    assert refs["ControlCoordinator", "_leader"] == "r1projoylo"
    assert refs["r1prowebxrjoystick", "_motion_limits"] == "ControlCoordinator"


def test_simulation_is_the_hardware_stack_on_a_simulated_robot():
    real, sim = r1pro_openarm_joylo_real, r1pro_openarm_joylo_sim2
    hardware = {atom.name: atom for atom in real.active_blueprints}
    simulated = {atom.name: atom for atom in sim.active_blueprints}
    assert set(hardware) - set(simulated) == HARDWARE
    assert set(simulated) - set(hardware) == SIMULATION
    shared = set(hardware) & set(simulated)
    for name in shared:
        assert hardware[name].module is simulated[name].module, name

    # One recorder contract: the same inputs, features, rates and quality rules,
    # each fed by exactly one producer of the same name and type.
    recorder = simulated["recorder"]
    assert recorder.kwargs["input_types"] == hardware["recorder"].kwargs["input_types"]
    assert recorder.kwargs["recording_schema"] == hardware["recorder"].kwargs["recording_schema"]
    for port in recorder.streams:
        if port.name not in {*recorder.kwargs["input_types"], "status"}:
            continue
        assert len(_publishers(sim, port.name, port.type)) == 1, port.name
        assert len(_publishers(real, port.name, port.type)) == 1, port.name

    # One control contract: the same coordinator tasks on the same joints, the
    # same module references, and every shared module on the same streams.
    def tasks(atom):
        return [(t.name, t.type, tuple(t.joint_names), t.priority) for t in atom.kwargs["tasks"]]

    assert tasks(simulated["ControlCoordinator"]) == tasks(hardware["ControlCoordinator"])
    assert resolve_module_refs(sim) == resolve_module_refs(real)
    for name in shared - {"usdmanager"}:  # It serves a scene here and a robot model there.
        for blueprint, atom in ((real, hardware[name]), (sim, simulated[name])):
            assert {
                (s.direction, blueprint.remapping_map.get((name, s.name), s.name), s.type)
                for s in atom.streams
            } == {
                (s.direction, real.remapping_map.get((name, s.name), s.name), s.type)
                for s in hardware[name].streams
            }, name

    # The cameras write what R1SpatialSensors writes, on its names and types.
    def outputs(blueprint, names):
        return {
            (blueprint.remapping_map.get((atom.name, s.name), s.name), s.type)
            for atom in blueprint.active_blueprints
            if atom.name in names
            for s in atom.streams
            if s.direction == "out" and s.name != "tf"
        }

    assert outputs(sim, {"head_camera", "left_wrist_camera", "right_wrist_camera"}) == outputs(
        real, {"r1spatialsensors"}
    )
    for key in ("color_image", "left_wrist", "right_wrist"):
        kind = recorder.kwargs["input_types"][key]
        assert sim.transport_map[key, kind] == real.transport_map[key, kind]
    # Only the simulation starts with no leader arms attached.
    assert simulated["r1proleaderconnection"].kwargs["required"] is False
    assert "required" not in hardware["r1proleaderconnection"].kwargs


def test_a_launch_option_puts_the_simulation_in_another_scene(tmp_path):
    scene = tmp_path / "kitchen" / "scene.usda"
    parsed = BlueprintConfigParser(r1pro_openarm_joylo_sim2).parse(
        [
            f"--usdmanager.model={scene}",
            f"--recorder.recording={tmp_path / 'take'}",
            "--episodes.task=pick up the bottle",
        ],
        environ={},
    )
    kwargs = parsed.module_kwargs("usdmanager")
    config = UsdManagerConfig(**kwargs)
    assert isinstance(config.model, UsdModel)
    assert config.model.source_path == scene
    # Nothing else about the scene manager changes with the scene.
    assert config.publish_joint_tf is False


def test_the_simulation_names_its_scene_and_joints_for_another_environment():
    scene, joints = blueprint_robot(r1pro_openarm_joylo_sim2)
    assert scene.source_path == OFFICE_SCENE
    assert joints == JOINTS
    with pytest.raises(ValueError, match="not a sim2 blueprint"):
        blueprint_robot(r1pro_openarm_joylo_real)
