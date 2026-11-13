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

from dimos.core.coordination.module_coordinator import resolve_module_refs
from dimos.imitation.dataprep.build import inspect_recording, run_dataprep
from dimos.imitation.dataprep.schema import OutputConfig, RecordingSchema
from dimos.robot.galaxea.r1pro._collection_test_fakes import write_collection
from dimos.robot.galaxea.r1pro.blueprints.teleop.r1pro_joylo_desktop import (
    r1pro_openarm_joylo_collect,
    r1pro_openarm_joylo_real,
)
from dimos.robot.galaxea.r1pro.collection import CAMERAS, r1pro_collection


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


@pytest.mark.parametrize("blueprint", [r1pro_openarm_joylo_real, r1pro_openarm_joylo_collect])
def test_hardware_entrypoints_record_robot_streams_with_robot_side_producers(blueprint):
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
        publishers = [
            atom.name
            for atom in blueprint.active_blueprints
            for output in atom.streams
            if output.direction == "out"
            and output.type is port.type
            and blueprint.remapping_map.get((atom.name, output.name), output.name) == topic
        ]
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
