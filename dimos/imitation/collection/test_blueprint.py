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

from __future__ import annotations

from types import SimpleNamespace

import pytest

from dimos.control.contract.keys import SEPARATOR
from dimos.control.coordinator import ControlCoordinator
from dimos.core.coordination.blueprint_config.errors import BlueprintConfigError
from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import IO
from dimos.imitation.collection import blueprint as collection
from dimos.imitation.collection.blueprint import (
    learning_collect_webxr_piper,
    learning_collect_webxr_xarm7,
)
from dimos.imitation.collection.episode_monitor import (
    EpisodeMonitorModule,
    EpisodeMonitorModuleConfig,
)
from dimos.imitation.collection.recorder import CollectionModule
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.get_all_blueprints import get_by_name
from dimos.robot.unitree.g1.blueprints.basic.unitree_g1_teleop import unitree_g1_teleop
from dimos.teleop.webxr.extensions import ArmTeleopModule

AGGREGATE = "coordinator_joint_state"


@pytest.mark.parametrize(
    "blueprint",
    [
        learning_collect_webxr_xarm7.solidify(),
        learning_collect_webxr_piper.solidify(),
        unitree_g1_teleop.solidify(),
    ],
)
def test_shipped_collection_monitor_configuration_is_valid(blueprint: Blueprint) -> None:
    monitor = next(
        atom for atom in blueprint.active_blueprints if atom.module is EpisodeMonitorModule
    )
    config = EpisodeMonitorModuleConfig(**monitor.kwargs)

    assert config.task.strip()
    assert any(
        stream.name == "teleop_buttons" and stream.direction == "in" for stream in monitor.streams
    )


@pytest.mark.parametrize(
    "blueprint",
    [learning_collect_webxr_xarm7.solidify(), learning_collect_webxr_piper.solidify()],
)
def test_collection_uses_sqlite_without_tf(blueprint: Blueprint) -> None:
    recorder = next(
        atom for atom in blueprint.blueprints if issubclass(atom.module, CollectionModule)
    )

    assert recorder.kwargs["format"] == "sqlite"
    schema = recorder.kwargs["recording_schema"]
    assert schema.action["action"] == schema.observation["state"]
    assert schema.observation["camera"].shape == (480, 848, 3)
    assert schema.sync.rate_hz == 30
    coordinator = next(
        atom for atom in blueprint.active_blueprints if atom.name == "ControlCoordinator"
    )
    joints = [
        *(
            joint
            for hardware in coordinator.kwargs.get("hardware", [])
            for joint in hardware.joints
        ),
        *(
            joint if SEPARATOR in joint else f"{connection.source}{SEPARATOR}{joint}"
            for connection in coordinator.kwargs.get("connections", [])
            for joint in connection.joint_commands
        ),
    ]
    assert joints
    assert schema.observation["state"].names == joints
    assert schema.observation["state"].shape == (len(joints),)


@pytest.mark.parametrize(
    "blueprint",
    [learning_collect_webxr_xarm7.solidify(), learning_collect_webxr_piper.solidify()],
)
def test_collect_stops_after_producers(blueprint: Blueprint) -> None:
    assert issubclass(blueprint.active_blueprints[0].module, CollectionModule)


@pytest.mark.parametrize(
    "blueprint",
    [learning_collect_webxr_xarm7.solidify(), learning_collect_webxr_piper.solidify()],
)
def test_episode_monitor_stops_after_input_producers(blueprint: Blueprint) -> None:
    assert blueprint.active_blueprints[1].module is EpisodeMonitorModule


@pytest.mark.parametrize(
    "blueprint",
    [learning_collect_webxr_xarm7.solidify(), learning_collect_webxr_piper.solidify()],
)
def test_collection_status_is_wired_to_webxr_hud(blueprint: Blueprint) -> None:
    hud = next(atom for atom in blueprint.blueprints if atom.module is ArmTeleopModule)
    status = next(stream for stream in hud.streams if stream.name == "status")

    assert status.direction == "in"
    assert status.type.__name__ == "String"


def _joint_streams(blueprint: Blueprint) -> dict[tuple[str, str], str]:
    """(instance, port) -> effective stream name, as ModuleCoordinator pairs streams."""
    return {
        (atom.name, stream.name): blueprint.remapping_map.get((atom.name, stream.name), stream.name)
        for atom in blueprint.active_blueprints
        for stream in atom.streams
        if stream.type is JointState
    }


@pytest.mark.parametrize(
    "blueprint",
    [learning_collect_webxr_xarm7.solidify(), learning_collect_webxr_piper.solidify()],
)
def test_recorder_reads_aggregate_joint_state(blueprint: Blueprint) -> None:
    streams = _joint_streams(blueprint)

    # Plain name pairing on both ends, no remap in between. The coordinator
    # atom carries its explicit instance_name (the RPC lookup contract).
    assert streams[("recorder", AGGREGATE)] == AGGREGATE
    assert streams[("ControlCoordinator", AGGREGATE)] == AGGREGATE


@pytest.mark.parametrize(
    ("blueprint", "required"),
    [
        (learning_collect_webxr_xarm7.solidify(), ["--xarmconnection.ip", "192.0.2.1"]),
        (learning_collect_webxr_piper.solidify(), ["--piperconnection.can-port", "can0"]),
    ],
)
@pytest.mark.parametrize("dimension,value", [("width", "640"), ("height", "360")])
def test_collection_rejects_camera_dimensions_that_disagree_with_saved_schema(
    blueprint, required, dimension, value
):
    parser = BlueprintConfigParser(blueprint)
    with pytest.raises(BlueprintConfigError, match=dimension):
        parser.parse([*required, f"--RealSenseCamera.{dimension}", value], environ={})


class DuplexCameraConfig(ModuleConfig):
    width: int = 4
    height: int = 2


class DuplexCamera(Module):
    config: DuplexCameraConfig
    color_image: IO[Image]


def test_collection_records_a_camera_that_publishes_on_a_bidirectional_port(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    # as in simulation: the teleop stack brings the camera, no RealSense is added
    monkeypatch.setattr(collection.global_config, "simulation", "mujoco")
    arm = SimpleNamespace(joints=["joint1"])
    teleop = autoconnect(ControlCoordinator.blueprint(hardware=[arm]), DuplexCamera.blueprint())

    recorder, *_ = collection._collection_components("demo", teleop, task="Teleoperate")

    schema = recorder.active_blueprints[0].kwargs["recording_schema"]
    assert schema.observation["camera"].shape == (2, 4, 3)


@pytest.mark.parametrize("name", ["collection-recorder", "collection-module"])
def test_profile_dependent_collection_module_is_not_a_standalone_command(name):
    with pytest.raises(ValueError, match="Unknown blueprint or module"):
        get_by_name(name)
