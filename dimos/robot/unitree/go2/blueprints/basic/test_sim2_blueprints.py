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


from collections.abc import Iterator
import contextlib
from dataclasses import replace
from typing import Any

import pytest

from dimos.core.coordination.blueprints import config_defaults
from dimos.core.hardware_files import hardware_file_topics
from dimos.core.hardware_module import HardwareModule
from dimos.core.transport_factory import make_transport
from dimos.msgs.files.FileInfo import FileInfo
from dimos.robot.assets.usd_utils.frames import usd_frame_names
from dimos.sim2.sensor import SimSensor
from dimos.sim2.sim import compose_usds, fetched_usd

pytest.importorskip("pxr")

_BLUEPRINTS = ["unitree_go2_sim2_mid360", "unitree_go2_sim2_l1"]


def _atoms(name: str) -> list[Any]:
    from importlib import import_module

    return list(getattr(import_module(f"{__package__}.{name}"), name).solidify().active_blueprints)


def _hardware_atoms(name: str) -> list[Any]:
    return [atom for atom in _atoms(name) if issubclass(atom.module, HardwareModule)]


@contextlib.contextmanager
def _hardware(name: str) -> Iterator[list[Any]]:
    modules = [atom.module(**atom.kwargs) for atom in _hardware_atoms(name)]
    try:
        yield modules
    finally:
        for module in modules:
            module.stop()


def _providing_hardware_files(module: HardwareModule) -> str:
    """Wire ``module``'s hardware_file streams to the topics a blueprint gives them and answer on them; its namespace."""
    request_topic, response_topic = hardware_file_topics(module.namespace)
    module.hardware_file_request.transport = make_transport(request_topic, FileInfo)
    module.hardware_file_response.transport = make_transport(response_topic, FileInfo)
    module.provide_hardware_files()
    return module.namespace


@pytest.mark.parametrize("name", _BLUEPRINTS)
def test_each_namespace_has_one_unmounted_hardware(name: str) -> None:
    HardwareModule.blueprint_checks(
        [
            replace(atom, kwargs={**config_defaults(atom.module), **atom.kwargs})
            for atom in _hardware_atoms(name)
        ]
    )


@pytest.mark.self_hosted  # the composed world reads the Go2 meshes and occlusion map from Git LFS
@pytest.mark.parametrize("name", _BLUEPRINTS)
def test_the_room_robot_and_sensors_compose_into_one_world(name: str) -> None:
    with _hardware(name) as hardware:
        frames = usd_frame_names(
            compose_usds(
                [
                    (fetched_usd(_providing_hardware_files(module)), module.attached_to())
                    for module in hardware
                ]
            ).usd
        )
        # The scene is a SimSensor too, but the only unmounted one.
        sensed = {
            module.frame_id
            for module in hardware
            if isinstance(module, SimSensor) and module.attached_to() is not None
        }

    assert len(sensed) == 2, f"{name} mounts the sensors {sorted(sensed)}"
    assert sensed <= frames, f"{name} leaves {sorted(sensed - frames)} out of the world"


@pytest.mark.self_hosted  # Go2Sim loads its policy and meshes from Git LFS
def test_a_simulated_go2_publishes_the_tf_a_real_one_does() -> None:
    from dimos.robot.unitree.go2.connection import ConnectionConfig
    from dimos.robot.unitree.go2.sim2 import Go2Sim
    from dimos.sim2.odom_tf import OdomTf

    (odom_atom,) = [atom for atom in _atoms("unitree_go2_sim2_l1") if atom.module is OdomTf]
    odom_tf = OdomTf(**odom_atom.kwargs)
    with _hardware("unitree_go2_sim2_l1") as hardware:
        go2 = next(module for module in hardware if isinstance(module, Go2Sim))
        edges = {(go2.attached_to(), go2.attach_frame), *odom_tf.tf_edge_ids}
        robot, body = go2.namespace, go2.tf_frames[go2.config.usd.root_link]
        announced = go2.simulated_robot().odom_frame
    odom_tf.stop()

    real_odom = ConnectionConfig.model_fields["odom_frame_id"].default
    # The mount holds odom, and odom (the OdomTf beside it) holds the body, as on a real Go2.
    assert edges == {(robot, real_odom), (real_odom, body)}
    assert body == f"{robot}/go2/Geometry/base_link"
    assert announced == real_odom
