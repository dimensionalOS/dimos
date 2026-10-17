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

"""Point-LIO's inputs must each come from one Mid360, stamped in its sensor frame.

autoconnect merges same-named streams, so a second IMU on the topic would feed the
estimator two sensors and it would silently stop publishing.
"""

import warnings

from pydantic import BaseModel
import pytest

from dimos.core.coordination.blueprints import Blueprint, BlueprintAtom
from dimos.core.global_config import global_config
from dimos.hardware.sensors.lidar.livox.module import Mid360, Mid360Config
from dimos.hardware.sensors.lidar.pointlio.module import PointLio, PointLioConfig
from dimos.robot.get_all_blueprints import OptionalDependencyError, load_blueprint
from dimos.robot.test_all_blueprints import UBUNTU_BLUEPRINTS

POINTLIO_INPUTS = ("lidar_raw", "imu_raw")


def _producers(blueprint: Blueprint, consumer: str, port: str) -> list[BlueprintAtom]:
    topic = blueprint.remapping_map.get((consumer, port), port)
    found = []
    for atom in blueprint.active_blueprints:
        for stream in atom.streams:
            if stream.direction != "out":
                continue
            if blueprint.remapping_map.get((atom.name, stream.name), stream.name) == topic:
                found.append(atom)
    return found


def _frame(atom: BlueprintAtom, field: str, config: type[BaseModel]) -> str:
    return str(atom.kwargs.get(field, config.model_fields[field].default))


def test_every_pointlio_input_has_one_producer(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(global_config, "robot_ips", "192.0.2.10,192.0.2.11")
    checked = []
    skipped = []
    for name in UBUNTU_BLUEPRINTS:
        try:
            blueprint = load_blueprint(name)
        except OptionalDependencyError as e:
            skipped.append(f"{name} ({e})")
            continue
        lio = [atom for atom in blueprint.active_blueprints if atom.module is PointLio]
        for atom in lio:
            for port in POINTLIO_INPUTS:
                producers = _producers(blueprint, atom.name, port)
                names = [f"{p.name}.{port}" for p in producers]
                assert len(producers) == 1, f"{name}: {atom.name}.{port} fed by {names}"
                assert producers[0].module is Mid360, f"{name}: {atom.name}.{port} fed by {names}"
            driver = _producers(blueprint, atom.name, "lidar_raw")[0]
            driver_frame = _frame(driver, "frame_id", Mid360Config)
            sensor_frame = _frame(atom, "sensor_frame_id", PointLioConfig)
            assert driver_frame == sensor_frame, f"{name}: {driver_frame} != {sensor_frame}"
            assert _frame(driver, "point_format", Mid360Config) == "full", name
            checked.append(name)
    assert checked, "no registered blueprint runs PointLio"
    if skipped:
        warnings.warn(
            f"checked {len(checked)} blueprints, could not import: {skipped}", stacklevel=1
        )
