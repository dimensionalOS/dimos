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

"""Point-LIO's inputs must each come from exactly one module.

autoconnect joins every stream of one name and type onto one topic, so a second
IMU or cloud publisher in the same blueprint would feed the estimator interleaved
samples from two sensors and it would silently stop publishing.
"""

import pytest

from dimos.core.coordination.blueprints import Blueprint
from dimos.core.global_config import global_config
from dimos.hardware.sensors.lidar.pointlio.module import PointLio
from dimos.robot.all_blueprints import all_blueprints
from dimos.robot.get_all_blueprints import get_blueprint_by_name
from dimos.robot.test_all_blueprints import OPTIONAL_DEPENDENCIES, OPTIONAL_ERROR_SUBSTRINGS

POINTLIO_INPUTS = ("lidar_raw", "imu_raw")


def _load(name: str) -> Blueprint | None:
    try:
        return get_blueprint_by_name(name)
    except ModuleNotFoundError as e:
        if e.name in OPTIONAL_DEPENDENCIES:
            return None
        raise
    except Exception as e:
        if any(substring in str(e) for substring in OPTIONAL_ERROR_SUBSTRINGS):
            return None
        raise


def _producers(blueprint: Blueprint, consumer: str, port: str) -> list[str]:
    topic = blueprint.remapping_map.get((consumer, port), port)
    found = []
    for atom in blueprint.active_blueprints:
        for stream in atom.streams:
            if stream.direction != "out":
                continue
            if blueprint.remapping_map.get((atom.name, stream.name), stream.name) == topic:
                found.append(f"{atom.name}.{stream.name}")
    return found


def test_every_pointlio_input_has_one_producer(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(global_config, "robot_ips", "192.0.2.10,192.0.2.11")
    checked = []
    for name in sorted(all_blueprints):
        blueprint = _load(name)
        if blueprint is None:
            continue
        lio = [atom for atom in blueprint.active_blueprints if atom.module is PointLio]
        for atom in lio:
            for port in POINTLIO_INPUTS:
                producers = _producers(blueprint, atom.name, port)
                assert len(producers) == 1, f"{name}: {atom.name}.{port} fed by {producers}"
            checked.append(name)
    assert checked, "no registered blueprint runs PointLio"
