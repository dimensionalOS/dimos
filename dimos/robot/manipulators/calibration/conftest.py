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

"""Calibration test fixtures: robot specs built only from models already on disk."""

from __future__ import annotations

from collections.abc import Iterator

import pytest

from dimos.robot.manipulators.calibration._calibration_test_helpers import (
    offline_robot_assets,
    require_model,
    robot_specs_or_skip,
)
from dimos.robot.manipulators.calibration.spec import ArmCalibrationSpec
from dimos.robot.manipulators.dual_openyam.calibration_workspace import (
    dual_openyam_calibration_specs,
)
from dimos.robot.manipulators.dual_openyam.config import DUAL_OPENYAM
from dimos.robot.manipulators.seeed_b601_dm.config import B601_DM


@pytest.fixture(scope="session", autouse=True)
def offline_assets() -> Iterator[None]:
    """No calibration test downloads or refreshes a robot model."""
    with offline_robot_assets():
        yield


@pytest.fixture(scope="session")
def dual_openyam_specs(offline_assets: None) -> list[ArmCalibrationSpec]:
    """Both Dual OpenYAM arms with the bench guard: left, then right."""
    require_model(DUAL_OPENYAM.gravity_model, "Dual OpenYAM")
    return dual_openyam_calibration_specs()


@pytest.fixture(scope="session")
def left_spec(dual_openyam_specs: list[ArmCalibrationSpec]) -> ArmCalibrationSpec:
    return dual_openyam_specs[0]


@pytest.fixture(scope="session")
def right_spec(dual_openyam_specs: list[ArmCalibrationSpec]) -> ArmCalibrationSpec:
    return dual_openyam_specs[1]


@pytest.fixture(scope="session")
def b601_spec(offline_assets: None) -> ArmCalibrationSpec:
    """The Seeed B601-DM, a single non-OpenYAM Damiao arm, with default guard and ranges."""
    return robot_specs_or_skip(B601_DM, "b601_dm", "Seeed B601-DM")[0]
