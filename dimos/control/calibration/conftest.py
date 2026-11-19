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

"""Calibration test fixtures: robot specs built only from models already on disk.

No calibration test reads the calibrations saved on the machine running it.
"""

from __future__ import annotations

from collections.abc import Iterator
from pathlib import Path

import pytest

from dimos.control.calibration._calibration_test_helpers import (
    offline_robot_assets,
    robot_specs_or_skip,
)
from dimos.control.calibration.spec import ArmCalibrationSpec, WorkspaceGuard
from dimos.robot.manipulators.dual_openyam.config import DUAL_OPENYAM
from dimos.robot.manipulators.seeed_b601_dm.config import B601_DM

# A table 30 mm below the bases, the fingertip 150 mm in front of them.
DUAL_OPENYAM_TEST_BENCH = WorkspaceGuard.for_bench(floor_z_m=-0.03, min_x_m=0.15)
# A table at the base's level, the fingertip 100 mm in front of the base.
B601_TEST_BENCH = WorkspaceGuard.for_bench(floor_z_m=0.0, min_x_m=0.10)


@pytest.fixture(autouse=True)
def calibration_root(
    tmp_path_factory: pytest.TempPathFactory, monkeypatch: pytest.MonkeyPatch
) -> Path:
    """An empty directory Damiao connections look in for saved calibrations."""
    root = tmp_path_factory.mktemp("arms")
    try:
        from dimos.robot.manipulators.damiao import connection
    except ImportError:
        return root
    monkeypatch.setattr(connection, "CALIBRATION_ROOT", root)
    return root


@pytest.fixture(autouse=True)
def offline_assets() -> Iterator[None]:
    """No calibration test downloads or refreshes a robot model."""
    with offline_robot_assets():
        yield


@pytest.fixture(scope="session")
def dual_openyam_specs() -> list[ArmCalibrationSpec]:
    """Both Dual OpenYAM arms on a bench, everything else from the model: left, then right."""
    return robot_specs_or_skip(DUAL_OPENYAM, "Dual OpenYAM", workspace=DUAL_OPENYAM_TEST_BENCH)


@pytest.fixture(scope="session")
def left_spec(dual_openyam_specs: list[ArmCalibrationSpec]) -> ArmCalibrationSpec:
    return dual_openyam_specs[0]


@pytest.fixture(scope="session")
def right_spec(dual_openyam_specs: list[ArmCalibrationSpec]) -> ArmCalibrationSpec:
    return dual_openyam_specs[1]


@pytest.fixture(scope="session")
def b601_spec() -> ArmCalibrationSpec:
    """The Seeed B601-DM, a single non-OpenYAM Damiao arm, on a bench at its base's level.

    Everything but the workspace guard is derived from the model.
    """
    return robot_specs_or_skip(B601_DM, "Seeed B601-DM", workspace=B601_TEST_BENCH)[0]
