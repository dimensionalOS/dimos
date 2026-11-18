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

"""Test helpers: robot models from disk only, never downloaded, and arms heavier than modelled.

Robot models come from Git LFS archives and cached description repositories.
Inside ``offline_robot_assets`` a cached checkout is used as it is (no fetch),
and a model that is not on disk raises ``ModelNotOnDiskError`` instead of
reaching the network; ``require_model`` turns that into a skip with the reason.
"""

from __future__ import annotations

from collections.abc import Iterator, Mapping
from contextlib import contextmanager
from pathlib import Path
from typing import Any
from unittest import mock

import numpy as np
from numpy.typing import NDArray
import pinocchio as pin
import pytest

from dimos.robot.assets.git_cache import GitAssetCache
from dimos.robot.assets.model import RobotModel
from dimos.robot.manipulators.calibration.gravity_poses import ArmModel
from dimos.robot.manipulators.calibration.spec import ArmCalibrationSpec, specs_from_damiao_robot
from dimos.robot.manipulators.damiao.robot import DamiaoRobot
import dimos.utils.data as lfs_data


class ModelNotOnDiskError(RuntimeError):
    """A robot model would have to be downloaded."""


def _cached_checkout(self: GitAssetCache, repo_url: str, ref: str, checkout_path: Path) -> Path:
    return checkout_path


def _no_clone(self: GitAssetCache, repo_url: str, ref: str, checkout_path: Path) -> Path:
    raise ModelNotOnDiskError(f"{repo_url} at {ref} is not in the robot asset cache")


def _local_archive(filename: str | Path) -> Path:
    path = lfs_data._get_lfs_dir() / f"{filename}.tar.gz"
    if not path.exists() or lfs_data._is_lfs_pointer_file(path):
        raise ModelNotOnDiskError(f"the LFS archive {path.name} has not been pulled")
    return path


@contextmanager
def offline_robot_assets() -> Iterator[None]:
    """Use cached description checkouts as they are and never pull LFS data."""
    with (
        mock.patch.object(GitAssetCache, "_refresh_cached", _cached_checkout),
        mock.patch.object(GitAssetCache, "_clone_missing", _no_clone),
        mock.patch.object(lfs_data, "_pull_lfs_archive", _local_archive),
    ):
        yield


def require_model(model: RobotModel, what: str) -> None:
    """Load ``model`` from disk or skip the test, saying what is missing."""
    try:
        with offline_robot_assets():
            model.load()
    except ModelNotOnDiskError as error:
        pytest.skip(f"{what} model is not on disk: {error}")


def robot_specs_or_skip(robot: DamiaoRobot, what: str, **options: Any) -> list[ArmCalibrationSpec]:
    """``specs_from_damiao_robot(robot, **options)`` when its gravity model is on disk, else skip."""
    require_model(robot.gravity_model, what)
    return specs_from_damiao_robot(robot, **options)


# A camera and bracket on the gripper, a heavier forearm: what the rig likely has.
# Link: (d mass kg, d(m*c) kg m).
PERTURBATION: dict[str, tuple[float, NDArray[np.float64]]] = {
    "left_gripper": (0.25, np.array([-0.02, 0.0, -0.01])),
    "left_link3": (0.15, np.array([0.0, 0.0, -0.02])),
}


def perturbed_model(
    arm: ArmModel, perturbation: Mapping[str, tuple[float, NDArray[np.float64]]]
) -> pin.Model:
    """``arm``'s model with masses and first moments added to the named links: the "true" arm."""
    model = arm.model.copy()
    for link, (d_mass, d_moment) in perturbation.items():
        joint_id = int(model.frames[model.getFrameId(link, pin.FrameType.BODY)].parentJoint)
        inertia = model.inertias[joint_id]
        mass = inertia.mass + d_mass
        com = (inertia.mass * inertia.lever + d_moment) / mass
        model.inertias[joint_id] = pin.Inertia(mass, com, inertia.inertia)
    return model
