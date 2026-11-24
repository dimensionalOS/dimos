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

"""Robot models from disk only, never downloaded.

Robot models come from Git LFS archives and cached description repositories.
Inside ``offline_robot_assets`` a cached checkout is used as it is (no fetch),
and a model that is not on disk raises ``ModelNotOnDiskError`` instead of
reaching the network; ``require_model`` turns that into a skip with the reason.
"""

from __future__ import annotations

from collections.abc import Iterator
from contextlib import contextmanager
from pathlib import Path
from unittest import mock

import pytest

from dimos.robot.assets.git_cache import GitAssetCache
from dimos.robot.assets.model import RobotModel
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
