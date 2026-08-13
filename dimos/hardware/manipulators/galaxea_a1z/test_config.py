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

from dataclasses import replace
import pickle

import pytest

from dimos.hardware.manipulators.galaxea_a1z.config import (
    A1ZConfig,
    A1ZGripperConfig,
    A1ZTeachingConfig,
)
from dimos.utils.data import LfsPath


@pytest.mark.parametrize("value", [-0.1, 1.1])
def test_config_rejects_gravity_compensation_outside_unit_interval(value: float) -> None:
    with pytest.raises(ValueError):
        A1ZConfig(gravity_comp_factor=value)


def test_config_requires_gripper_for_gripper_free_drive() -> None:
    teaching = A1ZTeachingConfig(gripper_free_drive=True)

    with pytest.raises(ValueError, match="requires a configured gripper"):
        A1ZConfig(teaching=teaching)


def test_config_preserves_lazy_path_without_resolving_it() -> None:
    lazy_path = LfsPath("a1z_description/model.urdf")

    config = A1ZConfig(urdf_path=lazy_path)

    assert config.urdf_path is lazy_path
    assert object.__getattribute__(lazy_path, "_lfs_resolved_cache") is None


def test_config_replace_revalidates_updates() -> None:
    config = A1ZConfig(gripper=A1ZGripperConfig())

    with pytest.raises(ValueError):
        replace(config, gravity_comp_factor=2.0)


def test_config_pickle_round_trip() -> None:
    config = A1ZConfig(
        gravity_comp_factor=0.5,
        gripper=A1ZGripperConfig(max_torque=0.7),
        teaching=A1ZTeachingConfig(gripper_free_drive=True),
    )

    restored = pickle.loads(pickle.dumps(config, protocol=pickle.HIGHEST_PROTOCOL))

    assert restored == config
