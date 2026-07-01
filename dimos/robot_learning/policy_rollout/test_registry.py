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

import sys

import pytest

from dimos.robot_learning.policy_rollout.backends.lerobot.backend import LeRobotBackend
from dimos.robot_learning.policy_rollout.registry import (
    PolicyBackendRegistry,
    RobotPolicyContractRegistry,
)
from dimos.robot_learning.policy_rollout.vla_jepa_libero_contract import (
    VlaJepaLiberoRobotContract,
)


def test_backend_registry_discovers_lerobot_without_importing_lerobot_policy() -> None:
    registry = PolicyBackendRegistry()

    assert "lerobot" in registry.available()
    assert "lerobot.policies" not in sys.modules


def test_backend_registry_creates_lerobot_backend() -> None:
    backend = PolicyBackendRegistry().create(
        "lerobot", checkpoint_id="lerobot/VLA-JEPA-LIBERO", device="cpu"
    )

    assert isinstance(backend, LeRobotBackend)
    assert backend.describe().checkpoint_id == "lerobot/VLA-JEPA-LIBERO"
    assert backend.describe().device == "cpu"


def test_contract_registry_creates_vla_jepa_libero_contract() -> None:
    contract = RobotPolicyContractRegistry().create("vla_jepa_libero")

    assert isinstance(contract, VlaJepaLiberoRobotContract)


def test_registry_unknown_type_and_duplicate_path_errors() -> None:
    registry = PolicyBackendRegistry()

    with pytest.raises(ValueError, match="Unknown policy backend type"):
        registry.create("missing")

    with pytest.raises(ValueError, match="Duplicate policy backend type"):
        registry.register_path("lerobot", "somewhere.else:create_backend")
