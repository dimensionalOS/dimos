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

"""DimOS policy abstractions.

Public surface:

- `PolicyModule` and `PolicyModuleConfig`: the DimOS module that consumes
  multi-camera images, joint state, and a task description, and publishes
  coordinator-native joint position commands through a configured backend.
- `PolicyObservation` / `PolicyCommand`: the canonical observation and
  tagged command models exchanged with backends.
- `PolicyBackend`: the structural protocol every backend implements.
- `register_backend`, `create_backend`, `available_backends`: in-repo
  registry primitives. Built-in ``"test"`` and ``"lerobot"`` backends are
  registered when this package is imported.
- `TestPolicy`: dependency-free sinusoidal backend used for wiring checks.
- `policy_servo_task_config`: blueprint helper that yields a
  `JointServoTask` config with a priority strictly below any overlapping
  teleop task — enforces the teleop-preempts invariant.

The contract layer (`RobotContract`, `contracts/`) used by the LeRobot
backend is also exported from here for convenience.
"""

from __future__ import annotations

# Importing `backends` registers the built-in entries on the registry.
# `node` re-exports it transitively, but keep the explicit import here so
# `from dimos.manipulation.policy import create_backend` works without
# the user also importing `PolicyModule`.
from dimos.manipulation.policy import backends as _backends  # noqa: F401
from dimos.manipulation.policy.backend import PolicyBackend
from dimos.manipulation.policy.backends.test import TestPolicy
from dimos.manipulation.policy.blueprint import (
    policy_engage_buttons,
    policy_servo_task_config,
)
from dimos.manipulation.policy.command import (
    JointPositionCommand,
    NoOpCommand,
    PolicyCommand,
)
from dimos.manipulation.policy.config import CommandMode, PolicyModuleConfig
from dimos.manipulation.policy.contract import (
    GripperBinarization,
    LeRobotFrame,
    RobotContract,
)
from dimos.manipulation.policy.module import PolicyModule
from dimos.manipulation.policy.observation import PolicyObservation
from dimos.manipulation.policy.registry import (
    BackendFactory,
    available_backends,
    create_backend,
    is_registered,
    register_backend,
)
from dimos.manipulation.policy.rollout_toggle import RolloutToggle, RolloutToggleConfig

__all__ = [
    "BackendFactory",
    "CommandMode",
    "GripperBinarization",
    "JointPositionCommand",
    "LeRobotFrame",
    "NoOpCommand",
    "PolicyBackend",
    "PolicyCommand",
    "PolicyModule",
    "PolicyModuleConfig",
    "PolicyObservation",
    "RobotContract",
    "RolloutToggle",
    "RolloutToggleConfig",
    "TestPolicy",
    "available_backends",
    "create_backend",
    "is_registered",
    "policy_engage_buttons",
    "policy_servo_task_config",
    "register_backend",
]
