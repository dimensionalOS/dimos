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

"""Blueprint helpers for the fake simulator runtime module."""

from __future__ import annotations

from pathlib import Path

from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.core.runtime_environment import PythonProjectRuntimeEnvironment
from dimos_fake_runtime_sidecar.module import FakeRuntimeModule

FAKE_RUNTIME_ENV_NAME = "dimos-fake-runtime"
FAKE_RUNTIME_PROJECT = Path(__file__).resolve().parents[2]


def fake_runtime_blueprint(
    runtime: PythonProjectRuntimeEnvironment | None = None,
    *,
    robot_id: str = "fakebot",
    dof: int = 3,
    step_hz: int = 100,
) -> Blueprint:
    """Create a placed fake simulator runtime module blueprint."""

    environment = runtime or PythonProjectRuntimeEnvironment(
        name=FAKE_RUNTIME_ENV_NAME,
        project=FAKE_RUNTIME_PROJECT,
    )
    return (
        autoconnect(FakeRuntimeModule.blueprint(robot_id=robot_id, dof=dof, step_hz=step_hz))
        .runtime_environments(environment)
        .runtime_placements({FakeRuntimeModule: environment.name})
    )
