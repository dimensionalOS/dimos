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

"""Blueprint helpers for the Robosuite simulator runtime module."""

from __future__ import annotations

from pathlib import Path

from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.core.runtime_environment import PythonProjectRuntimeEnvironment
from dimos_robosuite_sidecar.module import RobosuiteRuntimeModule

ROBOSUITE_RUNTIME_ENV_NAME = "dimos-robosuite-runtime"
ROBOSUITE_RUNTIME_PROJECT = Path(__file__).resolve().parents[2]


def robosuite_runtime_blueprint(
    runtime: PythonProjectRuntimeEnvironment | None = None,
    *,
    env_name: str = "Lift",
    robot_id: str = "panda",
    robot_model: str = "Panda",
    controller: str = "JOINT_POSITION",
    control_freq: int = 20,
    horizon: int = 200,
    camera_name: str = "agentview",
    seed: int | None = None,
    visualize: bool = False,
    image_dump_dir: str | Path | None = None,
    image_dump_every: int = 0,
) -> Blueprint:
    """Create a placed Robosuite simulator runtime module blueprint."""

    environment = runtime or PythonProjectRuntimeEnvironment(
        name=ROBOSUITE_RUNTIME_ENV_NAME,
        project=ROBOSUITE_RUNTIME_PROJECT,
    )
    return (
        autoconnect(
            RobosuiteRuntimeModule.blueprint(
                env_name=env_name,
                robot_id=robot_id,
                robot_model=robot_model,
                controller=controller,
                control_freq=control_freq,
                horizon=horizon,
                camera_name=camera_name,
                seed=seed,
                visualize=visualize,
                image_dump_dir=image_dump_dir,
                image_dump_every=image_dump_every,
            )
        )
        .runtime_environments(environment)
        .runtime_placements({RobosuiteRuntimeModule: environment.name})
    )
