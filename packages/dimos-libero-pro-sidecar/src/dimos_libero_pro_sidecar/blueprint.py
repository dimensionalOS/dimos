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

"""Blueprint helpers for the LIBERO-PRO simulator runtime module."""

from __future__ import annotations

from pathlib import Path

from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.core.runtime_environment import PythonProjectRuntimeEnvironment
from dimos_libero_pro_sidecar.module import LiberoProRuntimeModule

LIBERO_PRO_RUNTIME_ENV_NAME = "dimos-libero-pro-runtime"
LIBERO_PRO_RUNTIME_PROJECT = Path(__file__).resolve().parents[2]


def libero_pro_runtime_blueprint(
    *,
    bddl_root: str | Path,
    init_states_root: str | Path,
    runtime: PythonProjectRuntimeEnvironment | None = None,
    benchmark_name: str = "libero_90",
    robot_id: str = "panda",
    task_order_index: int = 0,
    task_index: int = 0,
    init_state_index: int = 0,
    controller: str = "JOINT_POSITION",
    camera_names: tuple[str, ...] = ("agentview",),
    control_freq: int = 20,
    horizon: int = 1000,
    seed: int | None = None,
    allow_asset_bootstrap: bool = False,
    visualize: bool = False,
) -> Blueprint:
    """Create a placed LIBERO-PRO simulator runtime module blueprint."""

    environment = runtime or PythonProjectRuntimeEnvironment(
        name=LIBERO_PRO_RUNTIME_ENV_NAME,
        project=LIBERO_PRO_RUNTIME_PROJECT,
    )
    return (
        autoconnect(
            LiberoProRuntimeModule.blueprint(
                bddl_root=bddl_root,
                init_states_root=init_states_root,
                benchmark_name=benchmark_name,
                robot_id=robot_id,
                task_order_index=task_order_index,
                task_index=task_index,
                init_state_index=init_state_index,
                controller=controller,
                camera_names=camera_names,
                control_freq=control_freq,
                horizon=horizon,
                seed=seed,
                allow_asset_bootstrap=allow_asset_bootstrap,
                visualize=visualize,
            )
        )
        .runtime_environments(environment)
        .runtime_placements({LiberoProRuntimeModule: environment.name})
    )
