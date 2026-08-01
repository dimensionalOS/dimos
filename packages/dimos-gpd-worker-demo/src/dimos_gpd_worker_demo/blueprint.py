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

from __future__ import annotations

from pathlib import Path

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.runtime_environment import PythonProjectRuntimeEnvironment

GPD_DEMO_ENV_NAME = "dimos-gpd-worker-demo"
GPD_DEMO_PROJECT = Path(__file__).resolve().parents[2]


class GpdImportProbe(Module):
    @rpc
    def import_gpd_core(self) -> str:
        """Import gpd.core in the worker runtime and report success."""
        import gpd.core as gpd_core

        module_file = getattr(gpd_core, "__file__", "<unknown>")
        module_name = getattr(gpd_core, "__name__", "gpd.core")
        return f"gpd import ok: {module_name} ({module_file})"


def gpd_worker_demo_blueprint(
    runtime: PythonProjectRuntimeEnvironment | None = None,
):
    environment = runtime or PythonProjectRuntimeEnvironment(
        name=GPD_DEMO_ENV_NAME,
        project=GPD_DEMO_PROJECT,
    )
    return autoconnect(GpdImportProbe.blueprint()).runtime_environments(
        environment
    ).runtime_placements({GpdImportProbe: environment.name})
