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

"""RoboCasa smoke: ``data/robocasa/manifest.json`` → EvalCases.

    python -m dimos.evals.environments.lib.export_benchmark_scenes --robocasa PATH
    dimos evals run dimos.evals.suites.robocasa_smoke --agent dimos.evals.agents.pi
"""

from __future__ import annotations

from dimos.evals.environments.lib.benchmark_scenes import cases_from_manifest, load_manifest
from dimos.evals.types import Suite

SUITE: Suite = cases_from_manifest(load_manifest("robocasa"), extra_tags=("smoke",))
