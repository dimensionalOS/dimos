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

"""The simulated Go2 walking a route of waypoints, driven by the Orchestrator."""

from __future__ import annotations

from pathlib import Path

from dimos.core.coordination.blueprints import autoconnect
from dimos.orchestration.orchestrator import Orchestrator
from dimos.robot.unitree.go2.blueprints.navigation.go2_sim_nav import go2_sim_nav

# One worker per module.
go2_sim_patrol = autoconnect(
    go2_sim_nav,
    Orchestrator.blueprint(file=Path(__file__).parent / "go2_sim_patrol.toml"),
).global_config(n_workers=10)
