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

"""The M20 patrolling the premap's named locations and docking from each in turn, driven by the Orchestrator."""

from __future__ import annotations

from pathlib import Path

from dimos.core.coordination.blueprints import autoconnect
from dimos.orchestration.orchestrator import Orchestrator
from dimos.robot.deeprobotics.m20.blueprints.m20 import m20
from dimos.visualization.rerun.bridge import RerunBridgeModule
from dimos.visualization.rerun.websocket_server import RerunWebSocketServer

# Watched through the Controller app, so no Rerun.
m20_patrol = autoconnect(
    m20,
    Orchestrator.blueprint(file=Path(__file__).parent / "m20_patrol.toml"),
).disabled_modules(RerunBridgeModule, RerunWebSocketServer)
