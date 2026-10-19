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

from pathlib import Path

import pytest

from dimos.evals.sandbox import runner
from dimos.evals.sandbox.case import DimosRobot
from dimos.evals.sandbox.go2_home import SUITE
from dimos.evals.sandbox.runner import SandboxAgent, SandboxRunner, namespaced


def test_hidden_dimos_topics_go_under_the_namespace_and_exposed_ones_dont() -> None:
    hidden = SUITE[0].robot
    assert isinstance(hidden, DimosRobot) and not hidden.expose_dimos
    assert namespaced(hidden, "evals/run-1").env["ZENOH_NAMESPACE"] == "evals/run-1"  # type: ignore[union-attr]
    assert namespaced(hidden, "") == hidden
    exposed = DimosRobot(args=("run", "unitree-go2"))  # the agent is told these topics' keys
    assert namespaced(exposed, "evals/run-1") == exposed


def test_a_robot_recorded_on_demand_runs_without_record(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setattr(runner, "ROBOT_CACHE", tmp_path / "cache")
    on_demand = namespaced(SUITE[0].robot, "evals/run-1")
    service = SandboxRunner(SandboxAgent())._compose(on_demand, tmp_path)["services"]["robot"]
    assert "--record" not in service["command"]
    assert service["environment"]["ZENOH_NAMESPACE"] == "evals/run-1"
    whole_run = DimosRobot(args=("run", "unitree-go2"))
    service = SandboxRunner(SandboxAgent())._compose(whole_run, tmp_path)["services"]["robot"]
    assert service["command"][-4:] == ["--record", "sqlite", "run", "unitree-go2"]
