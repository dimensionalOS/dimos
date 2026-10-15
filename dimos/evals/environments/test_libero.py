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
from typing import Any

import pytest

from dimos.e2e_tests.dimos_cli_call import DimosCliCall
from dimos.evals.environments import libero
from dimos.evals.environments.libero import LiberoEnvironment, libero_success


def _status(*holds: bool) -> dict[str, Any]:
    return {"success": all(holds), "predicates": [["on", "a", "b", h] for h in holds]}


@pytest.mark.parametrize(
    ("statuses", "score"),
    [
        ([], 0.0),
        ([_status(False, False), _status(True, False)], 0.5),
        # LIBERO ends an episode on first success, so a later regression still passes.
        ([_status(False), _status(True), _status(False)], 1.0),
    ],
)
def test_libero_success(
    monkeypatch: pytest.MonkeyPatch, statuses: list[dict[str, Any]], score: float
) -> None:
    monkeypatch.setattr(libero, "task_statuses", lambda outcome: statuses)
    assert libero_success(None) == score  # type: ignore[arg-type]


def test_launch_selects_the_task_and_records_its_status(tmp_path: Path) -> None:
    bddl = tmp_path / "task.bddl"
    env = LiberoEnvironment(bddl=bddl, seed=3)
    proc = DimosCliCall()
    env.configure_launch(proc)
    assert proc.simulator is None
    assert proc.extra_env["LIBEROSIM__BDDL"] == str(bddl.resolve())
    assert proc.extra_env["LIBEROSIM__SEED"] == "3"
    assert "task_status" in proc.global_args[proc.global_args.index("--record-topics") + 1]
    assert env.config.blueprint[0] == "panda-libero-sim"
