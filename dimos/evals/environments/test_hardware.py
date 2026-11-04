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

import signal
import subprocess
from unittest.mock import MagicMock

import pytest

from dimos.e2e_tests.dimos_cli_call import DimosCliCall
from dimos.evals.environments.hardware import HardwareEnvironment
from dimos.evals.environments.live import LiveEnvironment
from dimos.evals.scorers import human


def environment(**kwargs):
    return HardwareEnvironment(
        blueprint=["xarm-grasp", "mcp-server"], task="pick the cup", **kwargs
    )


def test_robot_does_not_start_until_the_operator_confirms(mocker):
    prompts = []
    mocker.patch("builtins.input", side_effect=lambda text: prompts.append(text) or "n")
    launch = mocker.patch.object(LiveEnvironment, "start")

    with pytest.raises(RuntimeError, match="did not confirm"):
        environment().start(())

    assert "pick the cup" in prompts[0]
    launch.assert_not_called()


def test_stop_halts_the_robot_before_releasing_it(mocker):
    calls = []
    adapter = MagicMock()
    adapter.call_tool.side_effect = lambda name: calls.append(("halt", name)) or {"content": []}
    mocker.patch("dimos.evals.environments.hardware.McpAdapter", return_value=adapter)
    mocker.patch.object(LiveEnvironment, "stop", side_effect=lambda: calls.append(("release",)))
    env = environment(stop_tool="cancel")
    env._mcp_url = "http://localhost:9990/mcp"

    env.stop()

    assert calls == [("halt", "cancel"), ("release",)]


def test_a_failed_halt_warns_the_operator_and_still_stops_dimos(mocker):
    adapter = MagicMock()
    adapter.call_tool.return_value = {"isError": True, "content": [{"text": "not confirmed"}]}
    mocker.patch("dimos.evals.environments.hardware.McpAdapter", return_value=adapter)
    release = mocker.patch.object(LiveEnvironment, "stop")
    error = mocker.patch("dimos.evals.environments.hardware.logger.error")
    env = environment(stop_tool="cancel")
    env._mcp_url = "http://localhost:9990/mcp"

    env.stop()

    assert "HALT FAILED" in error.call_args.args[0]
    release.assert_called_once()


def test_the_real_robot_is_launched_without_simulation():
    proc = DimosCliCall()

    environment(module_env={"A__B": "1"}).configure_launch(proc)

    assert proc.simulator is None
    assert proc.extra_env["A__B"] == "1"


def test_hardware_needs_an_operator_terminal(mocker):
    mocker.patch("sys.stdin.isatty", return_value=False)
    with pytest.raises(RuntimeError, match="interactive terminal"):
        environment().preflight(MagicMock())


@pytest.mark.parametrize("stops_in_time", [True, False])
def test_dimos_is_stopped_like_dimos_stop_then_killed_if_it_hangs(mocker, stops_in_time):
    killpg = mocker.patch("dimos.evals.environments.live.os.killpg")
    proc = MagicMock()
    process = proc.process
    process.poll.return_value = None
    if not stops_in_time:
        process.wait.side_effect = [subprocess.TimeoutExpired("dimos", 30.0), 0]

    environment()._stop_process(proc)

    process.send_signal.assert_called_once_with(signal.SIGTERM)
    if stops_in_time:
        killpg.assert_not_called()
    else:
        killpg.assert_called_once_with(process.pid, signal.SIGKILL)
    assert proc.process is None  # reaped; DimosCliCall.stop has no group left to signal
    proc.stop.assert_called_once()


@pytest.mark.parametrize(("answer", "score"), [("y", 1.0), ("n", 0.0), ("", 0.0)])
def test_human_grade_is_the_operators_verdict(mocker, answer, score):
    mocker.patch("builtins.input", return_value=answer)
    outcome = MagicMock()
    outcome.trajectory.final_answer = "Holding the cup."

    assert human("pick the cup")(outcome) == score
