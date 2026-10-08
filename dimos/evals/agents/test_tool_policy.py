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

"""The same provider and tool contract across native Pi and dimcode runtimes."""

from dataclasses import replace
import json
import os
from pathlib import Path
import sys
from urllib.parse import urlsplit

import pytest

from dimos.evals.agents.conftest import NativeHarness, ScriptedProvider
from dimos.evals.agents.lib.pi_config import RunPaths


def test_allowed_tools_execute_and_excluded_tools_do_not(
    harness: NativeHarness, provider: ScriptedProvider
) -> None:
    provider.call("write", path=harness.path("forbidden.txt"), content="must not execute")
    provider.call("bash", command=f"printf selected-observation > {harness.path('facts.txt')}")
    provider.call("grep", pattern="selected-observation", path=harness.path("facts.txt"), context=1)
    result = harness.run(harness.agent(provider, ("bash", "grep")))
    assert result.extra.ended_by == "answer", result.extra
    assert result.final_answer == "OK"
    assert len(provider.requests) == 4
    endpoint, output_cap = {
        "openai": ("/responses", "max_output_tokens"),
        "anthropic": ("/v1/messages", "max_tokens"),
    }[provider.name]
    assert {urlsplit(route).path for route in provider.routes} == {endpoint}
    for request in provider.requests:
        assert request["model"] == provider.model
        assert request[output_cap] == 1024
        tools = request["tools"]
        assert isinstance(tools, list)
        names = set()
        for tool in tools:
            assert isinstance(tool, dict)
            names.add(str(tool["name"]))
        assert names == {"bash", "grep"}
    assert (harness.workspace / "facts.txt").read_text() == "selected-observation"
    assert not (harness.workspace / "forbidden.txt").exists()
    observation = result.steps[-2].observation
    assert observation is not None
    output = observation.results[0].content
    assert "selected-observation" in output
    assert json.dumps(output) in json.dumps(provider.requests[-1])
    assert result.final_metrics.total_prompt_tokens == 40
    assert result.final_metrics.total_completion_tokens == 20
    assert result.final_metrics.total_cost_usd is not None


def test_no_tools_blocks_even_a_provider_requested_call(
    harness: NativeHarness, provider: ScriptedProvider
) -> None:
    provider.call("write", path=harness.path("forbidden.txt"), content="must not execute")
    result = harness.run(harness.agent(provider, ()))
    assert result.extra.ended_by == "answer", result.extra
    assert all(request.get("tools", []) == [] for request in provider.requests)
    assert not (harness.workspace / "forbidden.txt").exists()


@pytest.mark.parametrize("harness", ["dimcode"], indirect=True)
def test_unknown_tool_fails_before_inference(
    harness: NativeHarness, provider: ScriptedProvider
) -> None:
    result = harness.run(harness.agent(provider, ("unknown_tool",)))
    assert result.extra.ended_by == "error"
    assert "unknown_tool" in result.extra.error
    assert provider.requests == []


def test_missing_runtime_extension_fails_before_inference(
    harness: NativeHarness,
    provider: ScriptedProvider,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    agent = harness.agent(provider, ("bash",))
    build = agent._build_pi_command
    # A previous run's marker must not let a missing extension bypass startup checks.
    ready = RunPaths.for_run(harness.root).config / "extensions/runtime-ready.json"
    ready.parent.mkdir(parents=True)
    ready.write_text('{"tools":["bash"],"unknown":[]}')

    def broken(inputs: str, prompt: str, paths: RunPaths) -> list[str]:
        command = build(inputs, prompt, paths)
        (paths.config / "extensions/runtime.js").unlink()
        return command

    monkeypatch.setattr(agent, "_build_pi_command", broken)
    result = harness.run(agent)
    assert result.extra.ended_by == "error"
    assert provider.requests == []


def test_excluded_keywords_deny_only_whole_word_matches(
    harness: NativeHarness, provider: ScriptedProvider
) -> None:
    provider.call("bash", command="python3 -c 'import dimos'")  # code mentioning it
    provider.call("read", path="/opt/DimOS/README.md")  # a path into it, any case
    provider.call("bash", command=f"echo dimosaurus > {harness.path('ok.txt')}")  # not a whole word
    provider.call("bash", command=f"cat {harness.path('ok.txt')}")  # the run's own path is exempt
    result = harness.run(harness.agent(provider, None, excluded_keywords=("dimos",)))
    assert result.extra.ended_by == "answer", result.extra
    assert result.final_answer == "OK"
    assert len(provider.requests) == 5
    outcomes = [
        (call.function_name, obs.results[0].content)
        for step in result.steps
        if step.tool_calls and (obs := step.observation)
        for call in step.tool_calls
    ]
    assert [name for name, _ in outcomes] == ["bash", "read", "bash", "bash"]
    denied = [text for _, text in outcomes[:2]]
    assert all("Tool call denied" in text and '"dimos"' in text for text in denied)
    assert not any("Tool call denied" in text for _, text in outcomes[2:])
    assert (harness.workspace / "ok.txt").read_text().strip() == "dimosaurus"
    assert "dimosaurus" in outcomes[3][1]
    assert result.extra.blocked_calls == 2


@pytest.mark.parametrize("provider", ["openai"], indirect=True)
def test_excluded_keyword_in_workspace_path_is_not_a_hit(
    harness: NativeHarness, provider: ScriptedProvider, tmp_path: Path
) -> None:
    root = tmp_path / "dimos" / "run"
    root.mkdir(parents=True)
    harness = replace(harness, root=root)
    provider.call("bash", command=f"printf inside > {harness.path('note.txt')}")
    result = harness.run(harness.agent(provider, None, excluded_keywords=("dimos",)))
    assert result.extra.ended_by == "answer", result.extra
    assert (root / "note.txt").read_text() == "inside"
    assert result.extra.blocked_calls == 0


@pytest.mark.parametrize("harness", ["pi"], indirect=True)
@pytest.mark.parametrize("provider", ["openai"], indirect=True)
def test_no_dimos_strips_dimos_from_the_environment(
    harness: NativeHarness,
    provider: ScriptedProvider,
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    # The probe needs a working python3 without dimOS. The one on the user's PATH may be a
    # version-manager shim that fails under the fixture's temporary HOME, so use the base
    # interpreter the venv was built from.
    bin_dir = tmp_path / "bin"
    bin_dir.mkdir()
    (bin_dir / "python3").symlink_to(Path(sys.base_prefix) / "bin" / "python3")
    monkeypatch.setenv("PATH", f"{bin_dir}{os.pathsep}{os.environ['PATH']}")
    # Spelling the name in two halves gets past the keyword guard on purpose: the
    # process environment must not have dimOS even when the guard is circumvented.
    probe = "python3 -c \"import importlib.util as u; print(u.find_spec('di'+'mos'))\"; command -v di''mos || echo no-cli"
    provider.call("bash", command=probe)
    provider.call("bash", command="echo $PATH")
    result = harness.run(harness.agent(provider, None, no_dimos=True))
    assert result.extra.ended_by == "answer", result.extra
    outputs = [
        obs.results[0].content
        for step in result.steps
        if step.tool_calls and (obs := step.observation)
    ]
    assert outputs[0].split() == ["None", "no-cli"]
    assert ".venv" not in outputs[1]
    assert result.extra.blocked_calls == 0
    prompt = (harness.root / "system-prompt.txt").read_text()
    assert "dimensionalOS" in prompt and "observations" in prompt
    assert not (harness.root / "recording.db").exists()


@pytest.mark.parametrize("provider", ["openai"], indirect=True)
def test_max_tool_seconds_clamps_bash(harness: NativeHarness, provider: ScriptedProvider) -> None:
    provider.call("bash", command="sleep 30; echo finished", timeout=600)
    result = harness.run(harness.agent(provider, None, max_tool_seconds=1))
    assert result.extra.ended_by == "answer", result.extra
    output = next(
        obs.results[0].content
        for step in result.steps
        if step.tool_calls and (obs := step.observation)
    )
    assert "finished" not in output
    assert "timed out" in output.lower() or "timeout" in output.lower()
