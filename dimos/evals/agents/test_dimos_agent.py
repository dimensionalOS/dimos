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

from contextlib import nullcontext
from dataclasses import replace
import json
import os
from pathlib import Path
import subprocess

import pytest

from dimos.agents.llm_trace import request_path, response_path
from dimos.evals.active_learning import HANDPICK_MEMORY
from dimos.evals.agents import dimos_agent, pi
from dimos.evals.agents.dimos_agent import (
    USER_PROMPT_MAX_CHARS,
    DimosAgentAdapter,
    _path_with_user_local_bin,
    recording_user_prompt,
)
from dimos.evals.agents.lib.pi_config import RunPaths
from dimos.evals.environments.dataset import Dataset
from dimos.evals.types import RunningEnvironment, ToolCall
from dimos.memory.store.sqlite import SqliteStore
from dimos.perception.identity.dump import sidecar_tracks_jsonl


@pytest.mark.parametrize(
    "option,value",
    [
        ("allowed_tools", ("bash",)),
        ("no_dimos", True),
        ("skills", ("robot.md",)),
        ("excluded_keywords", ("dimos",)),
        ("max_tool_seconds", 1),
    ],
)
def test_unsupported_tool_restrictions_are_rejected(option, value):
    with pytest.raises(ValueError, match=f"does not support {option}"):
        DimosAgentAdapter(**{option: value})


@pytest.mark.parametrize("returncode", [0, 1])
def test_preflight_checks_the_agent_installation(mocker, monkeypatch, returncode):
    monkeypatch.setenv("OPENAI_API_KEY", "test-key")
    mocker.patch("dimos.evals.agents.dimos_agent.shutil.which", return_value="/test/bin/dimos")
    version = mocker.patch(
        "dimos.evals.agents.dimos_agent.subprocess.run",
        return_value=subprocess.CompletedProcess([], returncode, stderr="agent build failed"),
    )
    agent = DimosAgentAdapter()
    expected = (
        pytest.raises(RuntimeError, match="agent build failed") if returncode else nullcontext()
    )

    with expected:
        agent.preflight(Dataset("unused"))

    version.assert_called_once_with(
        ["/test/bin/dimos", "agent", "version"], capture_output=True, text=True
    )


@pytest.mark.parametrize("outcome", ["done", "unanswered"])
def test_run_records_tool_results_and_reports_unanswered_cases(
    mocker, monkeypatch, tmp_path, outcome
):
    monkeypatch.setenv("OPENAI_API_KEY", "test-key")
    mocker.patch.object(
        dimos_agent, "model_trace_proxy", return_value=nullcontext("http://trace.test")
    )
    mocker.patch.object(pi, "kill_run_processes")
    mocker.patch.object(dimos_agent.subprocess, "run")
    listing = mocker.patch.object(pi, "tool_listing")
    mocker.patch("dimos.evals.agents.dimos_agent.shutil.which", return_value="/test/bin/dimos")
    spawn = mocker.patch.object(pi.subprocess, "Popen")
    process = spawn.return_value.__enter__.return_value
    process.returncode = 0
    assistant = {
        "role": "assistant",
        "responseId": "tool-response",
        "content": [
            {"type": "toolCall", "id": "read-1", "name": "read", "arguments": {"path": "facts.txt"}}
        ],
    }
    records = [
        {"item": {"kind": "user"}},
        {"item": {"kind": "assistant", "message": assistant}},
        {
            "item": {
                "kind": "tool_result",
                "result": {"toolCallId": "read-1", "content": [{"type": "text", "text": "42"}]},
            }
        },
        {
            "item": {
                "kind": "assistant",
                "message": {
                    "role": "assistant",
                    "responseId": "answer-response",
                    "content": [{"type": "text", "text": "42"}],
                },
            }
        },
        {"outcome": {"status": outcome, "reason": "model unavailable"}},
    ]
    agent = DimosAgentAdapter(model="test-model", thinking="low", max_output_tokens=128)
    env = RunningEnvironment(mcp_url="http://robot.test:4321/mcp", streams=(), artifacts={})
    raw = tmp_path / "raw"
    raw.mkdir()
    for seq, response_id in enumerate(("tool-response", "answer-response")):
        request_path(raw, seq).write_text("{}")
        response_path(raw, seq).write_text(json.dumps({"body": {"id": response_id}}))
    read_fd, write_fd = os.pipe()
    with os.fdopen(read_fd, "rb") as incoming, os.fdopen(write_fd, "wb") as outgoing:
        process.stdout = incoming
        outgoing.write("\n".join(json.dumps(record) for record in records).encode())
        outgoing.close()

        result = agent.run("Read facts.txt", env, tmp_path, timeout_s=5)

    runtime = None
    try:
        assert result.final_answer == "42", result.extra
        assert result.extra.ended_by == ("answer" if outcome == "done" else "error")
        assert result.extra.error == (
            "" if outcome == "done" else "dimos agent run ended unanswered: model unavailable"
        )
        tool_step = result.steps[1]
        assert tool_step.tool_calls == (
            ToolCall(tool_call_id="read-1", function_name="read", arguments={"path": "facts.txt"}),
        )
        assert tool_step.observation.results[0].content == "42"
        listing.assert_not_called()

        command = spawn.call_args.args[0]
        assert command == [
            "/test/bin/dimos",
            "agent",
            "run",
            "--json",
            "--cwd",
            str(tmp_path),
            "Read facts.txt",
        ]
        process_env = spawn.call_args.kwargs["env"]
        assert {
            name: process_env[name]
            for name in ("MCPSERVER__HOST", "MCPSERVER__PORT", "DIMOS_EVAL_API_KEY")
        } == {
            "MCPSERVER__HOST": "robot.test",
            "MCPSERVER__PORT": "4321",
            "DIMOS_EVAL_API_KEY": "test-key",
        }
        assert process_env["XDG_CONFIG_HOME"] == str(agent._eval_home / "config")
        assert process_env["XDG_STATE_HOME"] == str(agent._eval_home / "state")
        assert process_env["XDG_CACHE_HOME"] == str(agent._eval_home / "cache")
        assert process_env["XDG_RUNTIME_DIR"] == str(agent._eval_home / "runtime")
        assert process_env["DIMOS_AGENT_SKIP_BUILD"] == "1"
        config = Path(process_env["XDG_CONFIG_HOME"]) / "dimos" / "agent"
        assert json.loads((config / "config.json").read_text()) == {
            "model": {"provider": "openai", "modelId": "test-model"},
            "thinkingLevel": "low",
        }
        assert json.loads((config / "models.json").read_text()) == {
            "providers": {
                "openai": {
                    "baseUrl": "http://trace.test",
                    "apiKey": "$DIMOS_EVAL_API_KEY",
                    "modelOverrides": {"test-model": {"maxTokens": 128}},
                }
            },
        }
        process.terminate.assert_called_once()
        runtime = Path(process_env["XDG_RUNTIME_DIR"])
        assert runtime.is_dir()
    finally:
        agent._shutdown_eval_runtime()
    if runtime is not None:
        assert not runtime.exists()


def test_cases_share_one_agent_runtime(tmp_path, monkeypatch, mocker):
    monkeypatch.setenv("OPENAI_API_KEY", "test-key")
    mocker.patch.object(dimos_agent.subprocess, "run")
    agent = DimosAgentAdapter()
    agent._mcp_env = {}
    try:
        first = agent._build_process_env(RunPaths.for_run(tmp_path / "case-a"))
        second = agent._build_process_env(RunPaths.for_run(tmp_path / "case-b"))
        assert first["XDG_STATE_HOME"] == second["XDG_STATE_HOME"]
        assert first["XDG_RUNTIME_DIR"] == second["XDG_RUNTIME_DIR"]
        assert first["DIMOS_AGENT_SKIP_BUILD"] == "1"
        assert Path(first["XDG_STATE_HOME"]).parent == agent._eval_home
        local_bin = Path.home() / ".local" / "bin"
        if local_bin.is_dir():
            assert first["PATH"].split(os.pathsep)[0] == str(local_bin)
    finally:
        agent._shutdown_eval_runtime()


def test_eval_path_puts_user_local_bin_first(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setattr(dimos_agent.Path, "home", classmethod(lambda cls: tmp_path))
    (tmp_path / ".local" / "bin").mkdir(parents=True)
    assert _path_with_user_local_bin("/usr/bin:/bin") == os.pathsep.join(
        (str(tmp_path / ".local" / "bin"), "/usr/bin", "/bin")
    )
    assert _path_with_user_local_bin(
        str(tmp_path / ".local" / "bin") + os.pathsep + "/usr/bin"
    ) == os.pathsep.join((str(tmp_path / ".local" / "bin"), "/usr/bin"))


def test_its_own_mcp_server_gives_the_agent_dimos_md_not_a_tool_list(tmp_path, mocker):
    listing = mocker.patch("dimos.evals.agents.pi.tool_listing")
    run_dir = tmp_path / "case"
    run_dir.mkdir()
    env = RunningEnvironment(
        mcp_url="http://127.0.0.1:9990/mcp", streams=(), artifacts={}, python_guide="# Python API"
    )
    prompt = DimosAgentAdapter()._prepare_case(env, run_dir)
    assert f"- robot: {run_dir / 'DIMOS.md'}" in prompt
    assert "dimos mcp call" not in prompt
    listing.assert_not_called()


def test_recording_cases_point_at_the_memory_tool_not_sqlite(tmp_path: Path) -> None:
    src = tmp_path / "src.db"
    run_dir = tmp_path / "case"
    run_dir.mkdir()
    with SqliteStore(path=str(src)) as store:
        store.stream("color_image", str).append("frame", ts=1.0)
        env = RunningEnvironment(mcp_url="", streams=(store.streams.color_image,), artifacts={})
        prompt = DimosAgentAdapter()._prepare_case(env, run_dir)
    assert "inspect" in prompt
    assert "SqliteStore" not in prompt
    assert (run_dir / "recording.db").is_file()
    assert "memory" in DimosAgentAdapter.default_tools
    assert not (run_dir / "captions.jsonl").exists()


def test_recording_cases_dump_daily_caption_jsonl(tmp_path: Path) -> None:
    src = tmp_path / "src.db"
    run_dir = tmp_path / "case"
    run_dir.mkdir()
    with SqliteStore(path=str(src)) as store:
        store.stream("color_image", str).append("frame", ts=1.0)
        store.stream("color_image_caption", str).append(
            '{"setting": "lot", "people_count": 0, "people": [], "objects_and_text": ["red car"]}',
            ts=1.0,
        )
        env = RunningEnvironment(
            mcp_url="",
            streams=(store.streams.color_image, store.streams.color_image_caption),
            artifacts={},
        )
        prompt = DimosAgentAdapter()._prepare_case(env, run_dir)
    assert "captions.jsonl" in prompt
    assert "inspect" in prompt
    assert "tracks.jsonl" in prompt
    assert "Index first" in prompt
    assert "Must inspect (≥1)" in prompt
    assert "Do not find under $DIMOS_ROOT" in prompt
    assert "at most two inspect calls" in prompt
    assert "new episode" in prompt
    assert "distinct track ids" in prompt
    assert "If choices are timestamps" in prompt
    assert "Before the final answer" in prompt
    assert "why the rival choice loses" in prompt
    assert "multiple choice" in prompt
    assert "do not invent options" in prompt
    assert "color frame only" in prompt
    assert "Pass ask" in prompt
    assert "walkable path" not in prompt
    assert not (run_dir / "active-learning.md").exists()
    assert (run_dir / "captions.jsonl").is_file()
    assert (run_dir / "captions" / "manifest.jsonl").is_file()
    assert "SqliteStore" not in prompt
    user = recording_user_prompt("How many cars?", RunPaths.for_run(run_dir))
    assert "Captions index:" in user
    assert "Must inspect (≥1)" in user
    assert len(user) <= USER_PROMPT_MAX_CHARS


def test_active_learning_prompt_is_opt_in(tmp_path: Path) -> None:
    src = tmp_path / "src.db"
    run_dir = tmp_path / "case"
    run_dir.mkdir()
    with SqliteStore(path=str(src)) as store:
        store.stream("color_image", str).append("frame", ts=1.0)
        store.stream("color_image_caption", str).append(
            '{"setting": "lot", "people_count": 0, "people": [], "objects_and_text": ["red car"]}',
            ts=1.0,
        )
        env = RunningEnvironment(
            mcp_url="",
            streams=(store.streams.color_image, store.streams.color_image_caption),
            artifacts={},
        )
        adapter = DimosAgentAdapter(active_learning="handpick-memory")
        prompt = adapter._prepare_case(env, run_dir)
    notes = (run_dir / "active-learning.md").read_text(encoding="utf-8")
    assert "walkable path" not in prompt
    assert "the store is open" in notes
    assert "fences" in notes
    assert "roller shutter" in notes
    assert notes == HANDPICK_MEMORY.strip() + "\n"
    paths = RunPaths.for_run(run_dir)
    command = adapter._build_pi_command("Is Zara open?", prompt, paths)
    user = command[-1]
    al = (run_dir / "active-learning.md").resolve()
    captions = (run_dir / "captions.jsonl").resolve()
    assert f"Active-learning notes: {al}" in user
    assert f"Captions index: {captions}" in user
    assert "Read that file before answering." in user
    assert "Do not find under $DIMOS_ROOT" in user
    assert "Must inspect (≥1)" in user
    assert "at most two inspect calls" in user
    assert "why the rival choice loses" in user
    assert "walkable path" not in user
    assert "the store is open" not in user
    assert len(user) <= USER_PROMPT_MAX_CHARS
    assert user == recording_user_prompt("Is Zara open?", paths)


def test_recording_cases_copy_sidecar_tracks(tmp_path: Path) -> None:
    src = tmp_path / "foo.db"
    run_dir = tmp_path / "case"
    run_dir.mkdir()
    sidecar = sidecar_tracks_jsonl(src)
    sidecar.parent.mkdir(parents=True)
    sidecar.write_text(
        '{"t":0.0,"tracks":[{"id":7,"name":"person","xyxy":[1,2,3,4]}]}\n',
        encoding="utf-8",
    )
    with SqliteStore(path=str(src)) as store:
        store.stream("color_image", str).append("frame", ts=1.0)
        env = RunningEnvironment(
            mcp_url="",
            streams=(store.streams.color_image,),
            artifacts={"recording": str(src)},
        )
        DimosAgentAdapter()._prepare_case(env, run_dir)
    copied = json.loads((run_dir / "tracks.jsonl").read_text(encoding="utf-8").splitlines()[0])
    assert copied["tracks"][0]["id"] == 7


def test_dataset_eval_copies_sidecar_tracks_from_the_source_recording(tmp_path: Path) -> None:
    src = tmp_path / "foo.db"
    run_dir = tmp_path / "case"
    run_dir.mkdir()
    sidecar = sidecar_tracks_jsonl(src)
    sidecar.parent.mkdir(parents=True)
    sidecar.write_text(
        '{"t":1.0,"tracks":[{"id":3,"name":"car","xyxy":[0,0,1,1]}]}\n',
        encoding="utf-8",
    )
    with SqliteStore(path=str(src)) as store:
        store.stream("color_image", str).append("frame", ts=1.0)
    environment = Dataset(str(src), select=(lambda s: s.streams.color_image,))
    running = environment.start(())
    try:
        agent_env = replace(
            running,
            artifacts={
                name: running.artifacts[name] for name in environment.config.agent_artifacts
            },
        )
        DimosAgentAdapter()._prepare_case(agent_env, run_dir)
    finally:
        environment.stop()
    copied = json.loads((run_dir / "tracks.jsonl").read_text(encoding="utf-8").splitlines()[0])
    assert copied["tracks"][0]["id"] == 3
