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

import io
import json
from pathlib import Path
import subprocess
import sys
import types

import pytest

from dimos.deps.catalog import Plan
from dimos.deps.probe import ProbeError, ProbeRequest, check_backends, main, run_probe


def test_request_round_trip() -> None:
    plan = Plan(
        extras=frozenset({"web"}),
        backends={"onnxruntime": "cuda"},
        system=frozenset({"rclpy"}),
        tools=frozenset({"deno"}),
    )
    request = ProbeRequest.from_plan(
        plan, ("packages",), blueprints=("unitree-go2",), global_config={"staging": Path("/tmp/x")}
    )
    data = json.loads(json.dumps(request.to_json()))
    assert data["global_config"] == {"staging": "/tmp/x"} and data["system"] == ["rclpy"]
    assert ProbeRequest.from_json(data) == request
    assert request.plan().extras == {"web"} and request.plan().backends == {"onnxruntime": "cuda"}
    assert request.plan().system == {"rclpy"}


def test_main_rejects_bad_requests(
    monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture[str]
) -> None:
    monkeypatch.setattr(sys, "stdin", io.StringIO("not json"))
    assert main() == 2
    monkeypatch.setattr(sys, "stdin", io.StringIO(json.dumps({"schema": 7})))
    assert main() == 2
    assert "invalid probe request" in capsys.readouterr().err


def test_main_reports_cheap_checks(
    monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture[str]
) -> None:
    request = ProbeRequest(extras=(), tools=("python3",), checks=("tools",))
    monkeypatch.setattr(sys, "stdin", io.StringIO(json.dumps(request.to_json())))
    assert main() == 0
    report = json.loads(capsys.readouterr().out)
    assert report["tools"]["python3"]["status"] == "satisfied" and report["checks"] == ["tools"]


def test_run_probe_in_a_real_subprocess() -> None:
    request = ProbeRequest(extras=(), checks=("packages", "providers", "tools"), tools=("python3",))
    report = run_probe(Path(sys.executable), request)
    assert report.dimos_version is not None
    assert report.satisfied_for_launch, (report.missing, report.mismatched)
    assert report.prefix == sys.prefix


def test_run_probe_errors(monkeypatch: pytest.MonkeyPatch) -> None:
    with pytest.raises(ProbeError, match="could not run"):
        run_probe(Path("/nonexistent/python"), ProbeRequest(extras=()))

    def fake_run(*args: object, **kwargs: object) -> subprocess.CompletedProcess[str]:
        return subprocess.CompletedProcess(args=[], returncode=0, stdout="garbage")

    monkeypatch.setattr(subprocess, "run", fake_run)
    with pytest.raises(ProbeError, match="returned no report"):
        run_probe(Path(sys.executable), ProbeRequest(extras=()))

    def rejecting_run(*args: object, **kwargs: object) -> subprocess.CompletedProcess[str]:
        return subprocess.CompletedProcess(args=[], returncode=2, stdout="")

    monkeypatch.setattr(subprocess, "run", rejecting_run)
    with pytest.raises(ProbeError, match="older than this one"):
        run_probe(Path(sys.executable), ProbeRequest(extras=()))


def test_check_backends_with_fake_onnxruntime(monkeypatch: pytest.MonkeyPatch) -> None:
    fake = types.SimpleNamespace(get_available_providers=lambda: ["CPUExecutionProvider"])
    monkeypatch.setitem(sys.modules, "onnxruntime", fake)
    monkeypatch.setitem(sys.modules, "torch", None)
    results = check_backends({"onnxruntime": "cpu"})
    assert results["onnxruntime"].status == "satisfied"
    assert "CPUExecutionProvider" in str(results["onnxruntime"].detail)
    results = check_backends({"onnxruntime": "cuda"})
    assert results["onnxruntime"].status == "missing"
    assert check_backends({"other": "cpu"})["other"].status == "missing"
