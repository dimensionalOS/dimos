# Copyright 2025-2026 Dimensional Inc.
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

"""The public capture command must produce inspectable evidence from its own child."""

import json
import sys

from typer.testing import CliRunner

from dimos.cli.commands.diagnostics import app
from dimos.diagnostics.analysis import load_bundle


def test_capture_child_flushes_and_inspection_can_read_the_result(tmp_path):
    runner = CliRunner()
    script = "from dimos.diagnostics.runtime import publication, span\nwith publication('hand'), span('compute'):\n    pass\n"
    captured = runner.invoke(
        app,
        [
            "capture",
            "--directory",
            str(tmp_path),
            "--seconds",
            "10",
            "--run-id",
            "smoke",
            "--",
            sys.executable,
            "-c",
            script,
        ],
    )
    assert captured.exit_code == 0, captured.output
    assert json.loads(captured.output)["exit_code"] == 0
    manifests, events = load_bundle(tmp_path / "smoke")
    assert len(manifests) == 1
    assert [e["event"] for e in events] == ["sample.created", "compute.start", "compute.end"]
    inspected = runner.invoke(app, ["inspect", str(tmp_path / "smoke")])
    assert inspected.exit_code == 0, inspected.output
    assert json.loads(inspected.output)["stages"]["compute"]["n"] == 1


def test_capture_rejects_invalid_run_before_executing_anything(tmp_path):
    result = CliRunner().invoke(
        app,
        [
            "capture",
            "--directory",
            str(tmp_path),
            "--run-id",
            "../escape",
            "--",
            sys.executable,
            "-c",
            "raise AssertionError('must not run')",
        ],
    )
    assert result.exit_code != 0
    assert list(tmp_path.iterdir()) == []


def test_multiprocessing_worker_flushes_before_its_normal_exit(tmp_path):
    script = tmp_path / "worker.py"
    script.write_text(
        "from multiprocessing import get_context\nfrom dimos.diagnostics.runtime import span\ndef work():\n    with span('worker'):\n        pass\nif __name__ == '__main__':\n    worker = get_context('spawn').Process(target=work)\n    worker.start()\n    worker.join()\n    assert worker.exitcode == 0\n"
    )
    captures = tmp_path / "captures"
    result = CliRunner().invoke(
        app,
        [
            "capture",
            "--directory",
            str(captures),
            "--run-id",
            "workers",
            "--seconds",
            "10",
            "--",
            sys.executable,
            str(script),
        ],
    )
    assert result.exit_code == 0, result.output
    assert json.loads(result.output)["exit_code"] == 0
    manifests, events = load_bundle(captures / "workers")
    assert len(manifests) == 1
    assert [event["event"] for event in events] == ["worker.start", "worker.end"]
