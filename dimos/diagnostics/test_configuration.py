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

import os

import pytest

from dimos.core.coordination.python_worker import MethodCallProxy, PythonWorker
from dimos.core.core import rpc
from dimos.core.global_config import GlobalConfig
from dimos.core.module import Module
from dimos.diagnostics import runtime
from dimos.diagnostics.analysis import load_bundle
from dimos.diagnostics.runtime import get_runtime, publication, span


class DiagnosticWorker(Module):
    @rpc
    def measure(self):
        with publication("test/input"), span("test.compute"):
            runtime = get_runtime()
            return {
                "run": runtime.run_id,
                "mode": runtime.mode,
                "child_environment": os.environ["DIMOS_DIAGNOSTICS_RUN"],
            }


def test_resolved_run_config_reaches_spawned_worker_and_native_child_environment(tmp_path):
    worker = PythonWorker()
    worker.start_process()
    try:
        config = GlobalConfig(
            diagnostics="trace", diagnostics_run="distributed-test", diagnostics_dir=str(tmp_path)
        )
        actor = worker.deploy_module(DiagnosticWorker, config)
        assert MethodCallProxy(actor).measure().result() == {
            "run": "distributed-test",
            "mode": "trace",
            "child_environment": "distributed-test",
        }
    finally:
        worker.shutdown()
    manifests, events = load_bundle(tmp_path / "distributed-test")
    assert manifests
    assert any(event["event"] == "test.compute.end" for event in events)


@pytest.mark.parametrize("configured", ["", "relative-capture"])
def test_capture_root_is_resolved_on_the_worker_and_inherited_by_native_children(
    tmp_path, monkeypatch, configured
):
    monkeypatch.chdir(tmp_path)
    monkeypatch.setattr(runtime, "STATE_DIR", tmp_path / "host-state")
    monkeypatch.setattr(runtime, "_runtime", None)
    monkeypatch.setattr(runtime, "_mode", "off")
    monkeypatch.setattr(runtime, "_failed_pid", None)
    for key in ("DIMOS_DIAGNOSTICS", "DIMOS_DIAGNOSTICS_RUN", "DIMOS_DIAGNOSTICS_DIR"):
        monkeypatch.setenv(key, os.environ.get(key, ""))
    runtime.configure("off", "", configured)
    expected = tmp_path / configured if configured else tmp_path / "host-state" / "diagnostics"
    assert os.environ["DIMOS_DIAGNOSTICS_DIR"] == str(expected)
    assert GlobalConfig().diagnostics_dir == str(expected)  # inherited native/child environment


def test_documented_diagnostic_environment_survives_config_resolution(monkeypatch, tmp_path):
    monkeypatch.setenv("DIMOS_DIAGNOSTICS", "trace")
    monkeypatch.setenv("DIMOS_DIAGNOSTICS_RUN", "environment-run")
    monkeypatch.setenv("DIMOS_DIAGNOSTICS_DIR", str(tmp_path))
    config = GlobalConfig()
    assert (config.diagnostics, config.diagnostics_run, config.diagnostics_dir) == (
        "trace",
        "environment-run",
        str(tmp_path),
    )
    assert GlobalConfig(diagnostics="off").diagnostics == "off"
