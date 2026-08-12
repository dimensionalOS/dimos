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

import json
from pathlib import Path

from pytest_mock import MockerFixture

from dimos.benchmark.evaluation.models import RuntimeIdentity
from dimos.benchmark.evaluation.protocol import EvaluationContext, LiveAgentOutcome
import dimos.benchmark.vlnce_r2r.evaluation as evaluation
from dimos.benchmark.vlnce_r2r.models import VlnceConfig, VlnceTaskManifest

CASE = Path(__file__).parent / "cases/mp3d-example-episode-515/task.json"


def test_live_agent_starts_after_runtime_barrier_and_native_score_survives_render_failure(
    mocker: MockerFixture,
    tmp_path: Path,
) -> None:
    events: list[str] = []
    manifest = VlnceTaskManifest.model_validate_json(CASE.read_bytes())

    class FakeExecution:
        def start(self) -> None:
            events.append("agent.start")

        def failure(self) -> BaseException | None:
            return None

        def finish(self) -> LiveAgentOutcome:
            events.append("agent.finish")
            return LiveAgentOutcome("done", 4, 2.0)

    class FakeLiveRuntime:
        prompt_evidence = ()
        runtime_artifacts = ()
        identity = RuntimeIdentity(
            profile="live-agent-v1",
            driver_version="test",
            model="gpt-5.6-luna",
            thinking_level="medium",
        )

        def prepare(self, **kwargs: object) -> FakeExecution:
            events.append("agent.prepare")
            assert kwargs["episode_timeout_s"] == 600.0
            return FakeExecution()

    class FakeBenchmarkRuntime:
        def __init__(self, **kwargs: object) -> None:
            events.append("runtime.create")
            self.attempt_path = Path(kwargs["attempt_path"])
            self.memory_path = self.attempt_path / "live-memory/recording.db"
            self.result_path = self.attempt_path / "terminal-private/vlnce-result.v1.json"
            self.render_path = self.attempt_path / "native-render.mp4"
            self.log_path = self.attempt_path / "oci-runtime.log"
            self._returned = False

        def start(self) -> dict[str, object]:
            events.append("runtime.start")
            self.memory_path.parent.mkdir(parents=True)
            self.memory_path.touch()
            self.result_path.parent.mkdir(parents=True)
            self.log_path.touch()
            return {"ready": True}

        def begin(self) -> None:
            events.append("runtime.begin")

        def healthy(self) -> bool:
            return True

        def result_bytes(self) -> bytes | None:
            if self._returned:
                return None
            self._returned = True
            events.append("runtime.result")
            payload = _native_result(manifest, "run-1")
            encoded = json.dumps(payload).encode()
            self.result_path.write_bytes(encoded)
            return encoded

        def public_evidence(self) -> dict[str, object]:
            return {"public": True}

        def cancel_motion(self) -> None:
            events.append("runtime.cancel_motion")

        def close(self) -> None:
            events.append("runtime.close")

        def render_evidence(self) -> dict[str, object]:
            return {"schema_version": "native-render.v1", "status": "failed"}

    mocker.patch.object(evaluation, "prepare_public_assets", return_value=mocker.sentinel.assets)
    mocker.patch.object(evaluation, "resolve_oci_image", return_value="sha256:image")
    mocker.patch.object(evaluation, "preparation_evidence", return_value={"prepared": True})
    mocker.patch.object(evaluation, "VlnceExternalRuntime", FakeBenchmarkRuntime)
    context = EvaluationContext(
        run_id="run-1",
        spec_dir=CASE.parent,
        workspace=tmp_path,
        runtime=FakeLiveRuntime(),
        progress=None,
    )

    report = evaluation.vlnce_r2r.run(VlnceConfig(task_manifest="task.json"), context)

    assert report.summary[2].value is True
    assert report.summary[-1].value == "failed"
    assert events == [
        "runtime.create",
        "runtime.start",
        "agent.prepare",
        "runtime.begin",
        "agent.start",
        "runtime.result",
        "agent.finish",
        "runtime.cancel_motion",
        "runtime.close",
    ]


def _native_result(manifest: VlnceTaskManifest, attempt_id: str) -> dict[str, object]:
    return {
        "schema_version": "vlnce-result.v1",
        "attempt_id": attempt_id,
        "case_id": manifest.case_id,
        "terminal_reason": "submitted",
        "duration_seconds": 12.0,
        "trajectory": {"sha256": "a" * 64, "points": 2},
        "metrics": {
            "DISTANCE_TO_GOAL": 0.5,
            "SUCCESS": 1.0,
            "SPL": 0.8,
            "NDTW": 0.9,
            "PATH_LENGTH": 4.0,
            "ORACLE_SUCCESS": 1.0,
            "STEPS_TAKEN": 2.0,
        },
        "runtime": {"habitat_sim": "0.1.7"},
    }
