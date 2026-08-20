"""One complete built-in LIBERO-PRO code-policy Evaluation."""

from __future__ import annotations

from datetime import datetime, timezone
import json
import os
from pathlib import Path
import time
from typing import Literal, TypedDict

from pydantic import BaseModel

from dimos.benchmark.evaluation.models import (
    ArtifactReference,
    EvaluationReport,
    InlineNativeResult,
    SummaryItem,
)
from dimos.benchmark.evaluation.progress import StatusProgress, emit_progress
from dimos.benchmark.evaluation.protocol import (
    CodePolicyRuntime,
    EvaluationContext,
    PolicyArtifact,
    TrialOutcome,
    TrialRun,
)
from dimos.benchmark.libero_pro.assets import PreparedAssets, prepare_assets
from dimos.benchmark.libero_pro.blueprint import libero_trial_blueprint
from dimos.benchmark.libero_pro.control import EvaluationControlClient
from dimos.benchmark.libero_pro.models import LiberoProConfig, LiberoTaskManifest
from dimos.benchmark.libero_pro.podman import LiberoPodmanContainer
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.core.coordination.process_lifecycle import DIMOS_RUN_ID_ENV

EVALUATION_PROTOCOL = """Develop a task-specific synchronous policy for the exact task input.

Define `policy(app: Dimos) -> None`. Use discoverable typed DimOS modules and
read-only `app.memory`; no evaluator-only state is available. Call
`submit_policy(policy)` for up to five fresh Debug Trials. It returns an immutable
PolicyCandidate with a bounded Trial Evidence summary. Drill down only as needed via
`candidate.evidence.timeline()`, `.logs()`, `.frame()`, `.policy_output`,
`.open_memory()`, and `.artifacts`. Recorded facts may be unavailable; do not infer
missing causes.

Before finishing, call `freeze_policy(candidate)` exactly once. The frozen candidate
is evaluated in a clean policy-only process. Freezing is irreversible and later
submissions are rejected.
"""


class LiberoProEvaluation:
    name = "libero-pro"
    runtime_profile = "code-policy-v1"
    config_model: type[BaseModel] = LiberoProConfig

    def run(self, config: BaseModel, context: EvaluationContext) -> EvaluationReport:
        if not isinstance(config, LiberoProConfig):
            raise TypeError("libero-pro received the wrong configuration type")
        if not isinstance(context.runtime, CodePolicyRuntime):
            raise TypeError("libero-pro requires the code-policy-v1 runtime")
        runtime = context.runtime
        manifest_path = Path(config.task_manifest)
        if not manifest_path.is_absolute():
            manifest_path = context.spec_dir / manifest_path
        manifest = LiberoTaskManifest.model_validate_json(manifest_path.read_bytes())
        emit_progress(
            context.progress,
            StatusProgress(channel="eval", message="LIBERO-PRO preflight"),
        )
        assets = prepare_assets(manifest)
        LiberoPodmanContainer.ensure_image()

        def submit(policy: PolicyArtifact, number: int, path: Path) -> TrialRun:
            init_index = manifest.episodes.debug_init_state_indices[number - 1]
            return _run_trial(
                manifest,
                assets,
                policy,
                init_index=init_index,
                path=path,
                runtime=runtime,
                run_id=f"{context.run_id}-debug-{number}",
            )[0]

        exploration = runtime.explore(
            evaluation_protocol=EVALUATION_PROTOCOL,
            task_input=manifest.task.instruction,
            submit_debug_trial=submit,
        )
        if exploration.policy is None:
            raise RuntimeError(exploration.error or "exploration produced no policy")
        scored_path = context.workspace / "scored-trial"
        trial, native = _run_trial(
            manifest,
            assets,
            exploration.policy,
            init_index=manifest.episodes.scored_init_state_index,
            path=scored_path,
            runtime=runtime,
            run_id=f"{context.run_id}-scored",
        )
        if trial.outcome.status != "completed":
            raise RuntimeError(trial.outcome.error or "scored LIBERO-PRO trial failed")
        native_value = {
            "result_type": "single_trial",
            "case_id": manifest.case_id,
            "suite": manifest.task.suite,
            "task_name": manifest.task.task_name,
            "instruction": manifest.task.instruction,
            "init_state_index": manifest.episodes.scored_init_state_index,
            "success": native["success"],
            "score": native["score"],
            "reward": native["reward"],
            "terminal_reason": native["terminal_reason"],
            "policy_ticks": native["policy_ticks"],
            "backend_ticks": native["backend_ticks"],
            "debug_submissions": len(exploration.trials),
            "policy_sha256": exploration.policy.sha256,
            "policy_execution_status": native["policy_execution_status"],
            "source_revision": manifest.source.revision,
            "dataset_revision": manifest.source.dataset_revision,
            "deviations": ["continuous_real_time", "joint_target_to_native_osc_adapter"],
        }
        (scored_path / "score.json").write_text(
            json.dumps(native_value, indent=2, sort_keys=True) + "\n"
        )
        return EvaluationReport(
            summary=(
                SummaryItem(key="result_type", label="Result type", value="single_trial"),
                SummaryItem(key="suite", label="Suite", value=manifest.task.suite),
                SummaryItem(key="task", label="Task", value=manifest.task.instruction),
                SummaryItem(key="success", label="Native success", value=trial.outcome.success),
                SummaryItem(key="score", label="Native score", value=native["score"]),
                SummaryItem(
                    key="terminal_reason",
                    label="Terminal reason",
                    value=native["terminal_reason"],
                ),
                SummaryItem(key="policy_ticks", label="Policy ticks", value=native["policy_ticks"]),
                SummaryItem(
                    key="debug_trials", label="Debug trials", value=len(exploration.trials)
                ),
            ),
            native_result=InlineNativeResult(value=native_value),
            artifacts=(
                _artifact("scored-trial/task.json", "Task manifest", "application/json"),
                _artifact("scored-trial/score.json", "Native score", "application/json"),
                _artifact("scored-trial/trial.jsonl", "Trial log", "application/x-ndjson"),
                _artifact("scored-trial/container.log", "Container log", "text/plain"),
                _artifact("scored-trial/policy-output.log", "Policy output", "text/plain"),
                _artifact("scored-trial/trial.mp4", "Rendered trial", "video/mp4"),
                _artifact(
                    "scored-trial/recording.db", "Memory2 recording", "application/x-sqlite3"
                ),
            ),
        )


class NativeTrialResult(TypedDict):
    success: bool
    score: float
    reward: float
    terminal_reason: str
    policy_ticks: int
    backend_ticks: int
    policy_execution_status: str


def _run_trial(
    manifest: LiberoTaskManifest,
    assets: PreparedAssets,
    policy: PolicyArtifact,
    *,
    init_index: int,
    path: Path,
    runtime: CodePolicyRuntime,
    run_id: str,
) -> tuple[TrialRun, NativeTrialResult]:
    path.mkdir(parents=True, exist_ok=True)
    log_path = path / "trial.jsonl"
    memory_path = path / "recording.db"
    video_path = path / "trial.mp4"
    policy_output_path = path / "policy-output.log"
    discovery = str(path / "panda-shm")
    container = LiberoPodmanContainer(manifest, assets, artifact_dir=path)
    control = None
    coordinator = None
    execution = None
    started = time.monotonic()
    policy_output = ""
    native: NativeTrialResult
    previous_run_id = os.environ.get(DIMOS_RUN_ID_ENV)
    try:
        endpoints = container.start()
        _log(log_path, "container_started", run_id=run_id)
        control = EvaluationControlClient(endpoints.control, endpoints.control_token)
        control.wait_ready()
        control.initialize(manifest, init_index)
        blueprint = libero_trial_blueprint(
            policy_endpoint=endpoints.policy,
            discovery_address=discovery,
            memory_path=memory_path,
            video_path=video_path,
        )
        os.environ[DIMOS_RUN_ID_ENV] = run_id
        coordinator = ModuleCoordinator.build(blueprint)
        coordinator.start_rpc_service()
        execution = runtime.prepare(
            policy,
            memory_path=memory_path,
            startup_timeout_s=30.0,
        )
        control.start()
        execution.start()
        timeout = manifest.contract.horizon_ticks / manifest.contract.control_frequency_hz + 15.0
        terminal = control.wait_terminal(timeout)
        policy_result = execution.finish()
        policy_output = policy_result.output
        policy_execution_status = "completed" if terminal.success else policy_result.status
        native = {
            "success": bool(terminal.success),
            "score": float(terminal.score),
            "reward": float(terminal.reward),
            "terminal_reason": terminal.terminal_reason,
            "policy_ticks": int(terminal.policy_ticks),
            "backend_ticks": int(terminal.backend_ticks),
            "policy_execution_status": policy_execution_status,
        }
        _log(log_path, "trial_terminal", **native)
        status: Literal["completed", "policy_error", "infrastructure_error"]
        if terminal.terminal_reason == "failure" or policy_result.status == "infrastructure_error":
            status = "infrastructure_error"
        elif policy_execution_status == "policy_error":
            status = "policy_error"
        else:
            status = "completed"
        error = terminal.error or ("" if terminal.success else policy_result.error)
    except BaseException as exc:
        if execution is not None:
            try:
                policy_output = execution.finish().output
            except Exception:
                pass
        if control is not None:
            try:
                control.cancel()
            except Exception:
                pass
        status = "infrastructure_error"
        error = f"{type(exc).__name__}: {exc}"
        native = {
            "success": False,
            "score": 0.0,
            "reward": 0.0,
            "terminal_reason": "failure",
            "policy_ticks": 0,
            "backend_ticks": 0,
            "policy_execution_status": "infrastructure_error",
        }
        _log(log_path, "trial_error", error=error)
        if isinstance(exc, KeyboardInterrupt):
            raise
    finally:
        policy_output_path.write_text(policy_output, encoding="utf-8")
        if coordinator is not None:
            coordinator.stop()
        if control is not None:
            control.close()
        container.stop()
        if previous_run_id is None:
            os.environ.pop(DIMOS_RUN_ID_ENV, None)
        else:
            os.environ[DIMOS_RUN_ID_ENV] = previous_run_id
    if status == "completed" and (not video_path.is_file() or video_path.stat().st_size == 0):
        status = "infrastructure_error"
        error = "LIBERO trial did not produce a rendered video artifact"
    outcome = TrialOutcome(
        success=bool(native["success"]),
        reward=float(native["reward"]),
        status=status,
        error=error,
        duration_seconds=time.monotonic() - started,
    )
    return (
        TrialRun(
            run_id=run_id,
            outcome=outcome,
            artifacts=path,
            log_path=log_path,
            memory_path=memory_path,
            policy_output=policy_output,
        ),
        native,
    )


def _log(path: Path, event: str, **values: object) -> None:
    record = {
        "timestamp": datetime.now(timezone.utc).isoformat(),
        "module": "libero-pro-evaluation",
        "event": event,
        **values,
    }
    with path.open("a", encoding="utf-8") as handle:
        handle.write(json.dumps(record, sort_keys=True) + "\n")


def _artifact(path: str, label: str, media_type: str) -> ArtifactReference:
    return ArtifactReference(path=path, label=label, media_type=media_type)


libero_pro = LiberoProEvaluation()
