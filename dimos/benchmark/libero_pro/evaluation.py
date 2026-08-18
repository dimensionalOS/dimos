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

EVALUATION_PROTOCOL = """Develop a task-specific real-time DimOS policy.
Use submit_policy to test each revision. Every submission runs a fresh episode of
the exact task below. The final submitted callable is frozen and evaluated on a
fresh initial state. The simulator advances continuously at 20 Hz while the
callable runs, so policies must tolerate real execution latency.

Inspect the actual recorded RGB images after every debug submission. `python_exec`
supports rich image output, just as it does in the navigation benchmark:

    trial = submit_policy(policy)
    from IPython.display import display
    from PIL import Image as PILImage
    with trial.open_memory() as memory:
        frame = memory.stream("agentview_color_image").last().data
        display(PILImage.fromarray(frame.to_rgb().data))

Displayed images are delivered to you visually. Inspect both `agentview_color_image`
and `eye_in_hand_color_image` when the wrist view can disambiguate contact. Do not
substitute ASCII art, pixel statistics, database internals, or textual image
representations for viewing the RGB frames directly.

Inside policy(app), `app.memory` is the live read-only Memory2 recording and normal
modules are available by class name. Use the calibrated RGB-D helper rather than
guessing camera geometry. A policy can start while the first synchronized observation
is being recorded: if `latest_rgbd` raises `LookupError`, retry briefly (for at most two
seconds with a 50 ms sleep) rather than failing or returning:

    from dimos.perception.rgbd import latest_rgbd, project_depth
    observation = latest_rgbd(
        app.memory,
        color_stream="agentview_color_image",
        depth_stream="agentview_depth_image",
        camera_info_stream="agentview_camera_info",
        optical_frame="agentview_optical",
    )
    masks = app.GroundedSegmentationModule.segment(observation.color, ["object description"])
    objects = project_depth(masks, observation)

`objects` is an iterable of `Detection3DPC`. Each detection has `.name`, `.center`
(a `Vector3` property, not a method), `.bbox` (`x_min, y_min, x_max, y_max` pixels
in the displayed image), `.pose`, and `.pointcloud`. Use `.bbox` to connect multiple
same-label detections to the instance you selected visually. A `PointCloud2`
supports `len(pointcloud)`, the `.center` and `.bounding_box_dimensions` properties,
and `.as_numpy()`, which returns `(points, colors)`. These are the exact public shapes;
do not spend submissions rediscovering them with `dir()` or deliberate exceptions.

Grounded segmentation returns query-conditioned hypotheses, not verified object
identities: every proposal for a query can carry that query's name, including false
positives. A label match alone therefore never proves identity. When a query returns
multiple detections, do not select `[0]`, `min`, `max`, leftmost, or rightmost as an
unsupported shortcut. Print every candidate's `.bbox`, inspect the displayed RGB image,
and map the requested object's visual appearance to the matching box. If identity is
still ambiguous, refine the visual query and test on another fresh debug episode.

Resolve relational language from all named references rather than a positional guess.
Detect every reference object explicitly and use bbox centers in the same image. For
"between", project each candidate center onto the segment joining the two reference
centers and compare both the along-segment parameter and perpendicular distance; for
"not between", exclude the candidate supported by that relation. Confirm the resulting
identity in the displayed image before freezing the policy.

For one uniquely described object, avoid enumerating query hypotheses entirely:

    masks = app.GroundedSegmentationModule.segment_best(
        observation.color,
        "specific visual description of the requested object",
    )
    objects = list(project_depth(masks, observation))

`segment_best` point-grounds the requested identity first and returns at most one mask.
Use ordinary `segment` only when the task genuinely requires enumerating multiple
instances or references. Still inspect the returned box against the displayed image.

Request ranked grasps with
`grasps = app.GraspGenXModule.propose_grasps(objects[0].pointcloud)`. The returned
`GraspCandidateArray` is directly iterable and also exposes `.candidates`; it has no
`.poses` attribute and does not support indexing (`grasps[0]` is invalid; use
`grasps.candidates[0]` or iterate it). Candidates are already calibrated Panda TCP
poses. A proposal's header supplies its timestamp and frame. Construct targets with
the exact message imports and keywords:

    from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
    from dimos.msgs.geometry_msgs.Vector3 import Vector3
    target = PoseStamped(
        ts=grasps.header.timestamp,
        frame_id=grasps.header.frame_id,
        position=candidate.pose.position,
        orientation=candidate.pose.orientation,
    )

Use `Vector3(x, y, z)` when changing a target position. Inspect
`app.ManipulationModule.list_planning_groups()` and use a returned `.id` instead of
guessing group IDs. Try targets in rank order with `plan_to_poses({group.id: target})`,
then call `execute()` only for a successful PlanResult. Every operation returns a typed
result; inspect `.succeeded`. `PlanResult`, `ExecutionResult`, and `CommandResult` expose
`.message`. `MoveResult` instead exposes `.plan` and optional `.execution`; inspect
`move.plan.message` or `move.execution.message`, never `move.message`. Re-observe after
motion and keep the policy closed-loop. The Panda gripper positions are 0.04 m open and
0.0 m closed.
Use the ordinary manipulation RPC exactly as
`app.ManipulationModule.set_gripper_position(0.04, planning_group=group.id)` to open
and `set_gripper_position(0.0, planning_group=group.id)` to close; there is no separate
`GripperModule` in this blueprint.
Use collision-checked planning for the free-space approach. Once that route is clear,
`move_to_pose(..., check_collision=False)` provides the low-latency absolute Cartesian
updates needed for contact-rich motion; keep those updates small and verify each result.

For ordinary object grasps, prefer the general closed-loop executor after segmentation:

    result = app.GraspExecutionModule.execute_grasp_candidates(
        grasps,
        planning_group=group.id,
    )

It tries ranked candidates with a collision-checked pre-grasp, a short contact move,
gripper closure, verifies that the measured aperture retained an object, and retracts.
It retries after an empty closure. Inspect `result.succeeded` and `result.message`. Use
the lower-level primitives directly when the task needs contact without retracting,
such as hooking an articulated handle.

For a tabletop pick-and-place with uniquely described source and destination, use the
single grounded executor. Pass the current calibrated observation explicitly; the RPC
does not read a hidden camera stream:

    result = app.GraspExecutionModule.grounded_pick_and_place(
        observation,
        "specific visual description of the source",
        "specific visual description of the destination",
        planning_group=group.id,
    )

This point-grounds both identities from the supplied RGB image, projects their masks
through the supplied depth and calibration, and executes the pick and place as one
operation. Use the task's object nouns plus visible attributes in each description.
Do not replace it with agent-side candidate enumeration unless this call returns an
explicit grounding failure in a debug trial. For already-grounded geometry, the lower
level equivalent is:

    result = app.GraspExecutionModule.pick_and_place_pointclouds(
        source.pointcloud,
        destination.pointcloud,
        planning_group=group.id,
    )

It aligns the gripper's closing direction to elongated source geometry while retaining
the downward tool axis, approaches through collision-checked free space, and verifies
object retention after retracting from contact. It places onto thin supporting surfaces
or inside tall receptacles. Prefer it over assembling an unverified sequence of raw plan
calls. Re-observe before each additional object in a multi-object task.

Use the first debug submission for a perception-and-motion attempt, not module or type
discovery. Print concise typed results and inspect `trial.policy_output` after every
submission. Keep reachability searches to the first six ranked candidates so planning
does not consume the real-time horizon. Never hard-code scene coordinates or planning
group IDs, and never swallow broad exceptions: fresh initial states require perception,
returned group IDs, and explicit result checks. Leave the last accepted submission as
a complete task policy; do not replace a working policy with a diagnostic probe or an
unverified hard-coded rewrite.

Plan with the complete hand volume, not only the TCP. Infer the grasp and motion axes
from observed geometry. For articulated handles, compare a force-closure pinch with a
geometric hook: approach through free space, place closed fingers behind the handle,
engage it orthogonally, then withdraw along the articulation axis. Test equivalent
wrist-roll candidates because the fingers can reach while the wrist or forearm still
collides with nearby objects. If IK fails at a joint boundary, retry nearby poses a few
millimeters away instead of abandoning the strategy. Prefer short, smooth, measured
waypoint updates over one large target jump, but do not waste the real-time horizon on
long sleeps. A held final setpoint continues executing after policy(app) returns, so
finish the callable once the commanded motion is safely engaged.
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
