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

"""One live-agent VLN-CE R2R Evaluation under the DimOS geometry condition."""

from __future__ import annotations

import json
from pathlib import Path
import threading
import time

from pydantic import BaseModel

from dimos.benchmark.evaluation.models import (
    ArtifactNativeResult,
    ArtifactReference,
    EvaluationReport,
    SummaryItem,
)
from dimos.benchmark.evaluation.progress import StatusProgress, emit_progress
from dimos.benchmark.evaluation.protocol import EvaluationContext, LiveAgentRuntime
from dimos.benchmark.vlnce_r2r.external_runtime import (
    VlnceExternalRuntime,
    preparation_evidence,
)
from dimos.benchmark.vlnce_r2r.models import VlnceConfig, VlnceTaskManifest
from dimos.benchmark.vlnce_r2r.native_result import validate_native_result
from dimos.benchmark.vlnce_r2r.preparation import prepare_public_assets, resolve_oci_image
from dimos.benchmark.vlnce_r2r.prompt import vlnce_task_prompt

SYSTEM_PROMPT = """You are the live policy for a real-time navigation Evaluation.

Use python_exec throughout the episode. The persistent session provides `app.memory` for
public observations and `app` for ordinary DimOS RPCs. The relevant Memory2 streams are
`color_image`, `depth_image`, `depth_pointcloud`, `global_costmap`, and `odom`.

Inspect the actual RGB image immediately and after every short motion:

    from IPython.display import display
    from PIL import Image as PILImage
    display(PILImage.fromarray(app.memory.stream("color_image").last().data.data))

Displayed images are delivered to you visually. The `global_costmap` stream is the
complete world-frame traversability grid; `depth_pointcloud` is only the latest
camera-local depth geometry. These expressions access the map and robot pose:

    costmap = app.memory.stream("global_costmap").last().data
    cells = costmap.grid
    pose = app.memory.stream("odom").last().data
    cell = costmap.world_to_grid((pose.x, pose.y))  # cell.x is column, cell.y is row
    world = costmap.grid_to_world((column, row))

Cells with value zero are traversable. For planner-driven motion, select a traversable
waypoint, convert it with `grid_to_world()`, and call
`app.NavigationSkillContainer.navigate_to_position(x, y)`. Poll
`app.NavigationSkillContainer.navigation_status()` until it reports `IDLE`, then
inspect fresh RGB and odometry before choosing the next waypoint.

The map contains geometry, not room or object identity. Use RGB—not the shape of free
map cells—to choose between branches. At an ambiguous doorway or junction, call
`app.VlnceConnection.turn(angle_degrees)` and inspect fresh RGB in both directions
before translating. Positive angles turn counterclockwise and negative angles turn
clockwise. If a view still shows the room you were instructed to exit, do not move
deeper into that room.

Keep control closed-loop. Never issue a long or unbounded movement loop. For manual
control, call `app.VlnceConnection.move(twist)` without a duration. You may send a
bounded burst of controls, waiting at least 0.1 seconds between them; then inspect
fresh RGB, odometry, and planner state before deciding again.

The benchmark continues while you reason. Call app.VlnceConnection.submit_route()
only after fresh RGB evidence confirms the final named place or object, odometry shows
meaningful route progress, the robot is stopped, and the complete language route is
finished. Never infer success from a semantic-memory match or point-to-point navigation
completion alone. If the final named object is not visible, do not submit merely
because a branch ends or a waypoint fails.
"""


class VlnceR2REvaluation:
    name = "vlnce-r2r"
    runtime_profile = "live-agent-v1"
    config_model: type[BaseModel] = VlnceConfig

    def run(self, config: BaseModel, context: EvaluationContext) -> EvaluationReport:
        if not isinstance(config, VlnceConfig):
            raise TypeError("vlnce-r2r received the wrong configuration type")
        if not isinstance(context.runtime, LiveAgentRuntime):
            raise TypeError("vlnce-r2r requires the live-agent-v1 runtime")
        manifest_path = Path(config.task_manifest)
        if not manifest_path.is_absolute():
            manifest_path = context.spec_dir / manifest_path
        manifest = VlnceTaskManifest.model_validate_json(manifest_path.read_bytes())
        emit_progress(context.progress, StatusProgress(channel="eval", message="VLN-CE preflight"))
        preparation = prepare_public_assets(manifest.source, manifest.task)
        image_id = resolve_oci_image(manifest.source.preparation.image)
        attempt_path = context.workspace / "attempt"
        attempt_path.mkdir()
        runtime = VlnceExternalRuntime(
            case=manifest,
            attempt_id=context.run_id,
            attempt_path=attempt_path,
            preparation=preparation,
            image_id=image_id,
            render="native",
        )
        execution = None
        payload: bytes | None = None
        agent_outcome = None
        public_evidence: dict[str, object] = {}
        try:
            startup = runtime.start()
            (attempt_path / "runtime-startup.json").write_text(
                json.dumps(startup, indent=2, sort_keys=True) + "\n", encoding="utf-8"
            )
            (attempt_path / "preparation.json").write_text(
                json.dumps(preparation_evidence(preparation, image_id), indent=2, sort_keys=True)
                + "\n",
                encoding="utf-8",
            )
            execution = context.runtime.prepare(
                prompt=vlnce_task_prompt(manifest.task.prompt),
                system_prompt=SYSTEM_PROMPT,
                memory_path=runtime.memory_path,
                episode_timeout_s=manifest.interaction.timeout_seconds,
            )
            runtime.begin()
            execution.start()
            emit_progress(
                context.progress,
                StatusProgress(channel="eval", message="live navigation agent active"),
            )
            grace_deadline = time.monotonic() + manifest.interaction.timeout_seconds + 30.0
            waiter = threading.Event()
            while payload is None:
                failure = execution.failure()
                if failure is not None:
                    raise RuntimeError(f"live agent failed: {type(failure).__name__}: {failure}")
                if not runtime.healthy():
                    raise RuntimeError("VLN-CE runtime became unavailable")
                payload = runtime.result_bytes()
                if payload is not None:
                    public_evidence = dict(runtime.public_evidence())
                    break
                if time.monotonic() >= grace_deadline:
                    raise TimeoutError("VLN-CE did not publish a terminal result")
                waiter.wait(0.1)
        finally:
            try:
                if execution is not None:
                    agent_outcome = execution.finish()
            finally:
                try:
                    runtime.cancel_motion()
                finally:
                    runtime.close()
        if payload is None:
            raise RuntimeError("VLN-CE ended without a native result")
        native = validate_native_result(payload, case=manifest, attempt_id=context.run_id)
        render = runtime.render_evidence() or {
            "schema_version": "native-render.v1",
            "status": "failed",
            "diagnostic": "renderer evidence was unavailable",
        }
        render_path = attempt_path / "native-render.v1.json"
        render_path.write_text(
            json.dumps(render, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )
        public_path = attempt_path / "public-terminal.json"
        public_path.write_text(
            json.dumps(public_evidence, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        metrics = native.metrics
        artifacts = [
            _artifact("attempt/terminal-private/vlnce-result.v1.json", "Native VLN-CE result"),
            _artifact("attempt/live-memory/recording.db", "Public observation recording"),
            _artifact("attempt/runtime-startup.json", "Runtime startup evidence"),
            _artifact("attempt/preparation.json", "Preparation evidence"),
            _artifact("attempt/native-render.v1.json", "Native render evidence"),
            _artifact("attempt/public-terminal.json", "Public terminal evidence"),
            _artifact("attempt/oci-runtime.log", "OCI runtime log", "text/plain"),
        ]
        if runtime.render_path.is_file():
            artifacts.append(
                _artifact("attempt/native-render.mp4", "Native episode video", "video/mp4")
            )
        return EvaluationReport(
            summary=(
                SummaryItem(key="case", label="Case", value=manifest.case_id),
                SummaryItem(
                    key="condition", label="Condition", value=manifest.source.condition_label
                ),
                SummaryItem(key="success", label="Official success", value=metrics.SUCCESS == 1.0),
                SummaryItem(key="spl", label="SPL", value=metrics.SPL),
                SummaryItem(key="ndtw", label="nDTW", value=metrics.NDTW),
                SummaryItem(
                    key="distance", label="Distance to goal", value=metrics.DISTANCE_TO_GOAL
                ),
                SummaryItem(
                    key="agent_calls",
                    label="Agent tool calls",
                    value=agent_outcome.tool_call_count if agent_outcome is not None else 0,
                ),
                SummaryItem(key="render", label="Native render", value=str(render.get("status"))),
            ),
            native_result=ArtifactNativeResult(artifact=artifacts[0]),
            artifacts=tuple(artifacts),
        )


def _artifact(
    path: str,
    label: str,
    media_type: str = "application/json",
) -> ArtifactReference:
    return ArtifactReference(path=path, label=label, media_type=media_type)


vlnce_r2r = VlnceR2REvaluation()
