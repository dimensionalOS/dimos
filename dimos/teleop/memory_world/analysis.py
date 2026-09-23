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

"""`analyze_memory`: the agent writes a program about the recording and we run it.

From `andrew/feat/vr_demo`, where this was built. The search tools answer "where is X";
this answers everything else -- how far, how wide, how long, how many of the thing you
just found, what the floor does between here and there -- by handing the agent the
recording's own streams and letting it measure. The measuring is the point: an LLM
guessing a corridor width from photographs is not an answer, and the same LLM writing
six lines of numpy over the lidar map is.

Three things make it safe enough to expose:

- It runs in a SEPARATE PROCESS. A runaway loop is killed at the timeout and a segfault
  in numpy takes the child, not the module serving the demo.
- The child gets the recording read-only-ish and nothing else useful: no module, no
  network handle, no client list. It is not a security boundary -- the agent is trusted
  and `exec` is `exec` -- it is a blast radius.
- The result is VALIDATED against `MemoryQueryResult` before any of it is drawn, so a
  malformed answer is a failure the agent can read and retry, not a viewer that breaks.

The program is run one step at a time (`stepwise.py`) and each step is relayed to the
chat panel as it starts and finishes. That is not decoration: an analysis over a 4-million
point map takes tens of seconds, and a panel that says nothing until it is done is
indistinguishable from one that has hung.
"""

from __future__ import annotations

import json
import subprocess
import sys
import threading
import time
from typing import TYPE_CHECKING, Annotated, Any

from pydantic import Field as PydanticField, ValidationError

from dimos.agents.annotation import skill
from dimos.agents.skill_result import SkillResult
from dimos.teleop.memory_world.messages import encode_text
from dimos.teleop.memory_world.query import (
    MEMORY_ANALYSIS_BOOTSTRAP,
    RESULT_SENTINEL,
    STEP_SENTINEL,
    MemoryQueryResult,
    validation_summary,
)
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

# How long to wait for the step relay to drain after the child is gone. The thread is a
# daemon, so this is about getting the LAST step into the transcript, not about shutdown.
RELAY_DRAIN_S = 5.0


class MemoryAnalysis:
    """The `analyze_memory` skill. Expects the host module's ``config``,
    ``_clients_lock``, ``_broadcast``, ``_chat_history``, ``_publish_query_result``,
    ``_viewer_position``, ``_active_query_result`` and ``_ensure_world_cache``."""

    config: Any
    _clients_lock: threading.Lock
    _viewer_position: tuple[float, float, float] | None
    _active_query_result: dict[str, Any] | None

    if TYPE_CHECKING:
        from collections import deque

        from dimos.agents.utils import ChatEntry

        _chat_history: deque[ChatEntry]

        def _broadcast(self, message: bytes | str) -> None: ...
        def _publish_query_result(self, result: MemoryQueryResult) -> str: ...
        def _ensure_world_cache(self) -> Any: ...

    @skill
    def analyze_memory(
        self,
        code: Annotated[str, PydanticField(min_length=1)],
        timeout: Annotated[float, PydanticField(gt=0.0, le=100.0)] = 100.0,
    ) -> SkillResult:
        """Measure something in the recorded memory by running Python over its streams.

        The program runs in a fresh process with ``store`` (the recording), ``np``,
        ``places`` (what the last search found, each with ``centre`` and ``radius``),
        ``viewer_position``, ``world_frame`` and ``sample_pose_path(max_points=200)``
        (the robot's trajectory as world xyz). Read streams with
        ``store.list_streams()`` and ``store.streams[name]``; an observation has ``ts``,
        ``id``, ``data`` and ``pose_tuple``. ``store.read_stream`` does not exist.
        Assign a dictionary to ``result`` with a required ``answer`` string and any of
        ``focus_point`` [x, y, z], ``points``, ``regions``, ``evidence_paths`` and
        ``observation_ids``. Every point is [x, y, z] in the world frame.

        Args:
            code: Complete Python source that assigns the result dictionary.
            timeout: Maximum execution time in seconds, up to 100 seconds.
        """
        started = time.monotonic()
        stdout, stderr, timed_out = self._run_analysis(code, timeout)
        if timed_out:
            return SkillResult.fail(
                "EXECUTION_TIMEOUT", f"Memory analysis timed out after {timeout:g} seconds"
            )

        # The LAST sentinel on stdout, because the program may print one itself -- as a
        # string in its own output, or by running an earlier analysis's source again.
        marker = stdout.rfind(RESULT_SENTINEL)
        if marker < 0:
            detail = (stderr or stdout or "analysis returned no result").strip()
            return SkillResult.fail("EXECUTION_FAILED", self._cap_analysis_output(detail))

        encoded = stdout[marker + len(RESULT_SENTINEL) :].splitlines()[0]
        if len(encoded) > self.config.memory_analysis_max_output_chars:
            return SkillResult.fail(
                "RESULT_TOO_LARGE",
                "Memory result exceeds the configured output limit of "
                f"{self.config.memory_analysis_max_output_chars} characters",
            )
        try:
            result = MemoryQueryResult.model_validate_json(encoded)
        except ValidationError as exc:
            return SkillResult.fail(
                "EXECUTION_FAILED", f"Invalid memory result: {validation_summary(exc)}"
            )
        except Exception as exc:
            return SkillResult.fail("EXECUTION_FAILED", f"Invalid memory result: {exc}")

        query_id = self._publish_query_result(result)
        return SkillResult(
            success=True,
            message=result.answer,
            duration_ms=(time.monotonic() - started) * 1000,
            metadata={
                "query_id": query_id,
                "regions": len(result.regions),
                "points": len(result.points),
                "evidence_paths": len(result.evidence_paths),
                "observation_ids": len(result.observation_ids),
            },
        )

    # ---- running one --------------------------------------------------------------

    def _run_analysis(self, code: str, timeout: float) -> tuple[str, str, bool]:
        """Run an analysis in the sandbox, relaying each step to the viewers as it runs.

        Returns the sandbox's stdout, its stderr minus the step reports, and whether it
        was killed for running past the timeout.
        """
        with self._clients_lock:
            viewer_position = self._viewer_position
            active = self._active_query_result or {}
        places = [
            {key: cluster.get(key) for key in ("index", "centre", "radius", "score", "label")}
            for cluster in active.get("clusters", [])
        ]
        process = subprocess.Popen(
            [
                sys.executable,
                "-c",
                MEMORY_ANALYSIS_BOOTSTRAP,
                str(self.config.store_path),
                json.dumps(viewer_position),
                json.dumps(places),
                json.dumps(self._trail_points()),
                str(self.config.world_frame),
            ],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
        )
        assert process.stdin and process.stdout and process.stderr
        stderr_lines: list[str] = []

        def relay_steps() -> None:
            assert process.stderr
            for line in process.stderr:
                if line.startswith(STEP_SENTINEL):
                    try:
                        self._on_analysis_step(json.loads(line[len(STEP_SENTINEL) :]))
                        continue
                    except ValueError:
                        pass
                stderr_lines.append(line)

        killed = threading.Event()

        def kill() -> None:
            killed.set()
            process.kill()

        reader = threading.Thread(target=relay_steps, daemon=True, name="MemoryAnalysisSteps")
        reader.start()
        timer = threading.Timer(timeout, kill)
        timer.start()
        try:
            process.stdin.write(code)
            process.stdin.close()
            stdout = process.stdout.read()
            process.wait()
        finally:
            timer.cancel()
            reader.join(timeout=RELAY_DRAIN_S)
        return stdout, "".join(stderr_lines), killed.is_set()

    def _trail_points(self) -> list[list[float]]:
        """The robot's trajectory as world xyz, from the cache the viewer already draws.

        Built here rather than in the child because on this build the poses come from the
        tf tree and tf is the module's to read -- an observation's own `pose_tuple` is in
        another frame, or absent, on every recording this demo has been pointed at.
        """
        import numpy as np

        try:
            trail = self._ensure_world_cache()[3]
        except Exception:
            logger.exception("could not read the trail for an analysis")
            return []
        if not trail or not trail[1]:
            return []
        return np.frombuffer(trail[1], dtype=np.float32).reshape(-1, 3).tolist()

    def _on_analysis_step(self, step: dict[str, Any]) -> None:
        """Show one step of a running analysis in every viewer's chat."""
        entry = {"role": "tool_step", **step}
        with self._clients_lock:
            self._chat_history.append(entry)  # type: ignore[arg-type]
        self._broadcast(encode_text("chat", **entry))

    def _cap_analysis_output(self, output: str) -> str:
        limit = self.config.memory_analysis_max_output_chars
        if len(output) <= limit:
            return output
        return output[:limit] + f"\n... [truncated, {len(output)} chars total]"
