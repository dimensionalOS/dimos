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

"""`analyze_memory`: run a caller's Python over the recording, in its own process.

Split out of module.py, which had reached the repo's 75KB per-file limit for the third
time -- the hook rejects the whole commit there and reports only a size, so the failure
reads as unrelated to whatever you were editing, and what got shortened to fit was twice
the comment explaining why a fix existed. A mixin, like VisualAnswers and ReplayServing
beside it, so the host class composes the same way it already does. Its tests are already
in test_analyze_memory.py.
"""

from __future__ import annotations

from collections.abc import Callable
import contextlib
import json
import os
import signal
import subprocess
import sys
import time
from typing import Annotated, Any

from pydantic import Field as PydanticField

from dimos.agents.annotation import skill
from dimos.agents.skill_result import SkillResult
from dimos.teleop.memory_world.query import (
    MEMORY_ANALYSIS_BOOTSTRAP,
    RESULT_SENTINEL,
    MemoryQueryResult,
)
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


def _end_group(child: subprocess.Popen[str]) -> None:
    """TERM the analysis and everything it started, then KILL what is left.

    Whether to send the second signal cannot be a question about the DIRECT CHILD, because
    the signal does not go to the direct child: it goes to the group, and the reason the
    group exists is that analysis code is free to spawn. Returning as soon as the child
    was gone left a grandchild that ignores SIGTERM running for ever -- measured, a loop
    appending to a file every 0.2 s was still writing after this returned, with
    `analyze_memory` reporting EXECUTION_TIMEOUT.

    The group id is read once, up front: the wait below reaps the child, and `getpgid` of
    a reaped pid raises. A `killpg` on a group that is already empty raises too, and is
    suppressed -- sending it costs nothing, and not sending it cost everything.
    """
    try:
        group = os.getpgid(child.pid)
    except (OSError, ProcessLookupError, PermissionError):
        return
    for sig in (signal.SIGTERM, signal.SIGKILL):
        with contextlib.suppress(OSError, ProcessLookupError, PermissionError):
            os.killpg(group, sig)
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline and child.poll() is None:
            time.sleep(0.05)


class MemoryAnalysis:
    """`analyze_memory` and the output cap it needs.

    The host supplies `config`, `_clients_lock`, `_viewer_position`,
    `_add_route_to_result` and `_publish_query_result`.
    """

    config: Any
    _clients_lock: Any
    _viewer_position: tuple[float, float, float] | None
    _add_route_to_result: Callable[[MemoryQueryResult], None]
    _publish_query_result: Callable[[MemoryQueryResult], str]

    @skill
    def analyze_memory(
        self,
        code: str,
        timeout: Annotated[float, PydanticField(gt=0.0, le=100.0)] = 100.0,
    ) -> SkillResult:
        """Analyze the recorded memory and display validated spatial results in VR.

        Run complete Python code in a fresh process with ``store`` (the recording,
        a mem2 store), ``np`` (NumPy), and ``viewer_position`` available. Inspect
        streams with ``store.list_streams()``, ``store.summary()``, and
        ``store.streams[name]``. Observations expose ``pose_tuple``, ``data``,
        and ``id``. ``store.read_stream`` does not exist. For a bounded xyz
        trajectory use ``sample_pose_path("odom", max_points=200)``. Assign a
        dictionary to ``result`` with a required ``answer`` and optional fields:
        ``focus_point`` [x,y,z], ``regions`` (polygon point lists),
        ``evidence_paths`` (path point lists), ``points``, and
        ``observation_ids``. Every point must be [x,y,z] in the world frame.
        A route from the current VR position is added automatically when
        ``focus_point`` and ``global_costmap`` are available.

        Args:
            code: Complete Python source that assigns the result dictionary.
            timeout: Maximum execution time in seconds, up to 100 seconds.
        """
        started = time.monotonic()
        with self._clients_lock:
            viewer_position = self._viewer_position
        # Its own session, so a timeout can end everything the analysis started. Analysis
        # code is free to spawn, and `subprocess.run(timeout=...)` signals only the child
        # it launched: measured, a snippet that spawned a background loop and then slept
        # past the timeout was reported as EXECUTION_TIMEOUT while that loop went on
        # writing, unmanaged, with nothing left holding a handle to it. `embed.py` learnt
        # the same lesson about `nix run`.
        child = subprocess.Popen(
            [
                sys.executable,
                "-c",
                MEMORY_ANALYSIS_BOOTSTRAP,
                self.config.store_path,
                json.dumps(viewer_position),
            ],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            start_new_session=True,
        )
        try:
            out, err = child.communicate(input=code, timeout=timeout)
            completed = subprocess.CompletedProcess(
                child.args, child.returncode, stdout=out, stderr=err
            )
        except subprocess.TimeoutExpired:
            _end_group(child)
            with contextlib.suppress(Exception):
                child.communicate(timeout=5)
            return SkillResult.fail(
                "EXECUTION_TIMEOUT", f"Memory analysis timed out after {timeout:g} seconds"
            )
        except BaseException:
            _end_group(child)
            raise

        # The bootstrap prints the sentinel at the START of its own line, so that is how
        # it is looked for. `rfind` over the whole of stdout searched INSIDE the printed
        # JSON too, and since the JSON follows the marker on the same line, an answer whose
        # own text contained the sentinel won the search -- a valid result came back as
        # EXECUTION_FAILED. It also indexed `splitlines()[0]` on whatever followed, which
        # for a marker at the very end of stdout is an empty list: IndexError, uncaught.
        encoded = None
        for line in reversed(completed.stdout.splitlines()):
            if line.startswith(RESULT_SENTINEL):
                encoded = line[len(RESULT_SENTINEL) :]
                break
        if encoded is None:
            detail = (completed.stderr or completed.stdout or "analysis returned no result").strip()
            return SkillResult.fail("EXECUTION_FAILED", self._cap_analysis_output(detail))

        # A result on stdout is not the same as a run that worked. The child can print the
        # sentinel and THEN die -- a teardown that raises, a segfault in a native library
        # closing its handles -- and everything below this point would have accepted the
        # printed answer and reported success, losing the failure entirely. Measured: a
        # recording stub whose cleanup raised gave returncode 1, a RuntimeError on stderr,
        # and `success=True` out of this method.
        if completed.returncode != 0:
            detail = (completed.stderr or "").strip() or (
                f"analysis exited with status {completed.returncode} after printing a result"
            )
            return SkillResult.fail("EXECUTION_FAILED", self._cap_analysis_output(detail))

        if len(encoded) > self.config.memory_analysis_max_output_chars:
            return SkillResult.fail(
                "RESULT_TOO_LARGE",
                "Memory result exceeds the configured output limit of "
                f"{self.config.memory_analysis_max_output_chars} characters",
            )
        try:
            result = MemoryQueryResult.model_validate_json(encoded)
            self._add_route_to_result(result)
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
                "evidence_paths": len(result.evidence_paths),
                "observation_ids": len(result.observation_ids),
                "route": result.route is not None,
            },
        )

    def _cap_analysis_output(self, output: str) -> str:
        limit = self.config.memory_analysis_max_output_chars
        if len(output) <= limit:
            return output
        return output[:limit] + f"\n... [truncated, {len(output)} chars total]"
