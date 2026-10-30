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

"""`launches.last_error`: a failed launch's error names its cause, not the last shutdown line."""

from pathlib import Path

from experimental.gateway import launches

ABORT = "*** Terminating app due to uncaught exception 'NSInternalInconsistencyException'"
SHUTDOWN = "12:00:00.002 [inf][worker_manager_python.py] All workers shut down"


def test_a_native_abort_is_the_error_not_the_shutdown_lines_after_it() -> None:
    # a real `dimos run go2-sim` on macOS: pygame aborts on a non-main thread, ~50 shutdown lines follow
    output = (Path(__file__).parent / "fixtures" / "launch-output-worker-abort.txt").read_text()
    assert launches.last_error(output).startswith(ABORT)


def test_a_traceback_is_the_error_and_lines_that_merely_mention_errors_are_not() -> None:
    output = "\n".join(
        [
            "dimos run demo",
            "objc[27424]: Class GLFWHelper is implemented in both /a/libglfw.dylib and /b/libglfw.dylib",
            "12:00:00.001 [wrn][a.py] a warning",
            "12:00:00.001 [inf][health.py] Error: count reset to 0",
            "Traceback (most recent call last):",
            '  File "/x/demo.py", line 3, in start',
            "    raise RuntimeError('boom')",
            "12:00:00.001 [inf][other_worker.py] another worker, between the frames and the exception",
            "RuntimeError: boom",
            SHUTDOWN,
        ]
    )
    assert launches.last_error(output) == "RuntimeError: boom"
    critical = "12:00:00.001 [cri][a.py] out of memory"
    assert launches.last_error(f"dimos run demo\n{critical}\n{SHUTDOWN}\n") == critical
    assert launches.last_error(f"dimos run demo\n{SHUTDOWN}\n") == SHUTDOWN
    assert launches.last_error("dimos run demo\n") == "dimos exited during startup"
