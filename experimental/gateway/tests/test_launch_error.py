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

import pytest

from experimental.gateway import launches

ABORT = "*** Terminating app due to uncaught exception 'NSInternalInconsistencyException'"
SHUTDOWN = "12:00:00.002 [inf][worker_manager_python.py] All workers shut down"
TRACEBACK = 'Traceback (most recent call last):\n  File "/x/demo.py", line 3, in start\n    run()'
ERROR = "12:00:00.001 [err][a.py] worker failed"
CRITICAL = "12:00:00.001 [cri][a.py] out of memory"
UNCAUGHT = "12:00:00.001 [err][dimos/utils/logging_config.py ] Uncaught exception occurred"
BOXED = (
    "╭─ Traceback (most recent call last) ─╮\n"
    "│ /x/demo.py:3 in                     │\n"
    "│ start                               │\n"
    "╰─────────────────────────────────────╯"
)


@pytest.mark.parametrize(
    "output,expected",
    [
        pytest.param(
            "objc[27424]: Class GLFWHelper is implemented in both /a/libglfw.dylib and /b/libglfw.dylib\n"
            "12:00:00.001 [wrn][a.py] a warning\n"
            f"{TRACEBACK}\n"
            "12:00:00.001 [inf][other_worker.py] another worker, between the frames and the exception\n"
            "INFO:     Started server process [42]\n"
            "RuntimeError: boom\n"
            "12:00:00.002 [inf][health.py] Error: count reset to 0\n"
            f"{SHUTDOWN}\n",
            "RuntimeError: boom",
            id="traceback-with-worker-chatter",
        ),
        pytest.param(
            f"{TRACEBACK}\n"
            "2026-10-22T05:28:53.624185Z  WARN ThreadId(01) zenoh::net::routing::hat::peer: "
            "Gossip is disabled: cannot de-duplicate data traversing region gateways\n"
            f"RuntimeError: boom\n{SHUTDOWN}\n",
            "RuntimeError: boom",
            id="traceback-with-native-warning",
        ),
        pytest.param(
            f"{TRACEBACK}\n{ERROR}\nRuntimeError: boom\n{SHUTDOWN}\n",
            "RuntimeError: boom",
            id="error-log-before-exception",
        ),
        pytest.param(
            f"{ERROR}\n"
            '12:00:00.002 [inf][health.py] captured "[err]", "[cri]", "*** Terminating app"\n'
            f"{SHUTDOWN}\n",
            ERROR,
            id="quoted-severity-after-cause",
        ),
        pytest.param(
            f"{TRACEBACK}\nKeyboardInterrupt\n{SHUTDOWN}\n",
            "KeyboardInterrupt",
            id="message-less-exception",
        ),
        pytest.param(
            f"{TRACEBACK}\npkg.worker.LaunchError: failed\n{SHUTDOWN}\n",
            "pkg.worker.LaunchError: failed",
            id="qualified-exception",
        ),
        pytest.param(f"{ERROR}\n{SHUTDOWN}\n", ERROR, id="error-level"),
        pytest.param(f"{CRITICAL}\n{SHUTDOWN}\n", CRITICAL, id="critical-level"),
        pytest.param(
            f"{ABORT}\n"
            "12:00:00.001 [err][worker_manager_python.py] health_check: worker died worker_id=0\n"
            f"{SHUTDOWN}\n{UNCAUGHT}\n{BOXED}\n"
            "RuntimeError: health check failed, a worker process died\n",
            ABORT,
            id="abort-before-failed-health-check",
        ),
        pytest.param(
            f"{UNCAUGHT}\n{BOXED}\nRuntimeError: boom\n{SHUTDOWN}\n",
            "RuntimeError: boom",
            id="boxed-traceback-after-uncaught-record",
        ),
        pytest.param(
            f"{UNCAUGHT}\n{BOXED}\n"
            "Exception: Demo has a module reference requesting a module that\n"
            "satisfies the WheelSpec spec. No module met that spec.\n"
            "2026-10-22T05:28:53.624185Z  WARN ThreadId(01) zenoh::net: closing\n",
            "Exception: Demo has a module reference requesting a module that "
            "satisfies the WheelSpec spec. No module met that spec.",
            id="boxed-traceback-with-wrapped-exception",
        ),
        pytest.param(
            "12:00:00.001 [err][zenohrpc.py] Exception in RPC handler for Demo/start: failed\n"
            f"{SHUTDOWN}\n{UNCAUGHT}\n{BOXED}\n"
            "ExceptionGroup: safe_thread_map failed (1 sub-exception)\n"
            "\n"
            "╭─ Sub-exception #1 ──────────────────────╮\n"
            "│ ╭─ Traceback (most recent call last) ─╮ │\n"
            "│ │ /x/demo.py:3 in                     │ │\n"
            "│ │ start                               │ │\n"
            "│ ╰─────────────────────────────────────╯ │\n"
            "│ RuntimeError: `sudo ifconfig lo0 alias` │\n"
            "│ failed: a password is required          │\n"
            "╰─────────────────────────────────────────╯\n",
            "RuntimeError: `sudo ifconfig lo0 alias` failed: a password is required",
            id="failed-module-in-exception-group",
        ),
        pytest.param(
            "Usage: dimos [OPTIONS] COMMAND [ARGS]...\n"
            "╭─ Error ───────────────╮\n"
            "│ No such option: --foo │\n"
            "╰───────────────────────╯\n",
            "No such option: --foo",
            id="cli-error-panel",
        ),
    ],
)
def test_last_error(output: str, expected: str) -> None:
    assert launches.last_error(f"dimos run demo\n{output}") == expected
