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
            f"{ABORT}\n"
            '12:00:00.002 [inf][health.py] captured "[err]", "[cri]", "*** Terminating app"\n'
            f"{SHUTDOWN}\n",
            ABORT,
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
    ],
)
def test_last_error(output: str, expected: str) -> None:
    assert launches.last_error(f"dimos run demo\n{output}") == expected
