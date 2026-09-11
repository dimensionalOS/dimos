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

"""Robot process groups are cleaned even if their parent has already exited."""

import os
import select
import signal
import subprocess
import sys
from contextlib import suppress

import pytest
from microduck_world.supervisor import RobotProcess


@pytest.fixture
def exited_parent_with_worker(tmp_path):
    # The surviving worker holds the stdout pipe open after its parent exits.
    script = """
import subprocess
import sys
child = subprocess.Popen([sys.executable, '-c', 'import signal; signal.pause()'])
print(child.pid, flush=True)
"""
    process = subprocess.Popen(
        [sys.executable, "-c", script], stdout=subprocess.PIPE, start_new_session=True
    )
    log = (tmp_path / "robot.log").open("ab")
    try:
        assert process.stdout is not None
        assert int(process.stdout.readline()) > 0
        assert process.wait(timeout=5) == 0
        yield RobotProcess("test", process, log)
    finally:
        with suppress(ProcessLookupError):
            os.killpg(process.pid, signal.SIGKILL)
        process.wait(timeout=5)
        if process.stdout is not None:
            process.stdout.close()
        log.close()


def test_close_reaps_workers_after_the_runtime_parent_exits(exited_parent_with_worker):
    runtime = exited_parent_with_worker
    assert not select.select([runtime.process.stdout], [], [], 0)[0]

    runtime.close()

    assert select.select([runtime.process.stdout], [], [], 5)[0]
    assert runtime.process.stdout.read() == b""
    assert runtime.log.closed
