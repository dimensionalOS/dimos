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

from collections.abc import Callable, Iterator
import subprocess
import sys

import pytest

from dimos.core.global_config import global_config
from dimos.simulation.dimsim.dimsim_process import DimSimProcess


@pytest.fixture
def fake_dimsim() -> Iterator[Callable[[str], DimSimProcess]]:
    """A DimSimProcess around a python child standing in for the deno bridge."""
    started: list[DimSimProcess] = []

    def make(script: str) -> DimSimProcess:
        proc = DimSimProcess(global_config)
        proc.process = subprocess.Popen(
            [sys.executable, "-u", "-c", script],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )
        proc._start_log_reader()
        started.append(proc)
        return proc

    yield make
    for proc in started:
        proc.stop()


def test_ready_line_unblocks(fake_dimsim: Callable[[str], DimSimProcess]) -> None:
    proc = fake_dimsim(
        "import time\n"
        "print('[bridge] :8090 (LCM bridge)')\n"
        "print('[bridge:default] ready')\n"
        "time.sleep(30)\n"
    )
    proc.wait_until_ready(timeout=10.0)


def test_exit_before_ready_fails_fast(fake_dimsim: Callable[[str], DimSimProcess]) -> None:
    proc = fake_dimsim("import sys\nprint('[dimsim] vite build failed (exit 1).')\nsys.exit(3)\n")
    with pytest.raises(RuntimeError, match="code 3"):
        proc.wait_until_ready(timeout=10.0)


def test_silent_process_times_out(fake_dimsim: Callable[[str], DimSimProcess]) -> None:
    proc = fake_dimsim("import time\nprint('[headless] Launching: render=cpu')\ntime.sleep(30)\n")
    with pytest.raises(TimeoutError, match="did not report ready"):
        proc.wait_until_ready(timeout=0.6)
