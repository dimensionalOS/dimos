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

from pathlib import Path
import subprocess
import threading
import time

import pytest

from dimos.core.transport import pLCMTransport

_TIMEOUT = 30


@pytest.fixture
def repl():
    proc = subprocess.Popen(
        ["python3", "-i"],
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1,
    )

    def _pump():
        for line in proc.stdout:
            print(line, end="")

    t = threading.Thread(target=_pump, daemon=True)
    t.start()
    yield proc
    proc.stdin.write("exit()\n")
    proc.stdin.flush()
    proc.wait(_TIMEOUT)
    t.join(_TIMEOUT)


@pytest.fixture
def greeting():
    transport = pLCMTransport("/greetings")
    transport.start()
    yield transport
    transport.stop()


@pytest.fixture
def response():
    transport = pLCMTransport("/response")
    transport.start()
    yield transport
    transport.stop()


def _create_event(transport, expected) -> threading.Event:
    event = threading.Event()

    def handler(msg):
        if msg == expected:
            event.set()

    transport.subscribe(handler)

    return event


def _wait_for_response(greeting, response, expected) -> None:
    event = _create_event(response, expected)
    deadline = time.time() + _TIMEOUT
    while not event.is_set() and time.time() < deadline:
        greeting.publish("John")
        event.wait(0.5)
    assert event.is_set(), f"Did not receive {expected!r} within timeout"


@pytest.fixture
def test_module_file():
    path = Path(__file__).parent / "_test_module.py"
    original = path.read_text()
    yield path
    path.write_text(original)


@pytest.mark.slow
def test_module_reloading(repl, greeting, response, test_module_file):
    repl.stdin.write("""
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.core.coordination import _test_module
mc = ModuleCoordinator.build(_test_module.AliceModule.blueprint())
""")
    repl.stdin.flush()

    _wait_for_response(greeting, response, "Hello John from Alice")

    test_module_file.write_text(test_module_file.read_text().replace("from Alice", "from Bob"))

    repl.stdin.write("mc.restart_module(_test_module.AliceModule)\n")
    repl.stdin.flush()

    _wait_for_response(greeting, response, "Hello John from Bob")
