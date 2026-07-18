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

"""kill_stale_port_holder must only kill LISTEN-state holders of the port.

A plain `lsof :port` also matches processes merely connected to the port -
in the field, the user's browser with a debug tab open to a crashed run's
relay - and killing those takes the browser down with the stale relay.
"""

import shutil
import subprocess
import sys

import pytest

from dimos.web.relay_bridge.relay_process import kill_stale_port_holder

_LISTENER = """
import socket, time
s = socket.socket()
s.bind(("127.0.0.1", 0))
s.listen()
print(s.getsockname()[1], flush=True)
conn, _ = s.accept()
time.sleep(60)
"""

_CLIENT = """
import socket, sys, time
c = socket.socket()
c.connect(("127.0.0.1", int(sys.argv[1])))
print("connected", flush=True)
time.sleep(60)
"""


@pytest.mark.skipif(shutil.which("lsof") is None, reason="lsof not installed")
def test_only_the_listener_is_killed() -> None:
    listener = subprocess.Popen(
        [sys.executable, "-c", _LISTENER], stdout=subprocess.PIPE, text=True
    )
    client = None
    try:
        assert listener.stdout is not None
        port = int(listener.stdout.readline())
        client = subprocess.Popen(
            [sys.executable, "-c", _CLIENT, str(port)], stdout=subprocess.PIPE, text=True
        )
        assert client.stdout is not None
        assert client.stdout.readline().strip() == "connected"

        kill_stale_port_holder(port)

        assert listener.wait(timeout=5) != 0  # SIGTERMed
        assert client.poll() is None  # the connected peer must survive
    finally:
        for process in (listener, client):
            if process is not None:
                process.kill()
                process.wait(timeout=5)
                if process.stdout is not None:
                    process.stdout.close()
