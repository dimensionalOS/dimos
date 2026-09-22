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

"""Exercise generated TF messages through real Python and Rust module workers."""

import os
from pathlib import Path
import re
import selectors
import signal
import socket
import subprocess
import sys
import time

import pytest

_ROOT = Path(__file__).resolve().parents[2]
_LOOKUP = re.compile(r"tf lookup .* x=(\S+) y=(\S+) z=(\S+)\n")


@pytest.mark.native_e2e
@pytest.mark.parametrize("backend", ["lcm", "zenoh"])
def test_python_and_rust_compose_generated_tf_through_coordinator(backend, tmp_path):
    for executable in ("tf_broadcaster", "tf_listener"):
        if not (_ROOT / "target" / "release" / executable).is_file():
            pytest.skip("Build dimos-native-module-examples with cargo build --release first")

    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as reservation:
        reservation.bind(("127.0.0.1", 0))
        port = reservation.getsockname()[1]
    env = {
        **os.environ,
        "DIMOS_TRANSPORT": backend,
        "LCM_DEFAULT_URL": f"udpm://239.255.76.67:{port}?ttl=0",
    }
    output = bytearray()
    samples = []
    with subprocess.Popen(
        [sys.executable, str(Path(__file__).with_name("rust_tf.py"))],
        cwd=_ROOT,
        env=env,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        start_new_session=True,
    ) as process:
        try:
            assert process.stdout is not None
            with selectors.DefaultSelector() as selector:
                selector.register(process.stdout, selectors.EVENT_READ)
                deadline = time.monotonic() + 45
                while len(samples) < 4 and time.monotonic() < deadline:
                    if not selector.select(timeout=max(0, deadline - time.monotonic())):
                        break
                    chunk = os.read(process.stdout.fileno(), 65536)
                    if not chunk:
                        break
                    output.extend(chunk)
                    samples = [
                        tuple(map(float, match.groups()))
                        for match in _LOOKUP.finditer(output.decode(errors="replace"))
                    ]
        finally:
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
            try:
                remaining, _ = process.communicate(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                remaining, _ = process.communicate(timeout=5)
            output.extend(remaining)
            (tmp_path / f"tf-{backend}.log").write_bytes(output)

    transcript = output.decode(errors="replace")
    assert len(samples) >= 4, transcript
    for x, y, z in samples:
        assert x == pytest.approx(1.5)
        assert y * y + z * z == pytest.approx(1.0)
    assert max(sample[1] for sample in samples) - min(sample[1] for sample in samples) > 0.1
    assert process.returncode == 0, transcript
