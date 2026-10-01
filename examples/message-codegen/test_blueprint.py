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

"""Run the same three-language blueprint users launch from the demo command."""

import os
from pathlib import Path
import signal
import socket
import subprocess
import sys

import pytest

_ROOT = Path(__file__).resolve().parents[2]


@pytest.mark.native_e2e
@pytest.mark.parametrize("backend", ["lcm", "zenoh"])
def test_three_language_blueprint_preserves_changing_messages(backend, tmp_path):
    for executable in ("build/native-cpp-examples/cdr_relay", "target/debug/cdr_relay"):
        if not (_ROOT / executable).is_file():
            pytest.skip("Build both native CDR relay examples first")
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as reservation:
        reservation.bind(("127.0.0.1", 0))
        port = reservation.getsockname()[1]
    env = {
        **os.environ,
        "LCM_DEFAULT_URL": f"udpm://239.255.76.67:{port}?ttl=0&recv_buf_size=4194304",
    }
    with subprocess.Popen(
        [
            sys.executable,
            str(Path(__file__).with_name("demo_blueprint.py")),
            "--transport",
            backend,
            "--samples",
            "3",
        ],
        cwd=_ROOT,
        env=env,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        start_new_session=True,
    ) as process:
        try:
            output, _ = process.communicate(timeout=75)
        except subprocess.TimeoutExpired:
            process.send_signal(signal.SIGINT)
            try:
                output, _ = process.communicate(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                output, _ = process.communicate(timeout=5)
            pytest.fail(output.decode(errors="replace"))
    transcript = output.decode(errors="replace")
    (tmp_path / f"blueprint-{backend}.log").write_text(transcript)
    assert process.returncode == 0, transcript
    for sample in range(3):
        assert (
            f"sample {sample}: Python weight={sample} → C++ {sample + 1} → Rust {sample + 2}; "
            f"Python verified 921,600 image bytes and stamp {1_700_000_000_123_456_789 + sample}"
        ) in transcript
    assert f"{backend}: three-language coordinator blueprint verified" in transcript
