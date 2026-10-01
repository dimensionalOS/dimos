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

import importlib.util
from pathlib import Path
import subprocess
import sys

import pytest


@pytest.mark.native_e2e
def test_external_schema_records_on_both_transports_with_unchanged_binary(tmp_path):
    executable = Path("target/debug/dimos-memory-recorder").resolve()
    if not executable.is_file() or importlib.util.find_spec("external_telemetry") is None:
        pytest.skip("Build the recorder and install the external message demo package first")
    result = subprocess.run(
        [
            sys.executable,
            str(Path(__file__).with_name("demo_native_recording.py")),
            "--executable",
            str(executable),
            "--output",
            str(tmp_path),
        ],
        capture_output=True,
        text=True,
        timeout=60,
    )
    assert result.returncode == 0, result.stdout + result.stderr
    assert "lcm: locally-added-field-2" in result.stdout
    assert "zenoh: locally-added-field-2" in result.stdout
    assert "PASS:" in result.stdout
