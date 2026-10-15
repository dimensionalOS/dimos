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
import sys

import pytest


@pytest.mark.native_e2e
def test_generated_whole_body_feedback_and_commands_on_both_transports():
    result = subprocess.run(
        [sys.executable, str(Path(__file__).with_name("demo_whole_body_transport.py"))],
        capture_output=True,
        text=True,
        timeout=30,
    )
    assert result.returncode == 0, result.stdout + result.stderr
    assert "PASS: lcm whole-body generated CDR commands and feedback" in result.stdout
    assert "PASS: zenoh whole-body generated CDR commands and feedback" in result.stdout
