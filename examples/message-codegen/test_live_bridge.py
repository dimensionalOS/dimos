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

"""Bound the headless CDR transport-to-viewer demonstration in CI."""

from pathlib import Path
import subprocess
import sys


def test_live_lcm_bridge_records_generated_image() -> None:
    demo = Path(__file__).with_name("demo_live_bridge.py")
    result = subprocess.run(
        [sys.executable, str(demo)], check=True, capture_output=True, text=True, timeout=30
    )
    assert "LCM CDR image received and rendered" in result.stdout
    recording = Path("build/message-codegen/demo/evidence/live-bridge.rrd")
    subprocess.run(
        [str(Path(sys.executable).with_name("rerun")), "rrd", "verify", str(recording)],
        check=True,
        capture_output=True,
        text=True,
        timeout=30,
    )
