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

"""Integration test: C++ robot-control binary talks to Python simplerobot via LCM."""

from __future__ import annotations

from pathlib import Path
import subprocess

import pytest

pytestmark = pytest.mark.interop


def test_cpp_receives_pose_and_publishes_twist(
    simplerobot: subprocess.Popen[str],
    cpp_binary: Path,
) -> None:
    """Run the C++ binary for a few seconds and verify message exchange."""
    try:
        result = subprocess.run(
            [str(cpp_binary)],
            capture_output=True,
            text=True,
            timeout=5,
        )
    except subprocess.TimeoutExpired as e:
        stdout = e.stdout or ""
        stderr = e.stderr or ""
    else:
        stdout = result.stdout
        stderr = result.stderr

    assert "[pose]" in stdout, (
        f"C++ binary never received a PoseStamped.\nstdout: {stdout!r}\nstderr: {stderr!r}"
    )
    assert "[twist]" in stdout, (
        f"C++ binary never published a Twist.\nstdout: {stdout!r}\nstderr: {stderr!r}"
    )
