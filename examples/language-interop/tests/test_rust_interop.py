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

"""Integration test: Rust robot-control binary talks to Python simplerobot via LCM."""

from __future__ import annotations

import os
from pathlib import Path
import subprocess

import pytest

pytestmark = pytest.mark.interop


def _run_rust_binary(rust_binary: Path, timeout: int = 5) -> tuple[str, str]:
    """Run the Rust binary and return (stdout, stderr)."""
    try:
        result = subprocess.run(
            [str(rust_binary)],
            capture_output=True,
            text=True,
            timeout=timeout,
        )
        return result.stdout, result.stderr
    except subprocess.TimeoutExpired as e:
        # text=True means stdout/stderr are str, but stubs type them as bytes | str | None
        return str(e.stdout or ""), str(e.stderr or "")


def test_rust_binary_publishes_twist(
    simplerobot: subprocess.Popen[str],
    rust_binary: Path,
) -> None:
    """Rust binary starts up and publishes Twist commands."""
    stdout, stderr = _run_rust_binary(rust_binary)

    assert "[twist]" in stdout, (
        f"Rust binary never published a Twist.\nstdout: {stdout!r}\nstderr: {stderr!r}"
    )


@pytest.mark.skipif(
    os.environ.get("CI_NO_MULTICAST") is not None,
    reason="multicast unavailable",
)
def test_rust_binary_receives_pose(
    simplerobot: subprocess.Popen[str],
    rust_binary: Path,
) -> None:
    """Rust binary receives PoseStamped from simplerobot via LCM multicast.

    This test requires working UDP multicast between processes.
    """
    stdout, stderr = _run_rust_binary(rust_binary)

    assert "[pose]" in stdout, (
        f"Rust binary never received a PoseStamped from simplerobot.\n"
        f"stdout: {stdout!r}\nstderr: {stderr!r}"
    )
