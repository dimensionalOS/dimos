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

"""Run the exact offline example and inspect its real RRD with the SDK's CLI."""

from pathlib import Path
import subprocess
import sys

import pytest


def test_actual_loopback_example_preserves_pose_axes_values_and_nanoseconds(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    # The pinned Rerun CLI disables analytics when CI is present. This changes
    # only the child environment, not the user's persistent viewer settings.
    monkeypatch.setenv("CI", "1")
    monkeypatch.delenv("FORCE_RERUN_ANALYTICS", raising=False)
    example = Path(__file__).resolve().parents[3] / "examples/message-viewer/demo_cdr_rerun.py"
    output = tmp_path / "pose.rrd"
    command = [sys.executable, "-I", str(example), str(output)]
    subprocess.run(command, check=True, capture_output=True, text=True, timeout=30)
    contents = output.read_bytes()
    assert contents.startswith(b"RRF")
    rerun = Path(sys.executable).parent / "rerun"
    result = subprocess.run(
        [str(rerun), "rrd", "print", "-vvv", str(output)],
        check=True,
        capture_output=True,
        text=True,
        timeout=30,
    )
    assert "TransformAxes3D:axis_length" in result.stdout
    assert "[[1.5, -2.5, 0.25]]" in result.stdout
    assert "[[0.0, 0.0, 0.6, 0.8]]" in result.stdout
    assert "tf#/map" in result.stdout
    assert "2023-11-14T22:13:20.123456789" in result.stdout
    assert '"nanosec": 123456789' in result.stdout
    assert "Exact message values" in result.stdout
    assert "Points3D:positions" in result.stdout
    assert "Points3D:colors" in result.stdout
    assert "Image:buffer" in result.stdout
    assert "Generated RGB pixels" in result.stdout
    repeated = subprocess.run(command, capture_output=True, text=True, timeout=30)
    assert repeated.returncode != 0
    assert "FileExistsError" in repeated.stderr
    assert output.read_bytes() == contents
