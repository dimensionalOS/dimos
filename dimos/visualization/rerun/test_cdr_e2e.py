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

"""Run the reviewer example and inspect its live bridge and native MCAP importer."""

from pathlib import Path
import subprocess
import sys

import pytest


def test_live_bridge_and_native_mcap_import_cover_scene(
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
    for marker in (
        "TransformAxes3D:axis_length",
        "[[1.5, -2.5, 0.25]]",
        "[[0.0, 0.0, 0.6, 0.8]]",
        "tf#/map",
        "tf#/camera",
        "2023-11-14T22:13:20.123456789",
        '"nanosec": 123456789',
        "Exact message values",
        "Points3D:positions",
        "Points3D:colors",
        "Boxes3D:centers",
        "Image:buffer",
        "Pinhole:image_from_camera",
        "DepthImage:meter",
        "EncodedImage:blob",
        "world/path",
        "world/odometry",
        "world/entities",
        "demo person",
        "world/legacy_pose",
        "world/late_image",
        "world/cloud_boxes",
        "world/cloud_height",
        "Generated RGB pixels",
    ):
        assert marker in result.stdout, marker
    recording = output.with_suffix(".mcap")
    assert recording.read_bytes().startswith(b"\x89MCAP")
    native = output.with_name(output.stem + "-native.rrd")
    subprocess.run(
        [str(rerun), "mcap", "convert", str(recording), "-o", str(native)],
        check=True,
        capture_output=True,
        text=True,
        timeout=30,
    )
    imported = subprocess.run(
        [str(rerun), "rrd", "print", "-vvv", str(native)],
        check=True,
        capture_output=True,
        text=True,
        timeout=30,
    )
    for marker in (
        "/pose",
        "/tf",
        "Image:buffer",
        "Points3D:positions",
        "Pinhole:image_from_camera",
        "InstancePoses3D",
        "2023-11-14T22:13:20.123456789",
    ):
        assert marker in imported.stdout, marker
    repeated = subprocess.run(command, capture_output=True, text=True, timeout=30)
    assert repeated.returncode != 0
    assert "FileExistsError" in repeated.stderr
    assert output.read_bytes() == contents
