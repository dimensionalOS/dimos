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
import os
from pathlib import Path
import subprocess
import sys

import pytest

from dimos.message_codegen.generate import generate
from dimos.message_codegen.ownership import Dependency


@pytest.mark.native_e2e
@pytest.mark.parametrize("backend", ["lcm", "zenoh"])
def test_external_schema_records_on_both_transports_with_unchanged_binary(tmp_path, backend):
    executable = Path(
        os.environ.get("DIMOS_NATIVE_RECORDER_BIN", "target/debug/dimos-memory-recorder")
    ).resolve()
    if not executable.is_file():
        pytest.skip("Build the native recorder before running the native_e2e recording gate")
    source = tmp_path / "interfaces/recording_msgs/msg/Reading.msg"
    source.parent.mkdir(parents=True)
    source.write_text("std_msgs/Header header\nfloat64 value\nstring application_note\n")
    spec = importlib.util.find_spec("dimos_generated_schemas")
    assert spec is not None and spec.origin is not None
    builtin = Dependency.load(Path(spec.origin).parent / "package")
    package = tmp_path / "generated"
    generate(
        [source.parents[2]],
        package,
        ["recording_msgs/msg/Reading"],
        "recording_messages",
        dependencies=(builtin,),
        languages=("python",),
    )
    site = tmp_path / "site"
    subprocess.run(
        [
            sys.executable,
            "-m",
            "pip",
            "install",
            "--no-deps",
            "--no-build-isolation",
            "--target",
            str(site),
            str(package / "python"),
        ],
        check=True,
        capture_output=True,
        text=True,
        timeout=60,
    )
    env = {
        **os.environ,
        "PYTHONPATH": os.pathsep.join(
            [str(site), str(Path.cwd()), os.environ.get("PYTHONPATH", "")]
        ),
    }
    result = subprocess.run(
        [
            sys.executable,
            str(Path(__file__).with_name("demo_native_recording.py")),
            "--executable",
            str(executable),
            "--transport",
            backend,
            "--output",
            str(tmp_path),
        ],
        capture_output=True,
        text=True,
        env=env,
        timeout=60,
    )
    assert result.returncode == 0, result.stdout + result.stderr
    assert f"{backend}: locally-added-field-2" in result.stdout
    assert "PASS:" in result.stdout
