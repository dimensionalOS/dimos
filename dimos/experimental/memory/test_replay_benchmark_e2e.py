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

import json
from pathlib import Path
import subprocess
import sys

import pytest

pytestmark = pytest.mark.self_hosted_large


def test_replay_benchmark_smoke_writes_all_artifacts(tmp_path: Path) -> None:
    subprocess.run(
        [
            sys.executable,
            "-m",
            "dimos.experimental.memory.tool_replay_benchmark",
            "--dataset",
            "go2_short",
            "--seek",
            "5",
            "--duration",
            "0.5",
            "--repeats",
            "1",
            "--profile",
            "isolated:4",
            "--transport",
            "zenoh",
            "--out",
            str(tmp_path),
        ],
        check=True,
    )

    for name in ("samples.jsonl", "runs.json", "summary.json", "report.md"):
        assert (tmp_path / name).is_file()
    summary = json.loads((tmp_path / "summary.json").read_text())
    assert len(summary["aggregates"]) == 2
    assert {item["engine"] for item in summary["aggregates"]} == {"python", "rust"}
    assert all(item["median_cpu_pct"] > 0 for item in summary["aggregates"])
