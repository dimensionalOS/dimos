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

import re

from typer.testing import CliRunner

from dimos.memory.cli.app import mem_app
from dimos.memory.store.sqlite import SqliteStore


def test_summary_counts_attached_poses_per_stream(sqlite_store: SqliteStore) -> None:
    lidar = sqlite_store.stream("lidar", float)
    lidar.append(1.0, ts=1.0, pose=(1, 2, 3))
    lidar.append(2.0, ts=2.0, pose=(1, 2, 3))
    lidar.append(3.0, ts=3.0)
    sqlite_store.stream("cmd_vel", float).append(1.0, ts=1.0)

    result = CliRunner().invoke(mem_app, ["summary", sqlite_store.config.path])

    assert result.exit_code == 0, result.output
    # Rich colours the output when CI forces a terminal.
    lines = re.sub(r"\x1b\[[0-9;]*m", "", result.output).splitlines()
    header = next(line for line in lines if "Stream" in line)
    column = [cell.strip() for cell in header.split("┃")].index("Poses")
    poses = {}
    for name in ("lidar", "cmd_vel"):
        row = next(line for line in lines if f" {name} " in line)
        poses[name] = row.split("│")[column].strip()
    assert poses == {"lidar": "2", "cmd_vel": "—"}
