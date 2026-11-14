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

from collections.abc import Iterator
from pathlib import Path

import numpy as np
import pytest
from pytest_mock import MockerFixture
import rerun as rr
from rerun.experimental import RrdReader
import typer
from typer.testing import CliRunner

from dimos.mapping.cli.map import main
from dimos.memory.store.sqlite import SqliteStore
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2


@pytest.fixture
def map_app() -> Iterator[typer.Typer]:
    app = typer.Typer()
    app.command()(main)
    try:
        yield app
    finally:
        rr.rerun_shutdown()


@pytest.mark.parametrize("pgo_option", ["--pgo", "--full-pgo", "--export-cloud"])
def test_map_pgo_finishes_when_window_contains_no_lidar(
    tmp_path: Path, map_app: typer.Typer, mocker: MockerFixture, pgo_option: str
) -> None:
    source = tmp_path / "source.db"
    output = tmp_path / "map.rrd"
    with SqliteStore(path=str(source)) as store:
        cloud = PointCloud2.from_numpy(
            np.array([[1.0, 2.0, 3.0]]), frame_id="world", timestamp=100.0
        )
        for ts in (100.0, 101.0, 102.0):
            store.stream("lidar", PointCloud2).append(
                cloud, ts=ts, pose=Transform(translation=Vector3(ts, 0.0, 0.0))
            )
        # Keep the CLI's source store under this test's resource ownership.
        mocker.patch("dimos.memory.cli.dataset.open_store", return_value=store)

        result = CliRunner().invoke(
            map_app,
            [
                str(source),
                "--out",
                str(output),
                "--seek",
                "0.25",
                "--duration",
                "0.1",
                "--device",
                "CPU:0",
                "--block-count",
                "16",
                "--no-gui",
                pgo_option,
            ],
        )

    assert result.exit_code == 0, result.output
    assert f"wrote {output}" in result.output
    RrdReader(output).store()  # The empty recording must still be finalized.
