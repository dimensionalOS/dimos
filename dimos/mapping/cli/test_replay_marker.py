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
import rerun as rr
from rerun.experimental import RrdReader
import typer
from typer.testing import CliRunner

from dimos.mapping.cli.replay_marker import main
from dimos.memory.codecs.lcm import LcmCodec
from dimos.memory.store.sqlite import SqliteStore
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2


@pytest.fixture
def marker_app() -> Iterator[typer.Typer]:
    app = typer.Typer()
    app.command()(main)
    try:
        yield app
    finally:
        rr.rerun_shutdown()


@pytest.mark.parametrize(
    ("duration", "expected_entities"),
    [
        (0.5, {"/world/camera", "/world/robot"}),
        (0.0, {"/world/camera"}),
        (
            2.0,
            {
                "/world/camera",
                "/world/robot",
                "/world/camera/image",
                "/world/camera/image/detections",
            },
        ),
    ],
)
def test_replay_marker_uses_shared_window_for_offset_camera(
    tmp_path: Path, marker_app: typer.Typer, duration: float, expected_entities: set[str]
) -> None:
    source = tmp_path / "source.db"
    output = tmp_path / "markers.rrd"
    with SqliteStore(path=str(source)) as store:
        cloud = PointCloud2.from_numpy(
            np.array([[1.0, 2.0, 3.0]]), frame_id="world", timestamp=100.0
        )
        store.stream("lidar", PointCloud2).append(cloud, ts=100.0, pose=Transform())
        image = Image(data=np.zeros((8, 8, 3), dtype=np.uint8), format=ImageFormat.RGB)
        store.stream("color_image", Image, codec=LcmCodec(Image)).append(image, ts=101.0)

    result = CliRunner().invoke(
        marker_app,
        [str(source), "--out", str(output), "--duration", str(duration), "--no-gui"],
    )

    assert result.exit_code == 0, result.output
    reader = RrdReader(output)
    reader.store()  # A finalized recording has a readable footer.
    assert set(reader.stream().collect().schema().entity_paths()) == expected_entities
