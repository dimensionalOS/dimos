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

"""Select recorded streams by compatible type, with explicit ambiguity errors."""

from dimos_generated.builtin_interfaces.msg import Time
from dimos_generated.geometry_msgs.msg import Quaternion, Transform, TransformStamped, Vector3
from dimos_generated.sensor_msgs.msg import Image, PointCloud2
from dimos_generated.std_msgs.msg import Header
from dimos_generated.tf2_msgs.msg import TFMessage
from dimos_message_build.registry import encode as cdr_encode, schema as cdr_schema
import numpy as np
import pytest
from typer.testing import CliRunner

from dimos.cli.commands.map import map_app
from dimos.mapping.cli.streams import select_stream
from dimos.msgs.pointcloud import pointcloud_from_xyz
from dimos.protocol.cdr_mcap import CdrMcapWriter


def recording(path, clouds, images=(), frame="world", transforms=()):
    cloud = pointcloud_from_xyz(
        np.array([[1.0, 2.0, 3.0]]), header=Header(frame_id=frame, stamp=Time(sec=0, nanosec=0))
    )
    image = Image(
        header=Header(frame_id="camera", stamp=Time(sec=0, nanosec=0)),
        height=1,
        width=1,
        encoding="rgb8",
        step=3,
        data=np.array([255, 0, 0], dtype=np.uint8),
        is_bigendian=0,
    )
    with CdrMcapWriter(path) as writer:
        for name, value in (
            [(n, cloud) for n in clouds]
            + [(n, image) for n in images]
            + ([("tf", TFMessage(transforms=list(transforms)))] if transforms else [])
        ):
            writer.write(
                name,
                cdr_encode(value),
                schema_name=value.__msgtype__,
                schema=cdr_schema(value.__msgtype__),
                log_time_ns=1_000_000_000,
                publish_time_ns=1_000_000_000,
            )


def test_unique_type_and_optional_missing_stream():
    assert (
        select_stream({"front_cloud": PointCloud2, "camera": Image}, PointCloud2, None, "--lidar")
        == "front_cloud"
    )
    assert (
        select_stream({"front_cloud": PointCloud2}, Image, None, "--image", required=False) is None
    )


@pytest.mark.parametrize("command", ["global", "replay", "replay-marker"])
def test_multiple_clouds_require_selection_even_when_one_is_named_lidar(tmp_path, command):
    source, output = tmp_path / "source.mcap", tmp_path / "out.rrd"
    recording(source, ["lidar", "second_cloud"], ["camera"])
    result = CliRunner().invoke(map_app, [command, str(source), "--no-gui", "--out", str(output)])
    assert result.exit_code == 2, result.output
    assert "Multiple compatible streams: lidar, second_cloud" in result.output
    assert "--lidar" in result.output
    assert not output.exists()


def test_cloud_only_replay_selects_unique_arbitrary_name(tmp_path):
    source, output = tmp_path / "source.mcap", tmp_path / "out.rrd"
    recording(source, ["front_cloud"])
    result = CliRunner().invoke(map_app, ["replay", str(source), "--no-gui", "--out", str(output)])
    assert result.exit_code == 0, result.output
    assert "[1/1]" in result.output
    assert output.stat().st_size > 0


def test_explicit_cloud_and_image_disambiguate_actual_replay(tmp_path):
    source, output = tmp_path / "source.mcap", tmp_path / "out.rrd"
    recording(source, ["front", "rear"], ["left_camera", "right_camera"])
    result = CliRunner().invoke(
        map_app,
        [
            "replay",
            str(source),
            "--lidar",
            "rear",
            "--image",
            "right_camera",
            "--no-gui",
            "--out",
            str(output),
        ],
    )
    assert result.exit_code == 0, result.output
    assert output.stat().st_size > 0


@pytest.mark.parametrize("selected", ["missing", "camera"])
def test_invalid_explicit_cloud_fails_before_output(tmp_path, selected):
    source, output = tmp_path / "source.mcap", tmp_path / "out.rrd"
    recording(source, ["front"], ["camera"])
    result = CliRunner().invoke(
        map_app, ["replay", str(source), "--lidar", selected, "--no-gui", "--out", str(output)]
    )
    assert result.exit_code == 2, result.output
    assert "not a compatible stream" in result.output
    assert not output.exists()


def test_multiple_images_require_selection(tmp_path):
    source, output = tmp_path / "source.mcap", tmp_path / "out.rrd"
    recording(source, ["front"], ["left", "right"])
    result = CliRunner().invoke(map_app, ["replay", str(source), "--no-gui", "--out", str(output)])
    assert result.exit_code == 2, result.output
    assert "Multiple compatible streams: left, right" in result.output
    assert "--image" in result.output
    assert not output.exists()


@pytest.mark.parametrize("command,extra", [("global", ["--markers"]), ("replay-marker", [])])
def test_marker_commands_reject_ambiguous_images_before_output(tmp_path, command, extra):
    source, output = tmp_path / "source.mcap", tmp_path / "out.rrd"
    recording(source, ["cloud"], ["left", "right"])
    result = CliRunner().invoke(
        map_app, [command, str(source), *extra, "--no-gui", "--out", str(output)]
    )
    assert result.exit_code == 2, result.output
    assert "Multiple compatible streams: left, right" in result.output
    assert not output.exists()


def test_world_cloud_global_needs_no_pose(tmp_path):
    source, output = tmp_path / "source.mcap", tmp_path / "out.rrd"
    recording(source, ["cloud"])
    result = CliRunner().invoke(
        map_app,
        [
            "global",
            str(source),
            "--device",
            "CPU:0",
            "--block-count",
            "100",
            "--no-gui",
            "--out",
            str(output),
        ],
    )
    assert result.exit_code == 0, result.output
    assert "kept [1/1]" in result.output
    assert output.stat().st_size > 0


@pytest.mark.parametrize("option", [["--pgo-tol", "0.3"], ["--pgo"]])
def test_world_cloud_trajectory_options_require_pose(tmp_path, option):
    source, output = tmp_path / "source.mcap", tmp_path / "out.rrd"
    recording(source, ["cloud"])
    result = CliRunner().invoke(
        map_app,
        ["global", str(source), "--device", "CPU:0", "--no-gui", "--out", str(output), *option],
    )
    assert result.exit_code == 2, result.output
    assert "trajectory" in result.output
    assert not output.exists()


@pytest.mark.parametrize("with_tf", [False, True])
def test_sensor_cloud_requires_real_registration_tf(tmp_path, with_tf):
    source, output = tmp_path / "source.mcap", tmp_path / "out.rrd"
    tf = TransformStamped(
        header=Header(frame_id="world", stamp=Time(sec=0, nanosec=0)),
        child_frame_id="sensor",
        transform=Transform(
            translation=Vector3(x=2.0, y=0.0, z=0.0),
            rotation=Quaternion(w=1.0, x=0.0, y=0.0, z=0.0),
        ),
    )
    recording(source, ["cloud"], frame="sensor", transforms=[tf] if with_tf else [])
    result = CliRunner().invoke(
        map_app,
        [
            "global",
            str(source),
            "--frame",
            "world",
            "--pgo-tol",
            "0.3",
            "--device",
            "CPU:0",
            "--block-count",
            "100",
            "--no-gui",
            "--out",
            str(output),
        ],
    )
    assert result.exit_code == (0 if with_tf else 2), result.output
    assert output.exists() == with_tf
    if with_tf:
        assert "kept [1/1]" in result.output
    else:
        assert "cannot register" in result.output
