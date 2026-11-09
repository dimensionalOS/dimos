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

from __future__ import annotations

from dataclasses import asdict
import json
import warnings

from dimos_cdrgen.registry import encode, schema
from mcap.writer import Writer
import numpy as np
import pytest
from typer.testing import CliRunner

from dimos.memory.cli.app import mem_app
from dimos.memory.cli.dataset import open_dataset, open_store
from dimos.memory.replay_module import stream_types_of
from dimos.memory.store.mcap import McapStore
from dimos.message_utils import time as message_time
from dimos.messages.geometry_msgs.msg import Quaternion, Vector3
from dimos.messages.sensor_msgs.msg import Imu
from dimos.messages.std_msgs.msg import Header, String
from dimos.robot.unitree.go2.dds.msgs.ControlEvent import ControlEvent
from dimos.robot.unitree.go2.dds.msgs.Telemetry import Telemetry
from dimos.robot.unitree.go2.dds.store import Go2McapStore


def _imu(ts: float, frame_id: str) -> Imu:
    """Generated Imu with the zero defaults the retired wrapper supplied."""
    return Imu(
        header=Header(stamp=message_time.time_from_seconds(ts), frame_id=frame_id),
        orientation=Quaternion(0.0, 0.0, 0.0, 1.0),
        orientation_covariance=np.zeros(9, dtype=np.float64),
        angular_velocity=Vector3(0.0, 0.0, 0.0),
        angular_velocity_covariance=np.zeros(9, dtype=np.float64),
        linear_acceleration=Vector3(0.0, 0.0, 0.0),
        linear_acceleration_covariance=np.zeros(9, dtype=np.float64),
    )


def _imu_channel(writer: Writer, topic: str, metadata: dict[str, str]) -> int:
    return writer.register_channel(
        topic=topic,
        message_encoding="cdr",
        schema_id=writer.register_schema(
            name=Imu.__msgtype__,
            encoding="ros2msg",
            data=schema(Imu.__msgtype__).encode(),
        ),
        metadata=metadata,
    )


def test_native_mcap_cli_and_replay_resolve_typed_stream(tmp_path):
    path = tmp_path / "native.mcap"
    value = _imu(12.5, "imu_link")
    with path.open("wb") as output:
        writer = Writer(output)
        writer.start(profile="dimos")
        channel = _imu_channel(
            writer,
            "imu",
            {"dimos.payload_type": Imu.__msgtype__, "dimos.observation_time": "publish_time"},
        )
        writer.add_message(
            channel_id=channel,
            log_time=20_000_000_000,
            publish_time=12_500_000_000,
            data=encode(value),
        )
        writer.finish()

    with warnings.catch_warnings(record=True) as emitted:
        warnings.simplefilter("always", DeprecationWarning)
        with open_dataset(path) as store:
            assert type(store) is McapStore
            assert encode(store.streams.imu.first().data) == encode(value)
            assert store.streams.imu.first().ts == 12.5
        assert not [item for item in emitted if item.category is DeprecationWarning]
    assert stream_types_of(str(path)) == {"imu": Imu}
    result = CliRunner().invoke(mem_app, ["summary", str(path)])
    assert result.exit_code == 0, result.output
    assert "raw bytes" not in result.output
    assert "imu" in result.output


@pytest.mark.parametrize("with_dds", [False, True])
def test_legacy_go2_aliases_json_and_log_time_remain_available(tmp_path, with_dds):
    path = tmp_path / "legacy.mcap"
    with path.open("wb") as output:
        writer = Writer(output)
        writer.start(profile="ros2")
        # A known DDS channel selects the legacy preset. Its empty stream must
        # still retain its alias; the JSON channel proves decoding and timing.
        if with_dds:
            writer.register_channel(topic="rt/utlidar/imu", message_encoding="cdr", schema_id=0)
        channel = writer.register_channel(topic="control_log", message_encoding="json", schema_id=0)
        writer.add_message(
            channel_id=channel,
            log_time=20_000_000_000,
            publish_time=12_500_000_000,
            data=b'{"type":"velocity_input","lx":0.5,"extra":"preserved in original"}',
        )
        writer.finish()

    with open_dataset(path) as store:
        assert type(store) is McapStore
        assert store.list_streams() == (["control_log", "imu"] if with_dds else ["control_log"])
        observation = store.streams.control_log.first()
        assert observation.ts == 20.0
        assert observation.data == ControlEvent(type="velocity_input", lx=0.5)
    with pytest.warns(DeprecationWarning, match="open_dataset"):
        with Go2McapStore(path=str(path)) as old:
            assert old.streams.control_log.first().data == observation.data


def test_mixed_native_channel_is_not_overridden_by_go2_preset(tmp_path):
    path = tmp_path / "mixed.mcap"
    value = _imu(12.5, "native_imu")
    with path.open("wb") as output:
        writer = Writer(output)
        writer.start(profile="dimos")
        writer.register_channel(topic="rt/lowstate", message_encoding="cdr", schema_id=0)
        channel = _imu_channel(writer, "imu", {"dimos.payload_type": Imu.__msgtype__})
        writer.add_message(channel_id=channel, log_time=1, publish_time=1, data=encode(value))
        writer.finish()
    with open_dataset(path) as store:
        decoded = store.stream("imu").first().data
        assert isinstance(decoded, Imu)
        assert encode(decoded) == encode(value)


@pytest.mark.parametrize("profile", ["dimos", "ros2"])
@pytest.mark.parametrize("opener", [open_dataset, open_store])
def test_mixed_recording_preserves_legacy_json_types(tmp_path, profile, opener):
    path = tmp_path / "mixed-events.mcap"
    value = _imu(12.5, "native_imu")
    expected_telemetry = Telemetry(
        type="telemetry",
        battery=0.785,
        body_h=0.3,
        current_a=1.0,
        imu_hz=100.0,
        lidar=True,
        lidar_hz=10.0,
        lowstate_hz=50.0,
        mode=0,
        obstacle=False,
        odom_hz=50.0,
        points_per_s=1000.0,
        rage=False,
        recording={"active": False},
        rss_mb=128.0,
        sportmode_hz=50.0,
        vel=[0.0, 0.0],
        yaw=0.0,
    )
    with path.open("wb") as output:
        writer = Writer(output)
        writer.start(profile=profile)
        native = _imu_channel(writer, "imu", {"dimos.payload_type": Imu.__msgtype__})
        writer.add_message(channel_id=native, log_time=1, publish_time=1, data=encode(value))
        for topic, payload in [
            ("telemetry", json.dumps(asdict(expected_telemetry)).encode()),
            ("control_log", b'{"type":"velocity_input","lx":0.5}'),
        ]:
            channel = writer.register_channel(topic=topic, message_encoding="json", schema_id=0)
            writer.add_message(
                channel_id=channel,
                log_time=20_000_000_000,
                publish_time=12_500_000_000,
                data=payload,
            )
        writer.finish()

    with opener(path) as store:
        imu = store.streams.imu.first().data
        assert isinstance(imu, Imu)
        assert encode(imu) == encode(value)
        telemetry = store.streams.telemetry.first()
        assert isinstance(telemetry.data, Telemetry)
        assert telemetry.data == expected_telemetry
        assert telemetry.data.battery == 0.785
        assert telemetry.ts == 20.0
        control = store.streams.control_log.first()
        assert isinstance(control.data, ControlEvent)
        assert control.data.lx == 0.5
        assert control.ts == 20.0


@pytest.mark.parametrize("topic", ["telemetry", "control_log"])
@pytest.mark.parametrize(
    ("encoding", "metadata", "expected_type"),
    [
        ("json", {"dimos.payload_type": String.__msgtype__}, String),
        ("json", {"dimos.payload_type": "unknown.Payload"}, bytes),
        ("custom", {}, bytes),
    ],
)
def test_legacy_json_selection_respects_encoding_and_metadata(
    tmp_path, topic, encoding, metadata, expected_type
):
    path = tmp_path / "not-legacy.mcap"
    payload = b'{"event":"ready"}'
    with path.open("wb") as output:
        writer = Writer(output)
        writer.start(profile="dimos")
        channel = writer.register_channel(
            topic=topic, message_encoding=encoding, schema_id=0, metadata=metadata
        )
        writer.add_message(channel_id=channel, log_time=1, publish_time=1, data=payload)
        writer.finish()

    with open_dataset(path) as store:
        decoded = store.stream(topic).first().data
        assert isinstance(decoded, expected_type)
        if expected_type is String:
            assert decoded.data == payload.decode()
        else:
            assert decoded == payload
