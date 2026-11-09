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

from mcap.writer import Writer
import pytest
from typer.testing import CliRunner

from dimos.memory.cli.app import mem_app
from dimos.memory.cli.dataset import open_dataset, open_store
from dimos.memory.replay_module import stream_types_of
from dimos.memory.store.mcap import McapStore
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.std_msgs.String import String
from dimos.robot.unitree.go2.dds.msgs.ControlEvent import ControlEvent
from dimos.robot.unitree.go2.dds.msgs.Telemetry import Telemetry
from dimos.robot.unitree.go2.dds.store import Go2McapStore


def test_native_mcap_cli_and_replay_resolve_typed_stream(tmp_path):
    path = tmp_path / "native.mcap"
    value = Imu(ts=12.5, frame_id="imu_link")
    with path.open("wb") as output:
        writer = Writer(output)
        writer.start(profile="dimos")
        channel = writer.register_channel(
            topic="imu",
            message_encoding="lcm",
            schema_id=0,
            metadata={
                "dimos.payload_type": "dimos.msgs.sensor_msgs.Imu.Imu",
                "dimos.observation_time": "publish_time",
            },
        )
        writer.add_message(
            channel_id=channel,
            log_time=20_000_000_000,
            publish_time=12_500_000_000,
            data=value.lcm_encode(),
        )
        writer.finish()

    with warnings.catch_warnings(record=True) as emitted:
        warnings.simplefilter("always", DeprecationWarning)
        with open_dataset(path) as store:
            assert type(store) is McapStore
            assert store.streams.imu.first().data.lcm_encode() == value.lcm_encode()
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
    value = Imu(ts=12.5, frame_id="native_imu")
    with path.open("wb") as output:
        writer = Writer(output)
        writer.start(profile="dimos")
        writer.register_channel(topic="rt/lowstate", message_encoding="cdr", schema_id=0)
        channel = writer.register_channel(
            topic="rt/utlidar/imu",
            message_encoding="lcm",
            schema_id=0,
            metadata={"dimos.payload_type": "dimos.msgs.sensor_msgs.Imu.Imu"},
        )
        writer.add_message(channel_id=channel, log_time=1, publish_time=1, data=value.lcm_encode())
        writer.finish()
    with open_dataset(path) as store:
        decoded = store.stream("utlidar_imu").first().data
        assert isinstance(decoded, Imu)
        assert decoded.lcm_encode() == value.lcm_encode()


@pytest.mark.parametrize("profile", ["dimos", "ros2"])
@pytest.mark.parametrize("opener", [open_dataset, open_store])
def test_mixed_recording_preserves_legacy_json_types(tmp_path, profile, opener):
    path = tmp_path / "mixed-events.mcap"
    value = Imu(ts=12.5, frame_id="native_imu")
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
        native = writer.register_channel(
            topic="imu",
            message_encoding="lcm",
            schema_id=0,
            metadata={"dimos.payload_type": "dimos.msgs.sensor_msgs.Imu.Imu"},
        )
        writer.add_message(channel_id=native, log_time=1, publish_time=1, data=value.lcm_encode())
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
        assert imu.lcm_encode() == value.lcm_encode()
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
        ("json", {"dimos.payload_type": "dimos.msgs.std_msgs.String.String"}, String),
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
