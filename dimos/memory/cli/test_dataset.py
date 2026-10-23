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

import warnings

from mcap.writer import Writer
import pytest
from typer.testing import CliRunner

from dimos.memory.cli.app import mem_app
from dimos.memory.cli.dataset import open_dataset
from dimos.memory.replay_module import stream_types_of
from dimos.memory.store.mcap import McapStore
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.robot.unitree.go2.dds.msgs.ControlEvent import ControlEvent
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
