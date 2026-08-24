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

from pathlib import Path

import pytest

from dimos.memory.store.mcap import McapStore
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.Imu import Imu

mcap_writer = pytest.importorskip("mcap.writer", reason="mcap not installed")


def test_self_describing_lcm_channel_decodes_without_a_codec_registry(tmp_path: Path) -> None:
    path = tmp_path / "recording.mcap"
    expected = Imu(
        ts=12.5,
        frame_id="imu_link",
        angular_velocity=Vector3(1.0, 2.0, 3.0),
    )
    with path.open("wb") as output:
        writer = mcap_writer.Writer(output)
        writer.start(profile="dimos", library="test")
        channel_id = writer.register_channel(
            topic="imu",
            message_encoding="lcm",
            schema_id=0,
            metadata={
                "dimos.payload_type": "dimos.msgs.sensor_msgs.Imu.Imu",
                "dimos.observation_time": "publish_time",
            },
        )
        writer.add_message(
            channel_id=channel_id,
            log_time=13_000_000_000,
            publish_time=12_500_000_000,
            data=expected.lcm_encode(),
        )
        writer.add_message(
            channel_id=channel_id,
            log_time=14_000_000_000,
            publish_time=11_500_000_000,
            data=Imu(ts=11.5, frame_id="earlier").lcm_encode(),
        )
        writer.finish()

    with McapStore(path=str(path)) as store:
        assert store.list_streams() == ["imu"]
        observation = store.stream("imu").order_by("ts").first()
        assert observation.ts == 11.5
        assert observation.data.frame_id == "earlier"

        observation = store.stream("imu").order_by("ts", desc=True).first()
        assert observation.ts == 12.5
        assert observation.data.lcm_encode() == expected.lcm_encode()
