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

import numpy as np
import pytest

from dimos.memory.codecs.jpeg import JpegCodec
from dimos.memory.codecs.json import JsonCodec
from dimos.memory.codecs.lcm import LcmCodec
from dimos.memory.codecs.lz4 import Lz4Codec
from dimos.memory.store.mcap import McapStore
from dimos.memory.type.observation import Observation
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.std_msgs.String import String

mcap_writer = pytest.importorskip("mcap.writer", reason="mcap not installed")


@pytest.mark.parametrize("explicit", [False, True])
def test_lcm_channel_decodes_from_native_metadata(tmp_path: Path, explicit: bool) -> None:
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

    with McapStore(path=str(path), codecs={"imu": LcmCodec(Imu)} if explicit else None) as store:
        assert store.list_streams() == ["imu"]
        observation: Observation[Imu] = store.stream("imu").order_by("ts").first()
        assert observation.ts == 11.5
        assert observation.data.frame_id == "earlier"

        latest_observation: Observation[Imu] = store.stream("imu").order_by("ts", desc=True).first()
        assert latest_observation.ts == 12.5
        assert latest_observation.data.lcm_encode() == expected.lcm_encode()


@pytest.mark.parametrize("explicit", [False, True])
def test_wrapped_codec_decodes_from_native_metadata(tmp_path: Path, explicit: bool) -> None:
    path = tmp_path / "recording.mcap"
    expected = Imu(
        ts=12.5,
        frame_id="imu_link",
        angular_velocity=Vector3(1.0, 2.0, 3.0),
    )
    codec = Lz4Codec(LcmCodec(Imu))
    with path.open("wb") as output:
        writer = mcap_writer.Writer(output)
        writer.start(profile="dimos", library="test")
        channel_id = writer.register_channel(
            topic="imu",
            message_encoding="lz4+lcm",
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
            data=codec.encode(expected),
        )
        writer.finish()

    with McapStore(path=str(path), codecs={"imu": codec} if explicit else None) as store:
        observation: Observation[Imu] = store.stream("imu").first()
        assert observation.ts == 12.5
        assert observation.data.lcm_encode() == expected.lcm_encode()


def test_self_describing_jpeg_channel_decodes_without_a_codec_registry(tmp_path: Path) -> None:
    path = tmp_path / "recording.mcap"
    expected = Image(
        data=np.full((8, 8, 3), [20, 80, 140], dtype=np.uint8),
        format=ImageFormat.RGB,
        frame_id="camera",
        ts=12.5,
    )
    codec = JpegCodec()
    with path.open("wb") as output:
        writer = mcap_writer.Writer(output)
        writer.start(profile="dimos", library="test")
        channel_id = writer.register_channel(
            topic="color_image",
            message_encoding="jpeg",
            schema_id=0,
            metadata={
                "dimos.payload_type": "dimos.msgs.sensor_msgs.Image.Image",
                "dimos.observation_time": "publish_time",
            },
        )
        writer.add_message(
            channel_id=channel_id,
            log_time=13_000_000_000,
            publish_time=12_500_000_000,
            data=codec.encode(expected),
        )
        writer.finish()

    with McapStore(path=str(path)) as store:
        observation: Observation[Image] = store.stream("color_image").first()
        decoded = observation.data
        assert observation.ts == 12.5
        assert decoded.frame_id == "camera"
        assert decoded.format is ImageFormat.RGB
        assert decoded.data.shape == expected.data.shape
        assert np.mean(np.abs(decoded.data.astype(float) - expected.data.astype(float))) < 5


def test_lcm_metadata_does_not_import_payload_module(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    path = tmp_path / "untrusted.mcap"
    with path.open("wb") as output:
        writer = mcap_writer.Writer(output)
        writer.start(profile="dimos", library="test")
        channel_id = writer.register_channel(
            topic="untrusted",
            message_encoding="lcm",
            schema_id=0,
            metadata={"dimos.payload_type": "untrusted_module.Payload"},
        )
        writer.add_message(
            channel_id=channel_id,
            log_time=1,
            publish_time=1,
            data=b"raw payload",
        )
        writer.finish()

    def fail_import(name: str) -> None:
        raise AssertionError(f"artifact metadata imported {name!r}")

    monkeypatch.setattr("dimos.memory.codecs.base.importlib.import_module", fail_import)
    with McapStore(path=str(path)) as store:
        assert store.stream("untrusted").first().data == b"raw payload"


@pytest.mark.parametrize("encoding", ["pickle", "lcm", "lz4+lcm"])
@pytest.mark.parametrize(
    "payload_type", ["untrusted_module.Payload", "dimos.msgs.sensor_msgs.Imu.Other"]
)
def test_unknown_metadata_stays_raw(tmp_path, encoding, payload_type):
    path = tmp_path / "unknown.mcap"
    with path.open("wb") as output:
        writer = mcap_writer.Writer(output)
        writer.start(profile="dimos")
        channel = writer.register_channel(
            topic="raw",
            message_encoding=encoding,
            schema_id=0,
            metadata={"dimos.payload_type": payload_type},
        )
        writer.add_message(channel_id=channel, log_time=1, publish_time=1, data=b"untouched")
        writer.finish()
    with McapStore(path=str(path)) as store:
        assert store.streams.raw.first().data == b"untouched"
        assert "raw bytes" in store.summary()


def test_json_native_metadata_decodes_string(tmp_path):
    path = tmp_path / "events.mcap"
    value = String('{"ts":12.5,"event":"ready"}')
    with path.open("wb") as output:
        writer = mcap_writer.Writer(output)
        writer.start(profile="dimos")
        channel = writer.register_channel(
            topic="events",
            message_encoding="json",
            schema_id=0,
            metadata={
                "dimos.payload_type": "dimos.msgs.std_msgs.String.String",
                "dimos.observation_time": "publish_time",
            },
        )
        writer.add_message(
            channel_id=channel,
            log_time=20_000_000_000,
            publish_time=12_500_000_000,
            data=JsonCodec().encode(value),
        )
        writer.finish()
    with McapStore(path=str(path)) as store:
        observation = store.streams.events.first()
        assert isinstance(observation.data, String)
        assert observation.data.data == value.data
        assert observation.ts == 12.5


@pytest.mark.parametrize("conflicting", [False, True])
def test_duplicate_channel_counts_or_conflict(tmp_path, conflicting):
    path = tmp_path / "duplicates.mcap"
    with path.open("wb") as output:
        writer = mcap_writer.Writer(output)
        writer.start(profile="dimos")
        for encoding in ["lcm", "json" if conflicting else "lcm"]:
            channel = writer.register_channel(
                topic="imu",
                message_encoding=encoding,
                schema_id=0,
                metadata={"dimos.payload_type": "dimos.msgs.sensor_msgs.Imu.Imu"},
            )
            writer.add_message(
                channel_id=channel, log_time=1, publish_time=1, data=Imu(ts=0).lcm_encode()
            )
        writer.finish()
    if conflicting:
        with pytest.raises(ValueError, match="conflicting channel definitions"):
            McapStore(path=str(path))
    else:
        with McapStore(path=str(path)) as store:
            assert store.streams.imu.count() == 2
            assert len(store.streams.imu.to_list()) == 2


@pytest.mark.parametrize("changed_field", [None, "name", "encoding", "data"])
def test_duplicate_channels_compare_schema_content(tmp_path, changed_field):
    path = tmp_path / "schemas.mcap"
    with path.open("wb") as output:
        writer = mcap_writer.Writer(output)
        writer.start()
        for index in range(2):
            schema = {"name": "example", "encoding": "jsonschema", "data": b"{}"}
            if index == 1 and changed_field is not None:
                schema[changed_field] = b'{"type":"string"}' if changed_field == "data" else "other"
            schema_id = writer.register_schema(**schema)
            channel = writer.register_channel(
                topic="raw", message_encoding="custom", schema_id=schema_id
            )
            writer.add_message(
                channel_id=channel, log_time=index, publish_time=index, data=bytes([index])
            )
        writer.finish()
    if changed_field is not None:
        with pytest.raises(ValueError, match="conflicting channel definitions"):
            McapStore(path=str(path))
    else:
        with McapStore(path=str(path)) as store:
            assert store.streams.raw.count() == 2
            assert [observation.data for observation in store.streams.raw.to_list()] == [
                b"\x00",
                b"\x01",
            ]
