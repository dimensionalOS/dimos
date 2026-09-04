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
from dimos.memory.codecs.lcm import LcmCodec
from dimos.memory.codecs.lz4 import Lz4Codec
from dimos.memory.store.mcap import McapStore
from dimos.memory.type.observation import Observation
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.msgs.sensor_msgs.Imu import Imu

mcap_writer = pytest.importorskip("mcap.writer", reason="mcap not installed")


def test_lcm_channel_decodes_with_explicit_codec(tmp_path: Path) -> None:
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

    with McapStore(path=str(path), codecs={"imu": LcmCodec(Imu)}) as store:
        assert store.list_streams() == ["imu"]
        observation: Observation[Imu] = store.stream("imu").order_by("ts").first()
        assert observation.ts == 11.5
        assert observation.data.frame_id == "earlier"

        latest_observation: Observation[Imu] = store.stream("imu").order_by("ts", desc=True).first()
        assert latest_observation.ts == 12.5
        assert latest_observation.data.lcm_encode() == expected.lcm_encode()


def test_wrapped_codec_decodes_with_explicit_codec(tmp_path: Path) -> None:
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

    with McapStore(path=str(path), codecs={"imu": codec}) as store:
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
