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
import tracemalloc

from dimos_lcm.std_msgs import String as LcmString
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


@pytest.mark.parametrize("encoding", ["lcm", "lz4+lcm"])
def test_generated_lcm_channel_decodes_from_native_metadata(tmp_path, encoding):
    expected = LcmString("ready")
    path = tmp_path / "generated.mcap"
    payload_type = type(expected)
    codec = LcmCodec(payload_type)
    payload = Lz4Codec(codec).encode(expected) if encoding == "lz4+lcm" else codec.encode(expected)
    with path.open("wb") as output:
        writer = mcap_writer.Writer(output)
        writer.start(profile="dimos")
        channel = writer.register_channel(
            topic="status",
            message_encoding=encoding,
            schema_id=0,
            metadata={
                "dimos.payload_type": f"{payload_type.__module__}.{payload_type.__qualname__}",
                "dimos.observation_time": "publish_time",
            },
        )
        writer.add_message(
            channel_id=channel,
            log_time=13_000_000_000,
            publish_time=12_500_000_000,
            data=payload,
        )
        writer.finish()

    with McapStore(path=str(path)) as store:
        observation = store.streams.status.first()
        assert observation.ts == 12.5
        assert type(observation.data) is payload_type
        assert observation.data.data == expected.data
        assert "raw bytes" not in store.summary()


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


@pytest.mark.parametrize(
    ("encoding", "payload_type"),
    [
        ("lcm", "dimos.msgs.sensor_msgs.Imu.Other"),
        ("lcm", "dimos_lcm.std_msgs.Unknown.Unknown"),
        ("lcm", "dimos_lcm.untrusted_module.Payload.Payload"),
        ("lz4+lcm", "untrusted_module.Payload"),
        ("pickle", "dimos.msgs.sensor_msgs.Imu.Imu"),
    ],
)
def test_unsupported_metadata_stays_raw(tmp_path, encoding, payload_type):
    path = tmp_path / "unknown.mcap"
    payload = (
        Lz4Codec(LcmCodec(LcmString)).encode(LcmString("untouched"))
        if encoding == "lz4+lcm"
        else b"untouched"
    )
    with path.open("wb") as output:
        writer = mcap_writer.Writer(output)
        writer.start(profile="dimos")
        channel = writer.register_channel(
            topic="raw",
            message_encoding=encoding,
            schema_id=0,
            metadata={"dimos.payload_type": payload_type},
        )
        writer.add_message(channel_id=channel, log_time=1, publish_time=1, data=payload)
        writer.finish()
    with McapStore(path=str(path)) as store:
        assert store.streams.raw.first().data == payload
        assert "raw bytes" in store.summary()


@pytest.mark.parametrize(
    "operation", [lambda q: q.count(), lambda q: len(q.to_list())], ids=["count", "to_list"]
)
def test_near_warns_that_mcap_stores_no_poses(tmp_path, operation):
    path = tmp_path / "recording.mcap"
    with path.open("wb") as output:
        writer = mcap_writer.Writer(output)
        writer.start(profile="dimos")
        channel = writer.register_channel(
            topic="imu",
            message_encoding="lcm",
            schema_id=0,
            metadata={"dimos.payload_type": "dimos.msgs.sensor_msgs.Imu.Imu"},
        )
        writer.add_message(
            channel_id=channel, log_time=1, publish_time=1, data=Imu(ts=1.0).lcm_encode()
        )
        writer.finish()
    with McapStore(path=str(path)) as store:
        with pytest.warns(UserWarning, match="'imu' is read from MCAP.*no poses"):
            assert operation(store.streams.imu.near((0, 0, 0), 1.0)) == 0


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


@pytest.mark.parametrize("encoding", ["lcm", "lz4+lcm"])
def test_trusted_metadata_decodes_without_a_caller_codec(tmp_path, encoding):
    path = tmp_path / "native.mcap"
    expected = Imu(ts=2.5, frame_id="native")
    codec = LcmCodec(Imu)
    payload = Lz4Codec(codec).encode(expected) if encoding == "lz4+lcm" else codec.encode(expected)
    with path.open("wb") as output:
        writer = mcap_writer.Writer(output)
        writer.start()
        channel = writer.register_channel(
            topic="imu",
            message_encoding=encoding,
            schema_id=0,
            metadata={"dimos.payload_type": "dimos.msgs.sensor_msgs.Imu.Imu"},
        )
        writer.add_message(
            channel_id=channel,
            log_time=2_500_000_000,
            publish_time=2_500_000_000,
            data=payload,
        )
        writer.finish()
    with McapStore(path=str(path), decode_metadata=True) as store:
        observation = store.stream("imu").first()
        assert observation.ts == 2.5
        assert observation.data.lcm_encode() == expected.lcm_encode()


def test_explicit_codec_takes_precedence_over_metadata(tmp_path):
    path = tmp_path / "override.mcap"
    expected = Imu(ts=1, frame_id="explicit")
    with path.open("wb") as output:
        writer = mcap_writer.Writer(output)
        writer.start()
        channel = writer.register_channel(
            topic="imu",
            message_encoding="lcm",
            schema_id=0,
            metadata={"dimos.payload_type": "not_installed.Payload"},
        )
        writer.add_message(
            channel_id=channel,
            log_time=1_000_000_000,
            publish_time=1_000_000_000,
            data=expected.lcm_encode(),
        )
        writer.finish()
    with McapStore(path=str(path), codecs={"imu": LcmCodec(Imu)}, decode_metadata=True) as store:
        assert store.stream("imu").first().data.lcm_encode() == expected.lcm_encode()


def test_a_name_two_topics_fold_to_fails_only_when_read(tmp_path):
    path = tmp_path / "clash.mcap"
    with path.open("wb") as output:
        writer = mcap_writer.Writer(output)
        writer.start()
        for topic in ("rt/camera_info", "camera_info", "imu"):
            channel = writer.register_channel(topic=topic, message_encoding="cdr", schema_id=0)
            writer.add_message(channel_id=channel, log_time=1, publish_time=1, data=b"\0")
        writer.finish()
    with McapStore(path=str(path)) as store:
        assert store.list_streams() == ["imu"]
        assert store.stream("imu").count() == 1
        with pytest.raises(ValueError, match="pass streams="):
            store.stream("camera_info")


@pytest.mark.parametrize(
    ("native_times", "legacy_times", "expected"),
    [
        pytest.param(
            [(13.0, 12.5), (14.0, 11.5)], [(20.0, 1.0), (12.0, 2.0)], 11.5, id="native-first"
        ),
        pytest.param(
            [(13.0, 12.5), (14.0, 11.5)], [(20.0, 1.0), (10.0, 2.0)], 10.0, id="legacy-first"
        ),
        pytest.param([], [], None, id="empty"),
    ],
)
def test_first_ts_is_the_earliest_observation_time(
    tmp_path: Path,
    native_times: list[tuple[float, float]],
    legacy_times: list[tuple[float, float]],
    expected: float | None,
) -> None:
    path = tmp_path / "recording.mcap"
    with path.open("wb") as output:
        writer = mcap_writer.Writer(output)
        writer.start(profile="dimos", library="test")
        # A native channel is timed by publish_time, which can run against log order.
        native = writer.register_channel(
            topic="native",
            message_encoding="raw",
            schema_id=0,
            metadata={"dimos.observation_time": "publish_time"},
        )
        # A channel without the metadata is timed by log_time, whatever its publish_time.
        legacy = writer.register_channel(topic="legacy", message_encoding="raw", schema_id=0)
        for channel, times in ((native, native_times), (legacy, legacy_times)):
            for log_time, publish_time in times:
                writer.add_message(
                    channel_id=channel,
                    log_time=int(log_time * 1e9),
                    publish_time=int(publish_time * 1e9),
                    data=b"",
                )
        writer.finish()

    with McapStore(path=str(path)) as store:
        assert store.first_ts() == expected


def test_first_ts_does_not_hold_a_native_channel_in_memory(tmp_path: Path) -> None:
    path = tmp_path / "recording.mcap"
    payload = bytes(64 * 1024)
    total = 256
    with path.open("wb") as output:
        # Small chunks, so one chunk in memory stays far below the whole channel.
        writer = mcap_writer.Writer(output, chunk_size=4 * len(payload))
        writer.start(profile="dimos", library="test")
        channel = writer.register_channel(
            topic="camera",
            message_encoding="raw",
            schema_id=0,
            metadata={"dimos.observation_time": "publish_time"},
        )
        for i in range(total):
            writer.add_message(channel_id=channel, log_time=i, publish_time=total - i, data=payload)
        writer.finish()

    with McapStore(path=str(path)) as store:
        tracemalloc.start()
        try:
            first = store.first_ts()
            _, peak = tracemalloc.get_traced_memory()
        finally:
            tracemalloc.stop()

    assert first == 1e-9
    assert peak < len(payload) * total // 4, peak
