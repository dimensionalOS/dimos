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

"""CDR passes through real mem2 storage and self-describing MCAP replay."""

from contextlib import ExitStack
from functools import partial
import threading

from dimos_cdrgen.registry import encode, schema
from mcap.reader import make_reader
from mcap.writer import Writer
import pytest
from reactivex.scheduler import ThreadPoolScheduler

from dimos.core import transport as transport_module
from dimos.core.global_config import global_config
from dimos.core.transport_factory import make_transport
from dimos.memory.store.cdr_mcap import CdrMcapStore
from dimos.memory.store.sqlite import SqliteStore
from dimos.memory.tap import TransportRecorder
from dimos.messages.builtin_interfaces.msg import Time
from dimos.messages.dimos_msgs.msg import LineSegment3D, LineSegments3D
from dimos.messages.geometry_msgs.msg import Point
from dimos.messages.std_msgs.msg import Header
from dimos.protocol.pubsub.impl.lcmpubsub import LCM
from dimos.utils.threadpool import get_max_workers


def message(sec=17):
    return LineSegments3D(
        header=Header(stamp=Time(sec=sec, nanosec=250_000_000), frame_id="imu"),
        segments=[LineSegment3D(Point(1.0, 2.0, 3.0), Point(4.0, 5.0, 6.0), 0.75)],
    )


@pytest.mark.parametrize("backend", ["lcm", "zenoh"])
def test_factory_capture_sqlite_mcap_auto_named_replay(
    backend, monkeypatch, tmp_path, owned_scheduler
):
    # Isolate only session configuration; codecs, pub/sub and storage stay real.
    monkeypatch.setattr(transport_module, "LCM", partial(LCM, url="memq://"))
    for key, value in {
        "transport": backend,
        "robot_ip": None,
        "robot_ips": "",
        "zenoh_connect": "",
        "zenoh_mode": "peer",
        "zenoh_multicast": False,
        "zenoh_gossip": False,
        "zenoh_scouting": False,
    }.items():
        monkeypatch.setattr(global_config, key, value)
    expected = message()
    expected.header.stamp.nanosec = 123456789
    database = tmp_path / "live.db"
    received = threading.Event()
    with SqliteStore(path=str(database)) as store, ExitStack() as cleanup:
        transport = make_transport("sensor/segments", LineSegments3D)
        cleanup.callback(transport.stop)
        recorder = TransportRecorder(store)
        cleanup.callback(recorder.close)
        unsubscribe = recorder.tap("segments", LineSegments3D, transport)
        assert unsubscribe is not None
        cleanup.callback(unsubscribe)
        live = store.stream("segments", LineSegments3D).live()
        cleanup.callback(live.stop)
        cleanup.enter_context(live.subscribe(lambda _: received.set()))
        transport.publish(expected)
        assert received.wait(5), "default-factory payload was not recorded"
        assert recorder.dropped == 0
    path = tmp_path / "live.mcap"
    with SqliteStore(path=str(database), must_exist=True) as store, path.open("wb") as raw:
        writer = Writer(raw)
        writer.start(profile="ros2")
        captured = next(iter(store.streams.segments)).data
        assert type(captured) is LineSegments3D
        assert encode(captured) == encode(expected)
        schema_id = writer.register_schema(
            LineSegments3D.__msgtype__, "ros2msg", schema(LineSegments3D.__msgtype__).encode()
        )
        channel_id = writer.register_channel(transport.channel, "cdr", schema_id)
        writer.add_message(channel_id, 20_000_000_000, encode(captured), 17_123456789)
        writer.finish()
    with CdrMcapStore(path=str(path), streams={"sensor_segments": transport.channel}) as store:
        assert store.list_streams() == ["sensor_segments"]
        values = list(store.replay().streams.sensor_segments.iterate_ts())
        assert values == [(20.0, expected)]
        assert values[0][1].header.stamp.nanosec == 123456789
    with path.open("rb") as raw:
        definition, channel, recorded = next(make_reader(raw).iter_messages())
        assert recorded.publish_time == 17_123456789
        assert recorded.log_time == 20_000_000_000
        assert "MSG: std_msgs/Header" in definition.data.decode()
        assert channel.topic == transport.channel


@pytest.fixture
def owned_scheduler(monkeypatch):
    scheduler = ThreadPoolScheduler(max_workers=get_max_workers())
    monkeypatch.setattr("dimos.utils.reactive.get_scheduler", lambda: scheduler)
    monkeypatch.setattr("dimos.utils.threadpool.get_scheduler", lambda: scheduler)
    yield scheduler
    scheduler.executor.shutdown(wait=True)
