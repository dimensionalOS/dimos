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

"""Tests for RecordReplay."""

import asyncio
from collections.abc import Callable
from contextlib import suppress
from pathlib import Path
import threading
from typing import Any

import pytest

from dimos.record import RecordReplay


class FakeTopic:
    """Minimal topic for testing."""

    def __init__(self, name: str) -> None:
        self.topic = name
        self.lcm_type = None

    @property
    def pattern(self) -> str:
        return self.topic

    def __str__(self) -> str:
        return self.topic


class FakePubSub:
    """Minimal PubSub that supports subscribe_all for testing."""

    def __init__(self) -> None:
        self._subscribers: list[Callable[[Any, Any], Any]] = []
        self._lock = threading.Lock()
        self._started = False

    def start(self) -> None:
        self._started = True

    def stop(self) -> None:
        self._started = False

    def publish(self, topic: Any, message: Any) -> None:
        # Not needed for recording tests
        pass

    def subscribe_all(self, callback: Callable[[Any, Any], Any]) -> Callable[[], None]:
        with self._lock:
            self._subscribers.append(callback)

        def unsub() -> None:
            with self._lock:
                with suppress(ValueError):
                    self._subscribers.remove(callback)

        return unsub

    def emit(self, topic_name: str, msg: Any) -> None:
        """Test helper: simulate a message arriving."""
        topic = FakeTopic(topic_name)
        with self._lock:
            subs = list(self._subscribers)
        for cb in subs:
            cb(msg, topic)


class SimpleMsg:
    """A simple test message (not LCM, uses pickle codec)."""

    def __init__(self, value: float) -> None:
        self.value = value

    def __eq__(self, other: object) -> bool:
        return isinstance(other, SimpleMsg) and self.value == other.value


@pytest.fixture
def tmp_db(tmp_path: Path) -> str:
    return str(tmp_path / "test_recording.db")


class TestRecordReplay:
    async def test_record_and_list_streams(self, tmp_db: str) -> None:
        pubsub = FakePubSub()
        async with RecordReplay(tmp_db) as rec:
            rec.start_recording([pubsub])
            assert rec.is_recording

            pubsub.emit("/sensor/lidar", SimpleMsg(1.0))
            pubsub.emit("/sensor/odom", SimpleMsg(2.0))
            pubsub.emit("/sensor/lidar", SimpleMsg(3.0))
            await asyncio.sleep(0.05)  # let timestamps diverge

            rec.stop_recording()
            assert not rec.is_recording

            streams = rec.store.list_streams()
            assert "sensor_lidar" in streams
            assert "sensor_odom" in streams

    async def test_record_and_query(self, tmp_db: str) -> None:
        pubsub = FakePubSub()
        async with RecordReplay(tmp_db) as rec:
            rec.start_recording([pubsub])

            for i in range(10):
                pubsub.emit("/data", SimpleMsg(float(i)))
                await asyncio.sleep(0.01)

            rec.stop_recording()

            s = rec.store.stream("data")
            assert s.count() == 10
            first = s.first()
            assert isinstance(first.data, SimpleMsg)
            assert first.data.value == 0.0

    async def test_duration(self, tmp_db: str) -> None:
        pubsub = FakePubSub()
        async with RecordReplay(tmp_db) as rec:
            rec.start_recording([pubsub])
            pubsub.emit("/a", SimpleMsg(0.0))
            await asyncio.sleep(0.1)
            pubsub.emit("/a", SimpleMsg(1.0))
            rec.stop_recording()

            assert rec.duration >= 0.05  # at least some duration

    async def test_stream_info(self, tmp_db: str) -> None:
        pubsub = FakePubSub()
        async with RecordReplay(tmp_db) as rec:
            rec.start_recording([pubsub])
            for i in range(5):
                pubsub.emit("/sensor", SimpleMsg(float(i)))
                await asyncio.sleep(0.01)
            rec.stop_recording()

            infos = rec.stream_info()
            assert len(infos) == 1
            assert infos[0]["name"] == "sensor"
            assert infos[0]["count"] == 5

    async def test_delete_range(self, tmp_db: str) -> None:
        pubsub = FakePubSub()
        async with RecordReplay(tmp_db) as rec:
            rec.start_recording([pubsub])
            for i in range(20):
                pubsub.emit("/data", SimpleMsg(float(i)))
                await asyncio.sleep(0.01)
            rec.stop_recording()

            before = rec.store.stream("data").count()
            assert before == 20

            dur = rec.duration
            # Delete middle third
            deleted = rec.delete_range(dur / 3, 2 * dur / 3)
            assert deleted > 0

            after = rec.store.stream("data").count()
            assert after < before

    async def test_trim(self, tmp_db: str) -> None:
        pubsub = FakePubSub()
        async with RecordReplay(tmp_db) as rec:
            rec.start_recording([pubsub])
            for i in range(30):
                pubsub.emit("/data", SimpleMsg(float(i)))
                await asyncio.sleep(0.01)
            rec.stop_recording()

            before = rec.store.stream("data").count()
            dur = rec.duration
            # Trim to middle third
            rec.trim(dur / 3, 2 * dur / 3)

            after = rec.store.stream("data").count()
            assert after < before

    async def test_repr(self, tmp_db: str) -> None:
        async with RecordReplay(tmp_db) as rec:
            r = repr(rec)
            assert "RecordReplay" in r
            assert "streams=0" in r

    async def test_playback_runs(self, tmp_db: str) -> None:
        """Test that playback task starts and finishes."""
        pubsub = FakePubSub()
        async with RecordReplay(tmp_db) as rec:
            rec.start_recording([pubsub])
            for i in range(5):
                pubsub.emit("/data", SimpleMsg(float(i)))
                await asyncio.sleep(0.01)
            rec.stop_recording()

            rec.play(speed=100.0)  # very fast
            assert rec.is_playing
            async with asyncio.timeout(0.1):
                await rec._play_task
            assert not rec.is_playing

    async def test_stop_playback(self, tmp_db: str) -> None:
        pubsub = FakePubSub()
        async with RecordReplay(tmp_db) as rec:
            rec.start_recording([pubsub])
            for i in range(100):
                pubsub.emit("/data", SimpleMsg(float(i)))
                await asyncio.sleep(0.005)
            rec.stop_recording()

            rec.play(speed=0.1)  # slow
            await asyncio.sleep(0.1)
            assert rec.is_playing
            await rec.stop_playback()
            assert not rec.is_playing

    async def test_seek(self, tmp_db: str) -> None:
        pubsub = FakePubSub()
        async with RecordReplay(tmp_db) as rec:
            rec.start_recording([pubsub])
            for i in range(10):
                pubsub.emit("/data", SimpleMsg(float(i)))
                await asyncio.sleep(0.01)
            rec.stop_recording()

            rec.seek(0.05)
            assert rec.position == pytest.approx(0.05, abs=0.01)

    async def test_multiple_pubsubs(self, tmp_db: str) -> None:
        ps1 = FakePubSub()
        ps2 = FakePubSub()
        async with RecordReplay(tmp_db) as rec:
            rec.start_recording([ps1, ps2])
            ps1.emit("/from_ps1", SimpleMsg(1.0))
            ps2.emit("/from_ps2", SimpleMsg(2.0))
            rec.stop_recording()

            streams = rec.store.list_streams()
            assert "from_ps1" in streams
            assert "from_ps2" in streams
