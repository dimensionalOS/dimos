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

from itertools import islice
from pathlib import Path

from dimos_generated.geometry_msgs.msg import Point
import pytest

from dimos.memory.store.sqlite import SqliteStore
from dimos.utils.testing.moment import SensorMoment
from dimos.utils.testing.replay import TimedSensorReplay, _close_all, _resolve_db_path


@pytest.fixture
def recording(tmp_path):
    path = tmp_path / "recording.db"
    with SqliteStore(path=path) as store:
        stream = store.stream("robot_point", Point)
        for i in range(3):
            stream.append(Point(x=i, y=2 * i, z=3 * i), ts=100.0 + i)
    try:
        yield f"{path}/robot_point"
    finally:
        _close_all()


def test_explicit_database_seek_and_loop(recording):
    replay = TimedSensorReplay[Point](recording)
    assert replay.count() == 3
    assert [(ts, msg.x) for ts, msg in replay.iterate_ts(seek=1.0)] == [(101, 1), (102, 2)]
    assert [(ts, msg.x) for ts, msg in replay.iterate_ts(from_timestamp=102)] == [(102, 2)]
    assert [msg.x for _, msg in islice(replay.iterate_ts(seek=1, duration=0.5, loop=True), 3)] == [
        1,
        1,
        1,
    ]
    assert replay.find_closest(101.1, tolerance=0.2).x == 1
    assert replay.find_closest(110, tolerance=0.2) is None


def test_sensor_moment_publishes_selected_cdr_value_and_clears_missing_seek(recording, mocker):
    transport = mocker.Mock()
    moment = SensorMoment[Point](recording, transport)
    try:
        moment.publish()
        transport.publish.assert_not_called()
        moment.seek(1.0)
        moment.publish()
        (published,) = transport.publish.call_args.args
        assert Point.decode(published.encode()).x == 1
        moment.seek(10.0)
        assert moment.value is None
        moment.publish()
        assert transport.publish.call_count == 1
    finally:
        moment.stop()
    transport.stop.assert_called_once_with()


def test_missing_explicit_relative_database_does_not_download_a_doubled_suffix(mocker):
    download = mocker.patch("dimos.utils.testing.replay.get_data")
    assert _resolve_db_path("missing-recording.db") == Path("missing-recording.db")
    download.assert_not_called()
