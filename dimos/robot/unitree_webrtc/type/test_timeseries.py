from datetime import timedelta, datetime
from dimos.robot.unitree_webrtc.type.timeseries import TEvent, Timeseries
from typing import TypeVar

now = datetime.now().astimezone()
start = TEvent(now, 1)
end = TEvent(now + timedelta(seconds=5), 9)

T = TypeVar("T")


class TestTimeseries(list[TEvent[T]], Timeseries[TEvent[T]]):
    """A test class that inherits from both list and Timeseries."""

    ...


sample_list: TestTimeseries[TEvent[int]] = TestTimeseries([start, TEvent(now + timedelta(seconds=2), 5), end])


def test_equals():
    assert start == TEvent(start.ts, 1)
    assert start != TEvent(start.ts, 2)
    assert start != TEvent(start.ts + 1, 1)


def test_autotz():
    assert TEvent(datetime.now(), 3) == TEvent(datetime.now().astimezone(), 3)


def test_range():
    assert sample_list.time_range() == (start.ts, end.ts)


def test_duration():
    assert sample_list.duration() == timedelta(seconds=5)
