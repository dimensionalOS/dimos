# Copyright 2025 Dimensional Inc.
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

from datetime import datetime, timedelta

from dimos.robot.unitree_webrtc.type.timeseries import TEvent, TList

fixed_date = datetime(2025, 5, 13, 15, 2, 5).astimezone()
start_event = TEvent(fixed_date, 1)
end_event = TEvent(fixed_date + timedelta(seconds=10), 9)

sample_list = TList([start_event, TEvent(fixed_date + timedelta(seconds=2), 5), end_event])


def test_repr() -> None:
    assert (
        str(sample_list)
        == "Timeseries(date=2025-05-13, start=15:02:05, end=15:02:15, duration=0:00:10, events=3, freq=0.30Hz)"
    )


def test_equals() -> None:
    assert start_event == TEvent(start_event.ts, 1)
    assert start_event != TEvent(start_event.ts, 2)
    assert start_event != TEvent(start_event.ts + timedelta(seconds=1), 1)


def test_range() -> None:
    assert sample_list.time_range() == (start_event.ts, end_event.ts)


def test_duration() -> None:
    assert sample_list.duration() == timedelta(seconds=10)
