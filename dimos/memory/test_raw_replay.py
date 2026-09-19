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

"""Reading only as much of a recording as the caller is going to score.

The R1 Pro recording is fourteen hours and ten million transforms, and the
tools that read it score the first three. Holding all of it cost about 9 GB and
put the robot's own stack in danger of being killed, so the cut has to actually
skip the work rather than decode and discard.
"""

from __future__ import annotations

from dimos.memory.raw_replay import RawStream


class _CountingConnection:
    """A stand-in for the sqlite connection that tallies the rows asked for.

    Stubbing at the connection rather than overriding `_decode` keeps the real
    decode path under test, and it is the honest place to count: the thing
    being saved is the query and the blob it pulls back, not a Python call.
    """

    def __init__(self) -> None:
        self.rows: list[int] = []

    def execute(self, _sql: str, parameters: tuple) -> _CountingConnection:
        self.rows.append(parameters[0])
        return self

    def fetchone(self) -> tuple[bytes]:
        return (b"payload",)


class _Payload:
    """The smallest thing `RawStream._decode` will hand back."""

    @staticmethod
    def lcm_decode(data: bytes) -> str:
        return data.decode()


def _stream(stamps):
    connection = _CountingConnection()
    stream = RawStream(
        name="tf",
        connection=connection,
        payload_type=_Payload,
        stamps=tuple(stamps),
        ids=tuple(range(len(stamps))),
    )
    return stream, connection


def test_iterate_without_a_limit_reads_everything():
    stream, connection = _stream([1.0, 2.0, 3.0, 4.0])
    assert len(list(stream.iterate())) == 4
    assert connection.rows == [0, 1, 2, 3]


def test_iterate_stops_at_the_stamp_it_was_given():
    stream, connection = _stream([1.0, 2.0, 3.0, 4.0])
    assert len(list(stream.iterate(until_ts=2.0))) == 2


def test_the_rows_past_the_limit_are_never_read():
    # The whole point. Reading the blob is the cost -- filtering after the fact
    # would pull all ten million rows and then throw eleven hours of them away,
    # which is what this is here to stop.
    stream, connection = _stream([1.0, 2.0, 3.0, 4.0])
    list(stream.iterate(until_ts=2.0))
    assert connection.rows == [0, 1], "rows past until_ts were read anyway"


def test_a_limit_before_the_first_stamp_reads_nothing():
    stream, connection = _stream([1.0, 2.0, 3.0])
    assert list(stream.iterate(until_ts=0.5)) == []
    assert connection.rows == []
