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

"""Record generated values in mem2, reopen, export MCAP and replay both."""

import argparse
from pathlib import Path

from dimos_cdrgen.registry import encode, schema

from dimos.memory.store.mcap import McapStore
from dimos.memory.store.sqlite import SqliteStore
from dimos.messages.builtin_interfaces.msg import Time
from dimos.messages.dimos_msgs.msg import LineSegment3D, LineSegments3D
from dimos.messages.geometry_msgs.msg import Point
from dimos.messages.std_msgs.msg import Header
from dimos.protocol.cdr_mcap import CdrMcapWriter


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory", type=Path, help="New directory for the demo files")
    output = parser.parse_args().directory.resolve()
    output.mkdir(parents=True, exist_ok=False)
    expected = LineSegments3D(
        Header(Time(17, 250_000_000), "map"),
        [LineSegment3D(Point(1.0, 2.0, 3.0), Point(4.0, 5.0, 6.0), 0.75)],
    )
    with SqliteStore(path=str(output / "memory.db")) as store:
        store.stream("segments", LineSegments3D).append(expected, ts=17.25)
    with SqliteStore(path=str(output / "memory.db"), must_exist=True) as store:
        replayed = list(store.replay().streams.segments.iterate_ts())
        assert replayed == [(17.25, expected)]
        with CdrMcapWriter(output / "recording.mcap") as writer:
            writer.register_stream(
                "segments",
                schema_name=LineSegments3D.__msgtype__,
                schema=schema(LineSegments3D.__msgtype__),
                metadata={"dimos.observation_time": "publish_time"},
            )
            for timestamp, message in replayed:
                writer.write(
                    "segments",
                    encode(message),
                    schema_name=LineSegments3D.__msgtype__,
                    schema=schema(LineSegments3D.__msgtype__),
                    log_time_ns=20_000_000_000,
                    publish_time_ns=round(timestamp * 1_000_000_000),
                )
    with McapStore(path=str(output / "recording.mcap")) as store:
        assert list(store.replay().streams.segments.iterate_ts()) == [(17.25, expected)]
    print("PASS: mem2 SQLite close/reopen and replay preserve CDR values and source timestamp")
    print("PASS: MCAP embeds complete ros2msg schemas and replays the same generated type")
    print(output)


if __name__ == "__main__":
    main()
