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

"""Record generated values to SQLite and MCAP, then replay SQLite through module ports."""

from pathlib import Path
from tempfile import TemporaryDirectory
from threading import Event, Lock
from typing import Any

from dimos_generated.dimos_msgs.msg import LineSegment3D, LineSegments3D
from dimos_generated.geometry_msgs.msg import Point
from dimos_generated.std_msgs.msg import Header

from dimos.memory.codecs.base import codec_for, codec_id
from dimos.memory.replay_module import replay_module
from dimos.memory.store.mcap import McapStore
from dimos.memory.store.sqlite import SqliteStore
from dimos.msgs.time import time_from_nanoseconds, to_nanoseconds
from dimos.protocol.cdr_mcap import CdrMcapWriter


def main() -> None:
    messages = [
        LineSegments3D(
            header=Header(frame_id="map", stamp=time_from_nanoseconds(1700000000123456789 + index)),
            segments=[LineSegment3D(start=Point(x=index), end=Point(y=2), weight=index + 4)],
        )
        for index in range(3)
    ]
    with TemporaryDirectory(prefix="dimos-cdr-storage-") as directory:
        database = Path(directory) / "memory.db"
        artifact = Path(directory) / "recording.mcap"
        with SqliteStore(path=str(database)) as store, CdrMcapWriter(artifact) as writer:
            stream = store.stream("segments", LineSegments3D)
            for index, message in enumerate(messages):
                stream.append(message, ts=1 + index * 0.05)
                writer.write(
                    "segments",
                    message.encode(),
                    schema_name=message.msg_name,
                    schema=message.schema,
                    log_time_ns=1000000000 + index * 50000000,
                    publish_time_ns=to_nanoseconds(message.header.stamp),
                )
        print(f"SQLite default codec: {codec_id(codec_for(LineSegments3D))}")
        for source in [
            SqliteStore(path=str(database), must_exist=True),
            McapStore(path=str(artifact)),
        ]:
            with source:
                decoded = [observation.data for observation in source.stream("segments")]
                assert decoded == messages
                print(
                    f"{type(source).__name__}: weights={[value.segments[0].weight for value in decoded]}"
                )
        replay = replay_module(str(database))(dataset=str(database))
        received: list[Any] = []
        ready = Event()
        lock = Lock()

        def receive(message: LineSegments3D) -> None:
            with lock:
                received.append(message)
                print(
                    f"Module replay: weight={message.segments[0].weight}, stamp={to_nanoseconds(message.header.stamp)}"
                )
                if len(received) == len(messages):
                    ready.set()

        unsubscribe = replay.outputs["segments"].subscribe(receive)
        try:
            replay.start()
            assert ready.wait(5), "Replay did not deliver all messages"
            assert received == messages
        finally:
            unsubscribe()
            replay.stop()
    print(
        "PASS: CDR persisted, reopened by type/schema, replayed through Out ports; temporary files removed"
    )


if __name__ == "__main__":
    main()
