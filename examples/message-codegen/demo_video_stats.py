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

"""Record browser video metrics as generated CDR and inspect exact integer fields."""

from pathlib import Path
import tempfile

from dimos_generated.dimos_msgs.msg import VideoStats
from dimos_message_build.registry import decode as cdr_decode, encode as cdr_encode

from dimos.memory.store.sqlite import SqliteStore
from dimos.teleop.utils.video_stats import video_stats_from_dict


def main() -> None:
    message = video_stats_from_dict(
        {
            "ts": 1700000000.5,
            "fps": 28.0,
            "kbps": 2100.5,
            "width": 1280,
            "height": 720,
            "frames_dropped": 2**32 + 1,
        }
    )
    decoded = cdr_decode(cdr_encode(message), VideoStats)
    assert decoded == message
    with tempfile.TemporaryDirectory() as directory:
        path = Path(directory) / "video.db"
        with SqliteStore(path=str(path)) as store:
            store.stream("video_stats", VideoStats).append(decoded, ts=1700000000.5)
        with SqliteStore(path=str(path)) as store:
            restored = store.stream("video_stats").first().data
            assert restored == message
            print(
                f"Recorded {restored.__msgtype__}: {restored.width}x{restored.height}, {restored.fps} fps"
            )
            print(f"Exact dropped-frame counter: {restored.frames_dropped}")
            print(f"Source stamp: {restored.header.stamp.sec}s + {restored.header.stamp.nanosec}ns")
            print("PASS: browser JSON → generated CDR → SQLite → generated message")


if __name__ == "__main__":
    main()
