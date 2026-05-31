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

# Copyright 2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License").

"""Read a Go2 DDS mcap: list channels, iterate decoded messages.

Maps each DDS topic to a message class (decoded via :mod:`cdr`). Topics without
a registered class are listed by :func:`streams` but skipped by :func:`messages`.

    from dimos.robot.unitree.go2dds import reader
    for ch in reader.streams("data/go2_china_office_indoor.mcap"):
        print(ch["topic"], ch["schema"], ch["count"], "✓" if ch["decodable"] else "")
    for topic, ts, msg in reader.messages(path, "rt/lowstate"):
        print(ts, msg.bms_state.soc)
"""

from __future__ import annotations

from collections.abc import Callable, Iterator
from pathlib import Path
from typing import Any

from mcap.reader import make_reader

from dimos.robot.unitree.go2dds import ros
from dimos.robot.unitree.go2dds.cdr import decode
from dimos.robot.unitree.go2dds.msgs.LowState import LowState
from dimos.robot.unitree.go2dds.msgs.SportModeState import SportModeState


def _struct(cls: type) -> Callable[[bytes], Any]:
    """Decoder for a fixed CDR struct spec — the generic path."""
    return lambda b: decode(b, cls)[0]


# DDS topic -> decoder ``(cdr_bytes) -> message``. Unitree types decode to their
# own dataclasses; standard-ROS types decode into ``dimos.msgs`` (see :mod:`ros`).
REGISTRY: dict[str, Callable[[bytes], Any]] = {
    "rt/lowstate": _struct(LowState),
    "rt/sportmodestate": _struct(SportModeState),
    "rt/utlidar/cloud": ros.decode_pointcloud2,
    "rt/utlidar/imu": ros.decode_imu,
    "rt/utlidar/robot_odom": ros.decode_odometry,
    "rt/frontvideo": ros.decode_compressed_image,
}


def streams(path: str | Path) -> list[dict[str, Any]]:
    """List channels (no decode) — one dict per channel with keys
    ``topic``, ``schema``, ``encoding``, ``count``, ``decodable``.
    """
    with open(path, "rb") as f:
        s = make_reader(f).get_summary()
    if s is None:
        return []
    out = []
    for cid, ch in sorted(s.channels.items(), key=lambda kv: kv[1].topic):
        sch = s.schemas.get(ch.schema_id)
        n = s.statistics.channel_message_counts.get(cid, 0) if s.statistics else 0
        out.append(
            {
                "topic": ch.topic,
                "schema": sch.name if sch else "?",
                "encoding": ch.message_encoding,
                "count": n,
                "decodable": ch.topic in REGISTRY,
            }
        )
    return out


def messages(path: str | Path, *topics: str) -> Iterator[tuple[str, float, Any]]:
    """Yield ``(topic, ts_seconds, decoded_msg)`` in log order for registered topics.

    With no ``topics``, iterates every registered topic present in the file.
    """
    want = list(topics) or list(REGISTRY)
    unknown = [t for t in want if t not in REGISTRY]
    if unknown:
        raise KeyError(f"no decoder registered for {unknown}; known: {list(REGISTRY)}")
    with open(path, "rb") as f:
        for _schema, ch, m in make_reader(f).iter_messages(topics=want):
            yield ch.topic, m.log_time / 1e9, REGISTRY[ch.topic](m.data)
