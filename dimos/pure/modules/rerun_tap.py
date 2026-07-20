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

"""Rerun render tap: log a stream's ``to_rerun()`` into a recording, pass it through.

One tap per stream, rendering under ``entity_path`` at each message's ts. It is a
passthrough — ``step`` forwards its input unchanged — so it composes two ways
with no re-encode either way:

- **Recording / over()**: splice it inline (``B.over(x=RerunTap().over(msg=a))``).
  Forwarding is just an object reference and ``over()`` is lazy, so the pipeline
  stays streaming — one message in flight, never a materialized list.
- **Live**: bind ``msg`` to a topic and leave the ``msg`` Out **unbound**. The
  tap is then a parallel subscriber (pubsub fans out to it and the real
  consumer); an unbound Out publishes nothing, so nothing is re-encoded.

Deliberately NOT the ``subscribe_all`` bus bridge (``visualization/rerun``): a
pure module has a static typed port, so this renders *known, wired* streams — a
pipeline you built — identically live, recording, and hot-reload. Taps sharing
an ``app_id`` + ``recording_id`` merge into one recording, so a multi-stream
scene is N taps (each at its own cadence), not one multi-port sink.
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Any, Protocol, runtime_checkable

from dimos import pure as pm

if TYPE_CHECKING:
    import rerun as rr

__all__ = ["RerunRenderable", "RerunTap", "open_sink", "render_fields"]


@runtime_checkable
class RerunRenderable(Protocol):
    """Any message that can render itself to rerun (PointCloud2, OccupancyGrid, Path, ...)."""

    def to_rerun(self) -> Any:
        """A rerun Archetype, or a list of (entity_suffix, Archetype) pairs."""
        ...


class _Sink:
    """Owns the RecordingStream; ``dispose()`` (sniffed at teardown) flushes it."""

    def __init__(self, stream: rr.RecordingStream, timeline: str) -> None:
        self._stream = stream
        self._timeline = timeline

    def log(self, entity_path: str, ts: float, msg: RerunRenderable) -> None:
        """Log one message's archetype(s) under entity_path at data-time ts."""
        self._stream.set_time(self._timeline, timestamp=ts)
        data = msg.to_rerun()
        if isinstance(data, list):
            for suffix, archetype in data:
                self._stream.log(f"{entity_path}/{suffix}", archetype)
        else:
            self._stream.log(entity_path, data)

    def dispose(self) -> None:
        """Flush pending chunks to the sink at teardown."""
        self._stream.flush()


def open_sink(
    *,
    app_id: str,
    recording_id: str | None,
    sink: str,
    url: str | None,
    save_path: str | None,
    timeline: str,
) -> _Sink:
    """Open a rerun RecordingStream to the configured sink (shared by tap and sinks)."""
    import rerun as rr

    stream = rr.RecordingStream(application_id=app_id, recording_id=recording_id)
    if sink == "save":
        if not save_path:
            raise ValueError("rerun sink='save' needs save_path=<.rrd path>")
        stream.save(save_path)
    elif sink == "spawn":
        stream.spawn()
    elif sink == "grpc":
        stream.connect_grpc(url)
    else:
        raise ValueError(f"rerun: unknown sink {sink!r} (grpc|spawn|save)")
    return _Sink(stream, timeline)


def render_fields(rec: _Sink, entity_path: str, row: Any) -> None:
    """Log every present, renderable field of a row under entity_path/<field> at row.ts."""
    for name in type(row).fields():
        value = getattr(row, name)
        if value is not None and hasattr(value, "to_rerun"):
            rec.log(f"{entity_path}/{name}", row.ts, value)


class RerunTap(pm.PureModule):
    """Log a stream into rerun under entity_path; pass the message through unchanged."""

    entity_path: str = "world"  # rerun entity path this stream logs under
    app_id: str = "dimos"  # recording application id
    recording_id: str | None = None  # share across taps to merge into one recording
    timeline: str = "time"  # timeline name the message ts is stamped on
    sink: str = "grpc"  # "grpc" (connect a viewer) | "spawn" (launch one) | "save" (.rrd)
    url: str | None = None  # grpc address for sink="grpc"; None = rerun default
    save_path: str | None = None  # .rrd path for sink="save"

    class In(pm.In):
        msg: RerunRenderable = pm.tick()

    class Out(pm.Out):
        msg: RerunRenderable  # passthrough — forward the input; render is a side effect

    @pm.resource
    def rec(self) -> _Sink:
        """Open the recording stream to the configured sink; flushed at teardown."""
        return open_sink(
            app_id=self.app_id,
            recording_id=self.recording_id,
            sink=self.sink,
            url=self.url,
            save_path=self.save_path,
            timeline=self.timeline,
        )

    def step(self, i: In) -> Out:
        """Log the message, forward it unchanged."""
        self.rec.log(self.entity_path, i.ts, i.msg)
        return RerunTap.Out(msg=i.msg)
