# Copyright 2025-2026 Dimensional Inc.
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

"""Live adapters binding the drive sequencer to the coordinator + recorder.

These implement the ``CommandSink`` / ``EpisodeSink`` / ``Clock`` protocols from
:mod:`drive` against the real system. The publish functions are INJECTED (wired
to ``Out`` ports by the entry/blueprint), so the adapters themselves stay thin
and unit-testable with fakes. The live end-to-end (real coordinator + recorder)
cannot be validated without hardware/sim - that is a bench step - but the
mapping logic here is pinned by tests.
"""

from __future__ import annotations

from collections.abc import Callable
import time

from dimos.control.autotune.profile import RobotProfile
from dimos.learning.collection.episode_monitor import EpisodeStatus
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3

# Channel name -> (twist component, axis). The conventional twist-base mapping.
TWIST_CHANNEL_MAP: dict[str, tuple[str, str]] = {
    "vx": ("linear", "x"),
    "vy": ("linear", "y"),
    "wz": ("angular", "z"),
}


class TwistCommandSink:
    """Turns a per-channel command dict into a Twist on the coordinator's
    ``twist_command`` stream. Channels not in the command are zero; ``stop``
    publishes an all-zero twist."""

    def __init__(
        self,
        publish: Callable[[Twist], None],
        channel_map: dict[str, tuple[str, str]] | None = None,
    ) -> None:
        self._publish = publish
        self._map = channel_map or TWIST_CHANNEL_MAP

    def _twist(self, channel_values: dict[str, float]) -> Twist:
        lin = {"x": 0.0, "y": 0.0, "z": 0.0}
        ang = {"x": 0.0, "y": 0.0, "z": 0.0}
        for ch, value in channel_values.items():
            if ch not in self._map:
                raise KeyError(f"channel {ch!r} has no twist mapping")
            component, axis = self._map[ch]
            (lin if component == "linear" else ang)[axis] = value
        return Twist(Vector3(lin["x"], lin["y"], lin["z"]), Vector3(ang["x"], ang["y"], ang["z"]))

    def send(self, channel_values: dict[str, float]) -> None:
        self._publish(self._twist(channel_values))

    def stop(self) -> None:
        self._publish(self._twist({}))


class EpisodeStatusSink:
    """Emits ``EpisodeStatus`` start/save events on the SAME timeline the
    learning recorder reads - programmatically, not via teleop. We reuse the
    EpisodeStatus message and counter semantics; we do not fork EpisodeMonitor."""

    def __init__(
        self, publish: Callable[[EpisodeStatus], None], now: Callable[[], float] | None = None
    ) -> None:
        self._publish = publish
        self._now = now or time.time
        self._saved = 0
        self._discarded = 0

    def start_episode(self, label: str) -> None:
        self._publish(
            EpisodeStatus(
                ts=self._now(),
                state="recording",
                episodes_saved=self._saved,
                episodes_discarded=self._discarded,
                last_event="start",
                task_label=label,
            )
        )

    def save_episode(self) -> None:
        self._saved += 1
        self._publish(
            EpisodeStatus(
                ts=self._now(),
                state="idle",
                episodes_saved=self._saved,
                episodes_discarded=self._discarded,
                last_event="save",
            )
        )


class WallClock:
    """Real monotonic clock + sleep for live runs."""

    def now(self) -> float:
        return time.monotonic()

    def sleep(self, seconds: float) -> None:
        if seconds > 0:
            time.sleep(seconds)


def make_sinks(
    profile: RobotProfile,
    publish_twist: Callable[[Twist], None],
    publish_status: Callable[[EpisodeStatus], None],
) -> tuple[TwistCommandSink, EpisodeStatusSink, WallClock]:
    """Convenience: build the three live adapters for a twist-base profile."""
    if profile.command_interface != "twist":
        raise ValueError(
            f"live twist sinks require a twist command interface, got {profile.command_interface!r}"
        )
    return TwistCommandSink(publish_twist), EpisodeStatusSink(publish_status), WallClock()
