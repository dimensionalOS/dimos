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

"""Deterministic replay of the WebXR controller **input entry point**.

This module is not a second teleoperation path: it publishes exactly the two
streams :class:`~dimos.teleop.webxr.extensions.ArmTeleopModule` publishes, with
the same names and message types, so a blueprint can be exercised without a
headset and every downstream consumer (deadman classification, reference
anchoring, session handling) sees the same input it would see from a Quest.

It is *input replay*, never a human headset session: the poses come from a
fixed script - a hold, then a smooth eased arc, then a hold, then a deadman
release - and the module says so in its logs and in :meth:`describe`.

Two extra script knobs exist for the safety cases rather than for teleoperation:

* ``freeze_after_s`` stops publishing while the deadman is still held, which is
  an input timeout / stale-input case, not a release;
* ``restart_after_s`` presses the deadman again after a release, which proves a
  new session re-anchors instead of resuming the old reference.
"""

from __future__ import annotations

import math
import threading
import time

from pydantic import Field

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import Out
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.teleop.webxr.controller_types import Buttons
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

#: Label every artifact must carry: this is scripted input, not a headset run.
INPUT_SOURCE = "input_replay"


class WebXRReplayConfig(ModuleConfig):
    """Scripted controller input; all durations are seconds from start."""

    rate_hz: float = Field(default=90.0, gt=0)
    #: Where the (arbitrary, anchored) controller pose starts, in metres.
    origin: tuple[float, float, float] = (0.35, 0.0, 0.25)
    #: Deadman is pressed after this long; nothing moves before it.
    engage_after_s: float = 1.0
    #: After engagement the operator holds still, so the anchor is a real pose.
    settle_s: float = 0.5
    #: The scripted motion: eased sideways rise with a small wrist yaw.
    sweep_s: float = 3.0
    sweep_side_m: float = 0.06
    sweep_rise_m: float = 0.05
    sweep_yaw_rad: float = 0.20
    #: Hold the final pose while still engaged.
    hold_s: float = 1.0
    #: Release the deadman here; nothing is published after it by default.
    release_after_s: float = 6.0
    release_wait_s: float = 1.0
    #: Stop publishing (deadman still held) after this long: input timeout case.
    freeze_after_s: float | None = None
    #: Press the deadman again this long after the release: re-anchor case.
    restart_after_s: float | None = None
    #: Keep publishing after the release instead of going silent.
    publish_after_release: bool = False

    @property
    def total_s(self) -> float:
        """When the script is over, including any post-release observations."""
        end = self.release_after_s
        if self.restart_after_s is not None:
            end = max(end, self.restart_after_s + self.settle_s + self.hold_s)
        return end + self.release_wait_s


class WebXRReplayModule(Module):
    """Publish scripted Quest controller poses and deadman buttons.

    Outputs:
        - right_controller_output: PoseStamped, the right controller's absolute pose
        - teleop_buttons: Buttons, with ``right_grip`` as the deadman
    """

    config: WebXRReplayConfig

    right_controller_output: Out[PoseStamped]
    teleop_buttons: Out[Buttons]

    def __init__(self, **kwargs: object) -> None:
        super().__init__(**kwargs)
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None
        self._started_at: float | None = None

    @rpc
    def start(self) -> None:
        super().start()
        self._stop_event.clear()
        self._started_at = None
        self._thread = threading.Thread(target=self._run, name="WebXRReplayInput", daemon=True)
        self._thread.start()
        logger.info(
            "WebXR input replay started (scripted controller input, not a headset session)",
            source=INPUT_SOURCE,
            duration_s=self.config.total_s,
        )

    @rpc
    def stop(self) -> None:
        self._stop_event.set()
        thread = self._thread
        if thread is not None:
            thread.join(timeout=5.0)
        self._thread = None
        super().stop()

    @rpc
    def describe(self) -> dict[str, object]:
        """Report that these controller samples are scripted, and their schedule.

        Returns:
            The input source label and the script's timing knobs.
        """
        config = self.config
        return {
            "source": INPUT_SOURCE,
            "human_quest_session": False,
            "rate_hz": config.rate_hz,
            "engage_after_s": config.engage_after_s,
            "sweep_s": config.sweep_s,
            "release_after_s": config.release_after_s,
            "freeze_after_s": config.freeze_after_s,
            "restart_after_s": config.restart_after_s,
        }

    # ------------------------------------------------------------------ script
    def _run(self) -> None:
        config = self.config
        period = 1.0 / config.rate_hz
        self._started_at = time.monotonic()
        deadline = self._started_at
        elapsed = 0.0
        while not self._stop_event.is_set():
            elapsed = time.monotonic() - self._started_at
            self._publish(elapsed)
            deadline += period
            sleep_s = deadline - time.monotonic()
            if sleep_s > 0:
                timeout = sleep_s
            else:
                # Keep the schedule but never burst: a late tick is one tick.
                deadline = time.monotonic()
                timeout = 0.0
            if self._stop_event.wait(timeout):
                break
            if config.freeze_after_s is not None and elapsed >= config.freeze_after_s:
                logger.info(
                    "WebXR input replay stopped publishing with the deadman held "
                    "(stale-input case: no further samples reach the task)",
                    source=INPUT_SOURCE,
                    at_s=round(elapsed, 3),
                )
                # Publish nothing at all: a real headset that drops out does not
                # send a release. The task must notice the stale input itself.
                return
        logger.info("WebXR input replay finished", source=INPUT_SOURCE, at_s=round(elapsed, 3))

    def _engaged(self, t_s: float) -> bool:
        config = self.config
        if t_s < config.engage_after_s:
            return False
        if t_s < config.release_after_s:
            return True
        if config.publish_after_release:
            return False
        if config.restart_after_s is None:
            return False
        restart_at = config.restart_after_s
        window_end = restart_at + config.settle_s + config.hold_s
        return restart_at <= t_s < window_end

    def _pose(self, t_s: float) -> tuple[Vector3, Quaternion]:
        """Scripted controller pose: still, eased sweep, then still again."""
        config = self.config
        x, y, z = (float(value) for value in config.origin)
        engage_at = config.engage_after_s
        if config.restart_after_s is not None and t_s >= config.restart_after_s:
            engage_at = config.restart_after_s
        motion_at = engage_at + config.settle_s
        u = (t_s - motion_at) / config.sweep_s if config.sweep_s > 0 else 1.0
        u = min(max(u, 0.0), 1.0)
        # Cosine ease: zero velocity at both ends, so no reference step at the
        # start or the end of the sweep.
        ramp = 0.5 - 0.5 * math.cos(math.pi * u)
        yaw = config.sweep_yaw_rad * ramp
        half = 0.5 * yaw
        return (
            Vector3(x, y - config.sweep_side_m * ramp, z + config.sweep_rise_m * ramp),
            Quaternion(0.0, 0.0, math.sin(half), math.cos(half)),
        )

    # ---------------------------------------------------------------- publish
    def _publish(self, t_s: float) -> None:
        engaged = self._engaged(t_s)
        if (
            not engaged
            and not self.config.publish_after_release
            and t_s >= self.config.release_after_s
        ):
            if self.config.restart_after_s is None or t_s < self.config.restart_after_s:
                self._publish_buttons(False)
                return
        position, orientation = self._pose(t_s)
        self.right_controller_output.publish(
            PoseStamped(
                ts=time.time(),
                frame_id="world",
                position=position,
                orientation=orientation,
            )
        )
        self._publish_buttons(engaged)

    def _publish_buttons(self, engaged: bool) -> None:
        buttons = Buttons()
        buttons.right_grip = bool(engaged)
        self.teleop_buttons.publish(buttons)


webxr_input_replay = WebXRReplayModule.blueprint
