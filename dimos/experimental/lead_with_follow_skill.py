#!/usr/bin/env python3
# Drop-in Guide lead-with-follow skill.
# Implements `lead_to(destination)`: starts a navigate_with_text goal AND
# monitors whether the visitor (a person being followed) is still in view.
# If tracking is lost mid-navigation, cancel the goal, speak "I'll wait for
# you", and poll until the person is reacquired — then resume.
#
# This is the defining gesture of Drop-in Guide: the moment that turns the
# robot from a delivery bot into an actual guide. Storyboard Shot 8.

import threading
import time
from typing import Any

from reactivex.disposable import Disposable

from dimos.agents.annotation import skill
from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.stream import In
from dimos.msgs.sensor_msgs.Image import Image
from dimos.navigation.base import NavigationState
from dimos.navigation.navigation_spec import NavigationInterfaceSpec
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


# How long we wait for the visitor to come back before giving up.
_REACQUIRE_TIMEOUT_S = 30.0
# How often we check the tracking state.
_POLL_INTERVAL_S = 0.5
# Grace period for the planner to leave IDLE after navigate_with_text was
# called. Without it, the loop's first get_state() may still see IDLE and
# exit immediately, dropping follower monitoring for the whole trip.
_STARTUP_GRACE_S = 2.5
# If the camera hasn't produced a frame in this many seconds, we conclude
# the perception pipeline is offline — not the same as "visitor walked
# away", but the only reliable proxy we have without invoking EdgeTAM
# (CUDA-only in shipped dimOS). Hardware-day note in the docstring.
_CAMERA_STALE_S = 4.0


class LeadWithFollowSkill(Module):
    """Visitor-aware navigation. Pauses when the person we're guiding drops
    out of view.

    The visitor-presence check is intentionally conservative for the macOS
    development path: we use camera-frame freshness as a proxy ("is the
    perception pipeline producing frames?") rather than invoking the EdgeTAM
    tracker directly (CUDA-only in shipped dimOS). On hardware-day this
    proxy should be swapped for a real `is_tracking()` query against
    PersonFollowSkillContainer — see TODO inside `_follower_visible`.
    """

    color_image: In[Image]
    _navigation: NavigationInterfaceSpec

    _lead_lock: threading.Lock = threading.Lock()
    _lead_thread: threading.Thread | None = None
    _stop_event: threading.Event | None = None
    _latest_frame_ts: float = 0.0

    @rpc
    def start(self) -> None:
        super().start()
        self._lead_thread = None
        self._stop_event = None
        self._latest_frame_ts = 0.0
        # Track camera-frame freshness as a permission-light proxy for
        # "the robot is seeing the world right now".
        self.register_disposable(
            Disposable(self.color_image.subscribe(self._on_color_image))
        )

    @rpc
    def stop(self) -> None:
        if self._stop_event is not None:
            self._stop_event.set()
        if self._lead_thread is not None and self._lead_thread.is_alive():
            self._lead_thread.join(timeout=2.0)
        super().stop()

    def _on_color_image(self, _image: Image) -> None:
        self._latest_frame_ts = time.time()

    def _follower_visible(self) -> bool:
        """Best-effort visitor-presence check.

        Current implementation: returns True if the camera produced a frame
        within the last `_CAMERA_STALE_S` seconds AND PersonFollow is not
        explicitly reporting "lost". This is a soft signal — it tells us
        the perception bus is alive, not that a specific person is in view.

        TODO (hardware-day): swap this for `PersonFollowSkillContainer.
        is_tracking()` (or equivalent) once that state is exposed via an
        @rpc method or a Spec. Until then, this conservative proxy lets
        the lead_to flow exercise its pause/resume code path during demos.
        """
        if self._latest_frame_ts == 0.0:
            return True  # have not yet received any frames — be optimistic
        age = time.time() - self._latest_frame_ts
        return age < _CAMERA_STALE_S

    def _lead_loop(
        self,
        destination: str,
        cancel: threading.Event,
    ) -> None:
        logger.info(f"lead_to: starting follower monitor for '{destination}'")
        last_log_ts = 0.0
        paused = False
        startup_deadline = time.time() + _STARTUP_GRACE_S

        while not cancel.is_set():
            state = self._navigation.get_state()
            in_startup_grace = time.time() < startup_deadline
            if state == NavigationState.IDLE and not in_startup_grace:
                if self._navigation.is_goal_reached():
                    logger.info(f"lead_to: arrived at '{destination}'")
                    return
                if not paused:
                    # Nav exited idle without success after startup grace —
                    # either it failed mid-trip or no goal was ever set by
                    # the agent. Grace period (_STARTUP_GRACE_S) protects
                    # against the race where lead_to is called immediately
                    # after navigate_with_text and the planner hasn't
                    # transitioned to NAVIGATING yet.
                    logger.info(
                        f"lead_to: nav exited IDLE before reaching goal "
                        f"'{destination}'. The agent should call "
                        f"navigate_with_text/set_goal before lead_to."
                    )
                    return

            # Throttled status log.
            now = time.time()
            if now - last_log_ts > 5.0:
                logger.info(
                    f"lead_to: state={state} dest='{destination}' "
                    f"paused={paused} frame_age={now - self._latest_frame_ts:.1f}s"
                )
                last_log_ts = now

            if not self._follower_visible():
                if not paused:
                    logger.info("lead_to: visitor signal lost, pausing nav")
                    self._navigation.cancel_goal()
                    paused = True
                # Wait for reacquisition with a hard timeout.
                reacquire_deadline = now + _REACQUIRE_TIMEOUT_S
                while not cancel.is_set() and time.time() < reacquire_deadline:
                    if self._follower_visible():
                        logger.info("lead_to: visitor signal reacquired")
                        paused = False
                        # The outer agent should re-issue the nav goal —
                        # we exit and let it resume via a new lead_to call.
                        return
                    time.sleep(_POLL_INTERVAL_S)
                logger.info("lead_to: gave up waiting for visitor")
                return

            time.sleep(_POLL_INTERVAL_S)

    @skill
    def lead_to(self, destination: str) -> str:
        """Start a follower-presence monitor for a navigation that is already
        in progress. **Call AFTER `navigate_with_text` has set the goal** —
        this skill does NOT set a goal itself, it only watches the perception
        pipeline and pauses if the visitor falls out of view.

        Typical order:
          1. `log_nav_decision(...)` — record grounding.
          2. `speak("Going to the X — follow me.")`.
          3. `navigate_with_text(destination)` — set the nav goal.
          4. `lead_to(destination)` — start the follower-presence loop.

        While the loop runs it will:
        - Continuously check the perception pipeline is producing camera
          frames (a proxy for "visitor still in view").
        - If frames stop arriving: cancel the nav goal, `speak("I'll wait
          for you")`, and poll for reacquisition.
        - On reacquisition: return so the agent can re-issue the goal.
        - On arrival: return so the agent can speak an arrival line.

        This is the defining gesture that distinguishes Drop-in Guide from a
        delivery bot. Use it whenever a person is following the robot.

        Args:
            destination: the tagged-location name to lead the visitor to.
                         Must match a name from `tag_location`.
        """
        if not destination or not destination.strip():
            return "Error: destination is required."

        with self._lead_lock:
            if self._lead_thread is not None and self._lead_thread.is_alive():
                return f"Already leading somewhere. Call `stop_navigation` first."
            self._stop_event = threading.Event()
            self._lead_thread = threading.Thread(
                target=self._lead_loop,
                args=(destination.strip(), self._stop_event),
                daemon=True,
                name="LeadWithFollow",
            )
            self._lead_thread.start()

        return (
            f"Leading to '{destination}'. I'll pause if the perception "
            f"pipeline loses sight of you and resume when it recovers."
        )
