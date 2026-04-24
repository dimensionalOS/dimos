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

"""Go2 low-level standup example — joint-level control via ControlCoordinator.

Example module that computes joint positions and publishes them as
JointState commands.  The ControlCoordinator receives these over LCM,
runs them through per-joint arbitration, and writes to the
UnitreeGo2LowLevelAdapter.

Sequence:
  Phase 1: Interpolate from current → crouch position
  Phase 2: Interpolate from crouch → stand position
  Phase 3: Hold stand position (stabilize)
  Phase 4: Interpolate from stand → weight-shift position
  Phase 5: Lower each leg one by one (FR → FL → RR → RL)
  Phase 6: Lower diagonal pairs simultaneously (FR+RL, then FL+RR)
  Phase 7: Sit down
"""

from __future__ import annotations

import logging
import threading
import time
from typing import Any

from dimos.core import In, Module, Out, rpc
from dimos.msgs.sensor_msgs import JointState

logger = logging.getLogger(__name__)

# ---------------------------------------------------------------------------
# Target positions from Unitree SDK go2_stand_example.py
# Order: FR_0,1,2  FL_0,1,2  RR_0,1,2  RL_0,1,2
# ---------------------------------------------------------------------------
POS_CROUCH = [
    0.0,
    1.36,
    -2.65,  # FR
    0.0,
    1.36,
    -2.65,  # FL
    -0.2,
    1.36,
    -2.65,  # RR
    0.2,
    1.36,
    -2.65,  # RL
]

POS_STAND = [
    0.0,
    0.67,
    -1.3,  # FR
    0.0,
    0.67,
    -1.3,  # FL
    0.0,
    0.67,
    -1.3,  # RR
    0.0,
    0.67,
    -1.3,  # RL
]

POS_SHIFT = [
    -0.35,
    1.36,
    -2.65,  # FR
    0.35,
    1.36,
    -2.65,  # FL
    -0.5,
    1.36,
    -2.65,  # RR
    0.5,
    1.36,
    -2.65,  # RL
]

# Per-leg crouch positions (used for single-leg lower)
# Each is a 3-element list: [hip, thigh, calf]
LEG_CROUCH = [0.0, 1.36, -2.65]
LEG_STAND = [0.0, 0.67, -1.3]

# Leg index ranges within the 12-DOF array
LEG_FR = slice(0, 3)  # indices 0,1,2
LEG_FL = slice(3, 6)  # indices 3,4,5
LEG_RR = slice(6, 9)  # indices 6,7,8
LEG_RL = slice(9, 12)  # indices 9,10,11

# Phase durations in ticks (at CMD_HZ)
CMD_HZ = 50  # Command publish rate (Hz)
PHASE_1_TICKS = 250  # current → crouch
PHASE_2_TICKS = 250  # crouch → stand
PHASE_3_TICKS = 400  # hold stand
PHASE_4_TICKS = 400  # stand → shift
LEG_LOWER_TICKS = 150  # per-leg lower/raise
LEG_HOLD_TICKS = 100  # hold at bottom per leg
DIAG_LOWER_TICKS = 200  # diagonal pair lower/raise
DIAG_HOLD_TICKS = 150  # hold at bottom diagonal


class Go2LowLevelControl(Module):
    """Example module for Go2 low-level joint control.

    Computes joint position targets and publishes them as JointState.
    The ControlCoordinator handles arbitration and hardware IO.

    Ports:
        joint_state (In[JointState]): receives current joint positions
        joint_command (Out[JointState]): publishes target joint positions
    """

    joint_state: In[JointState]
    joint_command: Out[JointState]

    def __init__(self, *args: Any, **kwargs: Any) -> None:
        super().__init__(*args, **kwargs)
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None
        self._latest_state: JointState | None = None
        self._state_lock = threading.Lock()

    @rpc
    def start(self) -> None:
        super().start()
        self.joint_state.subscribe(self._on_joint_state)
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._run, daemon=True, name="go2-standup")
        self._thread.start()
        logger.info("Go2LowLevelControl started")

    @rpc
    def stop(self) -> None:
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=5.0)
            self._thread = None
        super().stop()
        logger.info("Go2LowLevelControl stopped")

    def _on_joint_state(self, msg: JointState) -> None:
        with self._state_lock:
            self._latest_state = msg

    def _get_positions(self) -> list[float] | None:
        """Extract position list from latest JointState."""
        with self._state_lock:
            if self._latest_state is None or not self._latest_state.position:
                return None
            return list(self._latest_state.position)

    def _get_joint_names(self) -> list[str]:
        """Get joint names from latest JointState."""
        with self._state_lock:
            if self._latest_state is None:
                return []
            return list(self._latest_state.name)

    # =========================================================================
    # Standup sequence
    # =========================================================================

    def _run(self) -> None:
        """Background thread: wait for state, countdown, then run standup."""
        # Wait for first joint state
        logger.info("Waiting for initial joint state...")
        while not self._stop_event.is_set():
            if self._get_positions() is not None:
                break
            time.sleep(0.1)
        if self._stop_event.is_set():
            return

        start_pos = self._get_positions()
        names = self._get_joint_names()
        if start_pos is None or not names:
            logger.error("No joint state received, aborting")
            return

        logger.info(f"Got initial state ({len(start_pos)} joints). Starting in 5s...")

        # Countdown (no input() — runs in Dask worker thread without stdin)
        for i in range(5, 0, -1):
            if self._stop_event.is_set():
                return
            logger.info(f"  {i}...")
            time.sleep(1.0)

        dt = 1.0 / CMD_HZ

        # Phase 1: current → crouch
        logger.info("Phase 1: crouch")
        self._interp(names, start_pos, POS_CROUCH, PHASE_1_TICKS, dt)

        # Phase 2: crouch → stand
        logger.info("Phase 2: stand")
        self._interp(names, POS_CROUCH, POS_STAND, PHASE_2_TICKS, dt)

        # Phase 3: hold stand
        logger.info("Phase 3: hold")
        self._hold(names, POS_STAND, PHASE_3_TICKS, dt)

        # Phase 4: stand → shift
        logger.info("Phase 4: shift")
        self._interp(names, POS_STAND, POS_SHIFT, PHASE_4_TICKS, dt)

        # Back to stand before leg lifts
        logger.info("Returning to stand...")
        self._interp(names, POS_SHIFT, POS_STAND, PHASE_2_TICKS, dt)
        self._hold(names, POS_STAND, PHASE_3_TICKS // 2, dt)

        # Phase 5: Lower each leg one by one
        logger.info("Phase 5: single leg lowers (FR → FL → RR → RL)")
        for leg_name, leg_slice in [("FR", LEG_FR), ("FL", LEG_FL), ("RR", LEG_RR), ("RL", LEG_RL)]:
            if self._stop_event.is_set():
                return
            logger.info(f"  Lowering {leg_name}...")
            self._lower_leg(names, POS_STAND, leg_slice, LEG_CROUCH, dt)

        # Phase 6: Diagonal pairs
        logger.info("Phase 6: diagonal pair lowers (FR+RL, then FL+RR)")
        for pair_name, slices in [("FR+RL", [LEG_FR, LEG_RL]), ("FL+RR", [LEG_FL, LEG_RR])]:
            if self._stop_event.is_set():
                return
            logger.info(f"  Lowering {pair_name}...")
            self._lower_diagonal(names, POS_STAND, slices, LEG_CROUCH, dt)

        # Phase 7: Sit down
        logger.info("Phase 7: sit down")
        self._interp(names, POS_STAND, POS_CROUCH, PHASE_1_TICKS, dt)
        self._hold(names, POS_CROUCH, PHASE_3_TICKS // 2, dt)

        logger.info("Full sequence complete!")

    def _interp(
        self,
        names: list[str],
        start: list[float],
        end: list[float],
        ticks: int,
        dt: float,
    ) -> None:
        """Linear interpolation between two poses over *ticks* steps."""
        for t in range(ticks):
            if self._stop_event.is_set():
                return
            alpha = (t + 1) / ticks
            pos = [(1 - alpha) * s + alpha * e for s, e in zip(start, end, strict=False)]
            self._pub(names, pos)
            time.sleep(dt)

    def _hold(self, names: list[str], pos: list[float], ticks: int, dt: float) -> None:
        """Hold a fixed pose for *ticks* steps."""
        for _ in range(ticks):
            if self._stop_event.is_set():
                return
            self._pub(names, pos)
            time.sleep(dt)

    def _lower_leg(
        self,
        names: list[str],
        base_pose: list[float],
        leg: slice,
        target: list[float],
        dt: float,
    ) -> None:
        """Lower a single leg from base_pose to target, hold, then raise back."""
        start = list(base_pose)
        end = list(base_pose)
        end[leg] = target

        # Lower
        self._interp(names, start, end, LEG_LOWER_TICKS, dt)
        # Hold
        self._hold(names, end, LEG_HOLD_TICKS, dt)
        # Raise back
        self._interp(names, end, start, LEG_LOWER_TICKS, dt)
        # Brief stabilize
        self._hold(names, start, LEG_HOLD_TICKS // 2, dt)

    def _lower_diagonal(
        self,
        names: list[str],
        base_pose: list[float],
        legs: list[slice],
        target: list[float],
        dt: float,
    ) -> None:
        """Lower a diagonal pair simultaneously, hold, then raise back."""
        start = list(base_pose)
        end = list(base_pose)
        for leg in legs:
            end[leg] = target

        # Lower both
        self._interp(names, start, end, DIAG_LOWER_TICKS, dt)
        # Hold
        self._hold(names, end, DIAG_HOLD_TICKS, dt)
        # Raise both back
        self._interp(names, end, start, DIAG_LOWER_TICKS, dt)
        # Brief stabilize
        self._hold(names, start, DIAG_HOLD_TICKS // 2, dt)

    def _pub(self, names: list[str], positions: list[float]) -> None:
        """Publish a JointState position command."""
        msg = JointState(name=names, position=positions)
        self.joint_command.publish(msg)


# Blueprint export
go2_low_level_control = Go2LowLevelControl.blueprint

__all__ = ["Go2LowLevelControl", "go2_low_level_control"]
