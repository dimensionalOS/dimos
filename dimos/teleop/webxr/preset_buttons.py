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

"""Quest controller buttons that send an arm to a saved pose during VR teleop."""

from __future__ import annotations

from collections.abc import Sequence
import threading
from typing import Any

from reactivex.disposable import Disposable

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In
from dimos.manipulation.manipulation_spec import ManipulationSpec, PlanningGroupInfo
from dimos.manipulation.planning.spec.models import PlanningGroupID
from dimos.manipulation.sdk import Arm
from dimos.teleop.webxr.controller_types import Buttons
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

# The planner's preset names, each with the Buttons bit that triggers it. Left X
# goes to "init", the pose the arm had when the stack started; left Y to "home".
_PRESET_BITS: dict[str, int] = {
    "init": 1 << Buttons.BITS["left_primary"],
    "home": 1 << Buttons.BITS["left_secondary"],
}
_GRIP_BITS = (1 << Buttons.BITS["left_grip"]) | (1 << Buttons.BITS["right_grip"])


class PresetButtonsConfig(ModuleConfig):
    # Planning group to move. None picks the model's only arm.
    planning_group: PlanningGroupID | None = None


class PresetButtonsModule(Module):
    """Quest buttons that send the arm to a saved pose: left X to its start pose, Y home.

    Each press plans a collision-free path and runs it. Presses are ignored while
    either grip is held (VR teleop engaged) or a preset move is running. Squeezing
    a grip during a preset move cancels it so the operator takes over.
    """

    config: PresetButtonsConfig

    teleop_buttons: In[Buttons]
    manipulation: ManipulationSpec

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._lock = threading.Lock()
        self._held = 0
        self._moving = False
        self._cancelling = False
        self._arm: Arm | None = None

    @rpc
    def start(self) -> None:
        super().start()
        self.register_disposable(Disposable(self.teleop_buttons.subscribe(self._on_buttons)))

    @rpc
    def stop(self) -> None:
        super().stop()

    def _on_buttons(self, msg: Buttons) -> None:
        """Start a preset move on a fresh X or Y press; cancel it when a grip is squeezed."""
        held = msg.data & Buttons.DIGITAL_MASK
        cancel = False
        preset: str | None = None
        with self._lock:
            pressed = held & ~self._held
            self._held = held
            if held & _GRIP_BITS:
                if self._moving and not self._cancelling:
                    self._cancelling = True
                    cancel = True
            elif not self._moving:
                preset = next((name for name, bit in _PRESET_BITS.items() if pressed & bit), None)
                if preset is not None:
                    self._moving = True
        # Planning and moving take seconds, so they run on their own thread
        # instead of holding up the button stream.
        if cancel:
            threading.Thread(target=self._cancel_move, name="preset-cancel", daemon=True).start()
        if preset is not None:
            threading.Thread(
                target=self._move_to, args=(preset,), name="preset-move", daemon=True
            ).start()

    def _move_to(self, preset: str) -> None:
        """Plan and run a move to `preset` ("init" or "home"), logging instead of raising."""
        try:
            self._selected_arm().move_to_preset(preset)
            logger.info("Arm reached preset", preset=preset)
        except Exception as exc:
            with self._lock:
                cancelled = self._cancelling
            if cancelled:
                logger.info("Preset move cancelled; VR teleop has the arm", preset=preset)
            else:
                logger.warning("Preset move not done", preset=preset, reason=str(exc))
        finally:
            with self._lock:
                self._moving = False
                self._cancelling = False

    def _cancel_move(self) -> None:
        """Stop the running preset move so VR teleop can take over."""
        try:
            self.manipulation.cancel()
        except Exception as exc:
            logger.warning("Could not cancel preset move", reason=str(exc))

    def _selected_arm(self) -> Arm:
        """The arm the buttons move, looked up from the planner on first use."""
        if self._arm is None:
            groups = self.manipulation.list_planning_groups()
            self._arm = Arm(self.manipulation, _pick_group(groups, self.config.planning_group))
        return self._arm


def _pick_group(
    groups: Sequence[PlanningGroupInfo], wanted: PlanningGroupID | None
) -> PlanningGroupInfo:
    """Pick the planning group to move: `wanted` if given, else the model's only arm.

    Among several arms, the only one with a gripper wins. Raises ValueError when
    that still leaves no single group.
    """
    if wanted is not None:
        matches = [group for group in groups if group.id == wanted]
    else:
        matches = [group for group in groups if group.tip_frame is not None]
        if len(matches) > 1:
            matches = [group for group in matches if group.has_gripper]
    if len(matches) != 1:
        raise ValueError(
            f"no single arm among planning groups {[group.id for group in groups]}; "
            "set planning_group"
        )
    return matches[0]
