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

"""Move planning groups to the zero joint pose when a WebXR button is pressed."""

from __future__ import annotations

import threading

from pydantic import Field, model_validator
from reactivex.disposable import Disposable

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In
from dimos.manipulation.manipulation_spec import ManipulationSpec
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.teleop.webxr.controller_types import Buttons
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class JointResetModuleConfig(ModuleConfig):
    # Buttons.BITS name -> planning groups that button resets.
    button_groups: dict[str, list[str]] = Field(default_factory=dict)

    @model_validator(mode="after")
    def _validate_buttons(self) -> JointResetModuleConfig:
        unknown = sorted(set(self.button_groups) - set(Buttons.BITS))
        if unknown:
            raise ValueError(f"unknown WebXR buttons: {unknown}")
        return self


class JointResetModule(Module):
    """Reset planning groups to zero, ignoring presses until the move ends.

    The move runs through the trajectory task, so engaged teleop overrides it.
    """

    config: JointResetModuleConfig

    button_pressed: In[Buttons]

    _manipulation: ManipulationSpec

    @rpc
    def start(self) -> None:
        super().start()
        self._busy = threading.Lock()
        self.register_disposable(Disposable(self.button_pressed.subscribe(self._on_button_pressed)))

    def _on_button_pressed(self, buttons: Buttons) -> None:
        groups = [
            group
            for button, button_groups in self.config.button_groups.items()
            if getattr(buttons, button)
            for group in button_groups
        ]
        if groups and self._busy.acquire(blocking=False):
            threading.Thread(target=self._reset, args=(groups,), daemon=True).start()

    def _reset(self, groups: list[str]) -> None:
        try:
            joints = {
                group.id: group.joint_names for group in self._manipulation.list_planning_groups()
            }
            targets = {
                group: JointState(name=list(joints[group]), position=[0.0] * len(joints[group]))
                for group in groups
            }
            plan = self._manipulation.plan_to_joints(targets)
            if not plan.succeeded or plan.plan is None:
                logger.warning("Joint reset planning failed", groups=groups, reason=plan.message)
                return
            result = self._manipulation.execute(plan_id=plan.plan.plan_id)
            logger.info("Joint reset", groups=groups, status=result.status.name)
        except Exception:
            logger.exception("Joint reset failed", groups=groups)
        finally:
            self._busy.release()
