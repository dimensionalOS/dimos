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

"""Translate Quest and coordinator events into policy rollout lifecycle calls."""

from __future__ import annotations

import threading
from typing import Protocol

from reactivex.disposable import Disposable

from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.stream import In
from dimos.imitation.policy.lerobot.module import RolloutStatus
from dimos.msgs.control_msgs.TaskPreemption import TaskPreemption
from dimos.spec.utils import Spec
from dimos.teleop.quest.quest_types import Buttons
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

POLICY_ROLLOUT_TASK_NAME = "policy_rollout"


class PolicyRolloutSpec(Spec, Protocol):
    def start_rollout(self, duration: float | None = None) -> RolloutStatus: ...

    def stop_rollout(self) -> RolloutStatus: ...

    def rollout_status(self) -> RolloutStatus: ...


class QuestRolloutControllerModule(Module):
    """Toggle rollout with Quest A and cancel it on coordinator preemption."""

    _policy: PolicyRolloutSpec

    teleop_buttons: In[Buttons]
    task_preempted: In[TaskPreemption]

    def __init__(self, **kwargs: object) -> None:
        super().__init__(**kwargs)
        self._lock = threading.Lock()
        self._a_was_pressed = False

    @rpc
    def start(self) -> None:
        super().start()
        self.register_disposable(Disposable(self.teleop_buttons.subscribe(self._on_buttons)))
        self.register_disposable(Disposable(self.task_preempted.subscribe(self._on_task_preempted)))

    def _on_buttons(self, buttons: Buttons) -> None:
        with self._lock:
            pressed = buttons.right_primary
            rising_edge = pressed and not self._a_was_pressed
            self._a_was_pressed = pressed
            if not rising_edge:
                return
            if buttons.left_grip or buttons.right_grip:
                logger.info("Ignoring rollout toggle while arm teleop is engaged")
                return

            if self._policy.rollout_status()["active"]:
                status = self._policy.stop_rollout()
                logger.info("Policy rollout stopped from Quest A", status=status)
            else:
                status = self._policy.start_rollout()
                logger.info("Policy rollout requested from Quest A", status=status)

    def _on_task_preempted(self, event: TaskPreemption) -> None:
        if event.preempted_task != POLICY_ROLLOUT_TASK_NAME:
            return
        with self._lock:
            status = self._policy.stop_rollout()
            logger.info(
                "Policy rollout cancelled by control preemption",
                preempting_task=event.preempting_task,
                joints=event.joints,
                status=status,
            )
