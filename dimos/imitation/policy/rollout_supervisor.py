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

"""Serialize operator requests into one fail-closed policy rollout lifecycle."""

from __future__ import annotations

from enum import Enum, auto
from queue import Queue
import threading
from typing import Any, Protocol

from reactivex.disposable import Disposable

from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In
from dimos.imitation.policy.lerobot.module import RolloutStatus
from dimos.msgs.std_msgs.Bool import Bool
from dimos.spec.utils import Spec
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

POLICY_ROLLOUT_TASK_NAME = "policy_rollout"
POLICY_GRIPPER_TASK_NAME = "policy_gripper"
POLICY_TASK_NAMES = (POLICY_ROLLOUT_TASK_NAME, POLICY_GRIPPER_TASK_NAME)


class PolicyRolloutSpec(Spec, Protocol):
    def start_rollout(self, duration: float | None = None) -> RolloutStatus: ...

    def stop_rollout(self) -> RolloutStatus: ...


class PolicyControlSpec(Spec, Protocol):
    def task_invoke(
        self,
        task_name: str,
        method: str,
        kwargs: dict[str, Any] | None = None,
    ) -> Any: ...


class PolicyRolloutSupervisorConfig(ModuleConfig):
    """Control tasks that must be activated as one policy rollout."""

    task_names: tuple[str, ...] = POLICY_TASK_NAMES


class _Request(Enum):
    START = auto()
    STOP = auto()
    TOGGLE = auto()
    OVERRIDE_STARTED = auto()
    OVERRIDE_ENDED = auto()
    SHUTDOWN = auto()


class PolicyRolloutSupervisor(Module):
    """Own policy/task activation without blocking input or RPC handlers."""

    config: PolicyRolloutSupervisorConfig

    _policy: PolicyRolloutSpec
    _control: PolicyControlSpec

    rollout_toggle: In[Bool]
    manual_override: In[Bool]

    def __init__(self, **kwargs: object) -> None:
        super().__init__(**kwargs)
        self._requests: Queue[_Request] = Queue()
        self._worker: threading.Thread | None = None
        self._rollout_active = False
        self._override_active = False

    @rpc
    def start(self) -> None:
        super().start()
        self._worker = threading.Thread(
            target=self._run,
            name="PolicyRolloutSupervisor",
            daemon=True,
        )
        self._worker.start()
        self.register_disposable(Disposable(self.rollout_toggle.subscribe(self._on_toggle)))
        self.register_disposable(Disposable(self.manual_override.subscribe(self._on_override)))

    @rpc
    def stop(self) -> None:
        worker = self._worker
        if worker is not None and worker.is_alive():
            self._requests.put(_Request.SHUTDOWN)
            worker.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)
            if worker.is_alive():
                logger.error("Policy rollout supervisor did not stop")
        self._worker = None
        super().stop()

    @rpc
    def request_rollout_start(self) -> None:
        """Queue a rollout start request and return immediately."""
        self._requests.put(_Request.START)

    @rpc
    def request_rollout_stop(self) -> None:
        """Queue a rollout stop request and return immediately."""
        self._requests.put(_Request.STOP)

    @rpc
    def request_rollout_toggle(self) -> None:
        """Queue a rollout toggle request and return immediately."""
        self._requests.put(_Request.TOGGLE)

    def _on_toggle(self, message: Bool) -> None:
        if message.data:
            self.request_rollout_toggle()

    def _on_override(self, message: Bool) -> None:
        self._requests.put(_Request.OVERRIDE_STARTED if message.data else _Request.OVERRIDE_ENDED)

    def _run(self) -> None:
        while True:
            request = self._requests.get()
            try:
                if request is _Request.SHUTDOWN:
                    return
                self._handle(request)
            except Exception:
                logger.exception("Policy rollout request failed", request=request.name)
                self._stop_rollout()

    def _handle(self, request: _Request) -> None:
        if request is _Request.OVERRIDE_STARTED:
            self._override_active = True
            self._stop_rollout()
        elif request is _Request.OVERRIDE_ENDED:
            self._override_active = False
        elif request is _Request.STOP:
            self._stop_rollout()
        elif request is _Request.START:
            self._start_rollout()
        elif request is _Request.TOGGLE:
            if self._rollout_active:
                self._stop_rollout()
            else:
                self._start_rollout()

    def _start_rollout(self) -> None:
        if self._rollout_active or self._override_active:
            return

        activated: list[str] = []
        try:
            for task_name in self.config.task_names:
                if self._control.task_invoke(task_name, "activate") is not True:
                    raise RuntimeError(f"failed to activate control task {task_name!r}")
                activated.append(task_name)
            status = self._policy.start_rollout()
            self._rollout_active = status["active"]
            if not self._rollout_active:
                raise RuntimeError(status["last_error"] or "policy did not start")
            logger.info("Policy rollout started", status=status)
        except Exception:
            for task_name in reversed(activated):
                self._deactivate_task(task_name)
            raise

    def _stop_rollout(self) -> None:
        try:
            status = self._policy.stop_rollout()
            logger.info("Policy rollout stopped", status=status)
        except Exception:
            logger.exception("Policy rollout stop failed")
        finally:
            self._rollout_active = False
            for task_name in self.config.task_names:
                self._deactivate_task(task_name)

    def _deactivate_task(self, task_name: str) -> None:
        try:
            if self._control.task_invoke(task_name, "deactivate") is not True:
                logger.error("Control task did not deactivate", task_name=task_name)
        except Exception:
            logger.exception("Control task deactivation failed", task_name=task_name)
