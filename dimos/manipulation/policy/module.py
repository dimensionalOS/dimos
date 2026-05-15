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

"""`PolicyModule` DimOS module.

Connects a single typed camera image, the latest robot joint state, a
task description, and the teleop `Buttons` stream to a configured
`PolicyBackend`. Runs inference at `PolicyModuleConfig.policy_rate` on its
own thread (kept off the `ControlCoordinator` tick loop) and publishes
joint position commands as `JointState` on `joint_command`.

The single `image: In[Image]` stream slot is exposed under
`PolicyModuleConfig.camera_key` in the `PolicyObservation.images` dict
handed to the backend. Multi-camera support is intentionally out of
scope; a future change extends the slot set from this baseline when a
deployment requires it.

Teleop preempts: when any button in `teleop_engage_buttons` goes high,
publication is suspended and `backend.reset()` is called. On disengage,
publication resumes from a fresh `select_action()` against the current
observation; pre-engage buffered actions are never replayed. Engagement
is also re-checked after `backend.select_action()` returns so a command
computed against pre-engage state is dropped if engagement fires during
an in-flight inference call.

Rollout gating: the node starts with `_rollout_enabled = False` so the
operator must explicitly call `start_rollout()` before commands publish.
`stop_rollout()` re-disables publication. Both transitions call
`backend.reset()` so any buffered chunk is discarded across them.
`is_rollout_active()` reports the current state for sibling modules
(see `RolloutToggle`).

Mandatory buttons subscription: when `teleop_engage_buttons` is
non-empty, the `buttons` subscription is a hard requirement of `start()`
and publication is additionally gated on having received at least one
`Buttons` message (with a `buttons_grace_period` warning after timeout).
"""

from __future__ import annotations

from collections.abc import Mapping
import threading
import time
from typing import TYPE_CHECKING, Any

from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.stream import In, Out

# Importing the backends package registers built-in backends ("test",
# "lerobot") on the shared registry. Without this import, fresh sessions
# would only see backends registered by the caller.
import dimos.manipulation.policy.backends  # noqa: F401
from dimos.manipulation.policy.command import (
    JointPositionCommand,
    NoOpCommand,
    PolicyCommand,
)
from dimos.manipulation.policy.config import CommandMode, PolicyModuleConfig
from dimos.manipulation.policy.observation import PolicyObservation
from dimos.manipulation.policy.registry import create_backend
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.teleop.quest.quest_types import Buttons
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from dimos.manipulation.policy.backend import PolicyBackend

logger = setup_logger()


class PolicyModule(Module):
    """Streams perception → policy → coordinator-native commands.

    Inputs:
        image: Camera image. Surfaced to the backend under
            `PolicyModuleConfig.camera_key` in `PolicyObservation.images`.
        joint_state: Latest robot joint state.
        task_description: Plain text task. Falls back to
            `PolicyModuleConfig.default_task` when no value has arrived.
        buttons: Teleop button state used to detect engage/disengage and
            preempt the policy.

    Output:
        joint_command: `JointState` whose `name`/`position` are wired to
            `ControlCoordinator.joint_command`.
    """

    config: PolicyModuleConfig

    image: In[Image]
    joint_state: In[JointState]
    task_description: In[str]
    buttons: In[Buttons]

    joint_command: Out[JointState]

    def __init__(self, *args: Any, **kwargs: Any) -> None:
        super().__init__(*args, **kwargs)

        self._latest_lock = threading.Lock()
        self._latest_image: Image | None = None
        self._latest_joint_state: JointState | None = None
        self._latest_joint_state_ts: float = 0.0
        self._latest_task: str | None = None
        self._latest_buttons: Buttons | None = None

        self._engage_lock = threading.Lock()
        self._engaged: bool = False
        self._rollout_enabled: bool = False

        # First-Buttons-message gate (only consulted when teleop_engage_buttons
        # is non-empty). Both fields are protected by _engage_lock for
        # simplicity since they're touched by the inference loop and the
        # button-handler.
        self._first_buttons_received: bool = False
        self._start_monotonic: float = 0.0
        self._grace_warning_logged: bool = False

        self._backend: PolicyBackend | None = None
        self._backend_lock = threading.Lock()

        self._inference_thread: threading.Thread | None = None
        self._stop_event = threading.Event()
        self._unsub: list[Any] = []

    # ── lifecycle ─────────────────────────────────────────────────────────

    @rpc
    def start(self) -> None:
        super().start()
        with self._backend_lock:
            self._backend = create_backend(self.config.backend, **self.config.backend_config)
            self._backend.initialize()

        with self._engage_lock:
            self._first_buttons_received = False
            self._grace_warning_logged = False
            self._start_monotonic = time.monotonic()

        self._subscribe_inputs()

        self._stop_event.clear()
        thread = threading.Thread(
            target=self._inference_loop,
            name=f"policy-module[{self.config.backend}]",
            daemon=True,
        )
        self._inference_thread = thread
        thread.start()
        logger.info(
            f"PolicyModule started: backend={self.config.backend} rate={self.config.policy_rate}Hz"
        )

    @rpc
    def start_rollout(self) -> None:
        """Enable command publication. Calls `backend.reset()`."""
        with self._engage_lock:
            self._rollout_enabled = True
        self._reset_backend("rollout started")
        logger.info("PolicyModule: rollout started")

    @rpc
    def stop_rollout(self) -> None:
        """Disable command publication. Calls `backend.reset()`."""
        with self._engage_lock:
            self._rollout_enabled = False
        self._reset_backend("rollout stopped")
        logger.info("PolicyModule: rollout stopped")

    @rpc
    def is_rollout_active(self) -> bool:
        """Return whether the operator has enabled policy rollout.

        Decorated `@rpc` so sibling modules (e.g., `RolloutToggle`) can
        read it through the cross-module proxy.
        """
        with self._engage_lock:
            return self._rollout_enabled

    @rpc
    def stop(self) -> None:
        self._stop_event.set()
        for unsub in self._unsub:
            try:
                unsub()
            except Exception:
                logger.exception("PolicyModule: error unsubscribing input")
        self._unsub = []

        thread = self._inference_thread
        self._inference_thread = None
        if thread is not None and thread.is_alive():
            thread.join(timeout=2.0)

        with self._backend_lock:
            backend = self._backend
            self._backend = None
        if backend is not None:
            try:
                backend.close()
            except Exception:
                logger.exception("PolicyModule: backend.close() raised")

        super().stop()
        logger.info("PolicyModule stopped")

    # ── input subscription ────────────────────────────────────────────────

    def _subscribe_inputs(self) -> None:
        try:
            self._unsub.append(self.image.subscribe(self._on_image))
        except Exception:
            logger.warning("PolicyModule: could not subscribe to image")

        try:
            self._unsub.append(self.joint_state.subscribe(self._on_joint_state))
        except Exception:
            logger.warning("PolicyModule: could not subscribe to joint_state")

        try:
            self._unsub.append(self.task_description.subscribe(self._on_task_description))
        except Exception:
            logger.debug("PolicyModule: task_description not connected")

        if self.config.teleop_engage_buttons:
            # Hard requirement: subscription failure aborts startup.
            self._unsub.append(self.buttons.subscribe(self._on_buttons))
        else:
            try:
                self._unsub.append(self.buttons.subscribe(self._on_buttons))
            except Exception:
                logger.debug("PolicyModule: buttons not connected (no teleop preempt)")

    # ── handlers (also called directly from unit tests) ───────────────────

    def _on_image(self, msg: Image) -> None:
        with self._latest_lock:
            self._latest_image = msg

    def _on_joint_state(self, msg: JointState) -> None:
        with self._latest_lock:
            self._latest_joint_state = msg
            self._latest_joint_state_ts = time.time()

    def _on_task_description(self, msg: str) -> None:
        with self._latest_lock:
            self._latest_task = msg

    def _on_buttons(self, msg: Buttons) -> None:
        with self._latest_lock:
            self._latest_buttons = msg
        engaged_now = self._is_engaged(msg)
        with self._engage_lock:
            was_engaged = self._engaged
            self._engaged = engaged_now
            self._first_buttons_received = True

        if engaged_now and not was_engaged:
            self._reset_backend("teleop engaged")

    def _is_engaged(self, buttons: Buttons) -> bool:
        for field in self.config.teleop_engage_buttons:
            if getattr(buttons, field, False):
                return True
        return False

    # ── observation assembly ──────────────────────────────────────────────

    def assemble_observation(self) -> PolicyObservation:
        """Snapshot the latest inputs into a `PolicyObservation`.

        Public so blueprint examples and tests can build observations
        directly without going through the inference thread.
        """
        with self._latest_lock:
            image = self._latest_image
            joint_state = self._latest_joint_state
            task = self._latest_task
            joint_ts = self._latest_joint_state_ts
        if task is None:
            task = self.config.default_task
        images: dict[str, Image] = {self.config.camera_key: image} if image is not None else {}
        return PolicyObservation(
            images=images,
            joint_state=joint_state,
            task=task,
            timestamp=joint_ts,
        )

    # ── inference loop ────────────────────────────────────────────────────

    def _inference_loop(self) -> None:
        rate = max(0.1, float(self.config.policy_rate))
        period = 1.0 / rate

        while not self._stop_event.is_set():
            t0 = time.monotonic()
            try:
                self._tick_once()
            except Exception:
                logger.exception("PolicyModule: tick_once raised")
            elapsed = time.monotonic() - t0
            sleep = period - elapsed
            if sleep > 0:
                # Use Event.wait so stop() can interrupt the sleep promptly.
                self._stop_event.wait(timeout=sleep)

    def _tick_once(self) -> PolicyCommand | None:
        """Run a single inference step.

        Returns the command produced (`PolicyCommand`) for testing, or
        `None` if the step was skipped (preempted, missing inputs, rollout
        disabled, awaiting first Buttons message, or backend not yet
        initialized).
        """
        if not self._publication_allowed_pre_inference():
            return None

        backend = self._backend
        if backend is None:
            return None

        observation = self.assemble_observation()
        if observation.joint_state is None:
            return None

        if self.config.observation_max_age > 0.0:
            age = time.time() - observation.timestamp
            if age > self.config.observation_max_age:
                logger.warning(
                    f"PolicyModule: skipping tick, joint_state age {age:.3f}s exceeds "
                    f"observation_max_age {self.config.observation_max_age:.3f}s"
                )
                return None

        try:
            command = backend.select_action(observation)
        except Exception:
            logger.exception("PolicyModule: backend.select_action raised")
            return None

        # Re-check gates after inference: an engage edge or stop_rollout()
        # may have fired during the (potentially slow) select_action call.
        # Dropping the command here keeps preempt strict even when the
        # button-handler thread runs concurrently with inference.
        with self._engage_lock:
            if self._engaged or not self._rollout_enabled:
                return command

        self._publish_if_valid(command)
        return command

    def _publication_allowed_pre_inference(self) -> bool:
        """Gate checked before `select_action()` runs.

        Combines the engagement gate, the rollout gate, and (when
        `teleop_engage_buttons` is configured) the first-Buttons-message
        gate. The grace-period warning is logged here so it fires each
        tick the gate stays closed.
        """
        with self._engage_lock:
            if self._engaged:
                return False
            if not self._rollout_enabled:
                return False
            if self.config.teleop_engage_buttons and not self._first_buttons_received:
                elapsed = time.monotonic() - self._start_monotonic
                if elapsed > self.config.buttons_grace_period and not self._grace_warning_logged:
                    logger.warning(
                        "PolicyModule: no Buttons message received within "
                        f"{self.config.buttons_grace_period:.1f}s of start "
                        "(teleop_engage_buttons is configured); continuing "
                        "to suppress publication until a Buttons message "
                        "arrives."
                    )
                    self._grace_warning_logged = True
                return False
        return True

    # ── command validation + publishing ───────────────────────────────────

    def _publish_if_valid(self, command: PolicyCommand) -> None:
        if isinstance(command, NoOpCommand):
            if command.reason:
                logger.debug(f"PolicyModule: backend no-op ({command.reason})")
            return

        if isinstance(command, JointPositionCommand):
            if "joint_position" not in self.config.enabled_command_modes:
                logger.warning(
                    "PolicyModule: backend returned joint_position but the command mode "
                    "is not enabled in PolicyModuleConfig.enabled_command_modes"
                )
                return
            joint_state = self._joint_position_command_to_joint_state(command)
            if joint_state is None:
                return
            self.joint_command.publish(joint_state)
            return

        logger.warning(
            f"PolicyModule: unsupported PolicyCommand kind '{getattr(command, 'kind', '?')}'"
        )

    def _joint_position_command_to_joint_state(
        self, command: JointPositionCommand
    ) -> JointState | None:
        try:
            mapped_names = [self._map_joint_name(n) for n in command.joint_names]
        except KeyError as exc:
            logger.warning(f"PolicyModule: rejecting command with unmapped joint {exc}")
            return None

        configured = list(self.config.joint_names)
        if configured:
            if list(mapped_names) != configured:
                logger.warning(
                    "PolicyModule: rejecting command — backend joints do not match "
                    f"PolicyModuleConfig.joint_names\n  backend: {list(mapped_names)}\n  "
                    f"configured: {configured}"
                )
                return None

        return JointState(
            name=list(mapped_names),
            position=list(command.positions),
            frame_id=self.frame_id,
        )

    def _map_joint_name(self, name: str) -> str:
        return self.config.joint_name_map.get(name, name)

    # ── teleop preempt support ────────────────────────────────────────────

    def _reset_backend(self, reason: str) -> None:
        backend = self._backend
        if backend is None:
            return
        try:
            backend.reset()
            logger.info(f"PolicyModule: backend.reset() called ({reason})")
        except Exception:
            logger.exception("PolicyModule: backend.reset() raised")

    # ── test/inspection helpers ───────────────────────────────────────────

    def latest(self) -> Mapping[str, Any]:
        """Snapshot of the latest cached inputs (for tests/diagnostics).

        ``image`` is the latest cached `Image` or ``None`` if no frame has
        arrived yet. The corresponding entry under the configured
        ``camera_key`` is what shows up in `PolicyObservation.images`.
        """
        with self._latest_lock:
            return {
                "image": self._latest_image,
                "joint_state": self._latest_joint_state,
                "task": self._latest_task,
                "buttons": self._latest_buttons,
            }

    def is_engaged(self) -> bool:
        """Return whether teleop is currently treated as engaged."""
        with self._engage_lock:
            return self._engaged


__all__ = ["CommandMode", "PolicyModule", "PolicyModuleConfig"]
