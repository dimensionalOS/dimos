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

"""Scan, click-to-target and pick commands from an operator page."""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass, field, replace
import math
import threading
import time
from typing import Any

from reactivex.disposable import Disposable

from dimos.agents.skill_result import SkillResult
from dimos.control.coordinator import ControlCoordinator
from dimos.control.tasks.operator_hold_task.operator_hold_task import (
    OPERATOR_HOLD_TASK_NAME,
    OperatorEstopStatus,
)
from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.rpc_client import RpcCall
from dimos.core.stream import In, Out
from dimos.manipulation.pick_and_place_module import PickAndPlaceModule
from dimos.perception.experimental.object_scene_registration_spec import ObjectSceneRegistrationSpec
from dimos.utils.generic import finite_number
from dimos.utils.logging_config import setup_logger
from dimos.web.codecs import web_decoder

logger = setup_logger()

# A click outside every box still targets the nearest box centre this close, in pixels.
NEAR_CLICK_PX = 60.0
# PickAndPlaceModule's message for a pick that ended holding the object.
PICK_COMPLETE = "Pick complete"
# Every action operator_command accepts.
OPERATOR_ACTIONS = ("scan", "target", "pick", "clear")
# Commands that call other modules and can take minutes; they run on a worker thread.
_LONG_ACTIONS = ("scan", "pick")


@dataclass(frozen=True)
class OperatorCommand:
    """One command from the operator page.

    Attributes:
        action: "scan", "target", "pick" or "clear".
        prompts: Names of the objects to look for, e.g. ["red ball"]; read by "scan".
        x: Column of a click in colour-image pixels; -1 when there is no click.
        y: Row of a click in colour-image pixels; -1 when there is no click.
        object_id: Object to target by id instead of by click; "" for none.
    """

    action: str
    prompts: list[str] = field(default_factory=list)
    x: float = -1.0
    y: float = -1.0
    object_id: str = ""


@dataclass(frozen=True)
class ObjectBox:
    """One scanned object and where it is in the camera image.

    Attributes:
        object_id: Id from the scan; a pick names the object by it.
        name: What the object was detected as, e.g. "red ball".
        bbox: [x1, y1, x2, y2], the left, top, right and bottom edges of its
            box in colour-image pixels; [] when it has no box.
    """

    object_id: str
    name: str
    bbox: list[int] = field(default_factory=list)


@dataclass(frozen=True)
class PickResult:
    """How the last scan or pick ended.

    Attributes:
        ok: True when the scan ran, or the pick ended holding the object.
        message: The scan's or pick's own message, or the error.
        duration_ms: How long it ran, in milliseconds.
    """

    ok: bool
    message: str
    duration_ms: int = 0


@dataclass(frozen=True)
class OperatorState:
    """Everything the operator page shows about scanning, targeting and picking.

    Attributes:
        stamp: Unix time in seconds this state was sent.
        busy: True while a scan or pick runs.
        last_action: Action of the last command; "" before the first.
        error: Why the last command was refused or failed; "" when it was not.
        objects: The objects the last scan found.
        target_id: object_id of the object to pick; "" for none.
        target_name: Name of the object to pick; "" for none.
        released_hold: True when the last pick released an operator hold first.
        last_result: How the last scan or pick ended; None before the first.
    """

    stamp: float = 0.0
    busy: bool = False
    last_action: str = ""
    error: str = ""
    objects: list[ObjectBox] = field(default_factory=list)
    target_id: str = ""
    target_name: str = ""
    released_hold: bool = False
    last_result: PickResult | None = None


@web_decoder("operator_command.json.v1")
def decode_operator_command(value: dict[str, Any]) -> OperatorCommand:
    """Read one command the operator page sent; raise ValueError when it is malformed.

    Args:
        value: Parsed JSON, e.g. ``{"action": "scan", "prompts": ["ball"]}``,
            ``{"action": "target", "x": 130, "y": 140}`` (colour-image pixels),
            ``{"action": "target", "object_id": "..."}``, ``{"action": "pick"}``
            or ``{"action": "clear"}``. A missing key takes the OperatorCommand
            default.
    """
    if not isinstance(value, dict):
        raise ValueError(f"operator command must be an object, got {type(value).__name__}")
    action = value.get("action")
    if action not in OPERATOR_ACTIONS:
        raise ValueError(f"action must be one of {OPERATOR_ACTIONS}, got {action!r}")
    prompts = value.get("prompts", [])
    if not isinstance(prompts, list) or not all(isinstance(prompt, str) for prompt in prompts):
        raise ValueError(f"prompts must be a list of strings, got {prompts!r}")
    object_id = value.get("object_id", "")
    if not isinstance(object_id, str):
        raise ValueError(f"object_id must be a string, got {type(object_id).__name__}")
    return OperatorCommand(
        action=action,
        prompts=prompts,
        x=finite_number(value.get("x", -1.0), "x"),
        y=finite_number(value.get("y", -1.0), "y"),
        object_id=object_id,
    )


class _RefusedError(Exception):
    """A command that cannot run now; the message goes to the operator as the state's error."""


def object_at(objects: list[ObjectBox], x: float, y: float) -> ObjectBox | None:
    """The object an operator meant by clicking one camera pixel, or None.

    The smallest box around the pixel wins. With none around it, the object
    whose box centre is nearest wins if it is within NEAR_CLICK_PX.

    Args:
        objects: Scanned objects; an object with an empty bbox is skipped.
        x: Column of the click in colour-image pixels.
        y: Row of the click in colour-image pixels.
    """
    boxed = [obj for obj in objects if len(obj.bbox) == 4]
    inside = [
        obj for obj in boxed if obj.bbox[0] <= x <= obj.bbox[2] and obj.bbox[1] <= y <= obj.bbox[3]
    ]
    if inside:
        return min(inside, key=lambda o: (o.bbox[2] - o.bbox[0]) * (o.bbox[3] - o.bbox[1]))

    def centre_distance(obj: ObjectBox) -> float:
        x1, y1, x2, y2 = obj.bbox
        return math.hypot((x1 + x2) / 2 - x, (y1 + y2) / 2 - y)

    nearest = min(boxed, key=centre_distance, default=None)
    if nearest is None or centre_distance(nearest) > NEAR_CLICK_PX:
        return None
    return nearest


class OperatorCommandsModule(Module):
    """Scan for objects, pick the one the operator clicked on, and report progress.

    ``operator_command`` takes one OperatorCommand at a time:

    - "scan" finds the objects named in ``prompts`` and their boxes in the
      camera image, and forgets the target.
    - "target" chooses the object at the clicked pixel ``x``, ``y`` (see
      ``object_at``), or the one named by ``object_id``.
    - "pick" picks the target. Refused while the e-stop is latched; an
      operator hold that is on is released first.
    - "clear" forgets the objects and the target.

    Scan and pick run on a worker thread and can take minutes. A command sent
    meanwhile is refused with an error, not queued. stop() does not interrupt
    a scan or pick already running.

    ``operator_state`` carries the whole OperatorState once at start and after
    every change. ``operator_estop_status`` is the coordinator's e-stop latch.

    Give pick_object a long timeout in ``rpc_timeouts``: a pick takes minutes.
    """

    operator_command: In[OperatorCommand]
    operator_estop_status: In[OperatorEstopStatus]
    operator_state: Out[OperatorState]

    _pick: PickAndPlaceModule
    _scene: ObjectSceneRegistrationSpec
    # Any coordinator running the operator_hold task.
    _coordinator: ControlCoordinator

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._lock = threading.Lock()
        self._running: str | None = None  # the scan or pick on the worker thread
        self._worker: threading.Thread | None = None
        self._estopped = False
        self._state = OperatorState()

    @rpc
    def start(self) -> None:
        """Listen for commands and the e-stop state, then send the state once."""
        super().start()
        subscriptions: list[tuple[str, Callable[[], Callable[[], None]]]] = [
            ("operator_command", lambda: self.operator_command.subscribe(self._on_command)),
            (
                "operator_estop_status",
                lambda: self.operator_estop_status.subscribe(self._on_estop_status),
            ),
        ]
        for port, subscribe in subscriptions:
            try:
                self.register_disposable(Disposable(subscribe()))
            except Exception:
                logger.warning("Could not subscribe", port=port, exc_info=True)
        self._update()

    def _on_estop_status(self, msg: OperatorEstopStatus) -> None:
        self._estopped = msg.estopped

    def _on_command(self, msg: OperatorCommand) -> None:
        """Start or refuse one operator_command message; never raises."""
        try:
            action = msg.action
            if action not in OPERATOR_ACTIONS:
                logger.warning("Ignoring operator command", msg=msg)
                self._update(last_action="", error=f"unknown action {action!r}")
                return
            with self._lock:
                running = self._running
                if running is None and action in _LONG_ACTIONS:
                    self._running = action
            if running is not None:
                self._update(last_action=action, error=f"busy with {running}; wait for it")
            elif action in _LONG_ACTIONS:
                self._update(last_action=action, error="")
                self._worker = threading.Thread(
                    target=self._run, args=(action, msg), name=f"operator-{action}", daemon=True
                )
                self._worker.start()
            else:
                self._run(action, msg)
        except Exception:
            logger.exception("Operator command failed", msg=msg)

    def _run(self, action: str, msg: OperatorCommand) -> None:
        """Run one command and send the state it leaves behind."""
        # Each returns the OperatorState fields it changes.
        commands = {
            "scan": self._scan,
            "target": self._target,
            "pick": self._pick_target,
            "clear": self._clear,
        }
        started = time.monotonic()
        error = ""
        try:
            changes = commands[action](msg)
        except _RefusedError as e:
            changes, error = {}, str(e)
        except Exception as e:
            logger.exception("Operator command failed", action=action)
            error = f"{action} failed: {e}"
            changes = (
                {"last_result": PickResult(ok=False, message=error)}
                if action in _LONG_ACTIONS
                else {}
            )
        if "last_result" in changes:
            changes["last_result"] = replace(
                changes["last_result"], duration_ms=round((time.monotonic() - started) * 1000)
            )
        with self._lock:
            if self._running == action:
                self._running = None
        self._update(last_action=action, error=error, **changes)

    def _scan(self, msg: OperatorCommand) -> dict[str, Any]:
        prompts = msg.prompts
        if not any(prompt.strip() for prompt in prompts):
            raise _RefusedError('scan needs "prompts": a list of object names')
        # The pick module forgets its last scan's ids now, so do the same here.
        self._update(objects=[], target_id="", target_name="")
        logger.info("Operator scan", prompts=prompts)
        result: SkillResult = self._pick.scan_objects(prompts)
        boxes = self._scene.get_object_bboxes()
        objects = [
            ObjectBox(
                object_id=obj["object_id"],
                name=obj["name"],
                bbox=list(boxes.get(obj["object_id"]) or []),
            )
            for obj in result.metadata.get("objects", [])
        ]
        logger.info("Operator scan finished", message=result.message, objects=objects)
        return {"objects": objects, "last_result": PickResult(ok=True, message=result.message)}

    def _target(self, msg: OperatorCommand) -> dict[str, Any]:
        with self._lock:
            objects = self._state.objects
        if msg.object_id:
            match = next((obj for obj in objects if obj.object_id == msg.object_id), None)
            if match is None:
                raise _RefusedError(f"no object {msg.object_id!r} in the last scan")
        elif msg.x >= 0 and msg.y >= 0:
            match = object_at(objects, msg.x, msg.y)
            if match is None:
                raise _RefusedError("no object there")
        else:
            raise _RefusedError('target needs "x" and "y" in camera pixels, or an "object_id"')
        return {"target_id": match.object_id, "target_name": match.name}

    def _pick_target(self, msg: OperatorCommand) -> dict[str, Any]:
        with self._lock:
            target_id, target_name = self._state.target_id, self._state.target_name
        if self._estopped:
            raise _RefusedError("E-STOP is active; clear it before picking")
        if not target_id:
            raise _RefusedError("no target; scan and click an object first")
        hold = self._coordinator.task_invoke(OPERATOR_HOLD_TASK_NAME, "get_status", {})
        released = getattr(hold, "on", False) is True
        if released:
            logger.warning(
                "Releasing the operator hold to pick",
                route=hold.route,
                reason=hold.reason,
                object_id=target_id,
            )
            self._coordinator.task_invoke(OPERATOR_HOLD_TASK_NAME, "acknowledge", {})
        self._update(released_hold=released)
        logger.info("Operator pick", object_id=target_id, name=target_name)
        result = self._pick_object(target_id)
        ok = result.message == PICK_COMPLETE
        logger.info("Operator pick finished", ok=ok, message=result.message)
        return {"last_result": PickResult(ok=ok, message=result.message)}

    def _pick_object(self, object_id: str) -> SkillResult:
        """Call pick_object through this module's RPC client, so its rpc_timeouts apply.

        The PickAndPlaceModule handle carries its own client with the default
        timeouts.
        """
        call = self._pick.pick_object
        if isinstance(call, RpcCall):
            call.set_rpc(self.rpc)
        return call(object_id)

    def _clear(self, msg: OperatorCommand) -> dict[str, Any]:
        return {"objects": [], "target_id": "", "target_name": ""}

    def _update(self, **changes: Any) -> None:
        """Apply changes to the OperatorState fields and send all of it on operator_state."""
        with self._lock:
            self._state = replace(
                self._state, **changes, busy=self._running is not None, stamp=time.time()
            )
            # Sent under the lock so two threads cannot deliver states out of order.
            try:
                self.operator_state.publish(self._state)
            except Exception:
                logger.exception("Publishing operator state failed")
