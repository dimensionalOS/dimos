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

"""Shared helpers for simulator runtime modules."""

from __future__ import annotations

from collections.abc import Callable
from concurrent.futures import Future
from dataclasses import dataclass
from queue import Queue
import threading
from typing import Generic, Protocol, TypeVar, cast

from dimos_runtime_protocol.models import (
    EpisodeResetRequest,
    EpisodeResetResponse,
    ObservationFrame,
    RuntimeDescription,
    ScoreOutput,
    StepRequest,
    StepResponse,
)

from dimos.core.stream import Out, Transport

RuntimeDescribeResponse = RuntimeDescription
RuntimeResetRequest = EpisodeResetRequest
RuntimeResetResponse = EpisodeResetResponse
RuntimeStepRequest = StepRequest
RuntimeStepResponse = StepResponse
RuntimeScoreResponse = ScoreOutput
RuntimeEventFrame = ObservationFrame

MOTOR_STATE_STREAM = "motor_state"
COLOR_IMAGE_STREAM = "color_image"
DEPTH_IMAGE_STREAM = "depth_image"
CAMERA_INFO_STREAM = "camera_info"
RUNTIME_EVENT_STREAM = "runtime_event"

T = TypeVar("T")


@dataclass(frozen=True)
class _OwnerThreadCall(Generic[T]):
    func: Callable[[], T]
    future: Future[T]


class _OwnerThreadStop:
    pass


_OwnerThreadItem = _OwnerThreadCall[object] | _OwnerThreadStop


class SimulatorOwnerThread:
    """Serial executor for simulator APIs with thread-affinity constraints."""

    def __init__(self, name: str) -> None:
        self._queue: Queue[_OwnerThreadItem] = Queue()
        self._thread = threading.Thread(target=self._run, name=name, daemon=True)
        self._owner_thread_id: int | None = None
        self._stopped = False
        self._lock = threading.Lock()
        self._thread.start()

    @property
    def owner_thread_id(self) -> int | None:
        return self._owner_thread_id

    def call(self, func: Callable[[], T]) -> T:
        """Run `func` on the simulator owner thread and return its result."""

        if self._owner_thread_id == threading.get_ident():
            return func()
        future: Future[T] = Future()
        with self._lock:
            if self._stopped:
                raise RuntimeError("Simulator owner thread has been stopped")
            self._queue.put(cast("_OwnerThreadItem", _OwnerThreadCall(func=func, future=future)))
        return future.result()

    def stop(self, timeout_s: float = 2.0) -> None:
        """Stop the owner thread after it drains any currently running call."""

        with self._lock:
            if self._stopped:
                return
            self._stopped = True
            self._queue.put(_OwnerThreadStop())
        if self._owner_thread_id != threading.get_ident():
            self._thread.join(timeout=timeout_s)

    def _run(self) -> None:
        self._owner_thread_id = threading.get_ident()
        while True:
            item = self._queue.get()
            if isinstance(item, _OwnerThreadStop):
                return
            try:
                item.future.set_result(item.func())
            except BaseException as exc:
                item.future.set_exception(exc)


class SimulatorExecutor(Protocol):
    """Serial execution surface for simulator APIs with affinity constraints."""

    @property
    def owner_thread_id(self) -> int | None: ...

    def call(self, func: Callable[[], T]) -> T: ...

    def stop(self, timeout_s: float = 2.0) -> None: ...


class InlineSimulatorExecutor:
    """Execute simulator calls inline on the caller thread.

    This is intended for visual simulator modes where the current process main
    thread should own the MuJoCo/GLFW viewer context.
    """

    def __init__(self) -> None:
        self._owner_thread_id = threading.get_ident()
        self._stopped = False

    @property
    def owner_thread_id(self) -> int | None:
        return self._owner_thread_id

    def call(self, func: Callable[[], T]) -> T:
        if self._stopped:
            raise RuntimeError("Inline simulator executor has been stopped")
        return func()

    def stop(self, timeout_s: float = 2.0) -> None:
        self._stopped = True


def module_runtime_description(
    description: RuntimeDescription,
    *,
    camera_streams: list[str] | None = None,
) -> RuntimeDescription:
    """Rewrite sidecar-origin metadata to the DimOS module stream surface."""

    capabilities = [cap for cap in description.capabilities if cap != "sync-http"]
    metadata = dict(description.metadata)
    if camera_streams is not None:
        metadata["backend_camera_streams"] = camera_streams
    metadata["module_streams"] = {
        "motor_state": MOTOR_STATE_STREAM,
        "color_image": COLOR_IMAGE_STREAM,
        "camera_info": CAMERA_INFO_STREAM,
        "runtime_event": RUNTIME_EVENT_STREAM,
    }
    observation_streams = [COLOR_IMAGE_STREAM, CAMERA_INFO_STREAM, RUNTIME_EVENT_STREAM]
    return description.model_copy(
        update={
            "capabilities": capabilities,
            "observation_streams": observation_streams,
            "metadata": metadata,
        }
    )


def publish_output(output: Out[T] | object, value: T) -> None:
    """Publish through local `Out` streams or deployed `RemoteOut` transports."""

    publish = getattr(output, "publish", None)
    if callable(publish):
        cast("Callable[[T], None]", publish)(value)
        return

    transport = getattr(output, "transport", None)
    if transport is None:
        raise RuntimeError("Output stream has no publish method or transport")
    cast("Transport[T]", transport).publish(value)
