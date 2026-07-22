"""Daemon worker plumbing for execution-runtime effects."""

from collections.abc import Callable
from concurrent.futures import ThreadPoolExecutor
from concurrent.futures.thread import _worker
from dataclasses import dataclass
import threading
from typing import Any, cast
import weakref


@dataclass(frozen=True)
class EffectDone:
    action_id: str
    outcome: Any


@dataclass(frozen=True)
class AuxiliaryDone:
    action_id: str
    value: Any
    error: BaseException | None = None


@dataclass(frozen=True)
class StopDone:
    success: bool
    diagnostic: str = ""


class _DaemonThreadPoolExecutor(ThreadPoolExecutor):
    def _adjust_thread_count(self) -> None:
        if self._idle_semaphore.acquire(timeout=0):
            return
        if len(self._threads) >= self._max_workers:
            return

        def weakref_cb(_ref: Any, q: Any = self._work_queue) -> None:
            q.put(None)

        thread = threading.Thread(
            name=f"{self._thread_name_prefix}_{len(self._threads)}",
            target=_worker,
            args=(
                weakref.ref(self, weakref_cb),
                self._work_queue,
                self._initializer,  # type: ignore[attr-defined]
                self._initargs,  # type: ignore[attr-defined]
            ),
            daemon=True,
        )
        thread.start()
        cast("Any", self._threads).add(thread)


class ExecutionEffectRunner:
    """Execute already-admitted effects and post immutable completion events."""

    def __init__(self) -> None:
        self._executor = _DaemonThreadPoolExecutor(
            max_workers=4, thread_name_prefix="execution-rpc"
        )

    def submit_action(
        self, action_id: str, effect: Callable[[], Any], enqueue: Callable[[Any], None]
    ) -> None:
        future = self._executor.submit(effect)

        def done_callback(future: Any) -> None:
            try:
                outcome = future.exception()
                if outcome is None:
                    outcome = future.result()
            except BaseException as exc:
                outcome = exc
            enqueue(EffectDone(action_id, outcome))

        future.add_done_callback(done_callback)

    def submit_auxiliary(
        self, action_id: str, effect: Callable[[], Any], enqueue: Callable[[Any], None]
    ) -> None:
        future = self._executor.submit(effect)

        def done_callback(future: Any) -> None:
            try:
                error = future.exception()
                enqueue(
                    AuxiliaryDone(
                        action_id,
                        None if error is not None else future.result(),
                        error,
                    )
                )
            except BaseException as exc:
                enqueue(AuxiliaryDone(action_id, None, exc))

        future.add_done_callback(done_callback)

    def submit_stop(self, effect: Callable[[], Any], enqueue: Callable[[Any], None]) -> None:
        future = self._executor.submit(effect)

        def done_callback(future: Any) -> None:
            try:
                error = future.exception()
                enqueue(StopDone(error is None, "" if error is None else str(error)))
            except BaseException as exc:
                enqueue(StopDone(False, str(exc)))

        future.add_done_callback(done_callback)

    def shutdown(self) -> None:
        self._executor.shutdown(wait=False, cancel_futures=False)
