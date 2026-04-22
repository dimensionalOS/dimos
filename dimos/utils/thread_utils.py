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

"""Thread utilities: safe values, managed threads, safe parallel map."""

from __future__ import annotations

import asyncio
import collections
from concurrent.futures import Future, ThreadPoolExecutor, as_completed
import json
import signal
import subprocess
import threading
from typing import IO, TYPE_CHECKING, Any, Generic, Literal

from reactivex.disposable import Disposable

from dimos.utils.logging_config import setup_logger
from dimos.utils.typing_utils import ExceptionGroup, TypeVar

logger = setup_logger()

if TYPE_CHECKING:
    from collections.abc import Callable, Sequence

    from dimos.core.module import ModuleBase

T = TypeVar("T")
R = TypeVar("R")


class ThreadSafeVal(Generic[T]):
    """A thread-safe value wrapper.

    Forces lock usage in order to get access to a value (reduces unsafe value access)
    Three ways to use:
    1. `.set`
    2. `.get`
    3. via a context manager::
        with counter as value:
            # Lock is held for the entire block.
            # Other threads block on get/set/with until this exits.
            if value < 100:
                counter.set(value + 1)

        # Works with any type:
        status = ThreadSafeVal({"running": False, "count": 0})
        with status as s:
            status.set({**s, "running": True})

        # Bool check (for flag-like usage):
        stopping = ThreadSafeVal(False)
        stopping.set(True)
        if stopping.get():
            print("stopping!")
    """

    def __init__(self, initial: T) -> None:
        self._lock = threading.RLock()
        self._value = initial

    def get(self) -> T:
        """Return the current value (acquires the lock briefly)."""
        with self._lock:
            return self._value

    def set(self, value: T) -> None:
        """Replace the value (acquires the lock briefly)."""
        with self._lock:
            self._value = value

    def __enter__(self) -> T:
        self._lock.acquire()
        return self._value

    def __exit__(self, *exc: object) -> None:
        self._lock.release()

    def __getstate__(self) -> dict[str, Any]:
        return {"_value": self._value}

    def __setstate__(self, state: dict[str, Any]) -> None:
        self._lock = threading.RLock()
        self._value = state["_value"]

    def __repr__(self) -> str:
        return f"ThreadSafeVal({self._value!r})"


class ModuleThread:
    """A thread that registers cleanup with a module's disposables.

    Passes most kwargs through to ``threading.Thread``. On construction,
    registers a disposable with the module so that when the module stops,
    the thread is automatically joined. Cleanup is idempotent — safe to
    call ``stop()`` manually even if the module also disposes it.

    Example::

        class MyModule(Module):
            @rpc
            def start(self) -> None:
                self._worker = ModuleThread(
                    module=self,
                    target=self._run_loop,
                    name="my-worker",
                )
                self._worker.start()

            def _run_loop(self) -> None:
                while self._worker.status.get() == "running":
                    do_work()
    """

    def __init__(
        self,
        module: ModuleBase[Any],
        *,
        close_timeout: float = 2.0,
        **thread_kwargs: Any,
    ) -> None:
        thread_kwargs.setdefault("daemon", True)
        thread_kwargs.setdefault("name", f"{type(module).__name__}-thread")
        self._thread = threading.Thread(**thread_kwargs)
        self._close_timeout = close_timeout
        self.status: ThreadSafeVal[Literal["not_started", "running", "stopping", "stopped"]] = (
            ThreadSafeVal("not_started")
        )
        module._disposables.add(Disposable(self.stop))

    def start(self) -> None:
        self.status.set("running")
        self._thread.start()

    def stop(self) -> None:
        """Signal the thread to stop and join it.

        Safe to call multiple times, from any thread (including the
        managed thread itself — it will skip the join in that case).
        """
        with self.status as s:
            if s in ("stopping", "stopped"):
                return
            self.status.set("stopping")
        if self._thread.is_alive() and self._thread is not threading.current_thread():
            self._thread.join(timeout=self._close_timeout)
        self.status.set("stopped")

    def join(self, timeout: float | None = None) -> None:
        """Join the underlying thread."""
        self._thread.join(timeout=timeout)

    @property
    def is_alive(self) -> bool:
        return self._thread.is_alive()


class AsyncModuleThread:
    """A thread running an asyncio event loop, registered with a module's disposables.

    If a loop is already running in the current context, reuses it (no thread
    created).  Otherwise creates a new loop and drives it in a daemon thread.

    On stop (or module dispose), the loop is shut down gracefully and the
    thread is joined.  Idempotent — safe to call ``stop()`` multiple times.

    Example::

        class MyModule(Module):
            def __init__(self, **kwargs):
                super().__init__(**kwargs)
                self._async = AsyncModuleThread(module=self)

            @rpc
            def start(self) -> None:
                future = asyncio.run_coroutine_threadsafe(
                    self._do_work(), self._async.loop
                )

            async def _do_work(self) -> None:
                ...
    """

    def __init__(
        self,
        module: ModuleBase[Any],
        *,
        close_timeout: float = 2.0,
    ) -> None:
        self._close_timeout = close_timeout
        self._stopped = ThreadSafeVal(False)
        self._owns_loop = False
        self._thread: threading.Thread | None = None
        self._loop: asyncio.AbstractEventLoop | None = None
        self._module_name = type(module).__name__

        module._disposables.add(Disposable(self.stop))

    def start(self) -> None:
        """Create (or reuse) the event loop and start the driver thread.

        Safe to call multiple times — subsequent calls are no-ops.
        """
        if self._loop is not None:
            return

        try:
            self._loop = asyncio.get_running_loop()
        except RuntimeError:
            self._loop = asyncio.new_event_loop()
            asyncio.set_event_loop(self._loop)
            self._owns_loop = True
            self._thread = threading.Thread(
                target=self._loop.run_forever,
                daemon=True,
                name=f"{self._module_name}-event-loop",
            )
            self._thread.start()

    @property
    def loop(self) -> asyncio.AbstractEventLoop:
        """The managed event loop."""
        return self._loop

    @property
    def is_alive(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    def stop(self) -> None:
        """Stop the event loop and join the thread.

        No-op if the loop was not created by this instance (reused an
        existing running loop).  Safe to call multiple times.
        """
        with self._stopped as stopped:
            if stopped:
                return
            self._stopped.set(True)

        if self._owns_loop and self._loop is not None and self._loop.is_running():
            self._loop.call_soon_threadsafe(self._loop.stop)

        if self._thread is not None and self._thread.is_alive():
            self._thread.join(timeout=self._close_timeout)


class ModuleProcess:
    """A managed subprocess that pipes stdout/stderr through the logger.

    Registers with a module's disposables so the process is automatically
    stopped on module teardown. A watchdog thread monitors the process and
    calls ``on_exit`` if the process exits on its own (i.e. not via
    ``ModuleProcess.stop()``).

    Most constructor kwargs mirror ``subprocess.Popen``. ``stdout`` and
    ``stderr`` are always captured (set to ``PIPE`` internally).

    Example::

        class MyModule(Module):
            @rpc
            def start(self) -> None:
                self._proc = ModuleProcess(
                    module=self,
                    args=["./my_binary", "--flag"],
                    cwd="/opt/bin",
                    on_exit=self.stop,  # stops the whole module if process exits on its own
                )
                self._proc.start()

            @rpc
            def stop(self) -> None:
                super().stop()
    """

    def __init__(
        self,
        module: ModuleBase[Any],
        args: list[str] | str,
        *,
        env: dict[str, str] | None = None,
        cwd: str | None = None,
        shell: bool = False,
        on_exit: Callable[[], Any] | None = None,
        shutdown_timeout: float = 10.0,
        kill_timeout: float = 5.0,
        log_json: bool = False,
        log_tail_lines: int = 50,
        **popen_kwargs: Any,
    ) -> None:
        self._args = args
        self._env = env
        self._cwd = cwd
        self._shell = shell
        self._on_exit = on_exit
        self._shutdown_timeout = shutdown_timeout
        self._kill_timeout = kill_timeout
        self._log_json = log_json
        self._log_tail_lines = log_tail_lines
        self._popen_kwargs = popen_kwargs
        self._process: subprocess.Popen[bytes] | None = None
        self._watchdog: ModuleThread | None = None
        self._module = module
        self._stopped = ThreadSafeVal(False)
        self.last_stdout: collections.deque[str] = collections.deque(maxlen=log_tail_lines)
        self.last_stderr: collections.deque[str] = collections.deque(maxlen=log_tail_lines)

        module._disposables.add(Disposable(self.stop))

    @property
    def pid(self) -> int | None:
        return self._process.pid if self._process is not None else None

    @property
    def returncode(self) -> int | None:
        if self._process is None:
            return None
        return self._process.poll()

    @property
    def is_alive(self) -> bool:
        return self._process is not None and self._process.poll() is None

    def start(self) -> None:
        """Launch the subprocess and start the watchdog."""
        if self._process is not None and self._process.poll() is None:
            logger.warning("Process already running", pid=self._process.pid)
            return

        self._stopped.set(False)

        self.last_stdout = collections.deque(maxlen=self._log_tail_lines)
        self.last_stderr = collections.deque(maxlen=self._log_tail_lines)

        logger.info(
            "Starting process",
            cmd=self._args if isinstance(self._args, str) else " ".join(self._args),
            cwd=self._cwd,
        )
        self._process = subprocess.Popen(
            self._args,
            env=self._env,
            cwd=self._cwd,
            shell=self._shell,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            **self._popen_kwargs,
        )
        logger.info("Process started", pid=self._process.pid)

        self._watchdog = ModuleThread(
            module=self._module,
            target=self._watch,
            name=f"proc-{self._process.pid}-watchdog",
        )
        self._watchdog.start()

    def stop(self) -> None:
        """Send SIGTERM, wait, escalate to SIGKILL if needed. Idempotent."""
        with self._stopped as stopped:
            if stopped:
                return
            self._stopped.set(True)

        if self._process is not None and self._process.poll() is None:
            logger.info("Stopping process", pid=self._process.pid)
            try:
                self._process.send_signal(signal.SIGTERM)
            except OSError:
                pass  # process already dead (PID recycled or exited between poll and signal)
            else:
                try:
                    self._process.wait(timeout=self._shutdown_timeout)
                except subprocess.TimeoutExpired:
                    logger.warning(
                        "Process did not exit, sending SIGKILL",
                        pid=self._process.pid,
                    )
                    self._process.kill()
                    try:
                        self._process.wait(timeout=self._kill_timeout)
                    except subprocess.TimeoutExpired:
                        logger.error(
                            "Process did not exit after SIGKILL",
                            pid=self._process.pid,
                        )
        self._process = None

    def _watch(self) -> None:
        """Watchdog: pipe logs, detect crashes."""
        proc = self._process
        if proc is None:
            return

        stdout_t = self._start_reader(proc.stdout, "info")
        stderr_t = self._start_reader(proc.stderr, "warning")
        rc = proc.wait()
        stdout_t.join(timeout=2)
        stderr_t.join(timeout=2)

        if self._stopped.get():
            return

        last_stdout = "\n".join(self.last_stdout) or None
        last_stderr = "\n".join(self.last_stderr) or None
        logger.error(
            "Process died unexpectedly",
            pid=proc.pid,
            returncode=rc,
            last_stdout=last_stdout,
            last_stderr=last_stderr,
        )
        if self._on_exit is not None:
            self._on_exit()

    def _start_reader(self, stream: IO[bytes] | None, level: str) -> threading.Thread:
        t = threading.Thread(target=self._read_stream, args=(stream, level), daemon=True)
        t.start()
        return t

    def _read_stream(self, stream: IO[bytes] | None, level: str) -> None:
        if stream is None:
            return
        log_fn = getattr(logger, level)
        is_stderr = level == "warning"
        buf = self.last_stderr if is_stderr else self.last_stdout
        for raw in stream:
            line = raw.decode("utf-8", errors="replace").rstrip()
            if not line:
                continue
            buf.append(line)
            if self._log_json:
                try:
                    data = json.loads(line)
                    event = data.pop("event", line)
                    log_fn(event, **data)
                    continue
                except (json.JSONDecodeError, TypeError):
                    logger.warning("malformed JSON from process", raw=line)
            proc = self._process
            log_fn(line, pid=proc.pid if proc else None)
        stream.close()


def safe_thread_map(
    items: Sequence[T],
    fn: Callable[[T], R],
    on_errors: Callable[[list[tuple[T, R | Exception]], list[R], list[Exception]], Any]
    | None = None,
) -> list[R]:
    """Thread-pool map that waits for all items to finish before raising and a cleanup handler

    - Empty *items* → returns ``[]`` immediately.
    - All succeed → returns results in input order.
    - Any fail → calls ``on_errors(outcomes, successes, errors)`` where
      *outcomes* is a list of ``(input, result_or_exception)`` pairs in input
      order, *successes* is the list of successful results, and *errors* is
      the list of exceptions. If *on_errors* raises, that exception propagates.
      If *on_errors* returns normally, its return value is returned from
      ``safe_thread_map``. If *on_errors* is ``None``, raises an
      ``ExceptionGroup``.

    Example::

        def start_service(name: str) -> Connection:
            return connect(name)

        def cleanup(
            outcomes: list[tuple[str, Connection | Exception]],
            successes: list[Connection],
            errors: list[Exception],
        ) -> None:
            for conn in successes:
                conn.close()
            raise ExceptionGroup("failed to start services", errors)

        connections = safe_thread_map(
            ["db", "cache", "queue"],
            start_service,
            cleanup,  # called only if any start_service() raises
        )
    """
    if not items:
        return []

    outcomes: dict[int, R | Exception] = {}

    with ThreadPoolExecutor(max_workers=len(items)) as pool:
        futures: dict[Future[R], int] = {pool.submit(fn, item): i for i, item in enumerate(items)}
        for fut in as_completed(futures):
            idx = futures[fut]
            try:
                outcomes[idx] = fut.result()
            except Exception as e:
                outcomes[idx] = e

    successes: list[R] = []
    errors: list[Exception] = []
    for v in outcomes.values():
        if isinstance(v, Exception):
            errors.append(v)
        else:
            successes.append(v)

    if errors:
        if on_errors is not None:
            zipped = [(items[i], outcomes[i]) for i in range(len(items))]
            return on_errors(zipped, successes, errors)  # type: ignore[return-value, no-any-return]
        raise ExceptionGroup("safe_thread_map failed", errors)

    return [outcomes[i] for i in range(len(items))]  # type: ignore[misc]


def thread_start(
    module: ModuleBase[Any],
    *,
    close_timeout: float = 2.0,
    **thread_kwargs: Any,
) -> ModuleThread:
    """Create a :class:`ModuleThread`, start it immediately, and return it.

    Convenience wrapper equivalent to::

        t = ModuleThread(module, close_timeout=close_timeout, **thread_kwargs)
        t.start()
        return t

    Accepts the same arguments as :class:`ModuleThread`.

    Example::

        class MyModule(Module):
            @rpc
            def start(self) -> None:
                self._worker = thread_start(
                    self,
                    target=self._run_loop,
                    name="my-worker",
                )

            def _run_loop(self) -> None:
                while self._worker.status.get() == "running":
                    do_work()
    """
    t = ModuleThread(module, close_timeout=close_timeout, **thread_kwargs)
    t.start()
    return t
