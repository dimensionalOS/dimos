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

"""Run a keyboard window on its own main thread while the worker serves RPC."""

from __future__ import annotations

from abc import ABC, abstractmethod
from collections.abc import Callable
from contextlib import suppress
from dataclasses import dataclass
from multiprocessing import Pipe
from multiprocessing.connection import Connection
import subprocess
import sys
import threading
from typing import Any

from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.core.stream import Out
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.protocol.encode.cbor import decode, encode


@dataclass
class KeyboardWindow(ABC):
    """Child endpoint. Construct the display before acknowledging startup."""

    connection: Connection

    @abstractmethod
    def open(self) -> None: ...

    def publish(self, name: str, message: Any) -> None:
        self.connection.send_bytes(encode((name, message)))

    @abstractmethod
    def run(self) -> None: ...

    @classmethod
    def serve(cls) -> None:
        # Popen supplies the other end of a private duplex pipe as stdin.
        with Connection(sys.stdin.fileno()) as connection:
            settings = decode(connection.recv_bytes())
            try:
                window = cls(connection, **settings)
                window.open()
            except Exception as exc:
                connection.send_bytes(f"{type(exc).__name__}: {exc}".encode())
                raise
            connection.send_bytes(b"")
            window.run()


_WINDOW_START_TIMEOUT = 30.0


class WindowProcess:
    """Forward one window's typed output and publish a stop when it closes."""

    def __init__(
        self,
        module: str,
        settings: dict[str, Any],
        outputs: dict[str, Out[Any]],
        publish_stop: Callable[[], None],
    ) -> None:
        self._outputs = outputs
        self._publish_stop = publish_stop
        self._lock = threading.RLock()
        self._ended = False
        self._moving = False
        self._reader = threading.Thread(target=self._forward, daemon=True)
        self._connection, child = Pipe()
        try:
            with child:
                # DimOS workers are daemonic and cannot start multiprocessing.Process.
                self._process = subprocess.Popen(
                    [sys.executable, "-m", module], stdin=child.fileno()
                )
        except BaseException:
            self._connection.close()
            raise
        try:
            self._connection.send_bytes(encode(settings))
            if not self._connection.poll(_WINDOW_START_TIMEOUT):
                raise TimeoutError("Keyboard window did not finish opening")
            try:
                error = self._connection.recv_bytes()
            except (EOFError, ConnectionResetError) as exc:
                raise RuntimeError("Keyboard window closed before reporting ready") from exc
            if error:
                raise RuntimeError(f"Could not open keyboard window: {error.decode()}")
            self._reader.start()
        except BaseException:
            self.stop()
            raise

    @property
    def running(self) -> bool:
        return not self._ended and self._process.poll() is None

    def _forward(self) -> None:
        try:
            while not self._ended:
                name, message = decode(self._connection.recv_bytes())
                with self._lock:
                    if not self._ended:
                        if isinstance(message, Twist):
                            self._moving = bool(message)
                        self._outputs[name].publish(message)
        except (EOFError, ConnectionResetError):
            pass
        finally:
            try:
                self._finish()
            finally:
                self._connection.close()

    def _finish(self, *, explicit_stop: bool = False) -> None:
        try:
            with self._lock:
                if not self._ended:
                    self._ended = True
                    with suppress(BrokenPipeError, ConnectionResetError):
                        self._connection.send_bytes(b"")
                    if self._moving or explicit_stop:
                        self._publish_stop()
        finally:
            try:
                self._process.wait(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)
            except subprocess.TimeoutExpired:
                self._process.kill()
                self._process.wait()

    def stop(self) -> None:
        try:
            self._finish(explicit_stop=self._reader.ident is not None and self.running)
        finally:
            if self._reader.is_alive() and self._reader is not threading.current_thread():
                self._reader.join()
            self._connection.close()
