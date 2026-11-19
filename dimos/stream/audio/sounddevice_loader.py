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

"""sounddevice, importable on a machine with no audio server.

sounddevice starts PortAudio when it is imported and raises if that fails.
Since Ubuntu 26.04 PortAudio has a PulseAudio backend, and PortAudio fails to
start when that backend cannot reach a server. A server, a container and a
robot's onboard computer have no audio server, so `import sounddevice` raises
there. unitree_webrtc_connect imports sounddevice without using it, which made
the Go2 and G1 connections impossible to import on such a machine.

Keep the module available for Unitree's import. Retry PortAudio when a caller
opens an output stream or enumerates devices after the audio server starts.
"""

from collections.abc import Callable
from functools import wraps
import importlib.util
import sys
from threading import Lock
from typing import Any


def _defer_initialization(backend: Any) -> None:
    """Retry PortAudio initialization before a device query or stream open."""
    lock = Lock()

    def on_demand(function: Callable[..., Any]) -> Callable[..., Any]:
        @wraps(function)
        def call(*args: Any, **kwargs: Any) -> Any:
            with lock:
                # sounddevice counts successful initializations and balances them at exit.
                if not backend._initialized:
                    backend._initialize()
            return function(*args, **kwargs)

        return call

    backend.OutputStream.__init__ = on_demand(backend.OutputStream.__init__)
    backend.query_devices = on_demand(backend.query_devices)


try:
    import sounddevice as sounddevice  # type: ignore[import-untyped]
except Exception as error:
    if type(error).__name__ != "PortAudioError":
        raise
    # The call that starts PortAudio is the last statement of sounddevice, so
    # the module is complete when it raises. A failed import drops the module,
    # so it is run again here and kept.
    spec = importlib.util.find_spec("sounddevice")
    assert spec is not None and spec.loader is not None
    sounddevice = importlib.util.module_from_spec(spec)
    try:
        spec.loader.exec_module(sounddevice)
    except sounddevice.PortAudioError:
        _defer_initialization(sounddevice)
    sys.modules["sounddevice"] = sounddevice
