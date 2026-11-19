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

Importing this module loads sounddevice either way. Without an audio server
PortAudio stays uninitialized. Every audio call then raises PortAudioError,
which is what a machine with no audio device gives on older releases.
"""

import importlib.util
import sys

from dimos.utils.logging_config import setup_logger

logger = setup_logger()

try:
    import sounddevice as sounddevice  # type: ignore[import-untyped]
except Exception as error:
    if type(error).__name__ != "PortAudioError":
        raise
    logger.info("PortAudio did not start, audio is unavailable", error=str(error))
    # The call that starts PortAudio is the last statement of sounddevice, so
    # the module is complete when it raises. A failed import drops the module,
    # so it is run again here and kept.
    spec = importlib.util.find_spec("sounddevice")
    assert spec is not None and spec.loader is not None
    sounddevice = importlib.util.module_from_spec(spec)
    try:
        spec.loader.exec_module(sounddevice)
    except sounddevice.PortAudioError:
        pass
    sys.modules["sounddevice"] = sounddevice
