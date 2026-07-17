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

import os
import platform
import threading
from typing import Any, cast

import lcm as lcm_mod

from dimos.protocol.service.spec import BaseConfig, Service
from dimos.protocol.service.system_configurator.base import configure_system
from dimos.protocol.service.system_configurator.lcm_config import lcm_configurators
from dimos.utils.logging_config import setup_logger

logger = setup_logger()
_DEFAULT_LCM_URL = os.getenv("LCM_DEFAULT_URL", "udpm://239.255.76.67:7667?ttl=0")
_LCM_LOOP_TIMEOUT_MS = 50


def autoconf(check_only: bool = False) -> None:
    checks = lcm_configurators()
    if checks:
        configure_system(checks, check_only=check_only)
    else:
        logger.error(f"System configuration not supported on {platform.system()}")


class LCMConfig(BaseConfig):
    ttl: int = 0
    url: str = _DEFAULT_LCM_URL
    lcm: lcm_mod.LCM | None = None


class LCMService(Service):
    config: LCMConfig
    l: lcm_mod.LCM | None

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self.l = self.config.lcm or lcm_mod.LCM(self.config.url)
        self._stop_event = threading.Event()
        self._start_lock = threading.Lock()
        self._thread: threading.Thread | None = None

    def __getstate__(self) -> dict[str, Any]:
        state = self.__dict__.copy()
        for name in ("l", "_stop_event", "_start_lock", "_thread"):
            del state[name]
        return state

    def __setstate__(self, state: dict[str, Any]) -> None:
        self.__dict__.update(state)
        self.l = None
        self._stop_event = threading.Event()
        self._start_lock = threading.Lock()
        self._thread = None

    def start(self) -> None:
        with self._start_lock:
            if self._thread is not None:
                return
            if self.l is None:
                self.l = self.config.lcm or lcm_mod.LCM(self.config.url)
            self._stop_event.clear()
            self._thread = threading.Thread(
                target=self._run,
                args=(self.l,),
                name="lcm",
                daemon=True,
            )
            self._thread.start()

    def _run(self, handle: lcm_mod.LCM) -> None:
        while not self._stop_event.is_set():
            handle.handle_timeout(_LCM_LOOP_TIMEOUT_MS)

    @property
    def handle(self) -> lcm_mod.LCM:
        return cast("lcm_mod.LCM", self.l)

    def stop(self) -> None:
        with self._start_lock:
            self._stop_event.set()
            thread = self._thread
            if thread is not None:
                thread.join()
            self._thread = None
            if self.config.lcm is None:
                self.l = None
