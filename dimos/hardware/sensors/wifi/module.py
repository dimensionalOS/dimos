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

from __future__ import annotations

import asyncio
from collections.abc import AsyncIterator
import contextlib
import shutil
import subprocess
import time
from typing import Any

from pydantic import Field

from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import Out
from dimos.diagnostics.observations import record_wifi
from dimos.hardware.sensors.wifi.iw import read_scan, run_iw
from dimos.msgs.sensor_msgs.WifiScan import WifiScan
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class WifiMonitorConfig(ModuleConfig):
    interface: str
    poll_interval_s: float = Field(default=1.0, ge=0.2)
    max_scan_age_s: float = Field(default=5.0, gt=0)
    # Explicit survey mode: off-channel scanning can interrupt the data link.
    scan_interval_s: float = Field(default=0.0, ge=0)


class WifiMonitor(Module):
    """Observe Linux Wi-Fi without initiating scans unless survey mode is enabled."""

    config: WifiMonitorConfig
    wifi_scan: Out[WifiScan]

    @classmethod
    def check_dependencies(cls, **kwargs: Any) -> None:
        if shutil.which("iw") is None:
            raise ImportError("WifiMonitor requires Linux and the iw package")

    async def main(self) -> AsyncIterator[None]:
        self.check_dependencies()
        task = asyncio.create_task(self._poll())
        try:
            yield
        finally:
            task.cancel()
            with contextlib.suppress(asyncio.CancelledError):
                await task

    async def _poll(self) -> None:
        next_scan = 0.0
        while True:
            started = time.monotonic()
            if self.config.scan_interval_s and started >= next_scan:
                next_scan = started + self.config.scan_interval_s
                try:
                    await run_iw(self.config.interface, "scan", "trigger")
                except (OSError, subprocess.SubprocessError, asyncio.TimeoutError) as exc:
                    logger.warning(
                        "Wi-Fi scan trigger failed (survey mode needs CAP_NET_ADMIN)",
                        error=str(exc),
                    )
            try:
                sample = await read_scan(self.config.interface, self.config.max_scan_age_s)
                self.wifi_scan.publish(sample)
                record_wifi(sample)
            except (OSError, subprocess.SubprocessError, asyncio.TimeoutError) as exc:
                logger.warning(
                    "Wi-Fi observation failed", interface=self.config.interface, error=str(exc)
                )
            await asyncio.sleep(max(0, self.config.poll_interval_s - (time.monotonic() - started)))
