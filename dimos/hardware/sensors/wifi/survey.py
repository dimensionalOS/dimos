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
from ipaddress import ip_address
import shutil
import time
from typing import Any

from pydantic import Field, field_validator
import requests

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import Out
from dimos.hardware.sensors.wifi.desktop_metrics import DesktopMetrics
from dimos.hardware.sensors.wifi.probes import HostCounters, probe
from dimos.msgs.sensor_msgs.SurveyMetrics import SurveyMetrics, SurveySample
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class NetworkSurveyConfig(ModuleConfig):
    interface: str
    targets: dict[str, str] = Field(default_factory=dict, max_length=4)
    poll_interval_s: float = Field(default=1.0, ge=0.2)
    # This endpoint and the gateway clock must be on the robot's clock domain.
    desktop_diagnostics_url: str | None = None

    @field_validator("targets")
    @classmethod
    def validate_targets(cls, targets: dict[str, str]) -> dict[str, str]:
        for address in targets.values():
            ip_address(address)
        return targets


class NetworkSurvey(Module):
    """Small read-only probes and existing viewer telemetry for a spatial survey."""

    config: NetworkSurveyConfig
    survey_metrics: Out[SurveyMetrics]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._poll_task: asyncio.Task[None] | None = None
        self._last_published: float | None = None

    @rpc
    def get_status(self) -> dict[str, Any]:
        """Report sampler health independently of whether the archive file is open."""
        task = self._poll_task
        age = None if self._last_published is None else time.time() - self._last_published
        error = None
        if task is None or task.cancelled():
            state = "stopped"
        elif task.done():
            failure = task.exception()
            state = "failed" if failure else "stopped"
            error = repr(failure) if failure else None
        elif age is None:
            state = "starting"
        else:
            state = "stalled" if age > max(10, 3 * self.config.poll_interval_s) else "running"
        return {
            "state": state,
            "last_published": self._last_published,
            "sample_age_s": age,
            "error": error,
        }

    @classmethod
    def check_dependencies(cls, **kwargs: Any) -> None:
        if kwargs.get("targets") and shutil.which("ping") is None:
            raise ImportError("NetworkSurvey ICMP probes require Linux iputils ping")

    async def main(self) -> AsyncIterator[None]:
        self.check_dependencies(targets=self.config.targets)
        task = self._poll_task = asyncio.create_task(self._poll())
        try:
            yield
        finally:
            task.cancel()
            with contextlib.suppress(asyncio.CancelledError):
                await task

    async def _probe(self, name: str, target: str) -> SurveySample:
        try:
            return await probe(self.config.interface, name, target)
        # asyncio.TimeoutError is a distinct exception on the robot's Python 3.10.
        except (OSError, asyncio.TimeoutError) as exc:
            logger.warning("Survey probe failed", target=target, error=str(exc))
            return SurveySample(
                time.time(),
                f"ping/{name}",
                {},
                labels={"error": type(exc).__name__ + ": " + str(exc), "target": target},
            )

    def _desktop(self) -> dict[str, Any]:
        assert self.config.desktop_diagnostics_url is not None
        response = requests.get(self.config.desktop_diagnostics_url, timeout=2)
        response.raise_for_status()
        document: dict[str, Any] = response.json()
        if not isinstance(document, dict):
            raise ValueError("Expected a diagnostics JSON object")
        return document

    async def _poll(self) -> None:
        previous = HostCounters.read(self.config.interface)
        desktop = DesktopMetrics()
        while True:
            started = time.monotonic()
            samples = list(
                await asyncio.gather(
                    *(self._probe(name, target) for name, target in self.config.targets.items())
                )
            )
            try:
                current = HostCounters.read(self.config.interface)
                samples.append(current.since(previous, self.config.interface))
                previous = current
            except (OSError, KeyError) as exc:
                logger.warning(
                    "Survey interface unavailable", interface=self.config.interface, error=str(exc)
                )
                samples.append(
                    SurveySample(time.time(), "host", {}, labels={"error": "interface_unavailable"})
                )
            if self.config.desktop_diagnostics_url:
                try:
                    document = await asyncio.to_thread(self._desktop)
                    samples.extend(desktop.read(document, time.time()))
                except (requests.RequestException, ValueError) as exc:
                    logger.warning("Viewer survey unavailable", error=str(exc))
                    samples.append(
                        SurveySample(time.time(), "desktop", {}, labels={"error": str(exc)})
                    )
            self.survey_metrics.publish(SurveyMetrics(time.time(), self.config.interface, samples))
            self._last_published = time.time()
            await asyncio.sleep(max(0, self.config.poll_interval_s - (time.monotonic() - started)))
