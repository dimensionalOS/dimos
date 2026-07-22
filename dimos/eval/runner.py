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

"""Runs headless and defaults to replay/sim so no robot is required.
"""

from __future__ import annotations

from datetime import datetime, timezone
import time
import traceback
from typing import TYPE_CHECKING, cast

from dimos.core.global_config import global_config
from dimos.core.resource_monitor.monitor import StatsMonitor
from dimos.eval.hardware import detect_hardware
from dimos.eval.metrics import EvalResult, EvalTotals
from dimos.eval.sampler import (
    GpuSampler,
    InMemoryResourceLogger,
    aggregate_resources,
    aggregate_totals,
)
from dimos.robot.get_all_blueprints import get_by_name
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from dimos.core.coordination.worker_manager_python import WorkerManagerPython

logger = setup_logger()

_RUN_MODES = ("replay", "simulation", "real")
_SIMULATORS = ("mujoco", "dimsim")


def _config_overrides(run_mode: str, simulator: str) -> dict[str, object]:
    """Build the global_config overrides for a run mode.
    """
    if run_mode == "replay":
        return {"replay": True}
    if run_mode == "simulation":
        return {"simulation": simulator, "replay": False}
    return {}


def run_eval(
    blueprint: str,
    *,
    run_mode: str = "replay",
    simulator: str = "mujoco",
    duration: float = 15.0,
    warmup: float = 3.0,
    interval: float = 1.0,
) -> EvalResult:
    """Evaluate ``blueprint`` on the current machine and return an EvalResult.
    """
    if run_mode not in _RUN_MODES:
        raise ValueError(f"run_mode must be one of {list(_RUN_MODES)}, got {run_mode!r}")
    if run_mode == "simulation" and simulator not in _SIMULATORS:
        raise ValueError(f"simulator must be one of {list(_SIMULATORS)}, got {simulator!r}")

    hardware = detect_hardware()
    timestamp = datetime.now(timezone.utc).isoformat()

    global_config.update(viewer="none", **_config_overrides(run_mode, simulator))

    result = EvalResult(
        blueprint=blueprint,
        run_mode=run_mode,
        status="ok",
        timestamp=timestamp,
        hardware=hardware,
    )

    coordinator = None
    gpu = GpuSampler(interval=interval)
    resource_logger = InMemoryResourceLogger()

    try:
        bp = get_by_name(blueprint)

        build_start = time.monotonic()
        from dimos.core.coordination.module_coordinator import ModuleCoordinator

        coordinator = ModuleCoordinator.build(bp)
        result.startup_seconds = time.monotonic() - build_start

        worker_source = cast("WorkerManagerPython", coordinator._managers["python"])
        monitor = StatsMonitor(worker_source, resource_logger=resource_logger, interval=interval)

        monitor.start()
        gpu.start()
        time.sleep(warmup)
        resource_logger.reset()
        gpu.reset()

        sample_start = time.monotonic()
        time.sleep(duration)
        actual_duration = time.monotonic() - sample_start

        monitor.stop()
        gpu.stop()

        samples = resource_logger.samples
        coord_fp, worker_fps = aggregate_resources(samples)
        cpu_mean, pss_mean, pss_peak = aggregate_totals(samples)
        ram_total = hardware.ram_total_bytes or 1

        result.warmup_seconds = warmup
        result.duration_seconds = actual_duration
        result.sample_count = len(samples)
        result.coordinator = coord_fp
        result.workers = worker_fps
        result.gpus = gpu.footprints()
        result.totals = EvalTotals(
            cpu_percent_mean=cpu_mean,
            pss_mean_bytes=pss_mean,
            pss_peak_bytes=pss_peak,
            ram_used_pct_of_host=100.0 * pss_peak / ram_total,
        )
    except Exception as exc:
        result.status = "failed"
        result.error = f"{type(exc).__name__}: {exc}"
        logger.error("Eval failed", blueprint=blueprint, exc_info=True)
        traceback.print_exc()
    finally:
        gpu.stop()
        if coordinator is not None:
            try:
                coordinator.stop()
            except Exception:
                logger.error("Error stopping coordinator", exc_info=True)

    return result
