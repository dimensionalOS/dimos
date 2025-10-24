# Copyright 2025 Dimensional Inc.
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

"""
Note, to enable ps-spy to run without sudo you need:

    echo 0 | sudo tee /proc/sys/kernel/yama/ptrace_scope
"""

import subprocess
import threading
import re
import os
import shutil
from functools import lru_cache, partial
from typing import Optional
from distributed.client import Client

from distributed import get_client
from dimos.core import Module, rpc
from dimos.utils.actor_registry import ActorRegistry
from dimos.utils.logging_config import setup_logger


logger = setup_logger(__file__)


def print_data_table(data):
    headers = [
        "cpu_percent",
        "active_percent",
        "gil_percent",
        "n_threads",
        "pid",
        "worker_id",
        "modules",
    ]
    numeric_headers = {"cpu_percent", "active_percent", "gil_percent", "n_threads", "pid"}

    # Add registered modules.
    modules = ActorRegistry.get_all()
    for worker in data:
        worker["modules"] = ", ".join(
            module_name.split("-", 1)[0]
            for module_name, worker_id_str in modules.items()
            if worker_id_str == str(worker["worker_id"])
        )

    # Determine column widths
    col_widths = []
    for h in headers:
        max_len = max(len(str(d[h])) for d in data)
        col_widths.append(max(len(h), max_len))

    # Print header with DOS box characters
    header_row = " │ ".join(h.ljust(col_widths[i]) for i, h in enumerate(headers))
    border_parts = ["─" * w for w in col_widths]
    border_line = "─┼─".join(border_parts)
    print(border_line)
    print(header_row)
    print(border_line)

    # Print rows
    for row in data:
        formatted_cells = []
        for i, h in enumerate(headers):
            value = str(row[h])
            if h in numeric_headers:
                formatted_cells.append(value.rjust(col_widths[i]))
            else:
                formatted_cells.append(value.ljust(col_widths[i]))
        print(" │ ".join(formatted_cells))


class UtilizationThread(threading.Thread):
    _module: "UtilizationModule"
    _stop_event: threading.Event
    _monitors: dict

    def __init__(self, module):
        super().__init__(daemon=True)
        self._module = module
        self._stop_event = threading.Event()
        self._monitors = {}

    def run(self):
        while not self._stop_event.is_set():
            workers = self._module.client.scheduler_info()["workers"]
            pids = {pid: None for pid in get_worker_pids()}
            for worker, info in workers.items():
                pid = get_pid_by_port(worker.rsplit(":", 1)[-1])
                if pid is None:
                    continue
                pids[pid] = info["id"]
            data = []
            for pid, worker_id in pids.items():
                if pid not in self._monitors:
                    self._monitors[pid] = GilMonitorThread(pid)
                    self._monitors[pid].start()
                cpu, gil, active, n_threads = self._monitors[pid].get_values()
                data.append(
                    {
                        "cpu_percent": cpu,
                        "worker_id": worker_id,
                        "pid": pid,
                        "gil_percent": gil,
                        "active_percent": active,
                        "n_threads": n_threads,
                    }
                )
            data.sort(key=lambda x: x["pid"])
            self._fix_missing_ids(data)
            print_data_table(data)
            self._stop_event.wait(1)

    def stop(self):
        self._stop_event.set()
        for monitor in self._monitors.values():
            monitor.stop()
            monitor.join(timeout=2)

    def _fix_missing_ids(self, data):
        """
        Some worker IDs are None. But if we order the workers by PID and all
        non-None ids are in order, then we can deduce that the None ones are the
        missing indices.
        """
        if all(x["worker_id"] in (i, None) for i, x in enumerate(data)):
            for i, worker in enumerate(data):
                worker["worker_id"] = i


class UtilizationModule(Module):
    client: Optional[Client]
    _utilization_thread: Optional[UtilizationThread]

    def __init__(self):
        super().__init__()
        self.client = None
        self._utilization_thread = None

        if not os.getenv("MEASURE_GIL_UTILIZATION"):
            logger.info("Set `MEASURE_GIL_UTILIZATION=true` to print GIL utilization.")
            return

        if not _can_use_py_spy():
            logger.warning(
                "Cannot start UtilizationModule because in order to run py-spy without "
                "being root you need to enable this:\n"
                "\n"
                "    echo 0 | sudo tee /proc/sys/kernel/yama/ptrace_scope"
            )
            return

        if not shutil.which("py-spy"):
            logger.warning("Cannot start UtilizationModule because `py-spy` is not installed.")
            return

        self.client = get_client()
        self._utilization_thread = UtilizationThread(self)

    @rpc
    def start(self):
        super().start()

        if self._utilization_thread:
            self._utilization_thread.start()

    @rpc
    def stop(self):
        if self._utilization_thread:
            self._utilization_thread.stop()
            self._utilization_thread.join(timeout=2)
        super().stop()


utilization = UtilizationModule.blueprint


__all__ = ["UtilizationModule", "utilization"]


def _can_use_py_spy():
    try:
        with open("/proc/sys/kernel/yama/ptrace_scope") as f:
            value = f.read().strip()
        return value == "0"
    except Exception:
        pass
    return False


@lru_cache(maxsize=None)
def get_pid_by_port(port: int) -> int | None:
    try:
        result = subprocess.run(
            ["lsof", "-ti", f":{port}"], capture_output=True, text=True, check=True
        )
        pid_str = result.stdout.strip()
        return int(pid_str) if pid_str else None
    except subprocess.CalledProcessError:
        return None


def get_worker_pids():
    pids = []
    for pid in os.listdir("/proc"):
        if not pid.isdigit():
            continue
        try:
            with open(f"/proc/{pid}/cmdline", "r") as f:
                cmdline = f.read().replace("\x00", " ")
                if "spawn_main" in cmdline:
                    pids.append(int(pid))
        except (FileNotFoundError, PermissionError):
            continue
    return pids


class GilMonitorThread(threading.Thread):
    pid: int
    _latest_values: tuple[float, float, float, int]
    _stop_event: threading.Event
    _lock: threading.Lock

    def __init__(self, pid):
        super().__init__(daemon=True)
        self.pid = pid
        self._latest_values = (-1.0, -1.0, -1.0, -1)
        self._stop_event = threading.Event()
        self._lock = threading.Lock()

    def run(self):
        command = ["py-spy", "top", "--pid", str(self.pid), "--rate", "100"]
        process = None
        try:
            process = subprocess.Popen(
                command,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                bufsize=1,  # Line-buffered output
            )

            for line in iter(process.stdout.readline, ""):
                if self._stop_event.is_set():
                    break

                if "GIL:" not in line:
                    continue

                match = re.search(
                    r"GIL:\s*([\d.]+?)%,\s*Active:\s*([\d.]+?)%,\s*Threads:\s*(\d+)", line
                )
                if not match:
                    continue

                try:
                    cpu_percent = _get_cpu_percent(self.pid)
                    gil_percent = float(match.group(1))
                    active_percent = float(match.group(2))
                    num_threads = int(match.group(3))

                    with self._lock:
                        self._latest_values = (
                            cpu_percent,
                            gil_percent,
                            active_percent,
                            num_threads,
                        )
                except (ValueError, IndexError) as e:
                    pass
        except Exception as e:
            logger.error(f"An error occurred in GilMonitorThread for PID {self.pid}: {e}")
            raise
        finally:
            if process:
                process.terminate()
                process.wait(timeout=1)
            self._stop_event.set()

    def get_values(self):
        with self._lock:
            return self._latest_values

    def stop(self):
        self._stop_event.set()


def _get_cpu_percent(pid: int) -> float:
    try:
        result = subprocess.run(
            ["ps", "-p", str(pid), "-o", "%cpu="], capture_output=True, text=True, check=True
        )
        return float(result.stdout.strip())
    except Exception:
        return -1.0
