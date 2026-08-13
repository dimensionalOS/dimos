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

"""Detect what hardware DimOS is running on.

Produces a HardwareProfile describing the host: CPU, RAM, GPU/CUDA,
and the NVIDIA Jetson tier when relevant. The ``tier`` field is a short,
stable label used as the cross-board comparison key.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
from pathlib import Path
import platform
import re
import shutil
import socket
import subprocess

import psutil

_SMI_QUERY = "name,memory.total,driver_version"


@dataclass(frozen=True)
class GpuInfo:
    """Static description of a GPU (dynamic utilization is sampled separately)."""

    name: str
    memory_total_bytes: int = 0
    driver_version: str = ""
    cuda_version: str = ""


@dataclass(frozen=True)
class HardwareProfile:
    """A snapshot of the machine's compute resources."""

    hostname: str
    tier: str
    os: str
    os_release: str
    arch: str
    cpu_model: str
    cpu_cores_physical: int
    cpu_cores_logical: int
    ram_total_bytes: int
    python_version: str
    is_jetson: bool = False
    jetson_model: str = ""
    jetpack: str = ""
    gpus: list[GpuInfo] = field(default_factory=list)

    def to_dict(self) -> dict[str, object]:
        return asdict(self)


def detect_hardware() -> HardwareProfile:
    """Introspect the current host into a HardwareProfile."""
    jetson_model, jetpack = _detect_jetson()
    is_jetson = bool(jetson_model)
    gpus = _detect_gpus()

    return HardwareProfile(
        hostname=socket.gethostname(),
        tier=_derive_tier(is_jetson, jetson_model, gpus),
        os=platform.system(),
        os_release=platform.release(),
        arch=platform.machine(),
        cpu_model=_cpu_model(),
        cpu_cores_physical=psutil.cpu_count(logical=False) or 0,
        cpu_cores_logical=psutil.cpu_count(logical=True) or 0,
        ram_total_bytes=psutil.virtual_memory().total,
        python_version=platform.python_version(),
        is_jetson=is_jetson,
        jetson_model=jetson_model,
        jetpack=jetpack,
        gpus=gpus,
    )


def _cpu_model() -> str:
    """Best-effort human CPU name across Linux/macOS."""
    cpuinfo = Path("/proc/cpuinfo")
    if cpuinfo.exists():
        for line in cpuinfo.read_text().splitlines():
            if line.startswith("model name"):
                return line.split(":", 1)[1].strip()
        for line in cpuinfo.read_text().splitlines():
            if line.startswith(("Hardware", "Model")):
                return line.split(":", 1)[1].strip()
    if platform.system() == "Darwin":
        try:
            out = subprocess.check_output(
                ["sysctl", "-n", "machdep.cpu.brand_string"], text=True, timeout=5
            )
            return out.strip()
        except (OSError, subprocess.SubprocessError):
            pass
    return platform.processor() or "unknown"


def _detect_jetson() -> tuple[str, str]:
    """Return ``(model, jetpack)`` if this is a Jetson board, else ``("", "")``."""
    model = ""
    model_node = Path("/proc/device-tree/model")
    if model_node.exists():
        try:
            raw = model_node.read_bytes().decode("utf-8", "ignore").strip("\x00").strip()
            if "jetson" in raw.lower() or "tegra" in raw.lower() or "nvidia" in raw.lower():
                model = raw
        except OSError:
            pass

    jetpack = ""
    tegra_release = Path("/etc/nv_tegra_release")
    if tegra_release.exists():
        try:
            first = tegra_release.read_text().splitlines()[0]
            m = re.search(r"R(\d+).*REVISION:\s*([\d.]+)", first)
            if m:
                jetpack = f"L4T R{m.group(1)}.{m.group(2)}"
            if not model:
                model = "NVIDIA Jetson"
        except OSError:
            pass

    return model, jetpack


def _detect_gpus() -> list[GpuInfo]:
    """Query GPUs via ``nvidia-smi``. Returns [] when unavailable."""
    if shutil.which("nvidia-smi") is None:
        return []
    try:
        out = subprocess.check_output(
            ["nvidia-smi", f"--query-gpu={_SMI_QUERY}", "--format=csv,noheader,nounits"],
            text=True,
            timeout=10,
        )
    except (OSError, subprocess.SubprocessError):
        return []

    cuda_version = _cuda_version()
    gpus: list[GpuInfo] = []
    for line in out.strip().splitlines():
        parts = [p.strip() for p in line.split(",")]
        if len(parts) < 3:
            continue
        name, mem_mib, driver = parts[0], parts[1], parts[2]
        try:
            mem_bytes = int(float(mem_mib)) * 1024 * 1024
        except ValueError:
            mem_bytes = 0
        gpus.append(
            GpuInfo(
                name=name,
                memory_total_bytes=mem_bytes,
                driver_version=driver,
                cuda_version=cuda_version,
            )
        )
    return gpus


def _cuda_version() -> str:
    """Parse the CUDA driver version from ``nvidia-smi`` output."""
    try:
        out = subprocess.check_output(["nvidia-smi"], text=True, timeout=10)
    except (OSError, subprocess.SubprocessError):
        return ""
    m = re.search(r"CUDA Version:\s*([\d.]+)", out)
    return m.group(1) if m else ""


def _derive_tier(is_jetson: bool, jetson_model: str, gpus: list[GpuInfo]) -> str:
    """Collapse the hardware into a short, comparison-friendly tier label."""
    if is_jetson:
        m = jetson_model.lower()
        for key in ("agx orin", "orin nx", "orin nano", "orin", "xavier nx", "agx xavier", "nano"):
            if key in m:
                return "jetson-" + key.replace(" ", "-")
        return "jetson"
    if gpus:
        return "desktop-dgpu"
    return "cpu-only"
