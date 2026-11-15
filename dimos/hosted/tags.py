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

"""Host tags: key/value facts a Host advertises, e.g. lab, gpu=RTX 2070, ram_gb=31.

A bare tag is a key with an empty value. A placement requirement ``key`` needs the
key, ``key=value`` needs that exact value.
"""

from __future__ import annotations

from collections.abc import Callable, Iterable, Mapping
from dataclasses import dataclass
from enum import Enum
import os
from pathlib import Path
import platform
import subprocess

import psutil

from dimos.utils.logging_config import setup_logger

logger = setup_logger()

Tagger = Callable[[], dict[str, str]]


class Tag(str, Enum):  # TODO(PY311): switch to enum.StrEnum
    """Numeric Host tags for ``hosted(minimum=..., prefer=...)``; any other tag works by name."""

    # hosted() keys minimum by str(tag), so it must be the value, as with StrEnum.
    def __str__(self) -> str:
        return self.value

    # A Jetson's shared memory does not count.
    gpu_mem = "gpu_mem_gb"
    gpu_count = "gpu_count"
    ram = "ram_gb"
    cpus = "cpus"
    # What is free when the Host is asked.
    free_cpus = "free_cpus"
    free_ram = "free_ram_gb"
    free_gpu_mem = "free_gpu_mem_gb"


# Device nodes: desktop NVIDIA, then Jetson.
GPU_DEVICES = ("/dev/nvidia0", "/dev/nvgpu", "/dev/nvhost-gpu")
NVIDIA_SMI = ["nvidia-smi", "--query-gpu=name,memory.total", "--format=csv,noheader,nounits"]
NVIDIA_SMI_FREE = ["nvidia-smi", "--query-gpu=memory.free", "--format=csv,noheader,nounits"]


@dataclass(frozen=True)
class Gpu:
    name: str
    mem_gb: int | None = None


def _short(name: str) -> str:
    return name.replace("NVIDIA ", "").replace("GeForce ", "").strip()


def nvidia_smi_gpus(output: str | None = None) -> list[Gpu]:
    """Every GPU nvidia-smi lists; nothing without nvidia-smi."""
    if output is None:
        try:
            output = subprocess.run(NVIDIA_SMI, capture_output=True, text=True, timeout=10).stdout
        except (OSError, subprocess.TimeoutExpired):
            return []
    gpus = []
    for line in output.splitlines():
        name, _, mib = line.rpartition(",")
        if name.strip() and mib.strip().isdigit():
            gpus.append(Gpu(_short(name), round(int(mib) / 1024)))
    return gpus


def gpu_tags() -> dict[str, str]:
    if not any(Path(p).exists() for p in GPU_DEVICES):
        return {}
    if Path("/etc/nv_tegra_release").exists():
        return {"gpu": "jetson", Tag.gpu_count.value: "1"}
    gpus = nvidia_smi_gpus() or [Gpu("")]
    top = max(gpus, key=lambda g: g.mem_gb or 0)
    out = {"gpu": top.name, Tag.gpu_count.value: str(len(gpus))}
    if top.mem_gb is not None:
        out[Tag.gpu_mem.value] = str(top.mem_gb)
    return out


def as_tags(items: Iterable[str]) -> dict[str, str]:
    """Tags from ``key`` / ``key=value`` strings."""
    return dict(item.partition("=")[::2] for item in items)


def format_tags(tags: Mapping[str, str]) -> str:
    return ",".join(k if not v else f"{k}={v}" for k, v in sorted(tags.items()))


def missing(tags: Mapping[str, str], required: Mapping[str, str | bool]) -> list[str]:
    """The requirements ``tags`` does not meet, as ``key`` or ``key=value``."""
    return sorted(
        key if value is True else f"{key}={value}"
        for key, value in required.items()
        if key not in tags or (value is not True and tags[key] != value)
    )


def system_tags() -> dict[str, str]:
    return {
        "arch": platform.machine(),
        Tag.cpus.value: str(os.cpu_count() or 0),
        Tag.ram.value: str(round(psutil.virtual_memory().total / 2**30)),
    }


def jetson_tags() -> dict[str, str]:
    if not Path("/etc/nv_tegra_release").exists():
        return {}
    model = Path("/proc/device-tree/model")
    return {"jetson": model.read_text().strip("\x00\n ") if model.exists() else ""}


TAGGERS: tuple[Tagger, ...] = (system_tags, jetson_tags, gpu_tags)


def free_gpu_mem_gb() -> int | None:
    """The most free memory on any NVIDIA GPU; None without nvidia-smi."""
    try:
        output = subprocess.run(NVIDIA_SMI_FREE, capture_output=True, text=True, timeout=5).stdout
    except (OSError, subprocess.TimeoutExpired):
        return None
    free = [int(line) for line in output.split() if line.isdigit()]
    return round(max(free) / 1024) if free else None


def live_tags() -> dict[str, str]:
    """What is free right now; a Host answers these fresh every time it is described."""
    load = os.getloadavg()[0] if hasattr(os, "getloadavg") else 0.0
    out = {
        Tag.free_cpus.value: str(max(0, round((os.cpu_count() or 0) - load))),
        Tag.free_ram.value: str(round(psutil.virtual_memory().available / 2**30)),
    }
    if Path(GPU_DEVICES[0]).exists() and (free := free_gpu_mem_gb()) is not None:
        out[Tag.free_gpu_mem.value] = str(free)
    return out


def auto_tags(taggers: Iterable[Tagger] = TAGGERS) -> dict[str, str]:
    """Every tagger's tags merged; a tagger that raises is logged and skipped."""
    tags: dict[str, str] = {}
    for tagger in taggers:
        try:
            tags.update(tagger())
        except Exception:
            logger.warning("Host tagger failed", tagger=tagger.__qualname__, exc_info=True)
    return tags
