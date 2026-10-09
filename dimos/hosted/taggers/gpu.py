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

"""gpu=<name>, gpu_mem_gb, gpu_tflops and gpu_count, for this Host's best GPU.

gpu_tflops is theoretical FP32: CUDA cores x max SM clock x 2. Desktops ask NVML through
ctypes, so no CUDA stack is imported; a Jetson reads its board model and GPU clock.
"""

from __future__ import annotations

import ctypes
from dataclasses import dataclass
from glob import glob
from pathlib import Path

from dimos.utils.logging_config import setup_logger

logger = setup_logger()

# Device nodes: desktop NVIDIA, then Jetson.
GPU_DEVICES = ("/dev/nvidia0", "/dev/nvgpu", "/dev/nvhost-gpu")
NVML_CLOCK_SM = 1
# Board model substring -> CUDA cores; first match wins.
JETSON_CORES = (
    ("AGX Orin", 2048),
    ("Orin NX", 1024),
    ("Orin Nano", 1024),
    ("AGX Xavier", 512),
    ("Xavier NX", 384),
)
JETSON_GPU_FREQ = "/sys/devices/platform/bus@0/17000000.gpu/devfreq/*/max_freq"
_warned: set[str] = set()


@dataclass(frozen=True)
class Gpu:
    name: str
    mem_gb: int | None = None
    tflops: float | None = None


class _Memory(ctypes.Structure):
    _fields_ = [
        ("total", ctypes.c_ulonglong),
        ("free", ctypes.c_ulonglong),
        ("used", ctypes.c_ulonglong),
    ]


def _short(name: str) -> str:
    return name.replace("NVIDIA ", "").replace("GeForce ", "").strip()


def nvml_gpus(lib: ctypes.CDLL | None = None) -> list[Gpu]:
    """Every GPU NVML reports, or nothing when there is no NVML."""
    try:
        lib = lib or ctypes.CDLL("libnvidia-ml.so.1")
    except OSError:
        return []
    if lib.nvmlInit_v2() != 0:
        return []
    try:
        count = ctypes.c_uint()
        lib.nvmlDeviceGetCount_v2(ctypes.byref(count))
        gpus = []
        for index in range(count.value):
            handle = ctypes.c_void_p()
            if lib.nvmlDeviceGetHandleByIndex_v2(index, ctypes.byref(handle)) != 0:
                continue
            name = ctypes.create_string_buffer(96)
            lib.nvmlDeviceGetName(handle, name, 96)
            memory = _Memory()
            lib.nvmlDeviceGetMemoryInfo(handle, ctypes.byref(memory))
            cores, mhz = ctypes.c_uint(), ctypes.c_uint()
            known = (
                lib.nvmlDeviceGetNumGpuCores(handle, ctypes.byref(cores)) == 0
                and lib.nvmlDeviceGetMaxClockInfo(handle, NVML_CLOCK_SM, ctypes.byref(mhz)) == 0
            )
            gpus.append(
                Gpu(
                    _short(name.value.decode()),
                    round(memory.total / 2**30),
                    round(cores.value * mhz.value * 2 / 1e6, 1) if known else None,
                )
            )
        return gpus
    finally:
        lib.nvmlShutdown()


def jetson_gpu(model: str, freq_paths: list[str] | None = None) -> Gpu:
    """The Jetson's GPU; no tflops for a board not in JETSON_CORES."""
    cores = next((n for key, n in JETSON_CORES if key in model), None)
    paths = glob(JETSON_GPU_FREQ) if freq_paths is None else freq_paths
    hz = int(Path(paths[0]).read_text()) if paths else 0
    if cores is None or not hz:
        if model not in _warned:
            logger.warning("No gpu_tflops for this Jetson board", model=model)
            _warned.add(model)
        return Gpu("jetson")
    return Gpu("jetson", None, round(cores * hz * 2 / 1e12, 1))


def best(gpus: list[Gpu]) -> Gpu:
    return max(gpus, key=lambda g: (g.tflops or 0.0, g.mem_gb or 0))


def tags() -> dict[str, str]:
    if not any(Path(p).exists() for p in GPU_DEVICES):
        return {}
    model_path = Path("/proc/device-tree/model")
    if Path("/etc/nv_tegra_release").exists() and model_path.exists():
        gpus = [jetson_gpu(model_path.read_text().strip("\x00\n "))]
    else:
        gpus = nvml_gpus() or [Gpu("")]
    top = best(gpus)
    out = {"gpu": top.name, "gpu_count": str(len(gpus))}
    if top.mem_gb is not None:
        out["gpu_mem_gb"] = str(top.mem_gb)
    if top.tflops is not None:
        out["gpu_tflops"] = str(top.tflops)
    return out
