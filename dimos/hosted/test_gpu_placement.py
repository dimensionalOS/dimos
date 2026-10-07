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

from typing import Any

from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.module import Module
from dimos.hosted.daemon import HostDescriptor
from dimos.hosted.fragment_compiler import resolve_hosted_assignments
from dimos.hosted.taggers import gpu


class FakeNvml:
    """NVML's C calls over a list of (name, mem_gib, cores, mhz)."""

    def __init__(self, devices: list[tuple[str, int, int, int]]) -> None:
        self.devices = devices

    def _set(self, ref: Any, value: Any) -> None:
        ref._obj.value = value

    def nvmlInit_v2(self) -> int:
        return 0

    def nvmlShutdown(self) -> int:
        return 0

    def nvmlDeviceGetCount_v2(self, count: Any) -> int:
        self._set(count, len(self.devices))
        return 0

    def nvmlDeviceGetHandleByIndex_v2(self, index: int, handle: Any) -> int:
        self._set(handle, index + 1)
        return 0

    def _device(self, handle: Any) -> tuple[str, int, int, int]:
        return self.devices[(handle.value or 1) - 1]

    def nvmlDeviceGetName(self, handle: Any, buf: Any, size: int) -> int:
        buf.value = self._device(handle)[0].encode()
        return 0

    def nvmlDeviceGetMemoryInfo(self, handle: Any, memory: Any) -> int:
        memory._obj.total = self._device(handle)[1] * 2**30
        return 0

    def nvmlDeviceGetNumGpuCores(self, handle: Any, cores: Any) -> int:
        self._set(cores, self._device(handle)[2])
        return 0

    def nvmlDeviceGetMaxClockInfo(self, handle: Any, clock: int, mhz: Any) -> int:
        self._set(mhz, self._device(handle)[3])
        return 0


def test_nvml_reports_every_gpu_and_best_picks_the_fastest() -> None:
    nvml = FakeNvml(
        [
            ("NVIDIA GeForce RTX 2070 SUPER", 8, 2560, 2130),
            ("NVIDIA GeForce RTX 5080", 16, 10752, 3090),
        ]
    )
    gpus = gpu.nvml_gpus(nvml)  # type: ignore[arg-type]
    assert [g.name for g in gpus] == ["RTX 2070 SUPER", "RTX 5080"]
    assert gpus[0].tflops == 10.9 and gpus[1].tflops == 66.4
    assert gpu.best(gpus) == gpus[1]


def test_no_nvml_means_no_gpus() -> None:
    class Missing:
        def nvmlInit_v2(self) -> int:
            return 1

    assert gpu.nvml_gpus(Missing()) == []  # type: ignore[arg-type]


def test_jetson_table(tmp_path: Any) -> None:
    freq = tmp_path / "max_freq"
    freq.write_text("1173000000\n")
    orin = gpu.jetson_gpu("NVIDIA Jetson Orin NX Seeed recomputer classic Super", [str(freq)])
    assert orin.tflops == 2.4
    assert gpu.jetson_gpu("NVIDIA Jetson Mystery Board", [str(freq)]).tflops is None


class GpuWorkload(Module):
    pass


def _host(host_id: str, tags: dict[str, str], runs: int = 0) -> HostDescriptor:
    return HostDescriptor(
        host_id, "e", host_id, tags, {}, "available", tuple(map(str, range(runs)))
    )


def _place(prefer: str, hosts: list[HostDescriptor]) -> str:
    blueprint = autoconnect(GpuWorkload.blueprint().hosted(tags={"gpu"}, prefer=prefer))
    BlueprintConfigParser(blueprint).parse(environ={})
    return resolve_hosted_assignments(
        blueprint, hosts, local_host_id="local", application_revision="r"
    )["gpuworkload"]


def test_prefer_picks_the_highest_value_then_least_busy() -> None:
    laptop = _host("laptop", {"gpu": "5070", "gpu_tflops": "28.5"})
    compute = _host("compute", {"gpu": "5080", "gpu_tflops": "66.4"}, runs=3)
    jetson = _host("jetson", {"gpu": "jetson"})
    assert _place("gpu_tflops", [laptop, compute, jetson]) == "compute"
    twin = _host("twin", {"gpu": "5080", "gpu_tflops": "66.4"})
    assert _place("gpu_tflops", [compute, twin]) == "twin"
    assert _place("gpu_tflops", [jetson, laptop]) == "laptop"
