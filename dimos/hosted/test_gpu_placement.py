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

import pytest

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.module import Module
from dimos.hosted import tags
from dimos.hosted.fragment_compiler import resolve_hosted_assignments
from dimos.hosted.protocol import HostDescriptor


def test_nvidia_smi_lines_become_gpus() -> None:
    output = "NVIDIA GeForce RTX 2070 SUPER, 8192\nNVIDIA GeForce RTX 5080, 16303\nbroken line\n"
    gpus = tags.nvidia_smi_gpus(output)
    assert gpus == [tags.Gpu("RTX 2070 SUPER", 8), tags.Gpu("RTX 5080", 16)]
    assert tags.nvidia_smi_gpus("") == []


class GpuWorkload(Module):
    pass


def _host(host_id: str, tags: dict[str, str], runs: int = 0) -> HostDescriptor:
    return HostDescriptor(host_id, host_id, tags, "r", tuple(map(str, range(runs))))


def _place(prefer: str, hosts: list[HostDescriptor]) -> str:
    blueprint = autoconnect(GpuWorkload.blueprint().hosted(tags={"gpu": True}, prefer=prefer))
    return resolve_hosted_assignments(blueprint, hosts, local_host_id="local", revision="r")[
        "gpuworkload"
    ]


def test_prefer_picks_the_highest_value_then_least_busy() -> None:
    laptop = _host("laptop", {"gpu": "5070", "gpu_mem_gb": "8"})
    compute = _host("compute", {"gpu": "5080", "gpu_mem_gb": "16"}, runs=3)
    jetson = _host("jetson", {"gpu": "jetson"})
    assert _place("gpu_mem_gb", [laptop, compute, jetson]) == "compute"
    twin = _host("twin", {"gpu": "5080", "gpu_mem_gb": "16"})
    assert _place("gpu_mem_gb", [compute, twin]) == "twin"
    # A Jetson has no GPU memory of its own and ranks last.
    assert _place("gpu_mem_gb", [jetson, laptop]) == "laptop"


def _place_minimum(minimum: dict[str, float], hosts: list[HostDescriptor]) -> str:
    blueprint = autoconnect(GpuWorkload.blueprint().hosted(minimum=minimum))
    return resolve_hosted_assignments(blueprint, hosts, local_host_id="local", revision="r")[
        "gpuworkload"
    ]


def test_minimum_rejects_hosts_below_a_numeric_tag_including_arbitrary_ones() -> None:
    small = _host("small", {"gpu_mem_gb": "8", "bla": "30"})
    big = _host("big", {"gpu_mem_gb": "16"}, runs=5)
    assert _place_minimum({tags.Tag.gpu_mem: 11}, [small, big]) == "big"
    assert _place_minimum({"bla": 22}, [small, big]) == "small"
    with pytest.raises(ValueError, match=r"gpu_mem_gb is 8, needs >= 32.*gpu_mem_gb is 16"):
        _place_minimum({tags.Tag.gpu_mem: 32}, [small, big])
