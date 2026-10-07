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

from pathlib import Path
import shutil
import subprocess

# Device nodes, so Host start never imports a CUDA stack: desktop NVIDIA, then Jetson.
GPU_DEVICES = ("/dev/nvidia0", "/dev/nvgpu", "/dev/nvhost-gpu")


def tags() -> dict[str, str]:
    """gpu=<name> and gpu_mem_gb when nvidia-smi answers, else a bare gpu."""
    if not any(Path(p).exists() for p in GPU_DEVICES):
        return {}
    if shutil.which("nvidia-smi") is None:
        return {"gpu": ""}
    out = subprocess.run(
        ["nvidia-smi", "--query-gpu=name,memory.total", "--format=csv,noheader,nounits"],
        capture_output=True,
        text=True,
        timeout=5,
    ).stdout.splitlines()
    if not out:
        return {"gpu": ""}
    name, _, mem_mib = out[0].partition(",")
    return {"gpu": name.strip(), "gpu_mem_gb": str(round(int(mem_mib) / 1024))}
