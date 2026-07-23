# Copyright 2025-2026 Dimensional Inc.
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

"""Select MuJoCo's offscreen GL backend before MuJoCo is imported.

MuJoCo binds its render backend from ``MUJOCO_GL`` at import time. On a headless
host the default resolves to Mesa ``llvmpipe`` *software* rendering, so a
camera-heavy sim renders each 640x480 RGB+depth frame on the CPU (~30-90 ms)
and the sim falls to ~1% of real time. When an NVIDIA GPU is present, EGL
renders offscreen on the GPU (~0.7 ms/frame) and the sim runs at real time --
the interactive GLFW viewer (which uses its own windowed context, independent of
MUJOCO_GL) still works alongside it.

Import this module *before* the first ``import mujoco`` so the choice takes
effect. It is a no-op when the caller already set ``MUJOCO_GL`` (explicit choice
wins) or when no NVIDIA EGL driver is present (CPU-only / CI hosts keep MuJoCo's
default). Override the device with ``MUJOCO_EGL_DEVICE_ID``.
"""

from __future__ import annotations

import glob
import os


def _nvidia_egl_present() -> bool:
    if os.path.exists("/dev/nvidiactl"):
        return True
    return bool(
        glob.glob("/usr/lib/*/libEGL_nvidia.so*") or glob.glob("/usr/lib*/libEGL_nvidia.so*")
    )


def select_mujoco_gl_backend() -> str | None:
    """Default MuJoCo to EGL on an NVIDIA GPU. Returns the chosen backend or None."""
    if os.environ.get("MUJOCO_GL"):
        return None  # respect an explicit backend choice (e.g. MUJOCO_GL=glfw)
    if not _nvidia_egl_present():
        return None  # no GPU EGL driver -> leave MuJoCo's default backend
    os.environ["MUJOCO_GL"] = "egl"
    # The default EGL display can select Mesa's software device and fail; pin an
    # explicit device (0 is the NVIDIA GPU on single-GPU hosts).
    os.environ.setdefault("MUJOCO_EGL_DEVICE_ID", "0")
    return "egl"


# Run on import so simply importing this module before mujoco is sufficient.
_SELECTED_BACKEND = select_mujoco_gl_backend()
