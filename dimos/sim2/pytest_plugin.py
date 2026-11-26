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

"""Loaded by pytest for every test (``-p`` in pyproject): process-wide settings a MuJoCo test needs before anything imports it."""

from contextlib import suppress
from ctypes.util import find_library
import os
import platform

# Render offscreen on Linux where libEGL is there: MuJoCo's GLFW default needs a display, macOS has no EGL and renders headless anyway.
if platform.system() == "Linux" and not os.environ.get("MUJOCO_GL") and find_library("EGL"):
    os.environ["MUJOCO_GL"] = "egl"

# Drake before MuJoCo: on Linux, mujoco 4 loaded first breaks Drake's URDF parsing, and every test shares one process.
with suppress(ImportError):
    import pydrake.multibody.parsing  # noqa: F401
