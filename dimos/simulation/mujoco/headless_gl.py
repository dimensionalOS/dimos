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

"""MuJoCo's GL backend for offscreen rendering, chosen before anything imports mujoco.

MuJoCo binds its OpenGL backend once, when a process first imports it, from ``MUJOCO_GL``:

* macOS always renders through CGL, offscreen, whatever is set (``egl`` and ``osmesa`` are
  refused there: leave it unset);
* on Linux, unset means GLFW, which needs an X or Wayland display; ``egl`` renders headless on
  NVIDIA's driver or on Mesa's (Intel, AMD, or llvmpipe with no GPU at all); ``osmesa`` renders
  in software where there is neither EGL nor a display (``libosmesa6``).

Stdlib only: a render script in a venv with just mujoco and numpy imports it too.
"""

from __future__ import annotations

import os
import sys


def default_mujoco_gl() -> str | None:
    """Make headless rendering work on Linux: with no display and ``MUJOCO_GL`` unset, set it
    to ``egl``. Returns the backend mujoco will be told to use (None: its own default, GLFW on
    Linux, CGL on macOS). It only acts in a process that has not imported mujoco yet, and the
    processes it starts afterwards (a stack's workers) inherit it."""
    chosen = os.environ.get("MUJOCO_GL")
    if chosen is not None:
        return chosen
    has_display = bool(os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"))
    if sys.platform.startswith("linux") and not has_display:
        os.environ["MUJOCO_GL"] = "egl"
        return "egl"
    return None
