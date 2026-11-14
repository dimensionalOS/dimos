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

"""Picks MuJoCo's OpenGL backend before ``mujoco`` is first imported.

MuJoCo reads ``MUJOCO_GL`` once, when it is imported, so this module must be
imported before anything that imports it. A machine or launcher with no display
has no GLFW to render with: EGL is the default there. A ``MUJOCO_GL`` that is
already set is never changed, and neither is anything where a display exists.
"""

import os

if not (os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")):
    os.environ.setdefault("MUJOCO_GL", "egl")
