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

import os
from pathlib import Path

try:
    # Not a dependency, just the best way to get config path if available.
    from gi.repository import GLib  # type: ignore[import-untyped,import-not-found]
except ImportError:
    CONFIG_DIR = Path(os.environ.get("XDG_CONFIG_HOME", Path.home() / ".config"))
    STATE_DIR = Path(os.environ.get("XDG_STATE_HOME", Path.home() / ".local" / "state")) / "dimos"
    CACHE_DIR = Path(os.environ.get("XDG_CACHE_HOME", Path.home() / ".cache")) / "dimos"
else:
    CONFIG_DIR = Path(GLib.get_user_config_dir())
    STATE_DIR = Path(GLib.get_user_state_dir()) / "dimos"
    CACHE_DIR = Path(GLib.get_user_cache_dir()) / "dimos"

DIMOS_PROJECT_ROOT = Path(__file__).parent.parent

if (DIMOS_PROJECT_ROOT / ".git").exists():
    # Running from Git repository
    LOG_DIR = DIMOS_PROJECT_ROOT / "logs"
else:
    # Running from an installed package - use XDG_STATE_HOME
    LOG_DIR = STATE_DIR / "logs"

"""
Constants for shared memory
Usually, auto-detection for size would be preferred. Sadly, though, channels are made
and frozen *before* the first frame is received.
Therefore, a maximum capacity for color image and depth image transfer should be defined
ahead of time.
"""
# Default color image size: 1920x1080 frame x 3 (RGB) x uint8
DEFAULT_CAPACITY_COLOR_IMAGE = 1920 * 1080 * 3
# Default depth image size: 1280x720 frame * 4 (float32 size)
DEFAULT_CAPACITY_DEPTH_IMAGE = 1280 * 720 * 4

# From https://github.com/lcm-proj/lcm.git
LCM_MAX_CHANNEL_NAME_LENGTH = 63

# Default timeout (seconds) for thread.join() during shutdown.
DEFAULT_THREAD_JOIN_TIMEOUT = 2.0

DEFAULT_BUILD_NATIVE = False
