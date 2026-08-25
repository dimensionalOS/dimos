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
    CONFIG_DIR = Path(os.environ.get("XDG_CONFIG_HOME", Path.home() / ".config")) / "dimos"
    STATE_DIR = Path(os.environ.get("XDG_STATE_HOME", Path.home() / ".local" / "state")) / "dimos"
    CACHE_DIR = Path(os.environ.get("XDG_CACHE_HOME", Path.home() / ".cache")) / "dimos"
else:
    CONFIG_DIR = Path(GLib.get_user_config_dir()) / "dimos"
    STATE_DIR = Path(GLib.get_user_state_dir()) / "dimos"
    CACHE_DIR = Path(GLib.get_user_cache_dir()) / "dimos"

DIMOS_PROJECT_ROOT = Path(__file__).parent.parent

if (DIMOS_PROJECT_ROOT / ".git").exists():
    # Running from Git repository
    LOG_DIR = DIMOS_PROJECT_ROOT / "logs"
    RECORDINGS_DIR = DIMOS_PROJECT_ROOT / "recordings"
else:
    # Running from an installed package - use XDG_STATE_HOME
    LOG_DIR = STATE_DIR / "logs"
    RECORDINGS_DIR = STATE_DIR / "recordings"

CREDENTIALS_PATH = CONFIG_DIR / "credentials"


# From https://github.com/lcm-proj/lcm.git
LCM_MAX_CHANNEL_NAME_LENGTH = 63

# Default timeout (seconds) for thread.join() during shutdown.
DEFAULT_THREAD_JOIN_TIMEOUT = 2.0

DEFAULT_BUILD_NATIVE = False
