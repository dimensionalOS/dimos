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
import sys

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

# Per-boot files of running daemons; a plain `ssh host cmd` has no XDG_RUNTIME_DIR.
_USER_RUN = Path(f"/run/user/{os.getuid()}") if hasattr(os, "getuid") else None
if os.environ.get("XDG_RUNTIME_DIR"):
    RUNTIME_DIR = Path(os.environ["XDG_RUNTIME_DIR"]) / "dimos"
elif sys.platform == "darwin":
    RUNTIME_DIR = Path(os.confstr("CS_DARWIN_USER_TEMP_DIR")) / "dimos"
elif _USER_RUN is not None and _USER_RUN.is_dir():
    RUNTIME_DIR = _USER_RUN / "dimos"
else:
    RUNTIME_DIR = STATE_DIR

DIMOS_PROJECT_ROOT = Path(__file__).parent.parent

if (DIMOS_PROJECT_ROOT / ".git").exists():
    # Running from Git repository
    LOG_DIR = DIMOS_PROJECT_ROOT / "logs"
    RECORDINGS_DIR = DIMOS_PROJECT_ROOT / "recordings"
    DOWNLOADS_DIR = DIMOS_PROJECT_ROOT / "downloads"
else:
    # Running from an installed package - use XDG_STATE_HOME
    LOG_DIR = STATE_DIR / "logs"
    RECORDINGS_DIR = STATE_DIR / "recordings"
    DOWNLOADS_DIR = STATE_DIR / "downloads"

CREDENTIALS_PATH = CONFIG_DIR / "credentials"

# From https://github.com/lcm-proj/lcm.git
LCM_MAX_CHANNEL_NAME_LENGTH = 63

# Default timeout (seconds) for thread.join() during shutdown.
DEFAULT_THREAD_JOIN_TIMEOUT = 2.0

DEFAULT_BUILD_NATIVE = False

# streaming compression libraries sharing the open(path, mode) API: id -> (module, suffix)
CODEC_LIBS = {
    "lz4": ("lz4.frame", ".lz4"),
    "gzip": ("gzip", ".gz"),
    "bz2": ("bz2", ".bz2"),
    "xz": ("lzma", ".xz"),
}
