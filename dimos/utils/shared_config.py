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

"""The shared ``$DIMOS_HOME/config.yaml`` (``~/.dimos/config.yaml`` by default): the
settings a machine's dimos processes and its Desktop agree on.

It is read on every ask rather than cached, so a setting Desktop writes reaches the next
run without restarting anything, and so a test can point ``DIMOS_HOME`` somewhere else
after this module is imported.
"""

from __future__ import annotations

import os
from pathlib import Path
from typing import Any

import yaml

from dimos.constants import RECORDINGS_DIR


def dimos_home() -> Path:
    """``$DIMOS_HOME``, or ``~/.dimos``."""
    return Path(os.environ.get("DIMOS_HOME") or Path.home() / ".dimos")


def setting(section: str, key: str) -> Any:
    """A value from the shared ``config.yaml``, None when the file, the section or the key is
    missing (an empty value counts as unset: that is how Desktop clears a setting)."""
    try:
        loaded = yaml.safe_load((dimos_home() / "config.yaml").read_text()) or {}
    except (OSError, yaml.YAMLError):
        return None
    value = (loaded.get(section) or {}).get(key) if isinstance(loaded, dict) else None
    return None if value == "" else value


def recordings_dir() -> Path:
    """The one directory recordings go to: ``<recordings>/<run-id>/memory.db`` and the rest of
    a run's artifacts.

    ``--record-dir`` wins, because that is how dimos-gateway hands a launch the directory its
    ``GET /history/recordings`` answers with; then ``recordings.dir`` from the shared
    ``config.yaml``, which is the same answer reached without a flag (a plain ``dimos run``, a
    ``dimos cloud upload``, the gateway itself); then :data:`RECORDINGS_DIR`, the checkout's
    ``recordings/`` or the installed package's state directory, so a developer with no
    ``config.yaml`` keeps exactly what they have today.
    """
    # imported here and not at module scope: this module is low-level enough that
    # dimos.constants is all it should need to be imported, and GlobalConfig is a far
    # heavier import that pulls in pydantic-settings and the viewer's constants.
    from dimos.core.global_config import global_config

    if global_config.record_dir:
        return Path(global_config.record_dir).expanduser()
    configured = setting("recordings", "dir")
    return Path(configured).expanduser() if configured else RECORDINGS_DIR
