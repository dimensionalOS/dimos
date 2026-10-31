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

import json
import os
from pathlib import Path
from typing import Any

import yaml

from dimos.constants import RECORDINGS_DIR
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


def app_env() -> dict[str, Any]:
    """``DIMOS_APP``, as Desktop started this process with it: which component this is and the home it
    belongs to, as one JSON object instead of a variable per setting. Children inherit the variable.

    Unusable JSON is said out loud and then ignored rather than being fatal: what is in here locates a
    file that also has a default, so there is something to fall back to.
    """
    text = os.environ.get("DIMOS_APP")
    if not text:
        return {}
    try:
        loaded = json.loads(text)
    except ValueError as error:
        logger.warning("DIMOS_APP is not usable, so the home in it is ignored", error=str(error))
        return {}
    return loaded if isinstance(loaded, dict) else {}


def dimos_home() -> Path:
    """``DIMOS_APP``'s ``home``, else ``$DIMOS_HOME``, else ``~/.dimos``.

    ``DIMOS_HOME`` is still read because that is what every Desktop before 0.2.228 set, and because
    someone running this by hand exports it.
    """
    home = app_env().get("home") or os.environ.get("DIMOS_HOME")
    return Path(home) if home else Path.home() / ".dimos"


def config_file() -> Path:
    """The shared ``config.yaml`` itself, which may not exist."""
    return dimos_home() / "config.yaml"


def setting(section: str, key: str) -> Any:
    """A value from the shared ``config.yaml``, None when the file, the section or the key is
    missing (an empty value counts as unset: that is how Desktop clears a setting)."""
    try:
        loaded = yaml.safe_load(config_file().read_text()) or {}
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

    Always absolute, so a reader and a launch started in another directory agree: a relative
    ``--record-dir`` is the working directory's, a relative ``recordings.dir`` is
    :func:`dimos_home`'s, the folder its ``config.yaml`` is in.
    """
    # imported here and not at module scope: this module is low-level enough that
    # dimos.constants is all it should need to be imported, and GlobalConfig is a far
    # heavier import that pulls in pydantic-settings and the viewer's constants.
    from dimos.core.global_config import global_config

    if global_config.record_dir:
        return Path(global_config.record_dir).expanduser().absolute()
    configured = setting("recordings", "dir")
    if not configured:
        return RECORDINGS_DIR
    return (dimos_home() / Path(str(configured)).expanduser()).absolute()
