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

"""The gateway's settings, read once: `DIMOS_APP` (what Desktop starts it with), else `<home>/config.yaml`."""

from __future__ import annotations

from dataclasses import dataclass
import functools
import json
import os
from pathlib import Path
from typing import Any

import yaml

from dimos.utils.logging_config import setup_logger

logger = setup_logger()


@dataclass(frozen=True)
class SharedConfig:
    home: Path
    socket: str | None
    recordings_dir: str | None
    zenoh_namespace: str | None
    zenoh_connect: str | None
    zenoh_gossip: bool | None
    topics_enabled: bool | None
    # the blueprint view's remaining timers (config.yaml `blueprint-view:`), served to it by GET /config
    view_rates_poll_s: float = (
        2.0  # Desktop's topic rates have no event, so they're asked for this often
    )
    view_poll_without_events_s: float = (
        3.0  # without zenoh, the launch, stats and log are asked for this often
    )
    view_log_max_records: int = 5000  # a run's log keeps at most this many lines in the page


def _app_env() -> dict[str, Any]:
    """`DIMOS_APP` as JSON; an unusable one is warned about and ignored, since every field has a fallback."""
    text = os.environ.get("DIMOS_APP")
    if not text:
        return {}
    try:
        loaded = json.loads(text)
    except ValueError as error:
        logger.warning("DIMOS_APP is not JSON, so it is ignored", error=str(error))
        return {}
    return loaded if isinstance(loaded, dict) else {}


def _config_yaml(home: Path) -> dict[str, Any]:
    try:
        loaded = yaml.safe_load((home / "config.yaml").read_text()) or {}
    except (OSError, yaml.YAMLError):
        return {}
    return loaded if isinstance(loaded, dict) else {}


@functools.cache
def shared_config() -> SharedConfig:
    """Read on first use and kept: Desktop restarts the gateway when any of these change."""
    app = _app_env()
    home = Path(app["home"]) if app.get("home") else Path.home() / ".dimos"
    on_disk = _config_yaml(home)

    def pick(field: str | None, section: str, key: str) -> Any:
        """`DIMOS_APP`'s field, else config.yaml's; None when neither has it (empty counts as unset)."""
        if field and app.get(field) not in (None, ""):
            return app[field]
        value = (on_disk.get(section) or {}).get(key)
        return None if value == "" else value

    # only the blueprint-view keys config.yaml sets, the rest keep SharedConfig's defaults
    view: dict[str, Any] = {
        name: kind(value)
        for name, kind, key in (
            ("view_rates_poll_s", float, "rates-poll-s"),
            ("view_poll_without_events_s", float, "poll-without-events-s"),
            ("view_log_max_records", int, "log-max-records"),
        )
        if (value := pick(None, "blueprint-view", key)) is not None
    }
    return SharedConfig(
        home=home,
        socket=pick("socket", "dimos-gateway", "socket"),
        recordings_dir=pick("recordingsDir", "recordings", "dir"),
        zenoh_namespace=pick("zenohNamespace", "zenoh", "namespace"),
        zenoh_connect=pick("zenohConnect", "zenoh", "connect"),
        zenoh_gossip=pick(None, "zenoh", "gossip"),
        topics_enabled=pick(None, "topics", "enabled"),
        **view,
    )
