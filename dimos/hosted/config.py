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

"""Settings of this machine's Host daemon."""

from __future__ import annotations

from collections.abc import Mapping
from pathlib import Path
import socket

from dotenv import set_key, unset_key
from pydantic import Field, ValidationError
from pydantic_settings import BaseSettings, SettingsConfigDict

from dimos.constants import DIMOS_PROJECT_ROOT
from dimos.core.global_config import ENV_FILE

# The checkout's .env, wherever the command runs; the daemon itself runs in the checkout.
HOST_ENV_FILE = None if ENV_FILE is None else DIMOS_PROJECT_ROOT / ENV_FILE
# The daemon is its machine's zenoh router, on zenoh's own port, reachable from this machine
# only until opened up with `dimos host config listen=iface/<name>`.
DEFAULT_PORT = 7447
DEFAULT_LISTEN = f"tcp/127.0.0.1:{DEFAULT_PORT}"
# Hosts scout each other on their own multicast group, apart from any other zenoh fabric.
DIMOS_SCOUT_ADDR = "224.0.0.224:7449"


class HostConfig(BaseSettings):
    """`dimos host start` settings, set by HOST__<FIELD> in the environment or .env."""

    model_config = SettingsConfigDict(
        env_prefix="HOST__", env_file=HOST_ENV_FILE, env_file_encoding="utf-8", extra="ignore"
    )

    name: str = Field(default_factory=socket.gethostname)
    # Comma-separated tags, key or key=value, added to the auto-detected ones.
    tags: str = ""
    # Comma-separated router listen endpoints; iface/<name>[:<port>] is every address of one interface.
    listen: str = DEFAULT_LISTEN
    # Comma-separated routers this one links to; zenoh dials and redials them.
    connect: str = ""
    # Link to every Host scouted on scout_addr or heard of from a linked one; off, only connect links.
    autodiscovery: bool = False
    # Link directly to the Hosts a linked Host knows of.
    gossip: bool = True
    scout_addr: str = DIMOS_SCOUT_ADDR
    # Empty scouts every interface.
    scout_interface: str = ""
    # A fragment that has not reported ready by then is killed; a cold nav stack builds for minutes.
    startup_timeout: float = Field(default=180.0, gt=0)
    # A nav stack lies the robot down and stops a dozen workers before it is killed.
    stop_timeout: float = Field(default=15.0, gt=0)
    # How long a run's blueprint root may be unreachable before its fragment is stopped.
    run_loss_grace: float = Field(default=30.0, gt=0)

    @property
    def listen_endpoints(self) -> list[str]:
        """``listen`` as zenoh endpoints: ``iface/wlan0`` as ``tcp/0.0.0.0:7447#iface=wlan0``, a missing port 7447."""
        out = []
        for item in (e.strip() for e in self.listen.split(",")):
            kind, _, rest = item.partition("/")
            if kind == "iface":
                name, _, port = rest.partition(":")
                item = f"tcp/0.0.0.0:{port or DEFAULT_PORT}#iface={name}"
            elif rest:
                address, sep, options = rest.partition("#")
                if ":" not in address.rpartition("]")[2]:
                    item = f"{kind}/{address}:{DEFAULT_PORT}{sep}{options}"
            if item:
                out.append(item)
        return out


def write_settings(values: Mapping[str, str], env_file: Path | None = HOST_ENV_FILE) -> None:
    """Validate ``values`` (``listen`` or ``HOST__LISTEN``; empty removes) and write them to ``env_file``."""
    if env_file is None:
        raise ValueError("No .env to write to")
    fields = {key.lower().removeprefix("host__"): value for key, value in values.items()}
    if unknown := sorted(set(fields) - set(HostConfig.model_fields)):
        raise ValueError(f"Unknown settings {unknown}; known: {sorted(HostConfig.model_fields)}")
    try:
        HostConfig.model_validate({k: v for k, v in fields.items() if v})
    except ValidationError as exc:
        raise ValueError(str(exc)) from None
    env_file.touch()
    for key, value in fields.items():
        if value:
            set_key(env_file, f"HOST__{key.upper()}", value, quote_mode="never")
        else:
            unset_key(env_file, f"HOST__{key.upper()}")
