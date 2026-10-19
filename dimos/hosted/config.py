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

import socket

from pydantic import Field
from pydantic_settings import BaseSettings, SettingsConfigDict

from dimos.core.global_config import ENV_FILE

# The daemon is its machine's zenoh router, on zenoh's own port.
DEFAULT_LISTEN = "tcp/0.0.0.0:7447"
# Hosts scout each other on their own multicast group, apart from any other zenoh fabric.
DIMOS_SCOUT_ADDR = "224.0.0.224:7449"


class HostConfig(BaseSettings):
    """`dimos host start` settings, set by HOST__<FIELD> in the environment or .env."""

    model_config = SettingsConfigDict(
        env_prefix="HOST__", env_file=ENV_FILE, env_file_encoding="utf-8", extra="ignore"
    )

    name: str = Field(default_factory=socket.gethostname)
    # Comma-separated tags, key or key=value, added to the auto-detected ones.
    tags: str = ""
    # Comma-separated router listen endpoints.
    listen: str = DEFAULT_LISTEN
    # Comma-separated routers this one links to; zenoh dials and redials them.
    connect: str = ""
    # Link to every Host scouted on scout_addr or heard of from a linked one.
    autodiscovery: bool = True
    scout_addr: str = DIMOS_SCOUT_ADDR
    # Empty scouts every interface.
    scout_interface: str = ""
    # A fragment that has not reported ready by then is killed; a cold nav stack builds for minutes.
    startup_timeout: float = Field(default=180.0, gt=0)
    # A nav stack lies the robot down and stops a dozen workers before it is killed.
    stop_timeout: float = Field(default=15.0, gt=0)
    # How long a run's controller may be unreachable before its fragment is stopped.
    run_loss_grace: float = Field(default=30.0, gt=0)
