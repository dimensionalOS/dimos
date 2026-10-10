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

"""Dedicated evaluation I/O: explicit TCP, bounded JSON, no RPC or discovery."""

from __future__ import annotations

import json
import re
from typing import Any, Literal

from pydantic import BaseModel, ConfigDict, Field
import zenoh

MAX_COMMAND_BYTES = 4096
MAX_SENSOR_BYTES = 8 * 1024 * 1024
PREFIX = "mujoco-eval"


class Packet(BaseModel):
    model_config = ConfigDict(extra="forbid", strict=True, allow_inf_nan=False)
    version: Literal[1] = 1
    run: str = Field(min_length=1, max_length=64, pattern=r"^[a-zA-Z0-9_-]+$")
    episode: str = Field(min_length=1, max_length=64, pattern=r"^[a-zA-Z0-9_-]+$")
    sequence: int = Field(ge=0)


class Command(Packet):
    sent_at: float
    kind: Literal["position", "stop", "enable", "disable"]
    values: list[float] = Field(default_factory=list, max_length=8)


class State(Packet):
    position: list[float] = Field(max_length=8)
    velocity: list[float] = Field(max_length=8)
    effort: list[float] = Field(max_length=8)
    lower: list[float] = Field(max_length=8)
    upper: list[float] = Field(max_length=8)
    velocity_max: list[float] = Field(max_length=8)
    enabled: bool
    accepted_sequence: int


class Sensor(Packet):
    kind: Literal["color_image", "depth_image", "camera_info", "tf"]
    data: str = Field(max_length=MAX_SENSOR_BYTES)


def key(run: str, episode: str, channel: str) -> str:
    if not all(re.fullmatch(r"[a-zA-Z0-9_-]{1,64}", v) for v in (run, episode)):
        raise ValueError("Invalid run/episode ID")
    return f"{PREFIX}/{run}/{episode}/{channel}"


def session_config(endpoint: str, *, trusted: bool, run: str, episode: str) -> zenoh.Config:
    if not re.fullmatch(r"tcp/[a-zA-Z0-9_.-]+:[0-9]+", endpoint):
        raise ValueError("An explicit TCP endpoint is required")
    command = key(run, episode, "command")
    sensors = [key(run, episode, c) for c in ("state", "sensor")]
    config: dict[str, Any] = {
        "mode": "router" if trusted else "client",
        "listen": {"endpoints": [endpoint] if trusted else []},
        "connect": {
            "endpoints": [] if trusted else [endpoint],
            "timeout_ms": 5000,
            "exit_on_failure": True,
        },
        "scouting": {"multicast": {"enabled": False}, "gossip": {"enabled": False}},
        "adminspace": {"enabled": False},
        "transport": {
            "shared_memory": {"enabled": False},
            "link": {
                "rx": {"max_message_size": MAX_COMMAND_BYTES if trusted else MAX_SENSOR_BYTES}
            },
        },
    }
    if trusted:
        config["access_control"] = {
            "enabled": True,
            "default_permission": "deny",
            "rules": [
                {
                    "id": "actions",
                    "permission": "allow",
                    "flows": ["ingress"],
                    "messages": ["put"],
                    "key_exprs": [command],
                },
                {
                    "id": "subscribe",
                    "permission": "allow",
                    "flows": ["ingress"],
                    "messages": ["declare_subscriber"],
                    "key_exprs": sensors,
                },
                {
                    "id": "sensors",
                    "permission": "allow",
                    "flows": ["egress"],
                    "messages": ["put"],
                    "key_exprs": sensors,
                },
            ],
            "subjects": [{"id": "robot"}],
            "policies": [{"subjects": ["robot"], "rules": ["actions", "subscribe", "sensors"]}],
        }
    return zenoh.Config.from_json5(json.dumps(config))
