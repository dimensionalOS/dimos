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

"""Stack composition, process management, and shared transport settings.

Module and subsystem settings belong to their owners' configuration classes.
"""

from collections.abc import Mapping
import os
from threading import Lock
from typing import Any, Literal, TypeAlias, TypeVar

from pydantic import AliasChoices, AliasGenerator, Field, ValidationInfo, field_validator
from pydantic_settings import BaseSettings, SettingsConfigDict

from dimos.constants import DEFAULT_BUILD_NATIVE
from dimos.visualization.rerun.constants import ViewerBackend

TransportBackend: TypeAlias = Literal["lcm", "zenoh"]
# How one zenoh session joins the network.
ZenohMode: TypeAlias = Literal["peer", "client", "router"]
# How every session in every process joins it. A router binds a port only one
# process can hold, so it is pinned on the one session that owns that port.
ZenohProcessMode: TypeAlias = Literal["peer", "client"]

# pytest exports PYTEST_VERSION to the whole process tree. Tests must not pick up
# a developer's .env (ROBOT_IP, SIMULATION, ...); dimos/conftest.py exports the
# LLM API keys itself.
ENV_FILE = None if "PYTEST_VERSION" in os.environ else ".env"


# Only the shared instance defers reading the environment.
_pending_config: "GlobalConfig | None" = None


def _environment_aliases(first: str, *others: str) -> AliasChoices:
    return AliasChoices(first, *others, *(f"G__{name}" for name in (first, *others)))


class GlobalConfig(BaseSettings):
    robot_ip: str | None = None
    robot_ips: str | None = None
    simulation: str = ""
    replay: bool = False
    replay_db: str = "go2_short"
    record: Literal["", "sqlite", "mcap"] = ""
    record_engine: Literal["python", "rust"] = Field(default="python", validate_default=True)
    record_topics: str = "*"  # comma-separated globs on the topic slug (/a/b -> a_b)
    record_encoding_threads: int | None = Field(default=None, ge=1)
    # How every zenoh session this process opens joins the network.
    zenoh_mode: ZenohProcessMode = "peer"
    # Extra locators every session dials, alongside those derived from --robot-ip.
    # Comma-separated, e.g. tcp/127.0.0.1:7447. Names a router or any non-robot peer.
    zenoh_connect: str = ""
    # Discover zenoh peers across the network.
    # Toggling off drops back to loopback-only discovery:
    # Sibling worker processes still find each other,
    # remote peers come solely from the connect endpoints derived from --robot-ip
    zenoh_scouting: bool = False
    # Interface multicast scouting binds to, e.g. wlan0.
    # Empty derives it from zenoh_scouting.
    zenoh_interface: str = ""
    # Whether multicast scouting runs at all. zenoh_scouting only sets its reach.
    zenoh_multicast: bool = True
    # Multicast group scouting joins, e.g. 224.0.0.224:7446. Empty takes zenoh's
    # own. Moving it walks a session onto a private discovery bus, which is how
    # parallel sessions on one machine stay apart (LCM_DEFAULT_URL's analog).
    zenoh_scout_addr: str = ""
    # Whether peers propagate the peers they already know over established links.
    # Unlike multicast scouting this reaches nothing new on the LAN, and zenoh
    # needs it to resolve the key expressions a linked peer sends.
    zenoh_gossip: bool | None = True
    # Seconds ZenohService.start() blocks for the configured connect endpoints to
    # link before giving up and continuing. 0 disables the wait.
    zenoh_connect_timeout: float = Field(default=1.0, ge=0, le=86400)
    # Off: share a zenoh bus with a stack that already owns the Coordinator name.
    serve_coordinator_rpc: bool = True
    # Isolates this run's zenoh streams under dimos-domains/<name>/; a hosted run pins its run id.
    zenoh_domain: str = Field(default="", pattern=r"^[A-Za-z0-9_.-]*$")
    viewer: ViewerBackend = "rerun"
    n_workers: int = 2
    scene_package: str | None = None
    robot_model: str | None = None
    # Robot geometry shared by every navigation and patrol module in a stack;
    # a module config may still override it for itself.
    robot_width: float = 0.3
    robot_rotation_diameter: float = 0.6
    # `DIMOS_TRANSPORT` (or `.env`) is the single switch read by every process
    # (dimos, humancli, agentspy, dtop). The `transport` alias keeps the bare
    # env name and the `--transport` CLI flag (which sets the field by name) working.
    transport: TransportBackend = Field(
        default="zenoh", validation_alias=_environment_aliases("DIMOS_TRANSPORT", "transport")
    )
    build_native: bool = DEFAULT_BUILD_NATIVE
    dtop: bool = Field(default=False, description="Publish live resource stats for dtop.")
    listen_host: str = "127.0.0.1"
    local_relay: bool = False
    topic_metadata: bool = False
    """Publish bounded Zenoh stream descriptors for a trusted topic service."""
    relay_url: str | None = None
    """HTTP URL of a relay started elsewhere (e.g. http://localhost:7780); the
    bridge discovers its WebTransport endpoint through /api/info."""

    model_config = SettingsConfigDict(
        alias_generator=AliasGenerator(validation_alias=_environment_aliases),
        env_file=ENV_FILE,
        env_file_encoding="utf-8",
        extra="ignore",
        validate_assignment=True,
    )

    @field_validator("record_engine")
    @classmethod
    def _validate_record_engine(cls, value: str, info: ValidationInfo) -> str:
        if info.data.get("record") == "mcap" and value != "rust":
            raise ValueError("MCAP recording requires --record-engine rust")
        return value

    @field_validator("record_encoding_threads")
    @classmethod
    def _validate_record_encoding_threads(
        cls, value: int | None, info: ValidationInfo
    ) -> int | None:
        if value is not None and info.data.get("record_engine") != "rust":
            raise ValueError("--record-encoding-threads is valid only with --record-engine rust")
        return value

    def __getattribute__(self, name: str) -> Any:
        if self is _pending_config and name != "update":
            get_global_config()
        return super().__getattribute__(name)

    def __setattr__(self, name: str, value: Any) -> None:
        if self is _pending_config and name in type(self).model_fields:
            get_global_config(**{name: value})
        else:
            super().__setattr__(name, value)

    def update(self, **kwargs: object) -> None:
        """Validate the complete set of overrides before updating this instance."""
        for key in kwargs:
            if key not in type(self).model_fields:
                raise AttributeError(f"GlobalConfig has no field '{key}'")
        if self is _pending_config:
            get_global_config(**kwargs)
            return
        if not kwargs:
            return
        updated = type(self).model_validate({**self.model_dump(), **kwargs})
        self.__dict__.update(updated.__dict__)
        self.model_fields_set.update(kwargs)

    @property
    def parsed_robot_ips(self) -> tuple[str, ...]:
        """Parse robot IPs without requiring them during blueprint discovery."""
        return tuple(x.strip() for x in (self.robot_ips or "").split(",") if x.strip())

    @property
    def processed_robot_ips(self) -> tuple[str, ...]:
        ips = self.parsed_robot_ips
        if error := require_robot_ips(ips):
            raise ValueError(error)
        return ips


# Default arguments and existing imports all hold this same model instance.
global_config = GlobalConfig.model_construct()
_pending_config = global_config
_config_lock = Lock()


def get_global_config(**overrides: Any) -> GlobalConfig:
    """Validate the shared settings in place when their effective values are known."""
    global _pending_config
    if _pending_config is not None or overrides:
        with _config_lock:
            if _pending_config is not None:
                validated = GlobalConfig(**overrides)
                GlobalConfig.__setstate__(global_config, validated.__getstate__())
                _pending_config = None
            elif overrides:
                global_config.update(**overrides)
    return global_config


def global_environment_names(*, qualified: bool = True) -> dict[str, str]:
    names = {}
    for field, info in GlobalConfig.model_fields.items():
        alias = info.validation_alias
        choices = alias.choices if isinstance(alias, AliasChoices) else (alias,)
        for name in (*choices, field):
            if isinstance(name, str) and (qualified or not name.lower().startswith("g__")):
                names[name.lower()] = field
    return names


_T = TypeVar("_T")


def normalize_global_aliases(values: Mapping[str, _T], *, qualified: bool = False) -> dict[str, _T]:
    """Resolve competing spellings within one source before combining it with another."""
    names = global_environment_names(qualified=qualified)
    lower = {name.lower(): value for name, value in values.items()}
    result = {name: value for name, value in values.items() if name.lower() not in names}
    for name, field in names.items():
        if name in lower:
            result.setdefault(field, lower[name])
    return result


def require_robot_ips(ips: tuple[str, ...]) -> str | None:
    """Validate the IPs used to compose a blueprint, independent of runtime globals."""
    if not ips:
        return "No robot IPs specified. Set ROBOT_IPS or --robot-ips to at least one IP."
    return None
