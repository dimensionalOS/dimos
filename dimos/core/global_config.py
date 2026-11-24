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

import os
from pathlib import Path
import sys
import tempfile
from typing import Literal, TypeAlias

from pydantic import AliasChoices, Field, ValidationInfo, field_validator
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


def parse_periods(text: str) -> dict[str, float]:
    """Parse comma-separated ``glob=seconds`` pairs, keeping their order."""
    periods: dict[str, float] = {}
    for entry in filter(None, (part.strip() for part in text.split(","))):
        pattern, _, seconds = (part.strip() for part in entry.partition("="))
        try:
            if not pattern:
                raise ValueError
            periods[pattern] = float(seconds)
        except ValueError:
            raise ValueError(f"period {entry!r} is not glob=seconds") from None
    return periods


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
    # Thin long recordings and keep full-rate clips around messages on the clip_that topic.
    record_events: bool = False
    # Seconds between kept messages outside event recording clips, for every stream.
    record_default_period: float = Field(default=5.0, ge=0)
    # Overrides as comma-separated glob=seconds pairs. Payload type names (Image=10)
    # override the default, stream names (lidar=2) override both.
    record_type_periods: str = ""
    record_stream_periods: str = ""
    # Event recording windows go in one subfolder per run here. Linux keeps them in RAM.
    record_window_dir: str = str(
        Path("/dev/shm" if sys.platform == "linux" else tempfile.gettempdir()) / "dimos-recorder"
    )
    # Most event window data (MiB) waiting to be stored before the oldest segment is stored early.
    record_window_mib: float = Field(default=2048.0, gt=0)
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
        default="zenoh",
        validation_alias=AliasChoices("DIMOS_TRANSPORT", "transport"),
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

    @field_validator("record_events")
    @classmethod
    def _validate_record_events(cls, value: bool, info: ValidationInfo) -> bool:
        if value and info.data.get("record_engine") != "rust":
            raise ValueError("--record-events is valid only with --record-engine rust")
        return value

    @field_validator(
        "record_default_period",
        "record_type_periods",
        "record_stream_periods",
        "record_window_dir",
        "record_window_mib",
    )
    @classmethod
    def _validate_event_setting(cls, value: float | str, info: ValidationInfo) -> float | str:
        name = info.field_name or ""
        if value != cls.model_fields[name].default and not info.data.get("record_events"):
            raise ValueError(f"--{name.replace('_', '-')} is valid only with --record-events")
        if isinstance(value, str) and name.endswith("_periods"):
            parse_periods(value)
        return value

    def update(self, **kwargs: object) -> None:
        """Update config fields in place."""
        for key, value in kwargs.items():
            if key not in type(self).model_fields:
                raise AttributeError(f"GlobalConfig has no field '{key}'")
            setattr(self, key, value)

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


global_config = GlobalConfig()


def require_robot_ips(ips: tuple[str, ...]) -> str | None:
    """Validate the IPs used to compose a blueprint, independent of runtime globals."""
    if not ips:
        return "No robot IPs specified. Set ROBOT_IPS or --robot-ips to at least one IP."
    return None
