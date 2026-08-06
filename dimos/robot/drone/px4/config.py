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

"""MAVSDK Python autopilot connection configuration.

The connection URL identifies how the MAVSDK C++ core reaches the autopilot::

    serial:///dev/ttyTHS3:921600    direct serial (Jetson TX2 UART)
    udpin://0.0.0.0:14550           inbound UDP (drone sends to us)
    udpout://192.168.1.10:14550     outbound UDP (we push to autopilot)

MAVSDK v4 handles the transport internally; DIMOS only passes the URL string.
"""

from ipaddress import AddressValueError, IPv4Address
import os
from typing import ClassVar
from urllib.parse import SplitResult, urlsplit

from pydantic import ConfigDict, Field, field_validator

from dimos.protocol.service.spec import BaseConfig
from dimos.robot.drone.px4.errors import InvalidMavsdkConfigError


class MavsdkConfig(BaseConfig):
    """Validated MAVSDK Python connection settings."""

    model_config: ClassVar[ConfigDict] = ConfigDict(
        arbitrary_types_allowed=True,
        extra="forbid",
        frozen=True,
    )

    connection_url: str = "serial:///dev/ttyTHS3:921600"
    connection_timeout_s: float = Field(default=10.0, gt=0.0)

    @field_validator("connection_url")
    @classmethod
    def _validate_connection_url(cls, value: str) -> str:
        parsed = urlsplit(value)
        match parsed.scheme:
            case "serial":
                cls._validate_serial_url(parsed)
            case "udpin" | "udpout":
                cls._validate_udp_url(parsed)
            case _:
                raise InvalidMavsdkConfigError(
                    field="connection_url",
                    detail="must use serial://, udpin://, or udpout://",
                )
        return value

    @staticmethod
    def _validate_serial_url(parsed: SplitResult) -> None:
        if parsed.netloc or parsed.query or parsed.fragment:
            raise InvalidMavsdkConfigError(
                field="connection_url",
                detail="serial URL cannot include authority, query, or fragment",
            )
        device, separator, baud_text = parsed.path.rpartition(":")
        path_parts = device.split("/")
        valid_device = (
            device.startswith("/dev/") and ".." not in path_parts and "" not in path_parts[1:]
        )
        valid_baud = separator == ":" and baud_text.isdecimal() and int(baud_text) > 0
        if not valid_device or not valid_baud:
            raise InvalidMavsdkConfigError(
                field="connection_url",
                detail="serial URL must match serial:///dev/<device>:<baud>",
            )

    @staticmethod
    def _validate_udp_url(parsed: SplitResult) -> None:
        if parsed.path or parsed.query or parsed.fragment or parsed.username or parsed.password:
            raise InvalidMavsdkConfigError(
                field="connection_url",
                detail="UDP URL must contain only an IPv4 host and port",
            )
        try:
            host = parsed.hostname
            port = parsed.port
            if host is None:
                raise AddressValueError("missing IPv4 host")
            _ = IPv4Address(host)
        except (AddressValueError, ValueError) as error:
            raise InvalidMavsdkConfigError(
                field="connection_url",
                detail="UDP URL must contain a valid IPv4 host and port",
            ) from error
        if port is None or not 1 <= port <= 65_535:
            raise InvalidMavsdkConfigError(
                field="connection_url",
                detail="UDP URL port must be between 1 and 65535",
            )


def mavsdk_config_from_environment(*, default_connection_url: str | None = None) -> MavsdkConfig:
    """Build connection settings from environment values and blueprint defaults."""
    connection_url = os.environ.get("DIMOS_MAVSDK_CONNECTION_URL", default_connection_url)
    connection_timeout_s = os.environ.get("DIMOS_MAVSDK_CONNECTION_TIMEOUT_S")
    if connection_timeout_s is None:
        return (
            MavsdkConfig()
            if connection_url is None
            else MavsdkConfig(connection_url=connection_url)
        )
    timeout_s = float(connection_timeout_s)
    if connection_url is None:
        return MavsdkConfig(connection_timeout_s=timeout_s)
    return MavsdkConfig(connection_url=connection_url, connection_timeout_s=timeout_s)
