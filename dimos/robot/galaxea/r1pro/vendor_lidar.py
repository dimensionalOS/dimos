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

"""Where the R1's chassis Mid-360 is on the network, from the vendor's own config.

Galaxea's stack drives the chassis lidar through ``livox_ros_driver2``, and that
driver is configured by a ``MID360_config.json`` shipped on every R1 Pro. The
two addresses Point-LIO needs -- the lidar's IP and the host NIC the lidar
pushes its data to -- are already in that file, so rather than asking every
operator to find them and export two environment variables, read them from
where the vendor put them.

The environment variables still win when set (``DIMOS_POINTLIO_LIDAR_IP`` and
``DIMOS_POINTLIO_HOST_IP``, as :class:`PointLio` already honours), and the
file's location can be overridden with ``DIMOS_R1_MID360_CONFIG`` for a robot
whose vendor install lives somewhere else.
"""

from __future__ import annotations

from dataclasses import dataclass
import json
import os
from pathlib import Path

# The vendor install's config, the same path on every R1 Pro we have seen.
VENDOR_MID360_CONFIG = Path(
    "/home/nvidia/galaxea-dimos/install/livox_ros_driver2/share/livox_ros_driver2/config/MID360_config.json"
)
ENV_CONFIG_PATH = "DIMOS_R1_MID360_CONFIG"


@dataclass(frozen=True)
class LidarNetwork:
    lidar_ip: str
    host_ip: str


class VendorLidarConfigError(RuntimeError):
    """The vendor's lidar config could not be read; the message says what to do."""


def vendor_config_path() -> Path:
    """The config file to read: the override if set, else the vendor's location."""
    override = os.environ.get(ENV_CONFIG_PATH)
    return Path(override) if override else VENDOR_MID360_CONFIG


def read_vendor_lidar_network(path: Path | None = None) -> LidarNetwork:
    """The lidar and host IPs out of ``MID360_config.json``.

    The file's shape, as ``livox_ros_driver2`` writes it::

        {"MID360": {"host_net_info": {"cmd_data_ip": "192.168.2.150", ...}, ...},
         "lidar_configs": [{"ip": "192.168.2.100", ...}]}

    Raises :class:`VendorLidarConfigError` with an actionable message when the
    file is missing or does not carry both addresses.
    """
    path = path or vendor_config_path()
    how_to_fix = (
        f"Set DIMOS_POINTLIO_LIDAR_IP and DIMOS_POINTLIO_HOST_IP yourself, or point "
        f"{ENV_CONFIG_PATH} at the robot's livox_ros_driver2 MID360_config.json."
    )
    try:
        text = path.read_text()
    except OSError as error:
        raise VendorLidarConfigError(
            f"Cannot read the vendor lidar config at {path} ({error.strerror}). {how_to_fix}"
        ) from error
    try:
        data = json.loads(text)
    except json.JSONDecodeError as error:
        raise VendorLidarConfigError(f"{path} is not valid JSON: {error}. {how_to_fix}") from error

    lidars = data.get("lidar_configs") or []
    lidar_ip = lidars[0].get("ip") if lidars and isinstance(lidars[0], dict) else None
    host_ip = (data.get("MID360") or {}).get("host_net_info", {}).get("cmd_data_ip")
    if not lidar_ip or not host_ip:
        missing = [
            name
            for name, value in (
                ("lidar_configs[0].ip", lidar_ip),
                ("MID360.host_net_info.cmd_data_ip", host_ip),
            )
            if not value
        ]
        raise VendorLidarConfigError(
            f"{path} does not carry {' and '.join(missing)}, so the chassis lidar's address is "
            f"unknown. {how_to_fix}"
        )
    return LidarNetwork(lidar_ip=str(lidar_ip), host_ip=str(host_ip))
