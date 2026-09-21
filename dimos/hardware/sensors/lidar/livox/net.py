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

"""Shared host_ip auto-detection for Livox SDK modules (Mid360 driver, Point-LIO)."""

from __future__ import annotations

import ipaddress
import socket

from dimos.utils.generic import get_local_ips
from dimos.utils.logging_config import setup_logger

_logger = setup_logger()


def resolve_host_ip(lidar_ip: str, configured: str | None, *, label: str) -> str:
    """Resolve the local host IP the Livox SDK should bind to.

    Uses ``configured`` when it's one of our local IPs; otherwise auto-derives
    the local NIC on the lidar's /24 subnet. The chosen IP is UDP-bind-tested
    before returning. Raises ``RuntimeError`` with an actionable message when no
    local IP is on the lidar's subnet or the bind fails. ``label`` prefixes log
    and error messages (e.g. ``"PointLio"``, ``"Mid360"``).
    """
    local_ips = [ip for ip, _iface in get_local_ips()]

    if configured and configured in local_ips:
        host_ip = configured
    else:
        try:
            lidar_net = ipaddress.IPv4Network(f"{lidar_ip}/24", strict=False)
            same_subnet = [ip for ip in local_ips if ipaddress.IPv4Address(ip) in lidar_net]
        except (ValueError, TypeError):
            same_subnet = []
        if not same_subnet:
            subnet_prefix = ".".join(lidar_ip.split(".")[:3])
            msg = (
                f"{label}: cannot resolve host_ip — no local IP on the lidar's subnet "
                f"(lidar {lidar_ip}).\n"
                f"  Local IPs found: {', '.join(local_ips) or '(none)'}\n"
                f"  → Bring up the lidar NIC, or set host_ip explicitly.\n"
                f"  → Check: ip addr | grep {subnet_prefix}\n"
                f"  → Or assign: sudo ip addr add {subnet_prefix}.5/24 dev <iface>\n"
            )
            _logger.error(msg)
            raise RuntimeError(msg)
        host_ip = same_subnet[0]
        if configured:
            _logger.warning(
                f"{label}: host_ip={configured!r} not local; using {host_ip!r} "
                f"(on lidar {lidar_ip}'s subnet).",
            )

    _logger.info(f"{label} network check", host_ip=host_ip, lidar_ip=lidar_ip, local_ips=local_ips)

    # Bind a UDP socket on host_ip (port 0 = ephemeral) to catch a host already
    # holding the Livox SDK ports before the native binary starts.
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            sock.bind((host_ip, 0))
    except OSError as err:
        _logger.error(
            f"{label}: Cannot bind UDP socket on host_ip={host_ip!r}: {err}\n"
            f"  Another process may be using the Livox SDK ports.\n"
            f"  → Check: ss -ulnp | grep {host_ip}"
        )
        raise RuntimeError(
            f"{label}: Cannot bind UDP on {host_ip}: {err}. "
            f"Check if another Livox/PointLio process is running."
        ) from err

    _logger.info(f"{label} network check passed", host_ip=host_ip, lidar_ip=lidar_ip)
    return host_ip
