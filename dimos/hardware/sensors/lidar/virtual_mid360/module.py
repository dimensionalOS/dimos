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

"""Python NativeModule wrapper for the virtual_mid360 Rust binary.


Usage::

    from dimos.hardware.sensors.lidar.virtual_mid360.module import VirtualMid360
    from dimos.hardware.sensors.lidar.pointlio.module import PointLio
    from dimos.core.coordination.blueprints import autoconnect

    autoconnect(
        VirtualMid360.blueprint(pcap="/path/to/ruwik2.pcap"),
        PointLio.blueprint(),
    )
"""

from __future__ import annotations

import os
import subprocess
import sys
from typing import TYPE_CHECKING, Any

from pydantic import Field

from dimos.core.core import rpc
from dimos.core.native_module import NativeModule, NativeModuleConfig
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

# Synthetic /24 the host_ip + lidar_ip share so they route to each other.
_ALIAS_PREFIX_LEN = 24
_ALIAS_NETMASK = "255.255.255.0"
# Host-route prefix length for the point/IMU multicast + discovery broadcast.
_HOST_ROUTE_LEN = 32
# Livox SDK's discovery hello; the fake lidar answers it.
_DISCOVERY_BROADCAST = "255.255.255.255"
# macOS has no dummy interfaces — the synthetic IPs are aliased onto loopback.
_MACOS_IFACE = "lo0"


class VirtualMid360Config(NativeModuleConfig):
    cwd: str | None = "."
    executable: str = "result/bin/virtual_mid360"
    build_command: str | None = "nix build .#default"
    # The rust binary reads its config as a JSON object on stdin (required).
    stdin_config: bool = True
    # Keep the Python-only NIC knobs out of the CLI args mirrored to the binary.
    cli_exclude: frozenset[str] = frozenset({"setup_network", "alias_iface"})

    # pcap/lidar_ip/host_ip default from DIMOS_MID360_* env vars so blueprints
    # needn't restate them. pcap is required — empty makes the binary error.
    pcap: str = Field(default_factory=lambda: os.environ.get("DIMOS_MID360_PCAP", ""))
    # Replay speed; 1.0 = original timing.
    rate: float = 1.0
    # Seconds to wait before streaming begins.
    delay: float = 0.0
    # IP the fake lidar sends from (on the dummy alias interface).
    lidar_ip: str = Field(
        default_factory=lambda: os.environ.get("DIMOS_MID360_LIDAR_IP", "192.168.1.155")
    )
    # Host IP the data is delivered to (where the SDK listens).
    host_ip: str = Field(default_factory=lambda: os.environ.get("DIMOS_MID360_HOST_IP", ""))
    lidar_netns: str = Field(default_factory=lambda: os.environ.get("DIMOS_MID360_NETNS", ""))
    # Multicast group for point/IMU. 224.1.1.5 is the Livox default the SDK joins.
    mcast_data: str = "224.1.1.5"

    # Auto-configure the virtual NIC (host_ip + lidar_ip on a dummy interface,
    # with the Livox multicast/discovery routes) on start, via sudo. Set False
    # if the interface is provisioned externally or real hardware is present.
    setup_network: bool = True
    # Name of the dummy interface the synthetic IPs are aliased onto.
    alias_iface: str = "dimos-mid360"

    def to_config_dict(self) -> dict[str, Any]:
        return {k: v for k, v in super().to_config_dict().items() if k not in self.cli_exclude}


class VirtualMid360(NativeModule):
    config: VirtualMid360Config

    def _sudo(self, args: list[str], *, check: bool = True) -> None:
        """Run a privileged command via sudo, raising on failure (when check)."""
        result = subprocess.run(["sudo", *args], capture_output=True)
        if check and result.returncode != 0:
            stderr = result.stderr.decode("utf-8", errors="replace").strip()
            raise RuntimeError(
                f"[{self._module_label}] `sudo {' '.join(args)}` failed "
                f"(exit {result.returncode}): {stderr}"
            )

    def _teardown_virtual_nic(self) -> None:
        # Idempotent: missing aliases/routes/interface are fine (check=False).
        cfg = self.config
        if sys.platform == "darwin":
            for ip in (cfg.host_ip, cfg.lidar_ip):
                self._sudo(["ifconfig", _MACOS_IFACE, "-alias", ip], check=False)
            for dst in (cfg.mcast_data, _DISCOVERY_BROADCAST):
                self._sudo(
                    ["route", "-n", "delete", "-host", dst, "-interface", _MACOS_IFACE], check=False
                )
        else:
            self._sudo(["ip", "link", "del", cfg.alias_iface], check=False)

    def _setup_virtual_nic(self) -> None:
        self._teardown_virtual_nic()
        if sys.platform == "darwin":
            self._setup_macos()
        elif sys.platform.startswith("linux"):
            self._setup_linux()
        else:
            raise RuntimeError(
                f"[{self._module_label}] setup_network supports Linux (iproute2) and macOS; "
                f"got {sys.platform}. Provision the interface yourself and set setup_network=False."
            )
        logger.info(
            "Virtual Mid-360 NIC configured",
            module=self._module_label,
            platform=sys.platform,
            host_ip=self.config.host_ip,
            lidar_ip=self.config.lidar_ip,
        )

    def _setup_macos(self) -> None:
        """Alias host_ip + lidar_ip onto loopback and route the Livox point/IMU
        multicast (and the discovery broadcast) there, so the local SDK sees the
        fake sensor over lo0. macOS has no dummy interfaces / netns."""
        cfg = self.config
        for ip in (cfg.host_ip, cfg.lidar_ip):
            self._sudo(["ifconfig", _MACOS_IFACE, "alias", ip, "netmask", _ALIAS_NETMASK])
        self._sudo(["route", "-n", "add", "-host", cfg.mcast_data, "-interface", _MACOS_IFACE])
        # Best-effort: the limited broadcast may already be deliverable on lo0.
        self._sudo(
            ["route", "-n", "add", "-host", _DISCOVERY_BROADCAST, "-interface", _MACOS_IFACE],
            check=False,
        )

    def _setup_linux(self) -> None:
        cfg = self.config
        iface = cfg.alias_iface
        self._sudo(["ip", "link", "add", iface, "type", "dummy"])
        self._sudo(["ip", "addr", "add", f"{cfg.host_ip}/{_ALIAS_PREFIX_LEN}", "dev", iface])
        self._sudo(["ip", "addr", "add", f"{cfg.lidar_ip}/{_ALIAS_PREFIX_LEN}", "dev", iface])
        self._sudo(["ip", "link", "set", iface, "up"])
        self._sudo(["ip", "link", "set", iface, "multicast", "on"])
        self._sudo(["ip", "route", "add", f"{cfg.mcast_data}/{_HOST_ROUTE_LEN}", "dev", iface])
        self._sudo(
            ["ip", "route", "add", f"{_DISCOVERY_BROADCAST}/{_HOST_ROUTE_LEN}", "dev", iface]
        )

    @rpc
    def build(self) -> None:
        super().build()
        if self.config.setup_network:
            self._setup_virtual_nic()

    @rpc
    def stop(self) -> None:
        super().stop()
        if self.config.setup_network:
            self._teardown_virtual_nic()


# Verify the module constructs (mirrors the pointlio wrapper's check).
if TYPE_CHECKING:
    VirtualMid360()
