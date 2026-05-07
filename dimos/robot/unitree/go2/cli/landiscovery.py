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

"""LAN discovery for Unitree Go2 robots.

Go2s respond to a custom UDP-multicast probe (group 231.1.1.1, port 10131)
with a JSON payload containing their serial number and IP. This module sends
that probe and collects replies on port 10134.

Implemented from scratch (rather than reusing legion1581/unitree_webrtc_connect's
scanner) so we can pin the multicast send/recv to a specific interface — Tailscale
and other VPN tun devices install a 224.0.0.0/4 route in a separate table that
silently swallows the probe otherwise.
"""

from __future__ import annotations

import asyncio
from collections.abc import AsyncIterator, Iterator
from dataclasses import dataclass
import json
import socket
import struct
import time

import psutil

from dimos.utils.logging_config import setup_logger

logger = setup_logger()

MULTICAST_GROUP = "231.1.1.1"
QUERY_PORT = 10131
REPLY_PORT = 10134
QUERY_PAYLOAD = json.dumps({"name": "unitree_dapengche"}).encode("utf-8")


@dataclass(frozen=True)
class Go2Device:
    serial: str
    ip: str
    iface: str
    mac: str | None = None


def _read_arp(ip: str) -> str | None:
    try:
        with open("/proc/net/arp") as f:
            for line in f.readlines()[1:]:
                parts = line.split()
                if len(parts) >= 4 and parts[0] == ip and parts[3] != "00:00:00:00:00:00":
                    return parts[3].upper()
    except OSError:
        return None
    return None


def _resolve_mac(ip: str, retries: int = 3, retry_delay: float = 0.05) -> str | None:
    """Resolve MAC for `ip` via /proc/net/arp; nudges the cache with a probe UDP send.

    Linux-only. Returns None if the address can't be resolved (e.g. on macOS).
    """
    cached = _read_arp(ip)
    if cached:
        return cached
    for _ in range(retries):
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            try:
                s.sendto(b"", (ip, 9))  # discard port — kernel ARPs before send
            except OSError:
                pass
            s.close()
        except OSError:
            pass
        time.sleep(retry_delay)
        mac = _read_arp(ip)
        if mac:
            return mac
    return None


def _candidate_ifaces() -> Iterator[tuple[str, str]]:
    """Yield (iface_name, ipv4_addr) for non-loopback, non-tun IPv4 interfaces."""
    skip_prefixes = ("lo", "tailscale", "wg", "tun", "docker", "br-", "veth", "Meta")
    for name, addrs in psutil.net_if_addrs().items():
        if name.startswith(skip_prefixes):
            continue
        for a in addrs:
            if a.family == socket.AF_INET and not a.address.startswith("127."):
                yield name, a.address
                break


def discover(timeout: float = 2.0, iface_ip: str | None = None) -> list[Go2Device]:
    """Probe the LAN for Go2 robots.

    Args:
        timeout: seconds to wait for replies after sending the probe.
        iface_ip: pin multicast to this local IPv4 address. If None, probe every
            non-tunnel interfacel.
    """
    targets: list[tuple[str | None, str]] = (
        [(None, iface_ip)] if iface_ip else list(_candidate_ifaces())
    )
    if not targets:
        logger.warning("no usable interfaces found for Go2 discovery")
        return []

    found: dict[str, Go2Device] = {}
    for name, ip in targets:
        for dev in _probe_iface(ip, timeout):
            dev = Go2Device(
                serial=dev.serial, ip=dev.ip, iface=name or "?", mac=_resolve_mac(dev.ip)
            )
            found.setdefault(dev.serial, dev)
    return list(found.values())


def _probe_iface(iface_ip: str, timeout: float) -> Iterator[Go2Device]:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        sock.bind(("", REPLY_PORT))
    except OSError as e:
        logger.warning(f"could not bind UDP {REPLY_PORT} on {iface_ip}: {e}")
        sock.close()
        return

    iface_addr = socket.inet_aton(iface_ip)
    sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_IF, iface_addr)
    sock.setsockopt(
        socket.IPPROTO_IP,
        socket.IP_ADD_MEMBERSHIP,
        struct.pack("4s4s", socket.inet_aton(MULTICAST_GROUP), iface_addr),
    )

    try:
        sock.sendto(QUERY_PAYLOAD, (MULTICAST_GROUP, QUERY_PORT))
    except OSError as e:
        logger.warning(f"multicast send failed on {iface_ip}: {e}")
        sock.close()
        return

    sock.settimeout(timeout)
    try:
        while True:
            try:
                data, addr = sock.recvfrom(1024)
            except TimeoutError:
                return
            try:
                msg = json.loads(data.decode("utf-8"))
            except (UnicodeDecodeError, json.JSONDecodeError):
                continue
            sn = msg.get("sn")
            if not sn:
                continue
            yield Go2Device(serial=sn, ip=msg.get("ip", addr[0]), iface=iface_ip)
    finally:
        sock.close()


async def discover_lan(
    tick: float = 2.0,
    timeout: float = 1.5,
    iface_ip: str | None = None,
) -> AsyncIterator[Go2Device]:
    """Stream Go2 LAN discoveries. Polls multicast every `tick` seconds.

    Yields each device on every poll cycle it shows up in (so consumers see the
    device repeatedly while it remains alive on the network).
    """
    while True:
        loop = asyncio.get_running_loop()
        devs = await loop.run_in_executor(
            None, lambda: discover(timeout=timeout, iface_ip=iface_ip)
        )
        for d in devs:
            yield d
        await asyncio.sleep(max(0.0, tick - timeout))


def main() -> None:
    import argparse

    parser = argparse.ArgumentParser(description="Discover Unitree Go2 robots on the LAN")
    parser.add_argument("--timeout", type=float, default=2.0)
    parser.add_argument(
        "--iface-ip", help="pin probe to this local IPv4 (default: all non-tunnel ifaces)"
    )
    args = parser.parse_args()

    devices = discover(timeout=args.timeout, iface_ip=args.iface_ip)
    if not devices:
        print("no Go2s found")
        return
    for d in devices:
        print(f"{d.serial}\t{d.ip}\t(via {d.iface})")


if __name__ == "__main__":
    main()
