from __future__ import annotations

import os
import socket

from zeroconf import ServiceInfo, Zeroconf

SERVICE_TYPE = "_dimensional._tcp.local."
_PORT = 7667
_VIRTUAL_IFACE_PREFIXES = ("docker", "virbr", "lo", "tun", "veth", "br-")


def _local_ip() -> str:
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.connect(("8.8.8.8", 80))
            ip = s.getsockname()[0]
            if not ip.startswith("127."):
                return ip
    except OSError:
        pass
    try:
        import psutil
        for iface, addrs in psutil.net_if_addrs().items():
            if any(iface.startswith(p) for p in _VIRTUAL_IFACE_PREFIXES):
                continue
            for addr in addrs:
                if addr.family == socket.AF_INET and not addr.address.startswith("127."):
                    return addr.address
    except ImportError:
        pass
    raise OSError("no non-loopback IPv4 address found")


class Advertiser:
    """Registers a static mDNS beacon so the harness can discover this robot."""

    def __init__(self) -> None:
        self._robot_name = os.environ.get("DIMENSIONAL_ROBOT_NAME", socket.gethostname().split(".")[0])
        self._lcm_url = os.environ.get("LCM_DEFAULT_URL", "udpm://239.255.76.67:7667?ttl=1")
        self._zeroconf: Zeroconf | None = None
        self._info: ServiceInfo | None = None

    def start(self) -> None:
        ip = _local_ip()
        self._zeroconf = Zeroconf()
        self._info = ServiceInfo(
            SERVICE_TYPE,
            f"{self._robot_name}.{SERVICE_TYPE}",
            addresses=[socket.inet_aton(ip)],
            port=_PORT,
            properties={b"lcm_url": self._lcm_url.encode()},
        )
        self._zeroconf.register_service(self._info)

    def stop(self) -> None:
        if self._zeroconf is None or self._info is None:
            return
        self._zeroconf.unregister_service(self._info)
        self._zeroconf.close()
        self._zeroconf = None
        self._info = None
