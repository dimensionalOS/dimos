from __future__ import annotations

import socket

from zeroconf import ServiceInfo, Zeroconf

SERVICE_TYPE = "_dimensional._tcp.local."
_PORT = 7667
_VIRTUAL_IFACE_PREFIXES = ("docker", "virbr", "lo", "tun", "veth", "br-")


def _local_ip() -> str:
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.connect(("8.8.8.8", 80))
            return s.getsockname()[0]
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
    return socket.gethostbyname(socket.gethostname())


def _encode_properties(props: dict[str, str]) -> dict[bytes, bytes]:
    return {k.encode(): v.encode() for k, v in props.items()}


class Advertiser:
    """Registers a static mDNS beacon so the harness can discover this robot."""

    def __init__(self, robot_name: str, lcm_url: str) -> None:
        self._robot_name = robot_name.split(".")[0]
        self._lcm_url = lcm_url
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
            properties=_encode_properties({
                "lcm_url": self._lcm_url,
            }),
        )
        self._zeroconf.register_service(self._info)

    def stop(self) -> None:
        if self._zeroconf is None or self._info is None:
            return
        self._zeroconf.unregister_service(self._info)
        self._zeroconf.close()
        self._zeroconf = None
        self._info = None
