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

"""Read-only diagnostics for local Wi-Fi Unitree Go2 teleop."""

from __future__ import annotations

from dataclasses import asdict, dataclass, replace
from enum import Enum
import ipaddress
import re
import socket
import subprocess
import threading
from typing import Any

import psutil
import requests

from dimos.core.run_registry import RunEntry, get_most_recent
from dimos.robot.unitree.go2.cli.landiscovery import Go2Device, discover
from dimos.visualization.rerun.constants import RERUN_GRPC_PORT, RERUN_WEB_VIEWER_PORT

COMMAND_CENTER_PORT = 7779
PHONE_TELEOP_PORT = 8444
GO2_SIGNAL_PORT = 9991
DEFAULT_IMAGE_TOPICS = ("jpeg_lcm:/color_image", "pshm:color_image")


class CheckLevel(str, Enum):
    OK = "OK"
    WARN = "WARN"
    FAIL = "FAIL"
    SKIP = "SKIP"


@dataclass(frozen=True)
class LocalInterface:
    name: str
    ip: str
    netmask: str | None
    matches_robot_subnet: bool


@dataclass(frozen=True)
class RobotCheck:
    level: CheckLevel
    robot_ip: str | None
    signal_port: int
    signal_reachable: bool
    discovered: list[dict[str, str | None]]
    message: str


@dataclass(frozen=True)
class RunCheck:
    level: CheckLevel
    run_id: str | None
    pid: int | None
    blueprint: str | None
    log_dir: str | None
    message: str


@dataclass(frozen=True)
class UiEndpoint:
    name: str
    port: int
    scheme: str


@dataclass(frozen=True)
class PortCheck:
    level: CheckLevel
    name: str
    port: int
    listening: bool
    bind_hosts: list[str]
    lan_reachable: bool | None
    message: str


@dataclass(frozen=True)
class ImageCheck:
    level: CheckLevel
    checked: bool
    topic: str | None
    width: int | None
    height: int | None
    message: str
    attempted_topics: list[str]


@dataclass(frozen=True)
class Go2DoctorReport:
    robot: RobotCheck
    interfaces: list[LocalInterface]
    run: RunCheck
    ports: list[PortCheck]
    image: ImageCheck
    suggested_urls: list[str]

    def has_problems(self) -> bool:
        levels = [self.robot.level, self.run.level, self.image.level]
        levels.extend(p.level for p in self.ports)
        if any(level is CheckLevel.FAIL for level in levels):
            return True
        return any(level is CheckLevel.WARN for level in levels)

    def as_dict(self) -> dict[str, Any]:
        return asdict(self)


def default_ui_endpoints(rerun_websocket_port: int) -> list[UiEndpoint]:
    return [
        UiEndpoint("Command center", COMMAND_CENTER_PORT, "http"),
        UiEndpoint("Phone teleop", PHONE_TELEOP_PORT, "https"),
        UiEndpoint("Rerun web viewer", RERUN_WEB_VIEWER_PORT, "http"),
        UiEndpoint("Rerun gRPC proxy", RERUN_GRPC_PORT, "rerun+http"),
        UiEndpoint("Rerun keyboard WebSocket", rerun_websocket_port, "ws"),
    ]


def local_interfaces(robot_ip: str | None = None) -> list[LocalInterface]:
    interfaces: list[LocalInterface] = []
    for name, addrs in psutil.net_if_addrs().items():
        for addr in addrs:
            if addr.family != socket.AF_INET or addr.address.startswith("127."):
                continue
            interfaces.append(
                LocalInterface(
                    name=name,
                    ip=addr.address,
                    netmask=addr.netmask,
                    matches_robot_subnet=_same_subnet(addr.address, addr.netmask, robot_ip),
                )
            )
    return interfaces


def check_robot(
    robot_ip: str | None,
    *,
    signal_port: int = GO2_SIGNAL_PORT,
    connect_timeout: float = 1.0,
    discover_lan: bool = True,
    discovery_timeout: float = 1.5,
) -> RobotCheck:
    devices = discover(timeout=discovery_timeout) if discover_lan else []
    discovered = [_device_dict(d) for d in devices]

    if robot_ip is None:
        if len(devices) == 1:
            device = devices[0]
            reachable = _signal_endpoint_reachable(device.ip, signal_port, connect_timeout)
            suffix = (
                f"{device.ip}:{signal_port}/con_notify responded."
                if reachable
                else f"{device.ip}:{signal_port}/con_notify did not respond."
            )
            return RobotCheck(
                level=CheckLevel.OK if reachable else CheckLevel.WARN,
                robot_ip=device.ip,
                signal_port=signal_port,
                signal_reachable=reachable,
                discovered=discovered,
                message=f"Discovered one Go2 at {device.ip} via {device.iface}. {suffix}",
            )
        if devices:
            return RobotCheck(
                level=CheckLevel.WARN,
                robot_ip=None,
                signal_port=signal_port,
                signal_reachable=False,
                discovered=discovered,
                message="Multiple Go2 robots discovered; pass --robot-ip to select one.",
            )
        return RobotCheck(
            level=CheckLevel.WARN,
            robot_ip=None,
            signal_port=signal_port,
            signal_reachable=False,
            discovered=[],
            message="No robot IP configured and LAN discovery found no Go2 robots.",
        )

    reachable = _signal_endpoint_reachable(robot_ip, signal_port, connect_timeout)
    matching_devices = [d for d in devices if d.ip == robot_ip]
    if reachable:
        detail = f"{robot_ip}:{signal_port}/con_notify responded."
        if matching_devices:
            detail += " LAN discovery also found this robot."
        return RobotCheck(
            level=CheckLevel.OK,
            robot_ip=robot_ip,
            signal_port=signal_port,
            signal_reachable=True,
            discovered=discovered,
            message=detail,
        )

    discovered_ips = {d.ip for d in devices}
    if robot_ip in discovered_ips:
        message = (
            f"LAN discovery found {robot_ip}, but {signal_port}/con_notify did not respond."
        )
        level = CheckLevel.WARN
    elif discovered_ips:
        message = (
            f"{robot_ip}:{signal_port}/con_notify is not reachable; discovered Go2 IPs: "
            f"{', '.join(sorted(discovered_ips))}."
        )
        level = CheckLevel.FAIL
    else:
        message = (
            f"{robot_ip}:{signal_port}/con_notify is not reachable "
            "and LAN discovery found no Go2."
        )
        level = CheckLevel.FAIL

    return RobotCheck(
        level=level,
        robot_ip=robot_ip,
        signal_port=signal_port,
        signal_reachable=False,
        discovered=discovered,
        message=message,
    )


def check_run() -> RunCheck:
    entry = get_most_recent(alive_only=True)
    if entry is None:
        return RunCheck(
            level=CheckLevel.WARN,
            run_id=None,
            pid=None,
            blueprint=None,
            log_dir=None,
            message="No running DimOS instance found.",
        )
    return _run_check_from_entry(entry)


def check_ports(
    endpoints: list[UiEndpoint],
    *,
    probe_hosts: list[str] | None = None,
) -> list[PortCheck]:
    listeners = _tcp_listeners()
    hosts = probe_hosts or ["127.0.0.1"]
    return [
        _check_port(endpoint, listeners.get(endpoint.port, []), probe_hosts=hosts)
        for endpoint in endpoints
    ]


def check_image(
    *,
    enabled: bool,
    topics: list[str],
    timeout: float,
) -> ImageCheck:
    if not enabled:
        return ImageCheck(
            level=CheckLevel.SKIP,
            checked=False,
            topic=None,
            width=None,
            height=None,
            message="Image check skipped. Pass --check-image to wait for color_image.",
            attempted_topics=list(topics),
        )

    from dimos.msgs.sensor_msgs.Image import Image
    from dimos.protocol.pubsub.registry import subscribe_pubsub_uri

    found: dict[str, Image] = {}
    errors: list[str] = []
    ready = threading.Event()
    subscriptions: list[tuple[Any, Any]] = []

    def on_image(topic: str) -> Any:
        def _callback(msg: Image) -> None:
            if not ready.is_set():
                found[topic] = msg
                ready.set()

        return _callback

    try:
        for topic in topics:
            try:
                transport, unsubscribe = subscribe_pubsub_uri(
                    topic,
                    on_image(topic),
                    msg_type=Image,
                )
            except Exception as exc:
                errors.append(f"{topic}: {exc}")
                continue
            subscriptions.append((transport, unsubscribe))

        ready.wait(timeout=max(0.0, timeout))
    finally:
        for transport, unsubscribe in subscriptions:
            try:
                unsubscribe()
            except Exception:
                pass
            try:
                transport.stop()
            except Exception:
                pass

    if found:
        topic, image = next(iter(found.items()))
        return ImageCheck(
            level=CheckLevel.OK,
            checked=True,
            topic=topic,
            width=image.width,
            height=image.height,
            message=f"Received color_image frame on {topic}: {image.width}x{image.height}.",
            attempted_topics=list(topics),
        )

    if errors and len(errors) == len(topics):
        return ImageCheck(
            level=CheckLevel.FAIL,
            checked=True,
            topic=None,
            width=None,
            height=None,
            message="Could not subscribe to any image topic: " + "; ".join(errors),
            attempted_topics=list(topics),
        )

    message = f"No color_image frame received within {timeout:.1f}s."
    if errors:
        message += " Some subscriptions failed: " + "; ".join(errors)
    return ImageCheck(
        level=CheckLevel.WARN,
        checked=True,
        topic=None,
        width=None,
        height=None,
        message=message,
        attempted_topics=list(topics),
    )


def collect_report(
    *,
    robot_ip: str | None,
    discover_lan: bool,
    discovery_timeout: float,
    connect_timeout: float,
    signal_port: int,
    endpoints: list[UiEndpoint],
    check_image_enabled: bool,
    image_topics: list[str] | None = None,
    image_timeout: float = 2.0,
) -> Go2DoctorReport:
    robot = check_robot(
        robot_ip,
        signal_port=signal_port,
        connect_timeout=connect_timeout,
        discover_lan=discover_lan,
        discovery_timeout=discovery_timeout,
    )
    resolved_robot_ip = robot.robot_ip or robot_ip
    interfaces = local_interfaces(resolved_robot_ip)
    run = check_run()
    probe_hosts = ["127.0.0.1", *(iface.ip for iface in interfaces)]
    ports = check_ports(endpoints, probe_hosts=probe_hosts)
    topics = list(image_topics or DEFAULT_IMAGE_TOPICS)
    image = check_image(enabled=check_image_enabled, topics=topics, timeout=image_timeout)
    robot = _adjust_robot_check_with_runtime_evidence(robot, image)
    urls = suggested_urls(interfaces, ports)
    return Go2DoctorReport(
        robot=robot,
        interfaces=interfaces,
        run=run,
        ports=ports,
        image=image,
        suggested_urls=urls,
    )


def format_report(report: Go2DoctorReport) -> str:
    lines = ["Go2 local Wi-Fi teleop doctor", ""]

    lines.append("Robot")
    lines.append(f"  [{report.robot.level.value}] {report.robot.message}")
    if report.robot.discovered:
        for device in report.robot.discovered:
            serial = device.get("serial") or "?"
            ip = device.get("ip") or "?"
            iface = device.get("iface") or "?"
            mac = device.get("mac") or "-"
            lines.append(f"        discovered serial={serial} ip={ip} iface={iface} mac={mac}")

    lines.extend(["", "Local network"])
    if report.interfaces:
        for iface in report.interfaces:
            suffix = "same subnet as robot" if iface.matches_robot_subnet else "not matched"
            lines.append(f"  [OK] {iface.name}: {iface.ip} ({suffix})")
    else:
        lines.append("  [WARN] No non-loopback IPv4 interfaces found.")

    lines.extend(["", "DimOS run"])
    lines.append(f"  [{report.run.level.value}] {report.run.message}")
    if report.run.run_id:
        lines.append(f"        run_id={report.run.run_id} pid={report.run.pid}")
        lines.append(f"        blueprint={report.run.blueprint}")
        lines.append(f"        log={report.run.log_dir}")

    lines.extend(["", "UI listeners"])
    for port in report.ports:
        lines.append(f"  [{port.level.value}] {port.name} port {port.port}: {port.message}")

    lines.extend(["", "Video"])
    lines.append(f"  [{report.image.level.value}] {report.image.message}")

    if report.suggested_urls:
        lines.extend(["", "Suggested URLs"])
        lines.extend(f"  {url}" for url in report.suggested_urls)

    return "\n".join(lines)


def suggested_urls(interfaces: list[LocalInterface], ports: list[PortCheck]) -> list[str]:
    command = next((p for p in ports if p.port == COMMAND_CENTER_PORT and p.listening), None)
    phone = next((p for p in ports if p.port == PHONE_TELEOP_PORT and p.listening), None)
    if command is None and phone is None:
        return []

    preferred = [i for i in interfaces if i.matches_robot_subnet] or interfaces
    urls: list[str] = []
    for iface in preferred:
        if command is not None and _port_lan_accessible(command):
            urls.append(f"http://{iface.ip}:{COMMAND_CENTER_PORT}/command-center")
        if phone is not None and _port_lan_accessible(phone):
            urls.append(f"https://{iface.ip}:{PHONE_TELEOP_PORT}/teleop")
    return urls


def _same_subnet(local_ip: str, netmask: str | None, robot_ip: str | None) -> bool:
    if robot_ip is None or netmask is None:
        return False
    try:
        network = ipaddress.ip_network(f"{local_ip}/{netmask}", strict=False)
        return ipaddress.ip_address(robot_ip) in network
    except ValueError:
        return False


def _adjust_robot_check_with_runtime_evidence(
    robot: RobotCheck,
    image: ImageCheck,
) -> RobotCheck:
    if robot.level is not CheckLevel.FAIL or image.level is not CheckLevel.OK:
        return robot

    return replace(
        robot,
        level=CheckLevel.WARN,
        message=(
            f"{robot.message} Received color_image from the running Go2 stack, "
            "so the WebRTC data plane is active; the signal-port probe may not "
            "apply to this connection mode."
        ),
    )


def _tcp_reachable(host: str, port: int, timeout: float) -> bool:
    try:
        with socket.create_connection((host, port), timeout=timeout):
            return True
    except OSError:
        return False


def _signal_endpoint_reachable(host: str, port: int, timeout: float) -> bool:
    try:
        response = requests.get(f"http://{host}:{port}/con_notify", timeout=timeout)
    except requests.RequestException:
        return False
    return response.ok


def _device_dict(device: Go2Device) -> dict[str, str | None]:
    return {
        "serial": device.serial,
        "ip": device.ip,
        "iface": device.iface,
        "mac": device.mac,
    }


def _run_check_from_entry(entry: RunEntry) -> RunCheck:
    level = CheckLevel.OK if "go2" in entry.blueprint else CheckLevel.WARN
    if level is CheckLevel.OK:
        message = f"Running DimOS Go2 blueprint: {entry.blueprint}."
    else:
        message = f"Running DimOS blueprint is not Go2-specific: {entry.blueprint}."
    return RunCheck(
        level=level,
        run_id=entry.run_id,
        pid=entry.pid,
        blueprint=entry.blueprint,
        log_dir=entry.log_dir,
        message=message,
    )


def _tcp_listeners() -> dict[int, list[str]]:
    listeners: dict[int, set[str]] = {}
    try:
        connections = psutil.net_connections(kind="tcp")
    except (OSError, psutil.Error):
        return _lsof_tcp_listeners()

    for conn in connections:
        if conn.status != psutil.CONN_LISTEN or not conn.laddr:
            continue
        port = getattr(conn.laddr, "port", None)
        host = getattr(conn.laddr, "ip", None)
        if port is None or host is None:
            continue
        listeners.setdefault(int(port), set()).add(str(host))

    if not listeners:
        return _lsof_tcp_listeners()
    return {port: sorted(hosts) for port, hosts in listeners.items()}


def _check_port(
    endpoint: UiEndpoint,
    bind_hosts: list[str],
    *,
    probe_hosts: list[str],
) -> PortCheck:
    if not bind_hosts:
        connectable_hosts = _connectable_hosts(endpoint.port, probe_hosts)
        if connectable_hosts:
            lan_reachable = any(_host_is_lan_reachable(host) for host in connectable_hosts)
            level = CheckLevel.OK if lan_reachable else CheckLevel.WARN
            suffix = (
                "LAN reachable"
                if lan_reachable
                else "connectable only through loopback; LAN devices cannot reach it"
            )
            return PortCheck(
                level=level,
                name=endpoint.name,
                port=endpoint.port,
                listening=True,
                bind_hosts=[],
                lan_reachable=lan_reachable,
                message=(
                    f"connectable via {', '.join(connectable_hosts)} "
                    f"({suffix}; listener bind address unavailable)"
                ),
            )
        return PortCheck(
            level=CheckLevel.WARN,
            name=endpoint.name,
            port=endpoint.port,
            listening=False,
            bind_hosts=[],
            lan_reachable=False,
            message="not listening",
        )

    lan_reachable = any(_host_is_lan_reachable(host) for host in bind_hosts)
    if lan_reachable:
        return PortCheck(
            level=CheckLevel.OK,
            name=endpoint.name,
            port=endpoint.port,
            listening=True,
            bind_hosts=bind_hosts,
            lan_reachable=True,
            message=f"listening on {', '.join(bind_hosts)}; LAN devices should be able to connect",
        )
    return PortCheck(
        level=CheckLevel.WARN,
        name=endpoint.name,
        port=endpoint.port,
        listening=True,
        bind_hosts=bind_hosts,
        lan_reachable=False,
        message=f"listening only on {', '.join(bind_hosts)}; LAN devices cannot reach it",
    )


def _host_is_lan_reachable(host: str) -> bool:
    return host in {"0.0.0.0", "::"} or not (
        host.startswith("127.") or host in {"::1", "localhost"}
    )


def _port_lan_accessible(port: PortCheck) -> bool:
    return bool(port.listening and port.lan_reachable)


def _connectable_hosts(port: int, hosts: list[str], timeout: float = 0.15) -> list[str]:
    seen: set[str] = set()
    reachable: list[str] = []
    for host in hosts:
        if host in seen:
            continue
        seen.add(host)
        if _tcp_reachable(host, port, timeout):
            reachable.append(host)
    return reachable


def _lsof_tcp_listeners() -> dict[int, list[str]]:
    try:
        result = subprocess.run(
            ["lsof", "-nP", "-iTCP", "-sTCP:LISTEN"],
            capture_output=True,
            text=True,
            timeout=1.0,
            check=False,
        )
    except (OSError, subprocess.SubprocessError):
        return {}

    if result.returncode not in {0, 1}:
        return {}

    listeners: dict[int, set[str]] = {}
    for line in result.stdout.splitlines():
        parsed = _parse_lsof_tcp_listener(line)
        if parsed is None:
            continue
        host, port = parsed
        listeners.setdefault(port, set()).add(host)
    return {port: sorted(hosts) for port, hosts in listeners.items()}


def _parse_lsof_tcp_listener(line: str) -> tuple[str, int] | None:
    match = re.search(r"TCP\s+(.+):(\d+)\s+\(LISTEN\)", line)
    if match is None:
        return None
    host = match.group(1).strip()
    if host == "*":
        host = "0.0.0.0"
    host = host.strip("[]")
    return host, int(match.group(2))
