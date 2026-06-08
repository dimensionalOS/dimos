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

"""Guided Go2 Wi-Fi setup orchestration without Typer wiring."""

from __future__ import annotations

import asyncio
from collections.abc import Awaitable, Callable, Sequence
from dataclasses import dataclass, replace
from typing import Any, Protocol

from dimos.robot.unitree.go2.cli.verify import Go2VerifyStatus, verify_robot_ip


class BluetoothRobot(Protocol):
    @property
    def name(self) -> str: ...

    @property
    def address(self) -> str: ...

    @property
    def serial(self) -> str | None: ...


class LanRobot(Protocol):
    @property
    def serial(self) -> str: ...

    @property
    def ip(self) -> str: ...


BleDiscovery = Callable[[], Awaitable[Sequence[BluetoothRobot]]]
LanDiscovery = Callable[[], Awaitable[Sequence[LanRobot]]]
WifiProvisioner = Callable[[str, str, str, str], Awaitable[str | None]]
RobotVerifier = Callable[[str], Go2VerifyStatus]


@dataclass(frozen=True)
class Go2BleSelection:
    name: str | None
    address: str
    serial: str | None

    def to_dict(self) -> dict[str, Any]:
        return {
            "name": self.name,
            "address": self.address,
            "serial": self.serial,
        }


@dataclass(frozen=True)
class Go2LanSelection:
    serial: str
    ip: str
    mac: str | None = None
    iface: str | None = None

    def to_dict(self) -> dict[str, Any]:
        return {
            "serial": self.serial,
            "ip": self.ip,
            "mac": self.mac,
            "iface": self.iface,
        }


@dataclass(frozen=True)
class SetupStep:
    name: str
    ok: bool
    message: str

    def to_dict(self) -> dict[str, Any]:
        return {
            "name": self.name,
            "ok": self.ok,
            "message": self.message,
        }


@dataclass(frozen=True)
class Go2WifiSetupResult:
    ok: bool
    selected_ble: Go2BleSelection | None
    lan_robot: Go2LanSelection | None
    verification: Go2VerifyStatus | None
    next_steps: tuple[str, ...]
    steps: tuple[SetupStep, ...]
    error: str | None = None

    @property
    def robot_ip(self) -> str | None:
        return self.lan_robot.ip if self.lan_robot else None

    def to_dict(self) -> dict[str, Any]:
        return {
            "ok": self.ok,
            "selected_ble": self.selected_ble.to_dict() if self.selected_ble else None,
            "lan_robot": self.lan_robot.to_dict() if self.lan_robot else None,
            "robot_ip": self.robot_ip,
            "verification": self.verification.to_dict() if self.verification else None,
            "next_steps": list(self.next_steps),
            "steps": [step.to_dict() for step in self.steps],
            "error": self.error,
        }

    def summary_lines(self) -> tuple[str, ...]:
        lines = [
            f"{'OK' if step.ok else 'FAIL'} {step.name}: {step.message}" for step in self.steps
        ]
        if self.error:
            lines.append(f"Error: {self.error}")
        lines.extend(self.next_steps)
        return tuple(lines)


def _redact(text: str | None, secret: str) -> str | None:
    if text is None or not secret:
        return text
    return text.replace(secret, "[REDACTED]")


def _redact_status(status: Go2VerifyStatus, secret: str) -> Go2VerifyStatus:
    return replace(
        status,
        reason=_redact(status.reason, secret),
        error=_redact(status.error, secret),
    )


def _same_address(left: str, right: str) -> bool:
    return left.casefold() == right.casefold()


def _matches_ble(
    robot: BluetoothRobot,
    *,
    serial: str | None,
    name: str | None,
    address: str | None,
) -> bool:
    if serial is not None and robot.serial != serial:
        return False
    if name is not None and robot.name != name:
        return False
    if address is not None and not _same_address(robot.address, address):
        return False
    return True


def _criteria_text(serial: str | None, name: str | None, address: str | None) -> str:
    pairs = [
        ("serial", serial),
        ("name", name),
        ("address", address),
    ]
    parts = [f"{key}={value}" for key, value in pairs if value is not None]
    return ", ".join(parts) if parts else "available robot"


def _select_ble_robot(
    robots: Sequence[BluetoothRobot],
    *,
    serial: str | None,
    name: str | None,
    address: str | None,
) -> tuple[Go2BleSelection | None, str | None]:
    if address is not None:
        address_matches = [robot for robot in robots if _same_address(robot.address, address)]
        matches = [
            robot
            for robot in address_matches
            if _matches_ble(robot, serial=serial, name=name, address=address)
        ]
        if len(matches) > 1:
            return None, f"Ambiguous BLE robot selection: {len(matches)} robots matched address."
        if matches:
            robot = matches[0]
            return Go2BleSelection(
                name=robot.name, address=robot.address, serial=robot.serial
            ), None
        if address_matches:
            return None, f"No BLE robot matched {_criteria_text(serial, name, address)}."
        return Go2BleSelection(name=name, address=address, serial=serial), None

    matches = [
        robot for robot in robots if _matches_ble(robot, serial=serial, name=name, address=address)
    ]
    if not matches:
        return None, f"No BLE robot matched {_criteria_text(serial, name, address)}."
    if len(matches) > 1:
        return (
            None,
            f"Ambiguous BLE robot selection: {len(matches)} robots matched "
            f"{_criteria_text(serial, name, address)}.",
        )
    robot = matches[0]
    return Go2BleSelection(name=robot.name, address=robot.address, serial=robot.serial), None


def _optional_str_attr(value: Any) -> str | None:
    return value if isinstance(value, str) else None


def _lan_selection(robot: LanRobot) -> Go2LanSelection:
    return Go2LanSelection(
        serial=robot.serial,
        ip=robot.ip,
        mac=_optional_str_attr(getattr(robot, "mac", None)),
        iface=_optional_str_attr(getattr(robot, "iface", None)),
    )


def _select_lan_robot(
    robots: Sequence[LanRobot],
    *,
    serial: str | None,
) -> tuple[Go2LanSelection | None, str | None]:
    if serial is not None:
        matches = [robot for robot in robots if robot.serial == serial]
        if len(matches) == 1:
            return _lan_selection(matches[0]), None
        if len(matches) > 1:
            return None, f"Ambiguous LAN discovery: {len(matches)} robots matched serial={serial}."
        return None, f"No LAN robot matched serial={serial}."

    if len(robots) == 1:
        return _lan_selection(robots[0]), None
    if robots:
        return None, f"Ambiguous LAN discovery: {len(robots)} robots found and no serial is known."
    return None, "No LAN robots found."


def _next_steps(ip: str) -> tuple[str, ...]:
    return (
        f"export ROBOT_IP={ip}",
        f"dimos --robot-ip {ip} run unitree-go2",
    )


def _result(
    *,
    ok: bool,
    selected_ble: Go2BleSelection | None,
    lan_robot: Go2LanSelection | None,
    verification: Go2VerifyStatus | None,
    next_steps: tuple[str, ...],
    steps: list[SetupStep],
    error: str | None = None,
) -> Go2WifiSetupResult:
    return Go2WifiSetupResult(
        ok=ok,
        selected_ble=selected_ble,
        lan_robot=lan_robot,
        verification=verification,
        next_steps=next_steps,
        steps=tuple(steps),
        error=error,
    )


async def setup_go2_wifi(
    *,
    ssid: str,
    password: str,
    discover_ble: BleDiscovery,
    provision_wifi: WifiProvisioner,
    discover_lan: LanDiscovery,
    serial: str | None = None,
    name: str | None = None,
    address: str | None = None,
    country_code: str = "US",
    verify_robot: RobotVerifier | None = None,
    rediscovery_attempts: int = 5,
    rediscovery_delay_s: float = 2.0,
) -> Go2WifiSetupResult:
    """Provision Go2 Wi-Fi over BLE, rediscover it on LAN, then verify WebRTC.

    The caller injects BLE discovery, Wi-Fi provisioning, and LAN discovery
    functions. This keeps Typer prompting and concrete backends outside the
    orchestration so it can be tested without hardware.
    """
    steps: list[SetupStep] = []

    def add_step(name: str, ok: bool, message: str) -> None:
        steps.append(SetupStep(name=name, ok=ok, message=_redact(message, password) or ""))

    def fail(
        name: str,
        message: str,
        *,
        selected_ble: Go2BleSelection | None = None,
        lan_robot: Go2LanSelection | None = None,
        verification: Go2VerifyStatus | None = None,
    ) -> Go2WifiSetupResult:
        redacted = _redact(message, password) or ""
        add_step(name, False, redacted)
        return _result(
            ok=False,
            selected_ble=selected_ble,
            lan_robot=lan_robot,
            verification=verification,
            next_steps=(),
            steps=steps,
            error=redacted,
        )

    try:
        ble_robots = list(await discover_ble())
    except Exception as e:
        return fail("ble_discovery", f"BLE discovery failed: {e}")
    add_step("ble_discovery", True, f"Found {len(ble_robots)} BLE robot(s).")

    selected_ble, selection_error = _select_ble_robot(
        ble_robots, serial=serial, name=name, address=address
    )
    if selected_ble is None:
        return fail("select_robot", selection_error or "No BLE robot selected.")
    add_step(
        "select_robot",
        True,
        f"Selected BLE robot address={selected_ble.address} serial={selected_ble.serial or '?'}",
    )

    try:
        provisioned_serial = await provision_wifi(
            selected_ble.address,
            ssid,
            password,
            country_code,
        )
    except Exception as e:
        return fail("provision_wifi", f"Wi-Fi provisioning failed: {e}", selected_ble=selected_ble)
    add_step("provision_wifi", True, "Provisioned Wi-Fi credentials over BLE.")

    target_serial = provisioned_serial or selected_ble.serial
    lan_robot: Go2LanSelection | None = None
    lan_error: str | None = None
    attempts = max(1, rediscovery_attempts)
    for attempt in range(attempts):
        try:
            lan_robots = list(await discover_lan())
        except Exception as e:
            return fail("lan_discovery", f"LAN discovery failed: {e}", selected_ble=selected_ble)

        lan_robot, lan_error = _select_lan_robot(lan_robots, serial=target_serial)
        if lan_robot is not None:
            break
        if lan_error and lan_error.startswith("Ambiguous"):
            return fail("lan_discovery", lan_error, selected_ble=selected_ble)
        if attempt + 1 < attempts:
            await asyncio.sleep(rediscovery_delay_s)

    if lan_robot is None:
        return fail(
            "lan_discovery",
            lan_error or "No LAN robot found after Wi-Fi provisioning.",
            selected_ble=selected_ble,
        )
    add_step("lan_discovery", True, f"Found LAN robot {lan_robot.serial} at {lan_robot.ip}.")

    verifier = verify_robot or verify_robot_ip
    try:
        verification = _redact_status(verifier(lan_robot.ip), password)
    except Exception as e:
        return fail(
            "verify",
            f"Verification probe failed: {e}",
            selected_ble=selected_ble,
            lan_robot=lan_robot,
        )
    if not verification.ok:
        message = " ".join(verification.summary_lines())
        return fail(
            "verify",
            message,
            selected_ble=selected_ble,
            lan_robot=lan_robot,
            verification=verification,
        )
    add_step("verify", True, f"Verified Go2 WebRTC endpoint at {verification.url}.")

    return _result(
        ok=True,
        selected_ble=selected_ble,
        lan_robot=lan_robot,
        verification=verification,
        next_steps=_next_steps(lan_robot.ip),
        steps=steps,
    )
