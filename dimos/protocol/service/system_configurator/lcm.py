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

from __future__ import annotations

import json
import re
import resource
import subprocess

from dimos.constants import STATE_DIR
from dimos.protocol.service.system_configurator.base import (
    SystemConfigurator,
    _read_sysctl_int,
    _write_sysctl_int,
)
from dimos.utils import prompt

_SYSCTL_CONF = STATE_DIR / "sysctl.json"


def _load_sysctl_conf() -> dict[str, int]:
    try:
        return json.loads(_SYSCTL_CONF.read_text())  # type: ignore[no-any-return]
    except (OSError, json.JSONDecodeError):
        return {}


def _save_sysctl_conf(data: dict[str, int]) -> None:
    _SYSCTL_CONF.parent.mkdir(parents=True, exist_ok=True)
    _SYSCTL_CONF.write_text(json.dumps(data))


# specific checks: multicast


class MulticastConfiguratorLinux(SystemConfigurator):
    critical = True
    MULTICAST_PREFIX = "224.0.0.0/4"

    def __init__(self, loopback_interface: str = "lo") -> None:
        self.loopback_interface = loopback_interface

        self.loopback_ok: bool | None = None
        self.route_ok: bool | None = None

        self.enable_multicast_cmd = [
            "ip",
            "link",
            "set",
            self.loopback_interface,
            "multicast",
            "on",
        ]
        self.add_route_cmd = [
            "ip",
            "route",
            "add",
            self.MULTICAST_PREFIX,
            "dev",
            self.loopback_interface,
        ]

    def check(self) -> bool:
        # Verify `ip` exists (iproute2)
        try:
            subprocess.run(["ip", "-V"], capture_output=True, text=True, check=False)
        except FileNotFoundError as error:
            print(
                f"ERROR: `ip` not found (iproute2 missing, did you install system requirements?): {error}"
            )
            self.loopback_ok = self.route_ok = False
            return False
        except Exception as error:
            print(f"ERROR: failed probing `ip`: {error}")
            self.loopback_ok = self.route_ok = False
            return False

        # check MULTICAST on loopback
        try:
            result = subprocess.run(
                ["ip", "-o", "link", "show", self.loopback_interface],
                capture_output=True,
                text=True,
            )
            if result.returncode != 0:
                print(
                    f"ERROR: `ip link show {self.loopback_interface}` rc={result.returncode} "
                    f"stderr={result.stderr!r}"
                )
                self.loopback_ok = False
            else:
                match = re.search(r"<([^>]*)>", result.stdout)
                flags = {
                    flag.strip().upper()
                    for flag in (match.group(1).split(",") if match else [])
                    if flag.strip()
                }
                self.loopback_ok = "MULTICAST" in flags
        except Exception as error:
            print(f"ERROR: failed checking loopback multicast: {error}")
            self.loopback_ok = False

        # Check if multicast route exists
        try:
            result = subprocess.run(
                ["ip", "-o", "route", "show", self.MULTICAST_PREFIX],
                capture_output=True,
                text=True,
            )
            if result.returncode != 0:
                print(
                    f"ERROR: `ip route show {self.MULTICAST_PREFIX}` rc={result.returncode} "
                    f"stderr={result.stderr!r}"
                )
                self.route_ok = False
            else:
                self.route_ok = bool(result.stdout.strip())
        except Exception as error:
            print(f"ERROR: failed checking multicast route: {error}")
            self.route_ok = False

        return bool(self.loopback_ok and self.route_ok)

    def explanation(self) -> str | None:
        lines = []
        if not self.loopback_ok:
            lines.append(f"Enable multicast with: `sudo {' '.join(self.enable_multicast_cmd)}`")
        if not self.route_ok:
            lines.append(f"Add multicast route with: `sudo {' '.join(self.add_route_cmd)}`")
        return "\n".join(lines) if lines else None

    def fix(self) -> None:
        if not self.loopback_ok:
            prompt.sudo_run(*self.enable_multicast_cmd, check=True, text=True, capture_output=True)
        if not self.route_ok:
            prompt.sudo_run(*self.add_route_cmd, check=True, text=True, capture_output=True)


class MulticastConfiguratorMacOS(SystemConfigurator):
    critical = True

    def __init__(self, loopback_interface: str = "lo0") -> None:
        self.loopback_interface = loopback_interface
        self.add_route_cmd = [
            "route",
            "add",
            "-net",
            "224.0.0.0/4",
            "-interface",
            self.loopback_interface,
        ]

    def check(self) -> bool:
        # `netstat -nr` shows the routing table. We search for a 224/4 route entry
        # that points to the loopback interface (lo0). The route often exists on
        # en0 (WiFi/Ethernet), which causes cross-process LCM communication to fail.
        try:
            result = subprocess.run(["netstat", "-nr"], capture_output=True, text=True)
            if result.returncode != 0:
                print(f"ERROR: `netstat -nr` rc={result.returncode} stderr={result.stderr!r}")
                return False

            for line in result.stdout.splitlines():
                if "224.0.0.0/4" in line or "224.0.0/4" in line:
                    if self.loopback_interface in line:
                        return True
            return False
        except Exception as error:
            print(f"ERROR: failed checking multicast route via netstat: {error}")
            return False

    def explanation(self) -> str | None:
        return f"Enable multicast with: `sudo {' '.join(self.add_route_cmd)}`"

    def fix(self) -> None:
        # Delete any existing 224.0.0.0/4 route (e.g. on en0) before adding on lo0,
        # otherwise `route add` fails with "route already in use"
        prompt.sudo_run(
            "route",
            "delete",
            "-net",
            "224.0.0.0/4",
            check=False,
            text=True,
            capture_output=True,
        )
        prompt.sudo_run(*self.add_route_cmd, check=True, text=True, capture_output=True)


# specific checks: buffers

IDEAL_RMEM_SIZE = 67_108_864  # 64MB


class BufferConfiguratorLinux(SystemConfigurator):
    critical = False

    TARGET_RMEM_SIZE = IDEAL_RMEM_SIZE

    def __init__(self) -> None:
        self.needs: list[tuple[str, int]] = []  # (key, target_value)

    def check(self) -> bool:
        self.needs.clear()
        for key, target in [
            ("net.core.rmem_max", self.TARGET_RMEM_SIZE),
            ("net.core.rmem_default", self.TARGET_RMEM_SIZE),
        ]:
            current = _read_sysctl_int(key)
            if current is None or current < target:
                self.needs.append((key, target))
        return not self.needs

    def explanation(self) -> str | None:
        lines = []
        for key, target in self.needs:
            lines.append(f"- socket buffer optimization for LCM: sudo sysctl -w {key}={target}")
        return "\n".join(lines)

    def fix(self) -> None:
        for key, target in self.needs:
            _write_sysctl_int(key, target)


class BufferConfiguratorMacOS(SystemConfigurator):
    critical = False

    TARGET = IDEAL_RMEM_SIZE

    KEYS = (
        "kern.ipc.maxsockbuf",
        "net.inet.udp.recvspace",
        "net.inet.udp.maxdgram",
    )

    def __init__(self) -> None:
        self.needs: list[tuple[str, int, int]] = []  # (key, target, current)

    def check(self) -> bool:
        self.needs.clear()
        saved = _load_sysctl_conf()
        for key in self.KEYS:
            target = saved.get(key, self.TARGET)
            current = _read_sysctl_int(key) or 0
            if current < target:
                self.needs.append((key, target, current))
        return not self.needs

    def explanation(self) -> str | None:
        lines = []
        for key, target, _ in self.needs:
            lines.append(f"- socket buffer optimization for LCM: sudo sysctl -w {key}={target}")
        return "\n".join(lines)

    def fix(self) -> None:
        saved = _load_sysctl_conf()
        for key, target, current in self.needs:
            while target > current:
                try:
                    _write_sysctl_int(key, target)
                except subprocess.CalledProcessError:
                    target //= 2
                else:
                    saved[key] = target
                    break
        # Write current amounts to config to avoid requesting TARGET every startup.
        _save_sysctl_conf(saved)


# specific checks: ulimit


class MaxFileConfiguratorMacOS(SystemConfigurator):
    """Ensure the open file descriptor limit (ulimit -n) is at least TARGET_FILE_COUNT_LIMIT."""

    critical = False
    TARGET_FILE_COUNT_LIMIT = 65536

    def __init__(self, target: int = TARGET_FILE_COUNT_LIMIT) -> None:
        self.target = target
        self.current_soft: int = 0
        self.current_hard: int = 0
        self.can_fix_without_sudo: bool = False

    def check(self) -> bool:
        try:
            self.current_soft, self.current_hard = resource.getrlimit(resource.RLIMIT_NOFILE)
        except Exception as error:
            print(f"[ulimit] ERROR: failed to get RLIMIT_NOFILE: {error}")
            return False

        if self.current_soft >= self.target:
            return True

        # Check if we can raise to target without sudo (hard limit is high enough)
        self.can_fix_without_sudo = self.current_hard >= self.target
        return False

    def explanation(self) -> str | None:
        lines = []
        if self.can_fix_without_sudo:
            lines.append(
                f"- Raise soft file count limit to {self.target} for LCM (no sudo required)"
            )
        else:
            lines.append(
                f"- Raise soft file count limit to {min(self.target, self.current_hard)} for LCM"
            )
            lines.append(
                f"- Raise hard limit via: sudo launchctl limit maxfiles {self.target} {self.target} for LCM"
            )
        return "\n".join(lines)

    def fix(self) -> None:
        if self.current_soft >= self.target:
            return

        if self.can_fix_without_sudo:
            # Hard limit is sufficient, just raise the soft limit
            try:
                resource.setrlimit(resource.RLIMIT_NOFILE, (self.target, self.current_hard))
            except Exception as error:
                print(f"[ulimit] ERROR: failed to set soft limit: {error}")
                raise
        else:
            # Need to raise both soft and hard limits via launchctl
            try:
                prompt.sudo_run(
                    "launchctl",
                    "limit",
                    "maxfiles",
                    str(self.target),
                    str(self.target),
                    check=True,
                    text=True,
                    capture_output=True,
                )
            except subprocess.CalledProcessError as error:
                print(f"[ulimit] WARNING: launchctl failed: {error.stderr}")
                # Fallback: raise soft limit as high as the current hard limit allows
                if self.current_hard > self.current_soft:
                    try:
                        resource.setrlimit(
                            resource.RLIMIT_NOFILE, (self.current_hard, self.current_hard)
                        )
                    except Exception as fallback_error:
                        print(f"[ulimit] ERROR: fallback also failed: {fallback_error}")
                raise

            # After launchctl, try to apply the new limit to the current process
            try:
                resource.setrlimit(resource.RLIMIT_NOFILE, (self.target, self.target))
            except Exception as error:
                print(
                    f"[ulimit] WARNING: could not apply to current process (restart may be required): {error}"
                )
