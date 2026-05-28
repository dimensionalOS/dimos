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

"""PACK MIND — live preflight for the 2-dog / 2-laptop demo.

Run on EACH laptop the morning of the demo, BEFORE starting dogs. Walks the whole
AP-per-dog + Tailscale chain link by link and prints PASS/WARN/FAIL with the exact
fix for every failure. Pure stdlib (macOS-focused) — no dimos import, no extra deps.

    uv run python -m dimos.experimental.pack_mind.preflight \\
        --role alpha --peer <peer-tailscale-ip> --coordinator http://127.0.0.1:8090

Exit code 0 only when nothing is FAIL (WARN is allowed). See LIVE_RUNBOOK.md.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import json
import os
import shutil
import socket
import subprocess
import sys
from typing import Literal
import urllib.request

Status = Literal["PASS", "WARN", "FAIL"]

_GREEN = "\033[32m"
_YELLOW = "\033[33m"
_RED = "\033[31m"
_DIM = "\033[2m"
_RESET = "\033[0m"
_COLOR: dict[Status, str] = {"PASS": _GREEN, "WARN": _YELLOW, "FAIL": _RED}

_DOG_AP_SUBNET_PREFIX = "192.168.12."  # Unitree Go2 AP default
_TS_APP_CLI = "/Applications/Tailscale.app/Contents/MacOS/Tailscale"


@dataclass
class Check:
    status: Status
    name: str
    detail: str
    fix: str = ""


def _run(cmd: list[str], timeout: float = 6.0) -> tuple[int, str]:
    """Run a command; return (rc, combined stdout+stderr). rc=-1 on missing/timeout."""
    try:
        p = subprocess.run(
            cmd, capture_output=True, text=True, timeout=timeout, check=False
        )
        return p.returncode, (p.stdout + p.stderr).strip()
    except (FileNotFoundError, subprocess.TimeoutExpired) as exc:
        return -1, str(exc)


def _route_iface(target: str) -> str | None:
    """macOS `route -n get <target>` -> the egress interface name, or None."""
    rc, out = _run(["route", "-n", "get", target])
    if rc != 0:
        return None
    for line in out.splitlines():
        stripped = line.strip()
        if stripped.startswith("interface:"):
            return stripped.split(":", 1)[1].strip()
    return None


def _ifaddr(iface: str) -> str | None:
    rc, out = _run(["ipconfig", "getifaddr", iface])
    return out.strip() if rc == 0 and out.strip() else None


def _tailscale_bin() -> str | None:
    return shutil.which("tailscale") or (
        _TS_APP_CLI if os.path.exists(_TS_APP_CLI) else None
    )


# -- individual checks -------------------------------------------------------


def check_env(role: str) -> list[Check]:
    out: list[Check] = []
    dog = os.environ.get("PACK_DOG_NAME")
    if dog and dog == role:
        out.append(Check("PASS", "env PACK_DOG_NAME", dog))
    elif dog:
        out.append(
            Check("WARN", "env PACK_DOG_NAME", f"{dog!r} != --role {role!r}",
                  f"export PACK_DOG_NAME={role}")
        )
    else:
        out.append(Check("FAIL", "env PACK_DOG_NAME", "unset", f"export PACK_DOG_NAME={role}"))

    url = os.environ.get("PACK_COORDINATOR_URL")
    if url:
        out.append(Check("PASS", "env PACK_COORDINATOR_URL", url))
    else:
        out.append(Check("FAIL", "env PACK_COORDINATOR_URL", "unset",
                         "export PACK_COORDINATOR_URL=http://<coord>:8090"))

    key = os.environ.get("OPENAI_API_KEY")
    if key:
        out.append(Check("PASS", "env OPENAI_API_KEY", "set"))
    else:
        out.append(Check("WARN", "env OPENAI_API_KEY", "unset (only needed for autonomy path 4b)",
                         "export OPENAI_API_KEY=<key>"))
    return out


def check_dog(robot_ip: str) -> list[Check]:
    out: list[Check] = []
    rc, _ = _run(["ping", "-c1", "-t2", robot_ip], timeout=5)
    if rc != 0:
        out.append(
            Check("FAIL", "dog reachable", f"{robot_ip} no reply",
                  "join the dog's Wi-Fi AP; confirm ROBOT_IP; power-cycle the dog")
        )
        return out
    out.append(Check("PASS", "dog reachable", robot_ip))

    dog_iface = _route_iface(robot_ip)
    if dog_iface:
        out.append(Check("PASS", "route->dog", f"{robot_ip} via {dog_iface}"))
    else:
        out.append(Check("WARN", "route->dog", "no route entry", "reconnect the dog Wi-Fi"))
    return out


def check_internet(robot_ip: str) -> list[Check]:
    out: list[Check] = []
    dog_iface = _route_iface(robot_ip)
    wan_iface = _route_iface("8.8.8.8") or _route_iface("default")

    if not wan_iface:
        out.append(Check("FAIL", "default route", "none",
                         "connect the uplink (USB tether / Ethernet)"))
        return out

    if dog_iface and wan_iface == dog_iface:
        out.append(
            Check("FAIL", "default route", f"WAN routes via dog AP iface {wan_iface}",
                  "System Settings->Network->...->Set Service Order: put the uplink ABOVE Wi-Fi")
        )
    else:
        out.append(Check("PASS", "default route", f"WAN via {wan_iface} (dog via {dog_iface})"))

    wan_ip = _ifaddr(wan_iface)
    if wan_ip and wan_ip.startswith(_DOG_AP_SUBNET_PREFIX):
        out.append(
            Check("FAIL", "subnet collision",
                  f"uplink {wan_ip} collides with dog AP {_DOG_AP_SUBNET_PREFIX}x",
                  "change the uplink subnet or the dog AP subnet -- they must differ")
        )
    elif wan_ip:
        out.append(Check("PASS", "uplink subnet", wan_ip))

    try:
        with socket.create_connection(("1.1.1.1", 443), timeout=5):
            out.append(Check("PASS", "internet", "1.1.1.1:443 reachable"))
    except OSError as exc:
        out.append(Check("FAIL", "internet", str(exc),
                         "fix the uplink / service order before Tailscale will work"))
    return out


def check_tailscale(peer: str | None) -> list[Check]:
    out: list[Check] = []
    ts = _tailscale_bin()
    if not ts:
        out.append(
            Check("FAIL", "tailscale", "CLI not found",
                  f"install Tailscale; App Store build CLI at {_TS_APP_CLI}")
        )
        return out

    rc, status_txt = _run([ts, "status"])
    if rc != 0:
        out.append(Check("FAIL", "tailscale status", status_txt[:80] or "down", f"{ts} up"))
        return out

    rc, self_ip_raw = _run([ts, "ip", "-4"])
    self_ip = self_ip_raw.splitlines()[0].strip() if rc == 0 and self_ip_raw.strip() else ""
    if self_ip.startswith("100."):
        out.append(Check("PASS", "tailscale up", f"self {self_ip}"))
    else:
        out.append(Check("WARN", "tailscale up", "no 100.x self ip", f"{ts} up --accept-routes"))

    if peer:
        rc, pout = _run([ts, "ping", "--timeout=4s", "--c=1", peer], timeout=8)
        if rc == 0 and "pong" in pout.lower():
            out.append(Check("PASS", "peer reachable", f"tailscale ping {peer}"))
        else:
            rc2, _ = _run(["ping", "-c1", "-t4", peer], timeout=6)
            if rc2 == 0:
                out.append(Check("PASS", "peer reachable", f"icmp {peer}"))
            else:
                out.append(Check("FAIL", "peer reachable", f"{peer} no pong/icmp",
                                 "check peer laptop tailscale up + its service order/uplink"))
    return out


def check_coordinator(url: str) -> list[Check]:
    url = url.rstrip("/")
    try:
        with urllib.request.urlopen(f"{url}/state", timeout=5) as resp:
            data = json.loads(resp.read().decode())
        zones = data.get("zones") if isinstance(data, dict) else None
        if isinstance(zones, list) and zones:
            names = ",".join(str(z.get("name", "?")) for z in zones)
            return [Check("PASS", "coordinator /state", f"{url} zones=[{names}]")]
        return [Check("WARN", "coordinator /state", f"{url} reachable but no zones",
                      "start it with --zones north,east,south,west")]
    except Exception as exc:
        return [Check("FAIL", "coordinator /state", f"{url}: {exc}",
                      "start pack_coordinator_server on Laptop-A; check the tailscale URL/port")]


# -- driver ------------------------------------------------------------------


def main() -> int:
    ap = argparse.ArgumentParser(description="PACK MIND live preflight (per laptop)")
    ap.add_argument("--role", required=True, choices=["alpha", "bravo"])
    ap.add_argument("--peer", default=None, help="the OTHER laptop's tailscale IP")
    ap.add_argument(
        "--coordinator",
        default=os.environ.get("PACK_COORDINATOR_URL", "http://127.0.0.1:8090"),
    )
    ap.add_argument("--robot-ip", default=os.environ.get("ROBOT_IP", "192.168.12.1"))
    args = ap.parse_args()

    checks: list[Check] = []
    checks += check_env(args.role)
    checks += check_dog(args.robot_ip)
    checks += check_internet(args.robot_ip)
    checks += check_tailscale(args.peer)
    checks += check_coordinator(args.coordinator)

    print(f"\n  PACK MIND preflight -- role={args.role} dog={args.robot_ip}\n")
    width = max(len(c.name) for c in checks)
    for c in checks:
        print(f"  {_COLOR[c.status]}{c.status:<4}{_RESET} {c.name:<{width}}  {c.detail}")
        if c.status != "PASS" and c.fix:
            print(f"       {_DIM}fix: {c.fix}{_RESET}")

    fails = sum(c.status == "FAIL" for c in checks)
    warns = sum(c.status == "WARN" for c in checks)
    print(f"\n  {fails} FAIL · {warns} WARN · {len(checks) - fails - warns} PASS")
    if fails:
        print(f"  {_RED}NOT READY -- fix the FAILs above before starting dogs.{_RESET}\n")
        return 1
    print(f"  {_GREEN}READY.{_RESET}\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())
