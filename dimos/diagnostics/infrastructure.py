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

"""Read-only Linux AP collector over an explicitly configured SSH management path."""

from __future__ import annotations

import json
import re
import subprocess
import time
from typing import Any

from dimos.hardware.sensors.wifi.radio import linux_radio

# No shell interpolation of credentials, interfaces or output. Requires an existing
# SSH identity/agent and python3 + iw on the selected AP; no installation or writes.
_SCRIPT = """
import json,subprocess,time,socket
from pathlib import Path
interface = INTERFACE
result={'received_ns':time.monotonic_ns(),'host':socket.gethostname(),'probes':[]}
result['clock_domain']=result['host']+'/'+Path('/proc/sys/kernel/random/boot_id').read_text().strip()+'/monotonic'
for command in [['iw','dev',interface,'info'],['iw','dev',interface,'station','dump'],['iw','dev',interface,'survey','dump'],['tc','-s','qdisc','show','dev',interface]]:
 try:
  p=subprocess.run(command,capture_output=True,text=True,timeout=2)
  result['probes'].append({'command':command,'returncode':p.returncode,'stdout':p.stdout[:65536],'stderr':p.stderr[:1024]})
 except (OSError,subprocess.TimeoutExpired) as e:
  result['probes'].append({'command':command,'status':'unavailable','reason':type(e).__name__})
result['sent_ns']=time.monotonic_ns()
print(json.dumps(result))
"""


def ap_snapshot(host: str, interface: str) -> dict[str, Any]:
    """Measure AP client counters without treating the SSH management RTT as airtime."""
    if not re.fullmatch(r"[A-Za-z0-9_.@:-]+", host) or host.startswith("-"):
        raise ValueError("invalid AP SSH host")
    if not re.fullmatch(r"[A-Za-z0-9_.:-]{1,64}", interface):
        raise ValueError("invalid AP interface")
    began = time.monotonic_ns()
    try:
        result = subprocess.run(
            ["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=3", host, "python3", "-"],
            input=_SCRIPT.replace("INTERFACE", repr(interface)),
            capture_output=True,
            text=True,
            timeout=12,
            check=False,
        )
        ended = time.monotonic_ns()
        if result.returncode:
            return {"status": "unavailable", "source": host, "reason": result.stderr[-512:]}
        if len(result.stdout) > 1024 * 1024:
            raise ValueError("AP result exceeds bound")
        return normalize_ap(json.loads(result.stdout), host, interface, began, ended)
    except (OSError, subprocess.TimeoutExpired, ValueError, KeyError) as error:
        return {"status": "unavailable", "source": host, "reason": type(error).__name__}


def normalize_ap(
    data: dict[str, Any], host: str, interface: str, began: int, ended: int
) -> dict[str, Any]:
    """Preserve AP-side direction and provenance: AP TX is client downlink."""
    probes = data["probes"]
    info = next((p.get("stdout", "") for p in probes if p.get("command", [])[-1:] == ["info"]), "")
    identity = re.search(r"\baddr\s+([0-9a-fA-F:]{17})", info)
    bssid = identity[1].lower() if identity else None
    station = next(
        (p.get("stdout", "") for p in probes if p.get("command", [])[-2:] == ["station", "dump"]),
        "",
    )
    links = []
    for block in re.split(r"(?=^Station )", station, flags=re.M):
        client = re.match(r"Station ([0-9a-fA-F:]{17})", block)
        if not client:
            continue
        isolated = [p for p in probes if p.get("command", [])[-2:] != ["station", "dump"]]
        isolated.append(
            {
                "command": ["iw", "dev", interface, "station", "dump"],
                "returncode": 0,
                "stdout": block,
            }
        )
        radio = linux_radio(isolated, interface)
        radio.pop("bssid", None)
        links.append(
            {
                "source": f"ap/{bssid or host}/{interface}",
                "target": f"station/{client[1].lower()}",
                "association": bssid,
                "client_mac": client[1].lower(),
                "observed_ns": data["sent_ns"],
                "evidence": "AP-side nl80211 station counters over authenticated SSH; AP TX is client downlink",
                "radio": radio,
            }
        )
    return {
        "schema": "dimos.infrastructure/v1",
        "status": "ok" if bssid and links else "partial",
        "source": host,
        "clock_domain": data["clock_domain"],
        "exported_ns": data["sent_ns"],
        "links": links,
        "management_probe": {
            "local_send_ns": began,
            "remote_receive_ns": data["received_ns"],
            "remote_send_ns": data["sent_ns"],
            "local_receive_ns": ended,
            "scope": "SSH management exchange, not radio airtime",
        },
        "coverage": {
            "client_counters": bool(links),
            "ap_identity": bssid is not None,
            "per_packet_queue_delay": False,
        },
    }
