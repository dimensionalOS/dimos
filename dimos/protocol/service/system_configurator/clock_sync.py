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

import platform
import shutil
import socket
import struct
import time

from dimos.protocol.service.system_configurator.base import SystemConfigurator
from dimos.utils import prompt
from dimos.utils.human import human_duration


class ClockSyncConfigurator(SystemConfigurator):
    """Check that the local clock is within MAX_OFFSET_SECONDS of NTP time.

    Uses a pure-Python NTP query (RFC 4330 SNTPv4) so there are no external
    dependencies.  If the NTP server is unreachable the check *passes* — we
    don't want unrelated network issues to block robot startup.
    """

    critical = False
    MAX_OFFSET_SECONDS = 0.2  # 200 ms per issue spec
    NTP_SERVER = "pool.ntp.org"
    NTP_PORT = 123
    NTP_TIMEOUT = 2  # seconds

    def __init__(self) -> None:
        self._offset: float | None = None  # seconds, filled by check()
        self._fix_cmd: list[str] = []  # resolved by check()

    @staticmethod
    def _ntp_offset(server: str = "pool.ntp.org", port: int = 123, timeout: float = 2) -> float:
        """Return clock offset in seconds (local - NTP).  Raises on failure."""

        # Minimal SNTPv4 request: LI=0, VN=4, Mode=3 → first byte = 0x23
        msg = b"\x23" + b"\x00" * 47
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.settimeout(timeout)
        try:
            t1 = time.time()
            sock.sendto(msg, (server, port))
            data, _ = sock.recvfrom(1024)
            t4 = time.time()
        finally:
            sock.close()

        if len(data) < 48:
            raise ValueError(f"NTP response too short ({len(data)} bytes)")

        # Transmit Timestamp starts at byte 40 (seconds at 40, fraction at 44)
        ntp_secs: int = struct.unpack("!I", data[40:44])[0]
        ntp_frac: int = struct.unpack("!I", data[44:48])[0]
        # NTP epoch is 1900-01-01; Unix epoch is 1970-01-01
        ntp_time: float = ntp_secs - 2208988800 + ntp_frac / (2**32)

        # Simplified offset: assume symmetric delay
        t_server = ntp_time
        rtt = t4 - t1
        offset: float = t_server - (t1 + rtt / 2)
        return offset

    def _resolve_fix_cmd(self) -> list[str]:
        """Determine the best available NTP sync command for this platform."""
        system = platform.system()
        if system == "Darwin":
            return ["sntp", "-sS", self.NTP_SERVER]
        if system == "Linux":
            if shutil.which("ntpdate"):
                return ["ntpdate", self.NTP_SERVER]
            if shutil.which("sntp"):
                return ["sntp", "-sS", self.NTP_SERVER]
            if self._offset is not None:
                new_time = time.time() - self._offset
                return ["date", "-s", f"@{new_time:.3f}"]
        return []

    def check(self) -> bool:
        try:
            self._offset = self._ntp_offset(self.NTP_SERVER, self.NTP_PORT, self.NTP_TIMEOUT)
        except (TimeoutError, OSError, ValueError) as exc:
            print(f"[clock-sync] NTP query failed ({exc}); assuming clock is OK")
            self._offset = None
            return True  # graceful degradation — don't block on network issues

        if abs(self._offset) <= self.MAX_OFFSET_SECONDS:
            return True

        self._fix_cmd = self._resolve_fix_cmd()
        return False

    def explanation(self) -> str | None:
        if self._offset is None:
            return None
        if self._fix_cmd:
            cmd = f"sudo {' '.join(self._fix_cmd)}"
        else:
            cmd = "(no NTP tool found — install ntpdate or sntp, then re-run)"
        hint = ""
        if platform.system() == "Linux":
            hint = "\n  (Alternatively, you can install systemd-timesyncd.service)"
        return f"- clock sync: local clock is off by {human_duration(self._offset)}: {cmd}{hint}"

    def fix(self) -> None:
        if not self._fix_cmd:
            print(f"[clock-sync] No automatic fix available on {platform.system()}")
            return
        cmd = list(self._fix_cmd)
        # Recompute the corrected time at fix-time (not stale from check-time)
        if cmd[:2] == ["date", "-s"] and self._offset is not None:
            cmd[2] = f"@{time.time() - self._offset:.3f}"
        prompt.sudo_run(*cmd, check=True, text=True, capture_output=True)
