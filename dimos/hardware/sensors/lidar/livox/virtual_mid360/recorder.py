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

"""tcpdump-based recorder for raw Livox Mid-360 UDP — the input for VirtualMid360.

Captures only the Mid-360's UDP traffic (point/IMU/status) off the wire into a
pcap, so it can be replayed later with the VirtualMid360 module. It records
nothing else — no dimos streams, no db, just the raw Livox packets. Needs
CAP_NET_RAW (grant tcpdump once: `sudo setcap cap_net_raw,cap_net_admin=eip
$(which tcpdump)`).
"""

from __future__ import annotations

import asyncio
from datetime import datetime
import os
from pathlib import Path
import re
import shlex
import shutil
import signal
import subprocess
import textwrap
import time

from pydantic import Field

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


def _stamp() -> str:
    now = datetime.now()
    return now.strftime("%Y-%m-%d_%I-%M%p").lower()


def _default_pcap_path() -> Path:
    return Path("recordings") / f"mid360_{_stamp()}.pcap"


def _stop_when_parent_dies(cmd: list[str], grace_sec: float) -> list[str]:
    """Wrap cmd so it's reaped if this recorder dies — including via SIGKILL,
    which it can't intercept — otherwise tcpdump would outlive its session."""
    parent_pid = os.getpid()
    quoted = " ".join(shlex.quote(arg) for arg in cmd)
    # Foreground waits on tcpdump so a startup failure propagates its exit code.
    script = textwrap.dedent(f"""
        {quoted} &
        child=$!
        (
            while kill -0 {parent_pid} 2>/dev/null; do
                sleep 0.5
            done
            kill -INT "$child" 2>/dev/null
            sleep {grace_sec}
            kill -KILL "$child" 2>/dev/null
        ) &
        watcher=$!
        wait "$child"
        code=$?
        kill "$watcher" 2>/dev/null
        exit $code
    """).strip()
    return ["bash", "-c", script]


class Mid360PcapRecorderConfig(ModuleConfig):
    """Where/how to capture; the pcap defaults to recordings/mid360_<stamp>.pcap."""

    pcap_path: Path = Field(default_factory=_default_pcap_path)
    # Capture interface. Machine-specific, so it defaults from DIMOS_PCAP_IFACE
    # (falling back to enp2s0) rather than hardcoding a host-only value.
    iface: str = Field(default_factory=lambda: os.environ.get("DIMOS_PCAP_IFACE", "enp2s0"))
    # The Mid-360's IP — network-specific, so required (no default). Only its
    # UDP is captured (`src host <lidar_ip> and udp`).
    lidar_ip: str
    snaplen: int = 2048
    # Grace period per stop signal (SIGINT -> SIGTERM -> SIGKILL) at teardown.
    stop_timeout: float = 5.0


class Mid360PcapRecorder(Module):
    """Records the raw Livox Mid-360 UDP stream to a pcap via tcpdump.

    Owns nothing but the tcpdump process: on start() it captures every UDP
    packet from the lidar into pcap_path; on stop() it flushes + tears it down.
    The result replays bit-for-bit through VirtualMid360.
    """

    config: Mid360PcapRecorderConfig

    # tcpdump fails fast (EPERM, bad iface) within a few ms; pause so poll() catches it.
    _TCPDUMP_STARTUP_PROBE_SEC: float = 0.3
    # Declare the capture dead if nothing landed after this long.
    _PCAP_WATCHDOG_SEC: float = 5.0
    # A libpcap file with zero packets is just its 24-byte global header.
    _PCAP_GLOBAL_HEADER_BYTES: int = 24
    # How long the diagnostic sniff listens for *any* UDP source on the iface.
    _PCAP_DIAGNOSTIC_SNIFF_SEC: float = 3.0

    _pcap_proc: subprocess.Popen[bytes] | None = None

    @rpc
    def start(self) -> None:
        self._start_pcap()
        super().start()
        if self._pcap_proc is not None:
            self.spawn(self._pcap_watchdog())

    @rpc
    def stop(self) -> None:
        try:
            super().stop()
        finally:
            self._stop_pcap()

    def _filter(self) -> str:
        return f"src host {self.config.lidar_ip} and udp"

    def _start_pcap(self) -> None:
        cfg = self.config
        path = Path(cfg.pcap_path).expanduser()
        path.parent.mkdir(parents=True, exist_ok=True)

        tcpdump = shutil.which("tcpdump") or "tcpdump"
        cmd = [
            tcpdump,
            "-i",
            cfg.iface,
            "-w",
            str(path),
            "-s",
            str(cfg.snaplen),
            "-U",  # packet-buffered: flush each packet so a kill loses nothing
            "-n",
            self._filter(),
        ]
        # Own session so _stop_pcap can signal the wrapper + tcpdump without
        # touching the recorder, and Ctrl-C doesn't race shutdown.
        proc = subprocess.Popen(
            _stop_when_parent_dies(cmd, cfg.stop_timeout),
            stdout=subprocess.DEVNULL,
            stderr=subprocess.PIPE,
            start_new_session=True,
        )
        time.sleep(self._TCPDUMP_STARTUP_PROBE_SEC)
        if proc.poll() is not None:
            stderr = proc.stderr.read().decode(errors="replace") if proc.stderr else ""
            self._pcap_proc = None
            logger.error(
                f"Mid360PcapRecorder: tcpdump exited rc={proc.returncode} stderr={stderr.strip()}"
            )
            print(
                "[mid360_record] tcpdump cannot capture. Grant capture capability once with:\n"
                f"            sudo setcap cap_net_raw,cap_net_admin=eip {tcpdump}\n"
                "          then restart. (tcpdump stderr above.)",
                flush=True,
            )
            return

        logger.info(
            f"Mid360PcapRecorder capturing  path={path}  iface={cfg.iface}  "
            f"filter={self._filter()!r}"
        )
        self._pcap_proc = proc

    async def _pcap_watchdog(self) -> None:
        """If tcpdump captured nothing after a few seconds, report why — almost
        always a wrong lidar_ip or interface."""
        await asyncio.sleep(self._PCAP_WATCHDOG_SEC)
        proc = self._pcap_proc
        if proc is None:
            return
        path = Path(self.config.pcap_path).expanduser()
        try:
            size = path.stat().st_size
        except OSError:
            size = -1
        if size > self._PCAP_GLOBAL_HEADER_BYTES:
            logger.info(
                f"Mid360PcapRecorder healthy — {size} bytes captured in "
                f"{self._PCAP_WATCHDOG_SEC:.0f}s  path={path}"
            )
            return
        report = await asyncio.to_thread(self._build_empty_pcap_report, size, proc)
        logger.error(report)
        print(report, flush=True)

    def _build_empty_pcap_report(self, size: int, proc: subprocess.Popen[bytes]) -> str:
        cfg = self.config
        proc_alive = proc.poll() is None
        stderr_text = ""
        if not proc_alive and proc.stderr is not None:
            try:
                stderr_text = proc.stderr.read().decode(errors="replace").strip()
            except (OSError, ValueError):
                stderr_text = "<unreadable>"

        observed = self._observed_udp_sources()
        if observed:
            listing = "\n".join(
                f"            {source}  ({count} pkts)"
                for source, count in sorted(observed.items(), key=lambda kv: kv[1], reverse=True)
            )
            diagnosis = (
                f"          UDP traffic IS flowing on {cfg.iface}, but from other source(s):\n"
                f"{listing}\n"
                f"          None matched 'src host {cfg.lidar_ip}'. The lidar_ip is almost\n"
                f"          certainly wrong — set it to whichever address above is the lidar."
            )
        else:
            diagnosis = (
                f"          NO UDP traffic at all was seen on {cfg.iface} during a "
                f"{self._PCAP_DIAGNOSTIC_SNIFF_SEC:.0f}s probe.\n"
                f"          Wrong interface, unplugged cable, or the lidar is powered off."
            )

        neigh = self._run_quiet(["ip", "neigh", "show", cfg.lidar_ip]).strip()
        return textwrap.dedent(f"""
            ============================================================================
            [mid360_record] PCAP WATCHDOG: 0 packets captured after {self._PCAP_WATCHDOG_SEC:.0f}s
            ============================================================================
            tcpdump wrote an EMPTY pcap (size={size} bytes; an empty libpcap file is
            {self._PCAP_GLOBAL_HEADER_BYTES} bytes of global header).

            Capture config:
              interface : {cfg.iface}
              lidar_ip  : {cfg.lidar_ip}
              filter    : {self._filter()!r}
              pcap_path : {cfg.pcap_path}
              tcpdump   : alive={proc_alive} pid={proc.pid}{f" stderr={stderr_text!r}" if stderr_text else ""}

            Diagnosis:
            {diagnosis}

              arp/neigh for {cfg.lidar_ip}: {neigh or "<no entry>"}
            ============================================================================
        """).strip()

    def _observed_udp_sources(self) -> dict[str, int]:
        """Sniff the interface briefly and tally which source IPs send UDP."""
        tcpdump = shutil.which("tcpdump") or "tcpdump"
        cmd = [tcpdump, "-i", self.config.iface, "-nn", "-c", "60", "udp"]
        try:
            result = subprocess.run(
                cmd, capture_output=True, text=True, timeout=self._PCAP_DIAGNOSTIC_SNIFF_SEC
            )
            output = result.stdout
        except subprocess.TimeoutExpired as expired:
            stdout = expired.stdout
            output = (
                stdout.decode(errors="replace") if isinstance(stdout, bytes) else (stdout or "")
            )
        except OSError:
            return {}
        counts: dict[str, int] = {}
        for line in output.splitlines():
            match = re.search(r"\bIP6?\s+(\S+?)\.\d+\s+>", line)
            if match:
                counts[match.group(1)] = counts.get(match.group(1), 0) + 1
        return counts

    @staticmethod
    def _run_quiet(cmd: list[str]) -> str:
        try:
            return subprocess.run(cmd, capture_output=True, text=True, timeout=2.0).stdout
        except (OSError, subprocess.TimeoutExpired):
            return ""

    def _stop_pcap(self) -> None:
        proc = self._pcap_proc
        if proc is None:
            return
        self._pcap_proc = None
        if proc.poll() is not None:
            return
        # Signal the group so tcpdump gets it directly. SIGINT is its clean-stop
        # signal (flushes the pcap); escalate if it hangs.
        try:
            pgid = os.getpgid(proc.pid)
        except ProcessLookupError:
            return
        timeout = self.config.stop_timeout
        for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGKILL):
            try:
                os.killpg(pgid, sig)
            except ProcessLookupError:
                break
            try:
                proc.wait(timeout=timeout)
                break
            except subprocess.TimeoutExpired:
                logger.warning(
                    f"tcpdump did not exit on {sig.name}; escalating  path={self.config.pcap_path}"
                )
        else:
            proc.wait()
        logger.info(f"Mid360PcapRecorder stopped  path={self.config.pcap_path}")
