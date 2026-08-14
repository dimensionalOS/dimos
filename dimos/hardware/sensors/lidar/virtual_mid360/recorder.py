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

"""tcpdump-based recorder for raw Livox Mid-360"""

from __future__ import annotations

import asyncio
from datetime import datetime
import getpass
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

from dimos.constants import RECORDINGS_DIR
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


def _stamp() -> str:
    now = datetime.now()
    return now.strftime("%Y-%m-%d_%I-%M%p").lower()


def _default_pcap_path() -> Path:
    return RECORDINGS_DIR / f"mid360_{_stamp()}.pcap"


def _stop_when_parent_dies(cmd: list[str], grace_sec: float) -> list[str]:
    """complicated because of AppArmor label. Must kill with `sudo -n aa-exec -p unconfined`"""
    parent_pid = os.getpid()
    quoted = " ".join(shlex.quote(arg) for arg in cmd)
    # Resolved here so the failure echo can show real paths + the long-term fix.
    aa = shutil.which("aa-exec") or "/usr/sbin/aa-exec"
    kill = shutil.which("kill") or "/usr/bin/kill"
    user = getpass.getuser()
    sudoers = "/etc/sudoers.d/dimos-mid360-pcap-kill"
    # Narrow rule: passwordless for ONLY the unconfined kill, not all sudo.
    rule = f"{user} ALL=(root) NOPASSWD: {aa} -p unconfined -- {kill} *"
    long_term_fix = (
        f"Long-term fix (passwordless for ONLY this kill, not all sudo): "
        f"echo '{rule}' | sudo tee {sudoers} && sudo chmod 440 {sudoers}"
    )

    def _kill(sig: str) -> str:
        return (
            f'kill -{sig} "$child" 2>/dev/null'
            f' || sudo -n {aa} -p unconfined -- {kill} -{sig} "$child" 2>/dev/null'
            f' || echo "[mid360_record] FAILED to {sig} tcpdump pid $child'
            f" (AppArmor blocked it + sudo -n could not escalate) — it is ORPHANED."
            f" Kill it now: sudo {aa} -p unconfined -- {kill} -9 $child."
            f'    {long_term_fix}" >&2'
        )

    # Foreground waits on tcpdump so a startup failure propagates its exit code.
    script = textwrap.dedent(f"""
        {quoted} &
        child=$!
        (
            while kill -0 {parent_pid} 2>/dev/null; do
                sleep 0.5
            done
            {_kill("INT")}
            sleep {grace_sec}
            {_kill("KILL")}
        ) &
        watcher=$!
        wait "$child"
        code=$?
        kill "$watcher" 2>/dev/null
        exit $code
    """).strip()
    return ["bash", "-c", script]


class Mid360PcapRecorderConfig(ModuleConfig):
    pcap_path: Path = Field(default_factory=_default_pcap_path)
    iface: str = Field(default_factory=lambda: os.environ.get("DIMOS_MID360_PCAP_IFACE", ""))
    lidar_ip: str = Field(default_factory=lambda: os.environ.get("DIMOS_MID360_LIDAR_IP", ""))
    snaplen: int = 2048
    stop_timeout: float = 5.0


class Mid360PcapRecorder(Module):
    config: Mid360PcapRecorderConfig

    _TCPDUMP_STARTUP_PROBE_SEC: float = 0.3
    # Declare the capture dead if nothing landed after this long.
    _PCAP_WATCHDOG_SEC: float = 5.0
    _PCAP_GLOBAL_HEADER_BYTES: int = 24
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
        if not cfg.lidar_ip:
            raise ValueError(
                "Mid360PcapRecorder requires lidar_ip — pass lidar_ip=... or set "
                "DIMOS_MID360_LIDAR_IP. It's the real Mid-360's IP, used to filter the capture."
            )
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
            if not self._signal_group(pgid, sig):
                break
            try:
                proc.wait(timeout=timeout)
                break
            except subprocess.TimeoutExpired:
                logger.warning(
                    f"tcpdump did not exit on {sig.name}; escalating  path={self.config.pcap_path}"
                )
        # The bash wrapper can die while a confined tcpdump survives its
        # AppArmor-blocked signal (the unconfined fallback couldn't escalate) —
        # so check tcpdump directly rather than trusting proc.wait().
        if self._tcpdump_pid() is not None:
            self._scream_orphaned()
        else:
            logger.info(f"Mid360PcapRecorder stopped  path={self.config.pcap_path}")

    def _signal_group(self, pgid: int, sig: signal.Signals) -> bool:
        """Signal the tcpdump process group; False if it's already gone.

        tcpdump's AppArmor profile rejects signals from a confined (e.g.
        vscode-labeled) sender with EPERM, so a plain killpg silently fails
        there — fall back to re-issuing from an unconfined label, the same
        escape the `kd` command uses. No-op where AppArmor isn't in the way."""
        try:
            os.killpg(pgid, sig)
            return True
        except ProcessLookupError:
            return False
        except PermissionError:
            pass
        # kill -<signum> -- -<pgid>  (negative pid = the whole group)
        aa = shutil.which("aa-exec")
        if aa is None:
            return True
        cmd = [aa, "-p", "unconfined", "--", "kill", f"-{int(sig)}", "--", f"-{pgid}"]
        if os.geteuid() != 0 and shutil.which("sudo"):
            cmd = ["sudo", "-n", *cmd]
        try:
            subprocess.run(cmd, capture_output=True, timeout=3.0)
        except (OSError, subprocess.TimeoutExpired):
            pass
        return True

    def _tcpdump_pid(self) -> int | None:
        """PID of a tcpdump still writing our pcap, or None — used to detect an
        orphan that survived the stop because its signal was AppArmor-blocked."""
        path = str(Path(self.config.pcap_path).expanduser())
        try:
            result = subprocess.run(
                ["pgrep", "-af", "tcpdump"], capture_output=True, text=True, timeout=2.0
            )
        except (OSError, subprocess.TimeoutExpired):
            return None
        for line in result.stdout.splitlines():
            if path in line:
                try:
                    return int(line.split(None, 1)[0])
                except (ValueError, IndexError):
                    continue
        return None

    def _scream_orphaned(self) -> None:
        """Loudly report a tcpdump that outlived the stop, with the exact fix."""
        pid = self._tcpdump_pid()
        aa = shutil.which("aa-exec") or "/usr/sbin/aa-exec"
        kill = shutil.which("kill") or "/usr/bin/kill"
        user = getpass.getuser()
        # Narrow sudoers rule: passwordless for ONLY the unconfined kill.
        rule = f"{user} ALL=(root) NOPASSWD: {aa} -p unconfined -- {kill} *"
        banner = textwrap.dedent(f"""
            ############################################################################
              !!! kill failed - mid360_record WILL EAT YOUR DISK IF YOU DONT KILL !!!
            ############################################################################
            tcpdump pid={pid} is STILL RUNNING and writing {self.config.pcap_path}.
            AppArmor's tcpdump profile rejected the kill from this (confined) process,
            and the unconfined fallback could not escalate (sudo -n needs a password,
            or aa-exec is missing). It will NOT be reaped on its own.

            Kill it now:
                sudo {aa} -p unconfined -- {kill} -9 {pid}

            To let the recorder kill it itself next time — passwordless for ONLY this
            unconfined kill, not all sudo — install a narrow sudoers rule:
                echo '{rule}' | sudo tee /etc/sudoers.d/dimos-mid360-pcap-kill
                sudo chmod 440 /etc/sudoers.d/dimos-mid360-pcap-kill
            (Verify the paths match `command -v aa-exec` and `command -v kill`.)
            ############################################################################
        """).strip()
        logger.error(banner)
        print(banner, flush=True)
