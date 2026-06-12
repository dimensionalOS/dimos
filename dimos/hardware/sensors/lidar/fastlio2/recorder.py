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
from dimos.core.stream import In
from dimos.memory2.module import Recorder, RecorderConfig
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


def _stamp() -> str:
    now = datetime.now()
    return now.strftime("%Y-%m-%d") + "_" + now.strftime("%I-%M%p").lower() + "-PST"


def _stop_when_parent_dies(cmd: list[str], grace_sec: float) -> list[str]:
    """Reap cmd if the recorder dies, including via SIGKILL (which it can't
    intercept) — otherwise tcpdump's own session would outlive it."""
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


def _default_recording_dir() -> Path:
    return Path("recordings") / _stamp()


class FastLio2RecorderConfig(RecorderConfig):
    """One recording dir per session: <dir>/mem2.db plus <dir>/raw_mid360.pcap."""

    recording_dir: Path = Field(default_factory=_default_recording_dir)
    # Filled in by model_post_init below if left at the default.
    db_path: str | Path = ""

    default_frame_id: str = "base_link"

    # tcpdump configuration. Pcap recording is opt-in: set record_pcap=True to
    # enable. pcap_path defaults to <recording_dir>/mid360.pcap when unset.
    record_pcap: bool = False
    pcap_path: Path | None = None
    record_pcap_iface: str = "enp2s0"
    record_pcap_snaplen: int = 2048
    lidar_ip: str = "192.168.1.107"
    # Grace period for each stop signal (SIGINT→SIGTERM→SIGKILL) when tearing
    # down the tcpdump capture.
    pcap_stop_timeout: float = 5.0

    def model_post_init(self, __context: object) -> None:
        super().model_post_init(__context)
        if not self.db_path:
            self.db_path = self.recording_dir / "mem2.db"
        if self.record_pcap and self.pcap_path is None:
            self.pcap_path = self.recording_dir / "raw_mid360.pcap"


class FastLio2Recorder(Recorder):
    """Records FastLio2 inputs and outputs for offline replay: raw Livox
    Mid-360 lidar + IMU into the SDK, FastLio2's registered lidar and
    odometry out, plus any companion streams (e.g. Go2 camera/leg odom)
    that help interpret the run.

    Also owns the tcpdump process that captures the raw Mid-360 UDP
    packets — this is the ground-truth input the FastLio2 binary can be
    replayed against bit-for-bit. Single session = single timestamped dir
    holding both the sqlite memory store and the pcap.
    """

    config: FastLio2RecorderConfig

    lidar: In[PointCloud2]
    odometry: In[Odometry]

    # tcpdump fails fast (EPERM, bad iface) within a few ms; pause briefly so poll() catches that.
    _TCPDUMP_STARTUP_PROBE_SEC: float = 0.3
    # How long to let tcpdump run before declaring the capture dead if nothing landed.
    _PCAP_WATCHDOG_SEC: float = 5.0
    # A libpcap file with zero packets is just its 24-byte global header.
    _PCAP_GLOBAL_HEADER_BYTES: int = 24
    # How long the diagnostic sniff listens for *any* UDP source on the iface.
    _PCAP_DIAGNOSTIC_SNIFF_SEC: float = 3.0

    _pcap_proc: subprocess.Popen[bytes] | None = None

    @rpc
    def start(self) -> None:
        Path(self.config.recording_dir).mkdir(parents=True, exist_ok=True)
        if self.config.record_pcap:
            self._start_pcap()
        super().start()
        if self.config.record_pcap and self._pcap_proc is not None:
            self.spawn(self._pcap_watchdog())

    @rpc
    def stop(self) -> None:
        try:
            super().stop()
        finally:
            self._stop_pcap()

    def _start_pcap(self) -> None:
        cfg = self.config
        path = Path(cfg.pcap_path).expanduser()
        path.parent.mkdir(parents=True, exist_ok=True)

        # Capture every UDP packet originating from the lidar.
        packet_filter_expression = f"src host {cfg.lidar_ip} and udp"
        tcpdump = shutil.which("tcpdump") or "tcpdump"
        cmd = [
            tcpdump,
            "-i",
            cfg.record_pcap_iface,
            "-w",
            str(path),
            "-s",
            str(cfg.record_pcap_snaplen),
            "-U",
            "-n",
            packet_filter_expression,
        ]

        # Own session/group so _stop_pcap can signal the wrapper + tcpdump
        # without touching the recorder, and Ctrl-C doesn't race shutdown.
        proc = subprocess.Popen(
            _stop_when_parent_dies(cmd, cfg.pcap_stop_timeout),
            stdout=subprocess.DEVNULL,
            stderr=subprocess.PIPE,
            start_new_session=True,
        )
        # tcpdump exits within a few ms on EPERM; wait briefly so we can detect that.
        time.sleep(self._TCPDUMP_STARTUP_PROBE_SEC)
        if proc.poll() is not None:
            stderr = proc.stderr.read().decode(errors="replace") if proc.stderr else ""
            self._pcap_proc = None
            logger.error(
                f"FastLio2Recorder pcap recording failed to start — tcpdump exited"
                f" rc={proc.returncode} stderr={stderr.strip()}"
            )
            print(
                "[go2_record] pcap recording is enabled but tcpdump cannot capture.\n"
                "          Grant capture capability once with:\n"
                f"            sudo setcap cap_net_raw,cap_net_admin=eip {tcpdump}\n"
                "          then restart. (tcpdump stderr above.)",
                flush=True,
            )
            return

        logger.info(
            f"FastLio2Recorder pcap recording enabled  path={path}  "
            f"iface={cfg.record_pcap_iface}  filter={packet_filter_expression!r}"
        )
        self._pcap_proc = proc

    async def _pcap_watchdog(self) -> None:
        """If tcpdump captured nothing after a few seconds, dump everything we
        know about why — almost always a wrong lidar_ip or interface."""
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
                f"FastLio2Recorder pcap healthy — {size} bytes captured in "
                f"{self._PCAP_WATCHDOG_SEC:.0f}s  path={path}"
            )
            return
        report = await asyncio.to_thread(self._build_empty_pcap_report, size, proc)
        logger.error(report)
        print(report, flush=True)

    def _build_empty_pcap_report(self, size: int, proc: subprocess.Popen[bytes]) -> str:
        cfg = self.config
        packet_filter_expression = f"src host {cfg.lidar_ip} and udp"
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
                f"          UDP traffic IS flowing on {cfg.record_pcap_iface}, but from other source(s):\n"
                f"{listing}\n"
                f"          None matched 'src host {cfg.lidar_ip}'. The lidar_ip is almost certainly\n"
                f"          wrong — set LIDAR_IP to whichever address above is the lidar and restart."
            )
        else:
            diagnosis = (
                f"          NO UDP traffic at all was seen on {cfg.record_pcap_iface} during a "
                f"{self._PCAP_DIAGNOSTIC_SNIFF_SEC:.0f}s probe.\n"
                f"          Wrong interface, unplugged cable, or the lidar is powered off."
            )

        neigh = self._run_quiet(["ip", "neigh", "show", cfg.lidar_ip]).strip()
        return textwrap.dedent(f"""
            ============================================================================
            [go2_record] PCAP WATCHDOG: 0 packets captured after {self._PCAP_WATCHDOG_SEC:.0f}s
            ============================================================================
            Recording is enabled but tcpdump wrote an EMPTY pcap (size={size} bytes; an
            empty libpcap file is {self._PCAP_GLOBAL_HEADER_BYTES} bytes of global header).

            Capture config:
              interface : {cfg.record_pcap_iface}
              lidar_ip  : {cfg.lidar_ip}
              filter    : {packet_filter_expression!r}
              pcap_path : {cfg.pcap_path}
              tcpdump   : alive={proc_alive} pid={proc.pid}{f" stderr={stderr_text!r}" if stderr_text else ""}

            Diagnosis:
            {diagnosis}

              arp/neigh for {cfg.lidar_ip}: {neigh or "<no entry>"}
            ============================================================================
        """).strip()

    def _observed_udp_sources(self) -> dict[str, int]:
        """Sniff the interface briefly and tally which source IPs are sending UDP."""
        tcpdump = shutil.which("tcpdump") or "tcpdump"
        cmd = [tcpdump, "-i", self.config.record_pcap_iface, "-nn", "-c", "60", "udp"]
        try:
            result = subprocess.run(
                cmd,
                capture_output=True,
                text=True,
                timeout=self._PCAP_DIAGNOSTIC_SNIFF_SEC,
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
                source = match.group(1)
                counts[source] = counts.get(source, 0) + 1
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
        # Signal the group so tcpdump gets it directly. SIGINT is its
        # clean-stop signal (flushes the pcap); escalate if it hangs.
        try:
            pgid = os.getpgid(proc.pid)
        except ProcessLookupError:
            return
        timeout = self.config.pcap_stop_timeout
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
        logger.info(f"FastLio2Recorder pcap recording stopped  path={self.config.pcap_path}")
