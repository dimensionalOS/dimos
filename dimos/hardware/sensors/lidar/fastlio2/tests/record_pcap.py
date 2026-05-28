#!/usr/bin/env python3
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

"""Record raw UDP traffic from a Livox Mid-360 to a pcap file.

Captures the wire bytes before they reach the Livox SDK, so a downstream
decoder can verify hardware behavior independent of any SDK/fastlio code.

Default filter:
    src host <lidar_ip> and udp and dst portrange 56101-56501

That covers all five Mid-360 host-side ports (cmd 56101, push 56201,
point 56301, imu 56401, log 56501) per MID360_config.json.

Usage:
    uv run python -m dimos.hardware.sensors.lidar.fastlio2.tests.record_pcap
    uv run python -m dimos.hardware.sensors.lidar.fastlio2.tests.record_pcap --duration 30

Requires `tcpdump` (Linux). On Jeff's setup `florp` provides passwordless
sudo; falls back to `sudo` elsewhere.
"""

from __future__ import annotations

import argparse
from datetime import datetime
import os
from pathlib import Path
import select
import shutil
import signal
import subprocess
import sys
import time

DEFAULT_IFACE = "enp2s0"
DEFAULT_LIDAR_IP = "192.168.1.157"
DEFAULT_PORT_RANGE = "56101-56501"
DEFAULT_OUTPUT_DIR = Path("tests/data/mid360_pcap")


def build_filter(lidar_ip: str, port_range: str) -> str:
    return f"src host {lidar_ip} and udp and dst portrange {port_range}"


def start_tcpdump(
    iface: str,
    lidar_ip: str,
    port_range: str,
    output_path: Path,
    snaplen: int,
) -> subprocess.Popen[bytes]:
    bpf = build_filter(lidar_ip, port_range)
    tcpdump = shutil.which("tcpdump") or "tcpdump"
    args = [
        tcpdump,
        "-i",
        iface,
        "-w",
        str(output_path),
        "-s",
        str(snaplen),
        "-U",
        "-n",
        bpf,
    ]
    print(f"[record_pcap] launching: {' '.join(args)}", flush=True)
    return subprocess.Popen(
        args,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.PIPE,
        start_new_session=True,
    )


def stop_tcpdump(proc: subprocess.Popen[bytes], timeout: float = 3.0) -> None:
    if proc.poll() is not None:
        return
    proc.send_signal(signal.SIGINT)
    try:
        proc.wait(timeout=timeout)
        return
    except subprocess.TimeoutExpired:
        pass
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
    except ProcessLookupError:
        pass
    try:
        proc.wait(timeout=timeout)
    except subprocess.TimeoutExpired:
        proc.kill()
        proc.wait()


def wait_for_enter() -> None:
    while True:
        ready, _, _ = select.select([sys.stdin], [], [], 0.2)
        if ready:
            sys.stdin.readline()
            return


def pcap_size(path: Path) -> int:
    if not path.exists():
        return 0
    return path.stat().st_size


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output",
        "-o",
        default=None,
        help=(
            f"Output pcap path. If omitted, writes to {DEFAULT_OUTPUT_DIR}/mid360_<timestamp>.pcap"
        ),
    )
    parser.add_argument(
        "--iface",
        default=DEFAULT_IFACE,
        help=f"Network interface to capture on (default: {DEFAULT_IFACE})",
    )
    parser.add_argument(
        "--lidar-ip",
        default=DEFAULT_LIDAR_IP,
        help=f"Mid-360 source IP to filter on (default: {DEFAULT_LIDAR_IP})",
    )
    parser.add_argument(
        "--port-range",
        default=DEFAULT_PORT_RANGE,
        help=(
            "Host-side destination port range (default: "
            f"{DEFAULT_PORT_RANGE}, covers cmd/push/point/imu/log)"
        ),
    )
    parser.add_argument(
        "--snaplen",
        type=int,
        default=2048,
        help=(
            "Per-packet capture length in bytes. Mid-360 point packets "
            "are <1500; 2048 is safe (default: %(default)s)"
        ),
    )
    parser.add_argument(
        "--duration",
        "-d",
        type=float,
        default=None,
        help=(
            "Recording length in seconds. If omitted, interactively prompts to start and to stop."
        ),
    )
    parser.add_argument(
        "--warmup",
        type=float,
        default=1.5,
        help=(
            "Seconds tcpdump runs before the START prompt (lets BPF attach "
            "and the link settle). Default: %(default)s"
        ),
    )
    args = parser.parse_args()

    if args.output is None:
        DEFAULT_OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        output_path = DEFAULT_OUTPUT_DIR / f"mid360_{stamp}.pcap"
    else:
        output_path = Path(args.output)
        output_path.parent.mkdir(parents=True, exist_ok=True)

    print(f"[record_pcap] capturing to {output_path}", flush=True)
    print(f"[record_pcap] filter: {build_filter(args.lidar_ip, args.port_range)}", flush=True)

    proc = start_tcpdump(
        args.iface,
        args.lidar_ip,
        args.port_range,
        output_path,
        args.snaplen,
    )

    try:
        time.sleep(args.warmup)
        if proc.poll() is not None:
            stderr_bytes = proc.stderr.read() if proc.stderr else b""
            stderr = stderr_bytes.decode(errors="replace")
            print(
                f"[record_pcap] tcpdump exited during warmup (rc={proc.returncode})",
                flush=True,
            )
            if stderr:
                sys.stderr.write(stderr)
            tcpdump_path = shutil.which("tcpdump") or "tcpdump"
            print(
                "\n[record_pcap] tcpdump can't capture. Grant capability once with:\n"
                f"  sudo setcap cap_net_raw,cap_net_admin=eip {tcpdump_path}\n"
                "then re-run.",
                flush=True,
            )
            return 1

        size_at_start = pcap_size(output_path)

        if args.duration is None:
            print(
                "[record_pcap] READY.  Press Enter to mark START (capture is already running).",
                flush=True,
            )
            wait_for_enter()
            mark_start = time.time()
            print(
                "[record_pcap] RECORDING.  Press Enter (or Ctrl+C) to STOP.",
                flush=True,
            )
            try:
                wait_for_enter()
            except KeyboardInterrupt:
                print("\n[record_pcap] caught Ctrl+C, stopping…", flush=True)
            duration = time.time() - mark_start
        else:
            mark_start = time.time()
            print(f"[record_pcap] RECORDING for {args.duration:.1f}s …", flush=True)
            time.sleep(args.duration)
            duration = args.duration
    finally:
        stop_tcpdump(proc)

    size_at_end = pcap_size(output_path)
    delta = max(0, size_at_end - size_at_start)
    rate_mbps = (delta * 8) / max(duration, 1e-3) / 1e6

    print(f"\n[record_pcap] capture stopped after {duration:.2f}s", flush=True)
    print(
        f"[record_pcap] pcap size: {size_at_end / 1e6:.2f} MB "
        f"(+{delta / 1e6:.2f} MB during marked window, ~{rate_mbps:.1f} Mbps)",
        flush=True,
    )
    print(f"[record_pcap] wrote {output_path}", flush=True)

    if delta == 0:
        print(
            "[record_pcap] WARNING: no bytes captured during the marked window. "
            "Check iface, lidar IP, and that the lidar is publishing.",
            flush=True,
        )
        return 1
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        sys.exit(130)
