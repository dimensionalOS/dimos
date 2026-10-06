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

from __future__ import annotations

from collections.abc import Callable, Sequence
from dataclasses import asdict
from functools import partial
import json
from pathlib import PurePosixPath
import queue
import re
import shlex
import subprocess
import threading
import time
from typing import Any
import uuid

import zenoh

from dimos.cli.network.model import PROTOCOL_VERSION, RTT, Report, Settings, meets_thresholds
from dimos.cli.network.session import Endpoint, start_sender, zenoh_version


class PeerError(RuntimeError):
    """The SSH responder disconnected or emitted an invalid protocol response."""


class SessionCapError(TimeoutError):
    """The duration budget cannot fit another complete observation window."""


def ssh_command(remote: str, executable: str) -> list[str]:
    if (
        not remote
        or re.fullmatch(r"[A-Za-z0-9_.:@\[\]-]+", remote) is None
        or remote.startswith("-")
    ):
        raise ValueError("remote must be an SSH host alias or user@host, without shell syntax")
    if not PurePosixPath(executable).is_absolute() or any(c in executable for c in "\n\r\0"):
        raise ValueError("remote-dimos must be an absolute executable path")
    return [
        "ssh",
        "-T",
        "-o",
        "BatchMode=yes",
        "-o",
        "ConnectTimeout=10",
        "-o",
        "ServerAliveInterval=5",
        "-o",
        "ServerAliveCountMax=2",
        "--",
        remote,
        f"exec {shlex.quote(executable)} network peer --stdio",
    ]


class RemotePeer:
    def __init__(self, command: Sequence[str]) -> None:
        self._process = subprocess.Popen(
            command,
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            bufsize=1,
        )
        self._messages: queue.Queue[dict[str, Any] | Exception] = queue.Queue()
        self._stderr: list[str] = []
        self._stderr_lock = threading.Lock()
        self._threads = [
            threading.Thread(target=self._read, daemon=True),
            threading.Thread(target=self._read_errors, daemon=True),
        ]
        for thread in self._threads:
            thread.start()

    def _read(self) -> None:
        assert self._process.stdout is not None
        try:
            for line in self._process.stdout:
                response = json.loads(line)
                if not isinstance(response, dict):
                    raise ValueError("responder protocol requires JSON objects")
                self._messages.put(response)
        except (ValueError, OSError) as error:
            self._messages.put(error)
        finally:
            self._messages.put(EOFError("SSH responder exited or disconnected"))

    def _read_errors(self) -> None:
        assert self._process.stderr is not None
        for line in self._process.stderr:
            with self._stderr_lock:
                if len(self._stderr) < 20:
                    self._stderr.append(line[:500])

    def send(self, request: dict[str, Any]) -> None:
        assert self._process.stdin is not None
        self._process.stdin.write(json.dumps(request, allow_nan=False) + "\n")
        self._process.stdin.flush()

    def read(self, timeout: float = 10) -> dict[str, Any]:
        try:
            result = self._messages.get(timeout=timeout)
        except queue.Empty as error:
            raise TimeoutError("SSH responder did not reply before deadline") from error
        if not isinstance(result, dict):
            with self._stderr_lock:
                diagnostic = "".join(self._stderr).strip()
            raise PeerError(f"{result}; {diagnostic}") from result
        return result

    def request(self, message: dict[str, Any], timeout: float = 10) -> dict[str, Any]:
        self.send(message)
        return self.read(timeout)

    def close(self) -> str:
        confirmed = False
        try:
            if self._process.poll() is None:
                self.send({"op": "stop"})
            assert self._process.stdin is not None
            self._process.stdin.close()
            until = time.monotonic() + 3
            while time.monotonic() < until:
                try:
                    response = self.read(max(0.01, until - time.monotonic()))
                except (TimeoutError, RuntimeError):
                    break
                if response.get("op") == "bye":
                    confirmed = True
                    break
            self._process.wait(timeout=3)
        except (OSError, subprocess.TimeoutExpired):
            self._process.terminate()
            try:
                self._process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                self._process.kill()
                self._process.wait()
        finally:
            for thread in self._threads:
                thread.join(timeout=1)
            for stream in (self._process.stdout, self._process.stderr):
                if stream is not None:
                    stream.close()
        return "confirmed" if confirmed else "unconfirmed; remote EOF/lease bounds still apply"


def run_check(
    remote: str,
    remote_dimos: str,
    settings: Settings,
    *,
    peer_host: str | None = None,
    listen_host: str = "0.0.0.0",
    port: int = 0,
    progress: Callable[[str, Report], None] | None = None,
    peer_command: Sequence[str] | None = None,
) -> Report:
    """Run SSH orchestration; peer_command is an injectable local integration boundary."""
    settings.validate()
    command = ssh_command(remote, remote_dimos) if peer_command is None else peer_command
    report = Report(uuid.uuid4().hex, remote, remote_dimos, settings, zenoh_version())
    stop = threading.Event()
    lease = threading.Timer(settings.max_seconds, stop.set)
    deadline = time.monotonic() + settings.max_seconds
    peer: RemotePeer | None = None
    endpoint: Endpoint | None = None
    phase_label = "Idle RTT"
    lease.start()

    def notify(message: str) -> None:
        if progress is not None:
            progress(message, report)

    def remaining() -> float:
        value = deadline - time.monotonic()
        if value <= 0 or stop.is_set():
            raise SessionCapError("session duration cap reached")
        return value

    def ask(message: dict[str, Any]) -> dict[str, Any]:
        assert peer is not None
        return peer.request(message, min(10, remaining()))

    def tick(rtt: RTT, elapsed: float) -> None:
        value = rtt.summary()["p95_ms"]
        latency = f"{value:.2f} ms" if value is not None else "pending"
        notify(f"{phase_label}: {elapsed:.1f} s | RTT p95 {latency} | {rtt.timeouts} timeouts")

    try:
        notify("Starting SSH-owned peer; no installation or robot stack")
        peer = RemotePeer(command)
        hello = ask(
            {
                "protocol": PROTOCOL_VERSION,
                "token": report.session_id,
                "settings": asdict(settings),
                "listen_host": listen_host,
                "port": port,
            }
        )
        if hello.get("op") != "hello" or hello.get("protocol") != PROTOCOL_VERSION:
            raise ValueError("remote network-check protocol incompatible")
        if not isinstance(hello.get("zenoh"), str) or not isinstance(hello.get("port"), int):
            raise ValueError("remote network-check handshake missing Zenoh version or TCP port")
        if not 1 <= hello["port"] <= 65535:
            raise ValueError("remote network-check handshake contains an invalid TCP port")
        report.remote_zenoh = hello["zenoh"]
        if report.remote_zenoh != report.local_zenoh:
            raise ValueError("use the same Zenoh version on both endpoints for comparable results")
        host = peer_host or remote.rsplit("@", 1)[-1]
        host = host.strip("[]")
        if ":" in host:
            host = f"[{host}]"
        report.endpoint = f"tcp/{host}:{hello['port']}"
        endpoint = Endpoint(report.session_id, "local", report.endpoint, stop)
        # Validate subscriber propagation using a round trip, not a guessed sleep.
        while endpoint.probe(0.2) is None:
            remaining()
        notify("Measuring idle RTT")
        report.baseline = endpoint.probes(
            min(settings.idle_seconds, remaining()), settings, tick
        ).summary()
        remaining()
        phase = 0
        used_local_bytes = 0
        for direction in report.directions:
            for rate in settings.rates:
                remaining()
                # Warm-up uses another phase ID, so it cannot pollute counters.
                if settings.warmup_seconds:
                    phase += 1
                    if direction == "remote_to_local":
                        ask(
                            {
                                "op": "send",
                                "phase": phase,
                                "rate": rate,
                                "duration": min(settings.warmup_seconds, remaining()),
                            }
                        )
                    else:
                        warm = endpoint.send(
                            phase,
                            rate,
                            min(settings.warmup_seconds, remaining()),
                            settings,
                            settings.max_bytes - used_local_bytes,
                        )
                        used_local_bytes += warm["bytes"]
                phase += 1
                duration = settings.step_seconds
                # Reserve enough time to finish the receiver observation window and drain.
                if remaining() < duration + settings.drain_seconds + settings.probe_timeout:
                    raise SessionCapError(
                        "session cap leaves insufficient time for a complete step"
                    )
                phase_label = f"{direction} @ {rate:g} Mbps"
                notify(f"{phase_label}: {duration:g} s + drain")
                if direction == "remote_to_local":
                    endpoint.receiver.reset(phase, duration)
                    peer.send({"op": "send", "phase": phase, "rate": rate, "duration": duration})
                    rtt = endpoint.probes(duration, settings, tick).summary()
                    sender = peer.read(min(remaining(), 10))["sender"]
                    stop.wait(settings.drain_seconds)
                    receiver = endpoint.receiver.summary(sender["messages"])
                else:
                    ask({"op": "prepare", "phase": phase, "duration": duration})
                    thread, results, errors = start_sender(
                        partial(
                            endpoint.send,
                            phase,
                            rate,
                            duration,
                            settings,
                            settings.max_bytes - used_local_bytes,
                        )
                    )
                    try:
                        rtt = endpoint.probes(duration, settings, tick).summary()
                    except BaseException:
                        stop.set()
                        raise
                    finally:
                        thread.join(timeout=duration + 1)
                    if errors:
                        raise errors[0]
                    if thread.is_alive() or not results:
                        raise RuntimeError("sender did not stop within bounded step")
                    sender = results[0]
                    used_local_bytes += sender["bytes"]
                    stop.wait(settings.drain_seconds)
                    receiver = ask({"op": "stats", "sent": sender["messages"]})["receiver"]
                remaining()
                if sender["messages"] == 0:
                    if sender["budget_exhausted"]:
                        report.stop_reasons[direction] = "byte_cap"
                        raise SessionCapError(
                            "byte cap exhausted before a complete measurement step"
                        )
                    raise ValueError(
                        "step sent no messages; increase rate/duration or lower payload size"
                    )
                step = {"target_mbps": rate, "sender": sender, "receiver": receiver, "rtt": rtt}
                report.directions[direction].append(step)
                notify(f"{direction}: step completed")
                if sender["budget_exhausted"]:
                    report.stop_reasons[direction] = "byte_cap"
                    break
                if settings.min_goodput_mbps is not None and meets_thresholds(step, settings):
                    report.stop_reasons[direction] = "requested_thresholds_met"
                    break
            report.stop_reasons.setdefault(direction, "offered_rate_cap")
        report.status = "completed"
        if settings.has_thresholds:
            report.verdict = (
                "met"
                if all(
                    steps and meets_thresholds(steps[-1], settings)
                    for steps in report.directions.values()
                )
                else "not_met"
            )
    except KeyboardInterrupt:
        report.status = "cancelled"
        report.error = "cancelled by user"
        report.verdict = "inconclusive" if settings.has_thresholds else "not_requested"
    except (OSError, RuntimeError, ValueError, TimeoutError, zenoh.ZError) as error:
        budget_expired = stop.is_set() or time.monotonic() >= deadline
        report.status = (
            "capped" if budget_expired or isinstance(error, SessionCapError) else "error"
        )
        report.error = str(error)
        report.verdict = "inconclusive" if settings.has_thresholds else "not_requested"
    finally:
        stop.set()
        lease.cancel()
        if endpoint is not None:
            endpoint.close()
        if peer is not None:
            report.cleanup = peer.close()
    return report
