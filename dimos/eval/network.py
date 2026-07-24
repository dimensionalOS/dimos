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

"""Network-degradation eval for DimOS.

Design:
* Transports come from the same registry the transport benchmark uses and every
  run goes through the same measure_throughput function the pytest benchmark uses,
  producing the same BenchmarkResult.
* SHM/Memory/webrtc-loopback transports are same-host and immune to netem; they
  are excluded automatically.
* NetworkEvalResult embeds a HardwareProfile and serializes to JSON, so a
  cross-board/cross-link comparison is just "run it, diff the JSON".

Requirements for netem profiles: Linux + root + tc (iproute2).
"""

from __future__ import annotations

from contextlib import contextmanager
from dataclasses import asdict, dataclass, field
from datetime import datetime, timezone
import json
import os
import platform
import shutil
import subprocess
from typing import TYPE_CHECKING, Any

from dimos.eval.hardware import HardwareProfile, detect_hardware

if TYPE_CHECKING:
    from collections.abc import Iterator

    from dimos.protocol.pubsub.benchmark.type import BenchmarkResult

SAME_HOST: frozenset[str] = frozenset(
    {"memory", "shm_pickle", "shm_bytes", "shm_lcm", "webrtc_loopback"}
)

def _build_networked_cases() -> list[Any]:
    """Build transport cases directly from transport implementations.
    """
    from contextlib import contextmanager
    from types import SimpleNamespace

    cases: list[Any] = []

    from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
    import numpy as np

    # LCM (UDP multicast)
    from dimos.protocol.pubsub.impl.lcmpubsub import LCM, Topic as LCMTopic

    @contextmanager
    def _lcm_ctx():
        p = LCM(); p.start(); yield p; p.stop()

    def _lcm_msg(size: int):
        data = np.frombuffer(bytes(i % 256 for i in range(size)), dtype=np.uint8)
        h = max(1, int(len(data) ** 0.5)); w = len(data) // h
        return LCMTopic(topic="eval/lcm", lcm_type=Image), Image(
            data=data[: h * w].reshape(h, w, 1), format=ImageFormat.RGB)

    cases.append(SimpleNamespace(pubsub_context=_lcm_ctx, msg_gen=_lcm_msg, display_name="LCM"))

    # UdpBytes (raw UDP via LCM base)
    from dimos.protocol.pubsub.impl.lcmpubsub import LCMPubSubBase

    @contextmanager
    def _udp_ctx():
        p = LCMPubSubBase(); p.start(); yield p; p.stop()

    def _udp_msg(size: int):
        return LCMTopic(topic="eval/udp"), bytes(i % 256 for i in range(size))

    cases.append(SimpleNamespace(pubsub_context=_udp_ctx, msg_gen=_udp_msg, display_name="UdpBytes"))

    # Zenoh (single session — intra-session local routing)
    from dimos.protocol.pubsub.impl.zenohpubsub import Zenoh, Topic as ZenohTopic
    from dimos.protocol.service.zenohservice import ZenohSessionPool

    @contextmanager
    def _zenoh_ctx():
        pool = ZenohSessionPool()
        p = Zenoh(session_pool=pool); p.start(); yield p; p.stop(); pool.close_all()

    def _zenoh_msg(size: int):
        data = np.frombuffer(bytes(i % 256 for i in range(size)), dtype=np.uint8)
        h = max(1, int(len(data) ** 0.5)); w = len(data) // h
        return ZenohTopic("eval/zenoh", Image), Image(
            data=data[: h * w].reshape(h, w, 1), format=ImageFormat.RGB)

    cases.append(SimpleNamespace(pubsub_context=_zenoh_ctx, msg_gen=_zenoh_msg, display_name="Zenoh"))

    # ZenohPeers (two peer sessions — cross-process wire path)
    from dimos.utils.testing.waiting import wait_until

    @contextmanager
    def _zenoh_peers_ctx():
        pub_pool, sub_pool = ZenohSessionPool(), ZenohSessionPool()
        pub = Zenoh(session_pool=pub_pool); sub = Zenoh(session_pool=sub_pool)
        pub.start(); sub.start()
        wait_until(lambda: len(pub.session.info.peers_zid()) > 0, timeout=5.0,
                   message="Zenoh peers did not discover each other")

        class _Split:
            def publish(self, t, m): pub.publish(t, m)
            def subscribe(self, t, cb): return sub.subscribe(t, cb)

        yield _Split()
        pub.stop(); sub.stop(); pub_pool.close_all(); sub_pool.close_all()

    def _zenoh_peers_msg(size: int):
        data = np.frombuffer(bytes(i % 256 for i in range(size)), dtype=np.uint8)
        h = max(1, int(len(data) ** 0.5)); w = len(data) // h
        return ZenohTopic("eval/zenoh_peers", Image), Image(
            data=data[: h * w].reshape(h, w, 1), format=ImageFormat.RGB)

    cases.append(SimpleNamespace(pubsub_context=_zenoh_peers_ctx, msg_gen=_zenoh_peers_msg, display_name="ZenohPeers"))

    # Redis (optional — only when server is running)
    try:
        from dimos.protocol.pubsub.impl.redispubsub import Redis

        @contextmanager
        def _redis_ctx():
            p = Redis(); p.start(); yield p; p.stop()

        def _redis_msg(size: int):
            import base64
            return "eval/redis", {"data": base64.b64encode(bytes(i % 256 for i in range(size))).decode()}

        cases.append(SimpleNamespace(pubsub_context=_redis_ctx, msg_gen=_redis_msg, display_name="Redis"))
    except Exception:
        pass

    return cases


def networked_cases() -> list[Any]:
    """Transport cases that cross a real network link, with no pytest dependency."""
    return _build_networked_cases()


def _display_name(case: Any) -> str:
    return case.display_name



@dataclass(frozen=True)
class NetemProfile:
    """A named link condition expressed as tc netem parameters."""

    name: str
    delay_ms: float = 0.0
    jitter_ms: float = 0.0
    loss_pct: float = 0.0
    rate: str = ""
    reorder_pct: float = 0.0

    @property
    def is_baseline(self) -> bool:
        """A no-op profile — measured for the uncapped reference row."""
        return not (self.delay_ms or self.jitter_ms or self.loss_pct or self.rate)

    def netem_args(self) -> list[str]:
        """Render the tc qdisc netem args for this profile."""

        def n(v: float) -> str:
            return f"{v:g}"

        args: list[str] = []
        if self.delay_ms:
            args += ["delay", f"{n(self.delay_ms)}ms"]
            if self.jitter_ms:
                args += [f"{n(self.jitter_ms)}ms"]
        if self.loss_pct:
            args += ["loss", f"{n(self.loss_pct)}%"]
        if self.reorder_pct:
            args += ["reorder", f"{n(self.reorder_pct)}%"]
        if self.rate:
            args += ["rate", self.rate]
        return args


DEFAULT_PROFILES: tuple[NetemProfile, ...] = (
    NetemProfile("perfect"),
    NetemProfile("wifi", delay_ms=5, jitter_ms=2, loss_pct=0.5),
    NetemProfile("lte", delay_ms=50, jitter_ms=20, loss_pct=1.0),
    NetemProfile("congested", delay_ms=120, jitter_ms=40, loss_pct=3.0, rate="10mbit"),
    NetemProfile("satellite", delay_ms=600, jitter_ms=50, loss_pct=1.0, rate="5mbit"),
)


def netem_available() -> tuple[bool, str]:
    """Whether we can drive tc netem here. Returns (ok, reason-if-not)."""
    if platform.system() != "Linux":
        return False, "netem requires Linux"
    if shutil.which("tc") is None:
        return False, "`tc` (iproute2) not found on PATH"
    if hasattr(os, "geteuid") and os.geteuid() != 0:
        return False, "netem requires root (run under sudo)"
    return True, ""


@contextmanager
def impaired_link(iface: str, profile: NetemProfile) -> "Iterator[None]":
    """Apply ``profile`` to ``iface`` for the duration of the block, then clear.

    A baseline profile is a no-op. tc qdisc replace is idempotent, so a leaked
    qdisc from a crashed prior run is overwritten rather than erroring.
    """
    if profile.is_baseline:
        yield
        return

    cmd = ["tc", "qdisc", "replace", "dev", iface, "root", "netem", *profile.netem_args()]
    subprocess.run(cmd, check=True)
    try:
        yield
    finally:
        subprocess.run(
            ["tc", "qdisc", "del", "dev", iface, "root"],
            check=False,
            stderr=subprocess.DEVNULL,
        )


@dataclass
class NetworkSample:
    """One (transport, profile) measurement, flattened for the JSON artifact."""

    transport: str
    profile: str
    loss_pct: float
    latency_ms: float
    throughput_msgs: float
    cpu_cores: float
    passed: bool


@dataclass
class NetworkEvalResult:
    """Full result of grading a transport set across impairment profiles."""

    run_mode: str
    status: str  # "ok" | "failed"
    timestamp: str
    hardware: HardwareProfile
    iface: str
    msg_size_bytes: int
    max_loss_pct: float
    max_latency_ms: float
    samples: list[NetworkSample] = field(default_factory=list)
    error: str = ""

    @property
    def passed(self) -> bool:
        return self.status == "ok" and all(s.passed for s in self.samples)

    def to_dict(self) -> dict[str, object]:
        return asdict(self)

    def to_json(self, indent: int = 2) -> str:
        return json.dumps(self.to_dict(), indent=indent)


def _verdict(result: "BenchmarkResult", max_loss_pct: float, max_latency_ms: float) -> bool:
    return result.loss_pct <= max_loss_pct and (result.receive_time * 1000) <= max_latency_ms


def run_network_eval(
    *,
    profiles: tuple[NetemProfile, ...] = DEFAULT_PROFILES,
    transports: tuple[str, ...] | None = None,
    iface: str = "lo",
    msg_size: int = 4096,
    duration: float = 1.0,
    receive_timeout: float = 5.0,
    max_loss_pct: float = 5.0,
    max_latency_ms: float = 250.0,
) -> NetworkEvalResult:
    """Grade networked transports across ``profiles`` on ``iface``.

    For each profile the interface is impaired, then each transport is measured
    once at ``msg_size``. Loss/latency are graded against the thresholds to
    produce a PASS/FAIL per (transport, profile).

    Args:
        profiles: Impairment profiles to sweep (default: DEFAULT_PROFILES).
        transports: Restrict to these display names (case-insensitive); None = all.
        iface: Interface to impair (``lo`` for localhost transports).
        receive_timeout: Max drain wait per run; raise it above the worst
            profile's delay so added latency reads as latency, not loss.
    """
    result = NetworkEvalResult(
        run_mode="network",
        status="ok",
        timestamp=datetime.now(timezone.utc).isoformat(),
        hardware=detect_hardware(),
        iface=iface,
        msg_size_bytes=msg_size,
        max_loss_pct=max_loss_pct,
        max_latency_ms=max_latency_ms,
    )

    if any(not p.is_baseline for p in profiles):
        ok, reason = netem_available()
        if not ok:
            result.status = "failed"
            result.error = reason
            return result

    from dimos.eval.measure import measure_throughput

    wanted = {t.lower() for t in transports} if transports else None
    cases = [c for c in networked_cases() if wanted is None or _display_name(c).lower() in wanted]
    if not cases:
        result.status = "failed"
        result.error = f"no networked transports matched {transports!r}"
        return result

    try:
        for profile in profiles:
            with impaired_link(iface, profile):
                for case in cases:
                    name = _display_name(case)
                    topic, msg = case.msg_gen(msg_size)
                    with case.pubsub_context() as pubsub:
                        bench = measure_throughput(
                            pubsub, topic, msg, name, msg_size,
                            duration=duration, receive_timeout=receive_timeout,
                        )
                    result.samples.append(
                        NetworkSample(
                            transport=name,
                            profile=profile.name,
                            loss_pct=bench.loss_pct,
                            latency_ms=bench.receive_time * 1000,
                            throughput_msgs=bench.throughput_msgs,
                            cpu_cores=bench.cpu_cores,
                            passed=_verdict(bench, max_loss_pct, max_latency_ms),
                        )
                    )
    except Exception as exc:
        result.status = "failed"
        result.error = f"{type(exc).__name__}: {exc}"

    return result


def render_network_report(result: NetworkEvalResult) -> None:
    """Print a transport x profile loss/latency matrix."""
    from rich.console import Console
    from rich.table import Table

    console = Console()
    console.rule(f"[bold]DimOS Network Eval — {result.iface} @ {result.msg_size_bytes}B")
    if result.status != "ok":
        console.print(f"[bold red]STATUS: FAILED[/bold red] — {result.error}")
        return

    profiles = list(dict.fromkeys(s.profile for s in result.samples))
    transports = list(dict.fromkeys(s.transport for s in result.samples))
    by_key = {(s.transport, s.profile): s for s in result.samples}

    table = Table(title="loss% / latency (PASS/FAIL vs budget)")
    table.add_column("Transport", style="cyan")
    for p in profiles:
        table.add_column(p, justify="right")
    for t in transports:
        row = [t]
        for p in profiles:
            s = by_key.get((t, p))
            if s is None:
                row.append("-")
                continue
            style = "green" if s.passed else "red"
            row.append(f"[{style}]{s.loss_pct:.1f}% / {s.latency_ms:.0f}ms[/{style}]")
        table.add_row(*row)
    console.print(table)
    console.print(
        f"\n[bold]VERDICT[/bold]: {'PASS' if result.passed else 'FAIL'} "
        f"(budget: loss<={result.max_loss_pct}% , latency<={result.max_latency_ms}ms)"
    )


def _select_profiles(names: list[str] | None, baseline_only: bool) -> tuple[NetemProfile, ...]:
    if baseline_only:
        return (NetemProfile("perfect"),)
    if not names:
        return DEFAULT_PROFILES
    by_name = {p.name: p for p in DEFAULT_PROFILES}
    return tuple(by_name[n] for n in names)


def _list() -> None:
    print("Networked transports (impaired by this eval):")
    for case in networked_cases():
        print(f"  {_display_name(case)}")
    print("\nSame-host transports (skipped — immune to netem):")
    print(f"  {', '.join(sorted(SAME_HOST))}")
    print("\nProfiles:")
    for p in DEFAULT_PROFILES:
        detail = "baseline (no impairment)" if p.is_baseline else " ".join(p.netem_args())
        print(f"  {p.name:<10} {detail}")


def main(argv: list[str] | None = None) -> int:
    import argparse
    from pathlib import Path
    import socket

    parser = argparse.ArgumentParser(
        prog="python -m dimos.eval.network",
        description="Grade DimOS transports under tc netem link impairment.",
    )
    parser.add_argument("--list", action="store_true", help="List transports/profiles and exit.")
    parser.add_argument("--iface", default="lo", help="Interface to impair (default: lo).")
    parser.add_argument("--transport", action="append", metavar="NAME",
                        help="Restrict to this transport (repeatable); default: all.")
    parser.add_argument("--profile", action="append", metavar="NAME",
                        help="Restrict to this profile (repeatable); default: all.")
    parser.add_argument("--baseline-only", action="store_true",
                        help="Run only the 'perfect' profile (no root/netem needed).")
    parser.add_argument("--msg-size", type=int, default=4096, help="Payload bytes (default: 4096).")
    parser.add_argument("--duration", type=float, default=1.0, help="Publish window s (default: 1).")
    parser.add_argument("--max-loss", type=float, default=5.0, help="PASS loss%% budget (def: 5).")
    parser.add_argument("--max-latency", type=float, default=250.0, help="PASS ms budget (def: 250).")
    parser.add_argument("--output", "-o", type=Path, default=None, help="JSON artifact path.")
    parser.add_argument("--quiet", "-q", action="store_true", help="Skip the terminal report.")
    args = parser.parse_args(argv)

    if args.list:
        _list()
        return 0

    try:
        profiles = _select_profiles(args.profile, args.baseline_only)
    except KeyError as bad:
        parser.error(f"unknown profile {bad}; see --list")

    result = run_network_eval(
        profiles=profiles,
        transports=tuple(args.transport) if args.transport else None,
        iface=args.iface,
        msg_size=args.msg_size,
        duration=args.duration,
        max_loss_pct=args.max_loss,
        max_latency_ms=args.max_latency,
    )
    if not args.quiet:
        render_network_report(result)

    host = socket.gethostname().split(".")[0]
    out = args.output or Path.cwd() / f"neteval_{host}.json"
    out.write_text(result.to_json())
    print(f"Wrote artifact: {out}")
    return 0 if result.passed else 1


if __name__ == "__main__":
    import sys

    sys.exit(main())
