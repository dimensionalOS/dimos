#!/usr/bin/env python3

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

from collections.abc import Callable, Iterator, Sequence
from contextlib import AbstractContextManager
from dataclasses import dataclass, field
import pickle
import threading
import time
from typing import Any, Generic, TypeVar

import pytest

from dimos.msgs.geometry_msgs import Vector3
from dimos.msgs.sensor_msgs.Image import Image
from dimos.protocol.pubsub.lcmpubsub import LCM, Topic
from dimos.protocol.pubsub.memory import Memory
from dimos.protocol.pubsub.shmpubsub import PickleSharedMemory
from dimos.protocol.pubsub.spec import MsgT, PubSub, TopicT
from dimos.utils.data import get_data

MsgGen = Callable[[int], tuple[TopicT, MsgT]]

PubSubContext = Callable[[], AbstractContextManager[PubSub[TopicT, MsgT]]]


@dataclass
class Case(Generic[TopicT, MsgT]):
    pubsub_context: PubSubContext[TopicT, MsgT]
    msg_gen: MsgGen[TopicT, MsgT]

    def __iter__(self) -> Iterator[PubSubContext[TopicT, MsgT] | MsgGen[TopicT, MsgT]]:
        return iter((self.pubsub_context, self.msg_gen))

    def __len__(self) -> int:
        return 2


TestData = Sequence[Case[Any, Any]]


def _format_size(size_bytes: int) -> str:
    """Format byte size to human-readable string."""
    if size_bytes >= 1048576:
        return f"{size_bytes / 1048576:.1f} MB"
    if size_bytes >= 1024:
        return f"{size_bytes / 1024:.1f} KB"
    return f"{size_bytes} B"


def _format_throughput(bytes_per_sec: float) -> str:
    """Format throughput to human-readable string."""
    if bytes_per_sec >= 1e9:
        return f"{bytes_per_sec / 1e9:.2f} GB/s"
    if bytes_per_sec >= 1e6:
        return f"{bytes_per_sec / 1e6:.2f} MB/s"
    if bytes_per_sec >= 1e3:
        return f"{bytes_per_sec / 1e3:.2f} KB/s"
    return f"{bytes_per_sec:.2f} B/s"


@dataclass
class BenchmarkResult:
    transport: str
    duration: float  # Time spent publishing
    msgs_sent: int
    msgs_received: int
    msg_size_bytes: int
    receive_time: float = 0.0  # Time after publishing until all messages received

    @property
    def total_time(self) -> float:
        """Total time including latency."""
        return self.duration + self.receive_time

    @property
    def throughput_msgs(self) -> float:
        """Messages per second (including latency)."""
        return self.msgs_received / self.total_time if self.total_time > 0 else 0

    @property
    def throughput_bytes(self) -> float:
        """Bytes per second (including latency)."""
        return (
            (self.msgs_received * self.msg_size_bytes) / self.total_time
            if self.total_time > 0
            else 0
        )

    @property
    def loss_pct(self) -> float:
        """Message loss percentage."""
        return (1 - self.msgs_received / self.msgs_sent) * 100 if self.msgs_sent > 0 else 0


@dataclass
class BenchmarkResults:
    results: list[BenchmarkResult] = field(default_factory=list)

    def add(self, result: BenchmarkResult) -> None:
        self.results.append(result)

    def print_summary(self) -> None:
        if not self.results:
            return

        from rich.console import Console
        from rich.table import Table

        console = Console()

        table = Table(title="Benchmark Results")
        table.add_column("Transport", style="cyan")
        table.add_column("Msg Size", justify="right")
        table.add_column("Sent", justify="right")
        table.add_column("Recv", justify="right")
        table.add_column("Msgs/s", justify="right", style="green")
        table.add_column("Throughput", justify="right", style="green")
        table.add_column("Latency", justify="right")
        table.add_column("Loss", justify="right")

        for r in sorted(self.results, key=lambda x: (x.transport, x.msg_size_bytes)):
            loss_style = "red" if r.loss_pct > 0 else "dim"
            recv_style = "yellow" if r.receive_time > 0.1 else "dim"
            table.add_row(
                r.transport,
                _format_size(r.msg_size_bytes),
                f"{r.msgs_sent:,}",
                f"{r.msgs_received:,}",
                f"{r.throughput_msgs:,.0f}",
                _format_throughput(r.throughput_bytes),
                f"[{recv_style}]{r.receive_time * 1000:.0f}ms[/{recv_style}]",
                f"[{loss_style}]{r.loss_pct:.1f}%[/{loss_style}]",
            )

        console.print()
        console.print(table)

    def _print_heatmap(
        self,
        title: str,
        value_fn: Callable[[BenchmarkResult], float],
        format_fn: Callable[[float], str],
        high_is_good: bool = True,
    ) -> None:
        """Generic heatmap printer."""
        if not self.results:
            return

        def size_id(size: int) -> str:
            if size >= 1048576:
                return f"{size // 1048576}MB"
            if size >= 1024:
                return f"{size // 1024}KB"
            return f"{size}B"

        transports = sorted(set(r.transport for r in self.results))
        sizes = sorted(set(r.msg_size_bytes for r in self.results))

        # Build matrix
        matrix: list[list[float]] = []
        for transport in transports:
            row = []
            for size in sizes:
                result = next(
                    (
                        r
                        for r in self.results
                        if r.transport == transport and r.msg_size_bytes == size
                    ),
                    None,
                )
                row.append(value_fn(result) if result else 0)
            matrix.append(row)

        all_vals = [v for row in matrix for v in row if v > 0]
        if not all_vals:
            return
        min_val, max_val = min(all_vals), max(all_vals)

        # ANSI 256 gradient: red -> orange -> yellow -> green
        gradient = [
            52,
            88,
            124,
            160,
            196,
            202,
            208,
            214,
            220,
            226,
            190,
            154,
            148,
            118,
            82,
            46,
            40,
            34,
        ]
        if not high_is_good:
            gradient = gradient[::-1]

        def val_to_color(v: float) -> int:
            if v <= 0 or max_val == min_val:
                return 236
            t = (v - min_val) / (max_val - min_val)
            return gradient[int(t * (len(gradient) - 1))]

        reset = "\033[0m"
        size_labels = [size_id(s) for s in sizes]
        col_w = max(8, max(len(s) for s in size_labels) + 1)
        transport_w = max(len(t) for t in transports) + 1

        print()
        print(f"{title:^{transport_w + col_w * len(sizes)}}")
        print()
        print(" " * transport_w + "".join(f"{s:^{col_w}}" for s in size_labels))

        # Dark colors that need white text (dark reds)
        dark_colors = {52, 88, 124, 160, 236}

        for i, transport in enumerate(transports):
            row_str = f"{transport:<{transport_w}}"
            for val in matrix[i]:
                color = val_to_color(val)
                fg = 255 if color in dark_colors else 16  # white on dark, black on bright
                cell = format_fn(val) if val > 0 else "-"
                row_str += f"\033[48;5;{color}m\033[38;5;{fg}m{cell:^{col_w}}{reset}"
            print(row_str)
        print()

    def print_heatmap(self) -> None:
        """Print msgs/sec heatmap."""

        def fmt(v: float) -> str:
            return f"{v / 1000:.1f}k" if v >= 1000 else f"{v:.0f}"

        self._print_heatmap("Msgs/sec", lambda r: r.throughput_msgs, fmt)

    def print_bandwidth_heatmap(self) -> None:
        """Print bandwidth heatmap."""

        def fmt(v: float) -> str:
            if v >= 1e9:
                return f"{v / 1e9:.1f}G"
            if v >= 1e6:
                return f"{v / 1e6:.0f}M"
            if v >= 1e3:
                return f"{v / 1e3:.0f}K"
            return f"{v:.0f}"

        self._print_heatmap("Bandwidth", lambda r: r.throughput_bytes, fmt)

    def print_latency_heatmap(self) -> None:
        """Print latency heatmap (time waiting for messages after publishing)."""

        def fmt(v: float) -> str:
            if v >= 1:
                return f"{v:.1f}s"
            return f"{v * 1000:.0f}ms"

        self._print_heatmap("Latency", lambda r: r.receive_time, fmt, high_is_good=False)
