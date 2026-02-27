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

"""Human-readable formatters for durations, byte sizes, and numbers."""

from __future__ import annotations


def human_duration(seconds: float, signed: bool = True) -> str:
    """Format a duration in seconds to a human-readable string.

    Examples: ``"25.1 ms"``, ``"1.3 s"``, ``"4m 12s"``, ``"2h 3m"``.

    When *signed* is True (the default), the result is prefixed with ``+``
    or ``-``.  Set *signed=False* for unsigned values like latencies.
    """
    sign = ("+" if seconds >= 0 else "-") if signed else ""
    s = abs(seconds)
    if s < 1:
        return f"{sign}{s * 1000:.1f} ms"
    if s < 60:
        return f"{sign}{s:.1f} s"
    m, s = divmod(s, 60)
    if m < 60:
        return f"{sign}{int(m)}m {int(s)}s"
    h, m = divmod(m, 60)
    return f"{sign}{int(h)}h {int(m)}m"


def human_number(value: float, decimals: int = 1) -> str:
    """Format a number with SI suffixes (k, M, G, ...).

    Examples: ``"42"``, ``"1.5k"``, ``"3.2M"``.
    """
    for unit in ("", "k", "M", "G", "T"):
        if abs(value) < 1000:
            return f"{value:.{decimals}f}{unit}" if unit else f"{value:.0f}"
        value /= 1000
    return f"{value:.{decimals}f}P"


def human_bytes(value: float, concise: bool = False, decimals: int = 2) -> str:
    """Format bytes with IEC units (1024-based: KiB, MiB, GiB, ...).

    *concise=True* uses short suffixes without a space (``"1.50K"``).
    *decimals* controls the number of decimal places.
    """
    k = 1024.0
    units = ["B", "K", "M", "G", "T"] if concise else ["B", "KiB", "MiB", "GiB", "TiB"]

    for unit in units[:-1]:
        if abs(value) < k:
            return f"{value:.{decimals}f}{unit}" if concise else f"{value:.{decimals}f} {unit}"
        value /= k
    return f"{value:.{decimals}f}{units[-1]}" if concise else f"{value:.{decimals}f} {units[-1]}"
