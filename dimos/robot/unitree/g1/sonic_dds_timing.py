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

"""Offline DDS timing diagnostics; never converts receipt into acquisition time."""

from collections import defaultdict
from itertools import pairwise
from statistics import median
from typing import Any


def _milliseconds(values: list[int]) -> dict[str, float] | None:
    if not values:
        return None
    return {"min": min(values) / 1e6, "median": median(values) / 1e6, "max": max(values) / 1e6}


def summarize_dds_timing(samples: list[dict[str, Any]], *, duration_ns: int) -> dict[str, Any]:
    """Summarize a bounded take trace, preserving publisher and clock distinctions.

    Source-to-take differences include unknown clock offset and buffering.
    A take-call bracket bounds the reader operation, not acquisition or arrival.
    Tick decreases may be wraps or resets; the report does not choose a meaning.
    """
    if type(duration_ns) is not int or duration_ns <= 0:
        raise ValueError("positive integer duration_ns required")
    groups: dict[str, Any] = {}
    for row in samples:
        if row["group"] not in ("body", "left_hand", "right_hand"):
            raise ValueError("unknown measurement group")
        for field in (
            "source_timestamp_ns",
            "take_before_monotonic_ns",
            "take_after_monotonic_ns",
            "take_before_wall_ns",
            "take_after_wall_ns",
        ):
            if type(row[field]) is not int:
                raise ValueError(f"integer {field} required")
        if row["take_after_monotonic_ns"] < row["take_before_monotonic_ns"]:
            raise ValueError("reader monotonic bracket reversed")
        if type(row["valid_data"]) is not bool or not isinstance(row["publication_handle"], str):
            raise ValueError("validity and publisher identity required")
        tick = row.get("tick")
        if tick is not None and (type(tick) is not int or not 0 <= tick < 2**32):
            raise ValueError("tick must be uint32 or absent")
    for group in ("body", "left_hand", "right_hand"):
        rows = [row for row in samples if row["group"] == group]
        valid = [row for row in rows if row["valid_data"]]
        by_publisher: dict[str, list[dict[str, Any]]] = defaultdict(list)
        for row in valid:
            by_publisher[row["publication_handle"]].append(row)
        publishers = {}
        for handle, entries in by_publisher.items():
            stamped = [row for row in entries if row["source_timestamp_ns"] > 0]
            source_steps = [
                b["source_timestamp_ns"] - a["source_timestamp_ns"]
                for a, b in pairwise(entries)
                if a["source_timestamp_ns"] > 0 and b["source_timestamp_ns"] > 0
            ]
            tick_steps = [
                b["tick"] - a["tick"]
                for a, b in pairwise(entries)
                if a.get("tick") is not None and b.get("tick") is not None
            ]
            publishers[handle] = {
                "samples": len(entries),
                "missing_source_timestamps": len(entries) - len(stamped),
                "source_repeats": sum(step == 0 for step in source_steps),
                "source_regressions": sum(step < 0 for step in source_steps),
                "source_period_ms": _milliseconds(source_steps),
                "source_after_local_take": sum(
                    row["source_timestamp_ns"] > row["take_after_wall_ns"] for row in stamped
                ),
                "source_to_take_end_ms": _milliseconds(
                    [row["take_after_wall_ns"] - row["source_timestamp_ns"] for row in stamped]
                ),
                "take_call_ms": _milliseconds(
                    [
                        row["take_after_monotonic_ns"] - row["take_before_monotonic_ns"]
                        for row in entries
                    ]
                ),
                "wall_minus_monotonic_step_ms": _milliseconds(
                    [
                        (b["take_after_wall_ns"] - a["take_after_wall_ns"])
                        - (b["take_after_monotonic_ns"] - a["take_after_monotonic_ns"])
                        for a, b in pairwise(entries)
                    ]
                ),
                "tick_samples": sum(row.get("tick") is not None for row in entries),
                "tick_repeats": sum(step == 0 for step in tick_steps),
                "tick_decreases_wrap_or_reset": sum(step < 0 for step in tick_steps),
                "tick_negative_step_modulo_2_32": [step % 2**32 for step in tick_steps if step < 0],
                "tick_positive_step_range": None
                if not any(s > 0 for s in tick_steps)
                else [min(s for s in tick_steps if s > 0), max(s for s in tick_steps if s > 0)],
            }
        groups[group] = {
            "valid_samples": len(valid),
            "invalid_samples": len(rows) - len(valid),
            "observed_take_rate_hz": len(valid) / (duration_ns / 1e9),
            "publisher_transitions": sum(
                a["publication_handle"] != b["publication_handle"] for a, b in pairwise(valid)
            ),
            "publishers": publishers,
        }
    return {
        "acquisition_alignment_verified": False,
        "time_basis": "DDS source metadata and local take calls; sensor acquisition unverified",
        "groups": groups,
    }
