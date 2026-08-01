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

"""Typed, privacy-safe operational views for the spatial scheduler."""

from __future__ import annotations

import json

from rich.console import Group, RenderableType
from rich.live import Live
from rich.panel import Panel
from rich.table import Table

from .scheduler_models import OperationalSnapshot

_STATES = ("pending", "running", "succeeded", "failed", "interrupted", "cancelled")


def _require_snapshot(snapshot: OperationalSnapshot) -> OperationalSnapshot:
    if not isinstance(snapshot, OperationalSnapshot):
        raise TypeError("display input must be an OperationalSnapshot")
    return snapshot


def operational_projection(snapshot: OperationalSnapshot) -> dict[str, object]:
    """Return the deterministic JSON-safe projection of one snapshot."""
    snapshot = _require_snapshot(snapshot)
    return {
        "record_type": snapshot.record_type,
        "schema_version": snapshot.schema_version,
        "experiment_id": snapshot.experiment_id,
        "workers": snapshot.workers,
        "jobs": snapshot.jobs,
        "active": snapshot.active,
        "observation": snapshot.observation,
        "states": {state: getattr(snapshot.counts, state) for state in _STATES},
        "failures": [failure.model_dump(mode="json") for failure in snapshot.failures],
    }


def operational_json(snapshot: OperationalSnapshot) -> str:
    """Serialize one snapshot deterministically for operational consumers."""
    return json.dumps(operational_projection(snapshot), sort_keys=True, separators=(",", ":"))


def render_status(snapshot: OperationalSnapshot) -> Group:
    """Build a static Rich view from one authoritative snapshot."""
    return _render(_require_snapshot(snapshot), title="Experiment status")


def render_live(snapshot: OperationalSnapshot) -> Group:
    """Build one non-interactive Rich frame from one authoritative snapshot."""
    return _render(_require_snapshot(snapshot), title="Experiment running")


def live(snapshot: OperationalSnapshot) -> Live:
    """Return a Rich Live instance whose frame is derived from the snapshot."""
    return Live(render_live(snapshot), refresh_per_second=2, transient=False)


def _render(snapshot: OperationalSnapshot, *, title: str) -> Group:
    heading = Table.grid(padding=(0, 2))
    heading.add_column(style="dim")
    heading.add_column()
    heading.add_row("Experiment", snapshot.experiment_id)
    heading.add_row("Workers", str(snapshot.workers))
    heading.add_row("Observation", snapshot.observation)

    counts = Table.grid(expand=True, padding=(0, 2))
    counts.add_column(justify="right")
    counts.add_column()
    counts.add_column(justify="right")
    counts.add_column()
    for left, right in (
        ("pending", "running"),
        ("succeeded", "failed"),
        ("interrupted", "cancelled"),
    ):
        counts.add_row(
            f"{left.title()} {getattr(snapshot.counts, left)}",
            "",
            f"{right.title()} {getattr(snapshot.counts, right)}",
            "",
        )

    parts: list[RenderableType] = [
        Panel(heading, title=title, border_style="blue"),
        Panel(counts, title=f"Jobs · {snapshot.jobs}", border_style="grey50"),
    ]
    if snapshot.failures:
        failures = Table.grid(padding=(0, 1))
        failures.add_column(style="yellow", no_wrap=True)
        failures.add_column()
        for failure in snapshot.failures:
            failures.add_row(failure.job_id, failure.reason)
        parts.append(Panel(failures, title="Recent failures", border_style="yellow"))
    return Group(*parts)
