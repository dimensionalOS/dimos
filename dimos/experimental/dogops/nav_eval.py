from __future__ import annotations

import json
from pathlib import Path
from statistics import mean

from dimos.experimental.dogops.models import NavAction, NavEvent, NavSummary
from dimos.experimental.dogops.store import DogOpsStore

try:  # pragma: no cover - exercised only inside a full DimOS checkout.
    from dimos.core.module import Module
except ModuleNotFoundError:

    class Module:
        def __init__(self, **_: object) -> None:
            pass

        @classmethod
        def blueprint(cls, **kwargs: object) -> dict[str, object]:
            return {"module": cls.__name__, "kwargs": kwargs}


def summarize_nav_events(run_id: str, events: list[NavEvent]) -> NavSummary:
    waypoint_events = [event for event in events if event.action == NavAction.goto]
    reached = [event for event in waypoint_events if event.success]
    failed = [event for event in waypoint_events if not event.success]
    elapsed_values = [event.elapsed_s for event in waypoint_events if event.elapsed_s > 0]
    tag_attempts = [
        event
        for event in events
        if event.action == NavAction.search_tag or "tag" in event.note.lower()
    ]
    guided = [event for event in events if event.guided or event.action == NavAction.guided]
    worst = max(waypoint_events, key=lambda event: event.elapsed_s, default=None)
    notes = [event.note for event in events if event.note]
    route_targets = {event.target_id for event in waypoint_events if event.target_id is not None}
    reached_targets = {event.target_id for event in reached if event.target_id is not None}
    tag_successes = len([event for event in tag_attempts if event.success])
    return NavSummary(
        run_id=run_id,
        waypoints_total=len(waypoint_events),
        waypoints_reached=len(reached),
        waypoints_failed=len(failed),
        success_rate=(len(reached) / len(waypoint_events)) if waypoint_events else 0.0,
        route_targets=len(route_targets),
        unique_targets_reached=len(reached_targets),
        route_coverage=(len(reached_targets) / len(route_targets)) if route_targets else 0.0,
        retries_total=sum(event.retries for event in events),
        guided_interventions=len(guided),
        guided_fallback_used=bool(guided),
        tag_reacquisition_attempts=len(tag_attempts),
        tag_reacquisition_successes=tag_successes,
        tag_reacquisition_rate=(tag_successes / len(tag_attempts)) if tag_attempts else 0.0,
        mean_elapsed_s=mean(elapsed_values) if elapsed_values else 0.0,
        worst_target_id=worst.target_id if worst else None,
        safety_stops=len([event for event in events if "safety" in event.note.lower()]),
        notes=notes,
    )


class DogOpsNavEvalModule(Module):
    def __init__(self, **_: object) -> None:
        if _:
            super().__init__(**_)

    def summarize_run(self, run_dir: str | Path = ".dogops/runs/latest") -> str:
        store = DogOpsStore.load_existing(run_dir)
        state = store.state
        assert state is not None
        summary = summarize_nav_events(state.run.id, state.nav_events)
        return json.dumps(summary.model_dump(mode="json"), sort_keys=True)
