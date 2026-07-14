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

"""Populate the DimSim environment with humans doing scheduled everyday tasks.

Two tests:

* ``test_human_task_schedule_is_deterministic`` — pure, always runs: the seeded
  schedule of "randomly generated" tasks is identical on every build, covers the
  whole predefined task pool, and respects the configured interval bounds.
* ``test_dimsim_humans_run_scheduled_tasks`` — self-hosted: boots DimSim, spawns
  the human NPCs, soaks for the scenario duration while the humans walk to their
  scheduled task locations, then asserts each ended up at its last task's room.

The scenario is fixed by ``fixtures/human_tasks.json`` so both runs are identical.
"""

from collections.abc import Callable
import itertools
import json
import math
import os
from pathlib import Path
import time

import pytest

from dimos.e2e_tests.activity_patrol import ActivityPatrol
from dimos.e2e_tests.dimos_cli_call import DimosCliCall
from dimos.e2e_tests.human_activity import (
    HumanActivityDriver,
    HumanTaskScenario,
    Location,
    write_activity_rerun,
)
from dimos.e2e_tests.lcm_spy import LcmSpy

SCENARIO_PATH = Path(__file__).parent / "fixtures" / "human_tasks.json"


def _load_scenario() -> HumanTaskScenario:
    return HumanTaskScenario.from_json(SCENARIO_PATH)


def _write_ground_truth(path: Path, scenario: HumanTaskScenario, driver: HumanActivityDriver) -> None:
    """Sidecar of what each human was *actually* doing when — labels to check
    a perception/activity-recognition skill against the robot-observed .rrd."""
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(
            {
                "seed": scenario.seed,
                "duration_s": scenario.duration_s,
                "locations": {
                    n: {
                        "x": driver.position_of(n)[0],
                        "y": driver.position_of(n)[1],
                        "furniture": loc.furniture,
                    }
                    for n, loc in scenario.locations.items()
                },
                "events": [
                    {
                        "at_s": e.at_s,
                        "human": e.human,
                        "task": e.task.id,
                        "location": e.task.location,
                    }
                    for e in driver.schedule
                ],
            },
            indent=2,
        )
    )


def test_human_task_schedule_is_deterministic() -> None:
    scenario = _load_scenario()

    # "randomly generated ... but the same every time it is run"
    schedule_a = scenario.build_schedule()
    schedule_b = scenario.build_schedule()
    assert schedule_a == schedule_b
    assert len(schedule_a) > 0

    lo, hi = scenario.interval_s_range
    for human in scenario.humans:
        times = [e.at_s for e in schedule_a if e.human == human.name]
        assert times, f"{human.name} was scheduled no tasks"
        assert times == sorted(times)
        assert times[0] <= hi  # first task lands within one interval
        assert all(t < scenario.duration_s for t in times)
        gaps = [b - a for a, b in itertools.pairwise(times)]
        assert all(lo - 1e-6 <= g <= hi + 1e-6 for g in gaps), gaps

    # every one of the predefined tasks targets a real scene object: either a
    # furniture anchor in the apt scene, or (legacy) an explicit prop stand-in
    assert len(scenario.tasks) == 10
    anchors = set()
    for task in scenario.tasks:
        assert task.location in scenario.locations
        loc = scenario.locations[task.location]
        assert loc.furniture or task.prop, (
            f"{task.id} has neither a real-furniture anchor nor a prop to act on"
        )
        if loc.furniture:
            anchors.add(loc.furniture)
    assert len(anchors) == 10  # a distinct real object per activity

    # humans use the real scanned-person model
    assert scenario.model_url.endswith(".glb")

    # events are globally time-ordered
    all_times = [e.at_s for e in schedule_a]
    assert all_times == sorted(all_times)


def _last_task_location(
    scenario: HumanTaskScenario, driver: HumanActivityDriver, human: str
) -> Location:
    last = None
    for event in driver.schedule:
        if event.human == human:
            last = event
    assert last is not None, f"{human} had no scheduled tasks"
    return scenario.location_of(last.task)


def test_rerun_save_path_reads_env() -> None:
    """The new global `rerun_save_path` is populated from RERUN_SAVE_PATH (.env)."""
    from dimos.core.global_config import GlobalConfig

    prev = os.environ.get("RERUN_SAVE_PATH")
    os.environ["RERUN_SAVE_PATH"] = "/tmp/dimos_probe_recording.rrd"
    try:
        assert GlobalConfig().rerun_save_path == "/tmp/dimos_probe_recording.rrd"
    finally:
        if prev is None:
            os.environ.pop("RERUN_SAVE_PATH", None)
        else:
            os.environ["RERUN_SAVE_PATH"] = prev


@pytest.mark.self_hosted_large
def test_dimsim_humans_run_scheduled_tasks(
    lcm_spy: LcmSpy,
    start_blueprint: Callable[..., DimosCliCall],
    run_human_activity: Callable[[HumanTaskScenario], HumanActivityDriver],
    robot_patrol: Callable[[HumanActivityDriver], ActivityPatrol],
) -> None:
    scenario = _load_scenario()

    # Record everything the robot observes to a .rrd (default under recordings/,
    # override with RERUN_SAVE_PATH). The bridge reads this via global config, so
    # setting the env before boot tees the live stream to a file.
    rrd_path = Path(
        os.environ.get("RERUN_SAVE_PATH") or (Path.cwd() / "recordings" / "human_activity.rrd")
    ).expanduser()
    os.environ["RERUN_SAVE_PATH"] = str(rrd_path)

    # Boot just the DimSim scene + robot + rerun bridge. We deliberately use the
    # non-agentic `unitree-go2-basic` blueprint: this scaffold only populates the
    # environment and records what the robot observes — it needs no agent, MCP,
    # web-input, or LLM. That also keeps the test off the `misc` extra
    # (python-multipart, required by the agentic WebInput module) and off OpenAI.
    start_blueprint(
        "run",
        "unitree-go2-basic",
        simulator="dimsim",
    )

    # `run_human_activity` polls the DimSim sandbox itself (wait_for_scene) before
    # spawning, so no readiness topic is needed.
    driver = run_human_activity(scenario)

    # ground-truth labels beside the recording (what each human did, when)
    gt_path = rrd_path.with_suffix(".groundtruth.json")
    _write_ground_truth(gt_path, scenario, driver)

    # every activity is anchored to REAL furniture in the apt scene (the bed,
    # sofa, toilet, kitchen counter, ...) — this is what makes the recording
    # meaningful for testing vision models / object detectors
    for loc in scenario.locations.values():
        if loc.furniture:
            assert driver.dim_sim.object_exists(loc.furniture), (
                f"real furniture anchor missing from scene: {loc.furniture}"
            )
    resolved = driver.resolved_locations
    assert len(resolved) == len(scenario.locations)

    # all humans spawned — as the real scanned-person model, not fallbacks —
    # and are readable in the scene
    assert driver.fallback_humans == set(), (
        f"humans fell back to primitive cylinders (person GLB failed to load): "
        f"{sorted(driver.fallback_humans)}"
    )
    for human in scenario.humans:
        x, y = driver.dim_sim.get_human_position(human.name)
        assert math.isfinite(x) and math.isfinite(y)

    # drive the robot on an activity-aware patrol so its camera + lidar actually
    # observe the humans (a stationary robot only ever sees one fixed spot)
    patrol = robot_patrol(driver)

    # let the whole schedule play out, plus enough slack for a worst-case
    # cross-map walk if the final task lands near the end of the window
    time.sleep(scenario.duration_s + 15.0)

    # each human should have walked to its final task's REAL furniture position
    for human in scenario.humans:
        goal = _last_task_location(scenario, driver, human.name)
        gx, gy = driver.position_of(goal.name)
        x, y = driver.dim_sim.get_human_position(human.name)
        dist = math.hypot(x - gx, y - gy)
        assert dist < 1.0, (
            f"{human.name} ended at ({x:.2f}, {y:.2f}), expected near "
            f"{goal.name}/{goal.furniture} ({gx:.2f}, {gy:.2f}); dist={dist:.2f}m"
        )

    # the patrol should have actually moved the robot and observed the humans
    observed = patrol.observed_humans
    print(
        f"\nPatrol: robot travelled up to {patrol.max_displacement_m:.1f} m from spawn; "
        f"observed {len(observed)}/{len(scenario.humans)} humans: {sorted(observed)}"
    )
    assert patrol.max_displacement_m > 1.0, (
        f"robot barely moved ({patrol.max_displacement_m:.2f} m) — patrol/cmd_vel not driving it"
    )
    assert observed, "robot never got a framed look at any human during the patrol"

    # fixed-world activity view: the humans doing their tasks + the robot's real
    # patrol path driving among them (open this to watch it all in one place)
    activity_rrd = rrd_path.with_suffix(".activity.rrd")
    write_activity_rerun(
        activity_rrd,
        scenario,
        robot_trajectory=patrol.trajectory,
        locations_override=resolved,
    )

    # the robot-observed recording should have been written by the bridge
    size = rrd_path.stat().st_size if rrd_path.exists() else 0
    print(
        f"Robot-observed recording: {rrd_path} ({size / 1e6:.1f} MB)\n"
        f"Actor activity view:      {activity_rrd}   <- open this to watch the humans\n"
        f"Ground-truth labels:      {gt_path}\n"
        f"Open with: dimos-viewer {rrd_path}"
    )
    assert rrd_path.exists(), (
        f"expected a robot-observed recording at {rrd_path} — is the rerun bridge "
        "active in this blueprint?"
    )
