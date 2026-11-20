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

"""Orchestrator demos on a robot with no hardware: `dimos run orchestration-demo-<name>`."""

from pathlib import Path

from dimos.core.coordination.blueprints import autoconnect
from dimos.orchestration.demos.toy_robot import ToyRobot
from dimos.orchestration.orchestrator import Orchestrator

HERE = Path(__file__).parent


orchestration_demo_delivery = autoconnect(
    ToyRobot.blueprint(pick_failures=2),
    Orchestrator.blueprint(file=HERE / "delivery.toml"),
)
orchestration_demo_inspection = autoconnect(
    ToyRobot.blueprint(script=[(0.5, "gripper_ok", False)]),
    Orchestrator.blueprint(file=HERE / "inspection.toml"),
)
orchestration_demo_shift = autoconnect(
    ToyRobot.blueprint(drive_s=4.0, pick_s=1.5, place_s=1.0),
    Orchestrator.blueprint(file=HERE / "shift.toml"),
)
orchestration_demo_safety = autoconnect(
    ToyRobot.blueprint(
        pick_s=2.0,
        drive_s=4.0,
        script=[
            (9.0, "obstacle", True),
            (14.0, "obstacle", False),
            (18.5, "estop", True),
            (24.0, "estop", False),
        ],
    ),
    Orchestrator.blueprint(file=HERE / "safety.toml"),
)
orchestration_demo_jam = autoconnect(
    ToyRobot.blueprint(pick_s=1.5, place_s=1.5, jam_on_release=6),
    Orchestrator.blueprint(file=HERE / "jam.toml"),
)
orchestration_demo_patrol = autoconnect(
    ToyRobot.blueprint(
        battery_start_pct=50.0, drain_per_s=3.0, charge_per_s=15.0, drive_failures=1
    ),
    Orchestrator.blueprint(file=HERE / "patrol.toml"),
)
orchestration_demo_warehouse = autoconnect(
    ToyRobot.blueprint(
        pick_failures=3,
        pick_failures_every=8,
        script=[
            (0.2, "estop", False),
            (0.2, "gripper_ok", True),
            *[
                event
                for cycle in range(60)
                for event in (
                    (cycle * 90.0 + 17.0, "gripper_ok", False),
                    (cycle * 90.0 + 23.0, "gripper_ok", True),
                    (cycle * 90.0 + 50.0, "estop", True),
                    (cycle * 90.0 + 56.0, "estop", False),
                )
            ],
        ],
    ),
    Orchestrator.blueprint(file=HERE / "warehouse.toml"),
)
