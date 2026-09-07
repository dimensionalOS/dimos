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

"""Scripted SQLite collection and ACT rollout in the same simulated scene."""

from dataclasses import replace
from pathlib import Path

from dimos.constants import RECORDINGS_DIR
from dimos.control.coordinator import TaskConfig
from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.imitation.collection.episode_monitor import EpisodeMonitorModule
from dimos.imitation.policy.lerobot.module import DualOpenYamLeRobotPolicy
from dimos.imitation.policy.module import POLICY_ROLLOUT_INSTANCE_NAME, POLICY_ROLLOUT_TASK_NAME
from dimos.imitation.policy.skills import PolicySkills
from dimos.robot.manipulators.dual_openyam.blueprints.basic import DualOpenYamCoordinator
from dimos.robot.manipulators.dual_openyam.blueprints.simulation import dual_openyam_sim_pick_place
from dimos.robot.manipulators.dual_openyam.config import DUAL_OPENYAM_JOINTS
from dimos.robot.manipulators.dual_openyam.learning import (
    DUAL_OPENYAM_LEROBOT_IO,
    DUAL_OPENYAM_SIM_TASK,
    DualOpenYamSimRecorder,
)


def build_dual_openyam_sim_collection(
    *,
    recording: Path,
    task: str,
    cameras: dict[str, int | str] | None = None,
    resume: bool = False,
) -> Blueprint:
    """Record the simulation's own cameras and applied joint commands."""
    if cameras:
        raise ValueError("Simulation supplies cameras; omit --camera")
    streams = [source.stream for source in DUAL_OPENYAM_LEROBOT_IO.observations.values()]
    streams.append(DUAL_OPENYAM_LEROBOT_IO.action.demonstration.stream)
    base = dual_openyam_sim_pick_place
    atoms = tuple(
        replace(
            atom,
            kwargs={
                **atom.kwargs,
                "tasks": [
                    replace(task, params={**task.params, "hold_position_when_idle": True})
                    if task.type == "trajectory"
                    else task
                    for task in atom.kwargs["tasks"]
                ],
            },
        )
        if atom.module is DualOpenYamCoordinator
        else atom
        for atom in base.blueprints
    )
    return autoconnect(
        replace(base, blueprints=atoms),
        DualOpenYamSimRecorder.blueprint(
            db_path=recording,
            record_tf=False,
            poseless_streams=[*streams, "status"],
            on_existing="resume" if resume else "error",
        ),
        EpisodeMonitorModule.blueprint(task=task),
    )


def build_dual_openyam_sim_rollout(
    *,
    artifact: str,
    task: str,
    device: str | None = None,
    cameras: dict[str, int | str] | None = None,
    quest_control: bool = False,
) -> Blueprint:
    """Run ACT with exclusive trajectory priority until explicitly stopped."""
    if cameras or quest_control:
        raise ValueError("Simulation uses its own cameras and policy skills")
    base = dual_openyam_sim_pick_place
    atoms = tuple(
        replace(
            atom,
            kwargs={
                **atom.kwargs,
                "tasks": [
                    *atom.kwargs["tasks"],
                    TaskConfig(
                        name=POLICY_ROLLOUT_TASK_NAME,
                        type="trajectory",
                        joint_names=list(DUAL_OPENYAM_JOINTS),
                        priority=30,
                        params={"start_position_tolerance": 0.05},
                    ),
                ],
            },
        )
        if atom.module is DualOpenYamCoordinator
        else atom
        for atom in base.blueprints
    )
    return autoconnect(
        replace(base, blueprints=atoms),
        DualOpenYamLeRobotPolicy.blueprint(
            instance_name=POLICY_ROLLOUT_INSTANCE_NAME,
            artifact=artifact,
            task=task,
            device=device,
        ),
        PolicySkills.blueprint(),
    )


dual_openyam_sim_collect = build_dual_openyam_sim_collection(
    recording=RECORDINGS_DIR / "dual-openyam-sim.db", task=DUAL_OPENYAM_SIM_TASK
)
