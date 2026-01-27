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

import json
from pathlib import Path
import signal
import sys
import time
from typing import Any

import mujoco
import mujoco.viewer as viewer  # type: ignore[import-untyped]

from dimos.simulation.manipulators.mujoco_subprocess.shared_memory import ShmReader
from dimos.simulation.utils.xml_parser import JointMapping, build_joint_mappings

_MODE_POSITION = 0
_MODE_VELOCITY = 1
_MODE_EFFORT = 2


def _resolve_xml_path(config_path: Path) -> Path:
    resolved = config_path.expanduser()
    xml_path = resolved / "scene.xml" if resolved.is_dir() else resolved
    if not xml_path.exists():
        raise FileNotFoundError(f"MuJoCo XML not found: {xml_path}")
    return xml_path


def _current_position(data: mujoco.MjData, mapping: JointMapping) -> float:
    if mapping.joint_id is not None and mapping.qpos_adr is not None:
        return float(data.qpos[mapping.qpos_adr])
    if mapping.tendon_qpos_adrs:
        return float(
            sum(data.qpos[adr] for adr in mapping.tendon_qpos_adrs) / len(mapping.tendon_qpos_adrs)
        )
    if mapping.actuator_id is not None:
        return float(data.actuator_length[mapping.actuator_id])
    return 0.0


def _run_simulation(xml_path: Path, headless: bool, shm: ShmReader, dof: int) -> None:
    model = mujoco.MjModel.from_xml_path(str(xml_path))
    data = mujoco.MjData(model)

    joint_mappings = build_joint_mappings(xml_path, model)
    num_joints = len(joint_mappings)

    if num_joints != dof:
        raise ValueError(f"Shared memory DOF mismatch: shm={dof} model={num_joints}")

    joint_position_targets = [0.0] * num_joints
    joint_velocity_targets = [0.0] * num_joints
    joint_effort_targets = [0.0] * num_joints
    command_mode = _MODE_POSITION

    for i, mapping in enumerate(joint_mappings):
        current_pos = _current_position(data, mapping)
        joint_position_targets[i] = current_pos

    control_frequency = (
        1.0 / float(model.opt.timestep) if float(model.opt.timestep) > 0.0 else 100.0
    )
    dt = 1.0 / control_frequency

    def apply_control() -> None:
        if command_mode == _MODE_EFFORT:
            targets = joint_effort_targets
        elif command_mode == _MODE_VELOCITY:
            targets = joint_velocity_targets
        else:
            targets = joint_position_targets
        for i, mapping in enumerate(joint_mappings):
            if mapping.actuator_id is None or i >= len(targets):
                continue
            data.ctrl[mapping.actuator_id] = targets[i]

    def update_joint_state() -> tuple[list[float], list[float], list[float]]:
        positions = [0.0] * num_joints
        velocities = [0.0] * num_joints
        efforts = [0.0] * num_joints
        for i, mapping in enumerate(joint_mappings):
            if mapping.joint_id is not None:
                if mapping.qpos_adr is not None:
                    positions[i] = float(data.qpos[mapping.qpos_adr])
                if mapping.dof_adr is not None:
                    velocities[i] = float(data.qvel[mapping.dof_adr])
                    efforts[i] = float(data.qfrc_actuator[mapping.dof_adr])
                continue

            if mapping.tendon_qpos_adrs:
                pos_sum = sum(data.qpos[adr] for adr in mapping.tendon_qpos_adrs)
                positions[i] = float(pos_sum / len(mapping.tendon_qpos_adrs))
                if mapping.tendon_dof_adrs:
                    vel_sum = sum(data.qvel[adr] for adr in mapping.tendon_dof_adrs)
                    velocities[i] = float(vel_sum / len(mapping.tendon_dof_adrs))
            elif mapping.actuator_id is not None:
                positions[i] = float(data.actuator_length[mapping.actuator_id])

            if mapping.actuator_id is not None:
                efforts[i] = float(data.actuator_force[mapping.actuator_id])
        return positions, velocities, efforts

    def step_once(sync_viewer: bool) -> None:
        nonlocal command_mode
        loop_start = time.time()
        cmd = shm.read_command()
        if cmd is not None:
            mode, cmd_pos, cmd_vel, cmd_eff = cmd
            if mode == _MODE_POSITION:
                joint_position_targets[:] = cmd_pos.tolist()
                command_mode = _MODE_POSITION
            elif mode == _MODE_VELOCITY:
                joint_velocity_targets[:] = cmd_vel.tolist()
                command_mode = _MODE_VELOCITY
            elif mode == _MODE_EFFORT:
                joint_effort_targets[:] = cmd_eff.tolist()
                command_mode = _MODE_EFFORT
            else:
                command_mode = _MODE_POSITION

        apply_control()
        mujoco.mj_step(model, data)
        if sync_viewer:
            m_viewer.sync()

        positions, velocities, efforts = update_joint_state()
        shm.write_state(positions, velocities, efforts)

        elapsed = time.time() - loop_start
        sleep_time = dt - elapsed
        if sleep_time > 0:
            time.sleep(sleep_time)

    shm.signal_ready()

    if headless:
        while not shm.should_stop():
            step_once(sync_viewer=False)
        return

    with viewer.launch_passive(model, data, show_left_ui=False, show_right_ui=False) as m_viewer:
        while m_viewer.is_running() and not shm.should_stop():
            step_once(sync_viewer=True)


if __name__ == "__main__":

    def signal_handler(_signum: int, _frame: Any) -> None:
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    xml_path = Path(sys.argv[1])
    headless = bool(int(sys.argv[2]))
    dof = int(sys.argv[3])
    shm_names = json.loads(sys.argv[4])

    shm = ShmReader(shm_names, dof)
    try:
        _run_simulation(_resolve_xml_path(xml_path), headless, shm, dof)
    finally:
        shm.cleanup()
