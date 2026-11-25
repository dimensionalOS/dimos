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

"""Replay the same JoyLo elbow motion and external torque with each simulated follower."""

import argparse
from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
from numpy.typing import NDArray

from dimos.control.connection.connection_module import ConnectionConfig
from dimos.control.connection_source import ConnectionSource
from dimos.control.contract.convert import dynamic_joint_state_to_values, joint_state_to_values
from dimos.control.contract.keys import Interface
from dimos.control.contract.limits import PD_INTERFACES
from dimos.control.motion_limits import MotionLimiter
from dimos.control.task import CoordinatorState, JointStateSnapshot
from dimos.control.tasks.joint_impedance_task import joint_impedance_task
from dimos.control.tasks.trajectory_task import trajectory_task
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.assets.model import RobotModel
from dimos.robot.galaxea.r1pro.joylo import ARMS, JAWS, UPPER, JoyloConfig, JoyloController
from dimos.robot.galaxea.r1pro.joylo_coordinator import joylo_tasks
from dimos.robot.galaxea.r1pro.motion import motion_groups
from dimos.robot.galaxea.r1pro.sim.config import ARM_RATE, UPPER_RATES, joylo_impedance_task
from dimos.robot.galaxea.r1pro.sim.dynamics import UpperBodyServo
from dimos.robot.galaxea.r1pro.sim.model import VENDOR_MODEL_PATH
from dimos.teleop.joycon.mapping import JoyConReading

ELBOWS = (3, 10)
DT = 0.001


def arm_models(urdf: Path) -> tuple[RobotModel, mujoco.MjModel]:
    """Build the 18-joint servo model and fixed-torso arm physics from a 2025 R1 Pro URDF."""
    tree = ET.parse(urdf).getroot()
    for link in tree.findall("link"):
        for tag in ("visual", "collision"):
            for element in link.findall(tag):
                link.remove(element)
    for joint in tree.findall("joint"):
        name = "r1pro/" + joint.attrib["name"]
        if name in UPPER:
            joint.set("name", name)
        else:
            joint.set("type", "fixed")
    model = RobotModel.from_urdf_string(ET.tostring(tree, encoding="unicode"))
    for joint in tree.findall("joint"):
        if joint.attrib["name"] not in ARMS:
            joint.set("type", "fixed")
    physics = mujoco.MjModel.from_xml_string(ET.tostring(tree, encoding="unicode"))
    physics.opt.timestep = DT
    physics.opt.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST
    return model, physics


def replay(urdf: Path, impedance: bool, *, soft_position: bool = False) -> NDArray[np.float64]:
    """Return time, leader angle, elbow angles and spring torques for one trial."""
    model, physics = arm_models(urdf)
    data = mujoco.MjData(physics)
    ids = np.array([physics.joint(name).id for name in ARMS])
    qidx, vidx = physics.jnt_qposadr[ids], physics.jnt_dofadr[ids]
    friction = physics.dof_damping[vidx].copy()
    servo = UpperBodyServo(model.load())
    joylo = JoyloController(
        dict(zip(UPPER, zip(servo.lower, servo.upper, strict=True), strict=True)),
        JoyloConfig(leader_angle_period=2 * np.pi),
    )
    profile = joylo_impedance_task()
    cfg = ConnectionConfig(
        source="r1pro",
        model=model,
        joint_commands=dict.fromkeys(ARMS, PD_INTERFACES if impedance else (Interface.POSITION,)),
        max_stiffness=profile.params["stiffness"] if impedance else {},
        max_damping=profile.params["damping"] if impedance else {},
    )
    port = "motor_command" if impedance else "position_command"
    source = ConnectionSource(cfg, [port], ["joint_state"], 1.0)
    tasks = joylo_tasks(
        dict(zip(UPPER, UPPER_RATES, strict=True)), arm_impedance=profile if impedance else None
    )
    follower = (
        joint_impedance_task.create_task(
            next(t for t in tasks if t.type == "joint_impedance"), {"r1pro": source}
        )
        if impedance
        else trajectory_task.create_task(next(t for t in tasks if t.type == "trajectory"))
    )
    limiter = MotionLimiter(motion_groups(arm_rate=ARM_RATE, arm_ceiling=ARM_RATE))
    fields: dict[str, float] = {}
    target = np.zeros(18)
    trajectory = np.zeros((13000, 6))
    phase = 0.0
    mujoco.mj_forward(physics, data)
    for i in range(len(trajectory)):
        now = i * DT
        q, dq = data.qpos[qidx].copy(), data.qvel[vidx].copy()
        feedback = dict(zip(ARMS, q, strict=True))
        measured = np.r_[np.zeros(4), q]
        if i % 20 == 0:
            phase = -0.18 * (1 - np.cos(np.pi * (now - 0.5))) if 0.5 <= now < 8.5 else 0.0
            leader = dict.fromkeys(ARMS, 0.0)
            for elbow in ELBOWS:
                leader[ARMS[elbow]] = phase
            joylo.set_leader(leader, now)
            joylo.set_feedback(
                {**dict(zip(UPPER, measured, strict=True)), **dict.fromkeys(JAWS, 0.05)}, now
            )
            joylo.set_joycon(
                JoyConReading(),
                JoyConReading(frozenset({"X"}) if i in (100, 12400) else frozenset()),
                now,
            )
            output = joylo.step(now, 0.02)
            if impedance and not joylo.active:
                follower.on_joint_command(
                    JointState(
                        name=list(ARMS), velocity=[0.0] * len(ARMS), frame_id="dimos/guarded-joint"
                    ),
                    now,
                )
            elif output is not None:
                follower.on_joint_command(
                    JointState(
                        name=list(ARMS),
                        position=[output.positions[n] for n in ARMS],
                        frame_id="dimos/guarded-joint",
                    ),
                    now,
                )
        if i % 10 == 0:
            source.on_message(
                "joint_state",
                JointState(name=list(ARMS), position=q.tolist(), velocity=dq.tolist()),
            )
            state = CoordinatorState(
                joints=JointStateSnapshot(joint_positions=feedback),
                dt=0.01,
                t_now=now,
                commanded_positions=limiter.snapshot(feedback),
            )
            output_command = follower.compute(state)
            commands = {}
            if output_command is not None:
                for j, name in enumerate(output_command.joint_names):
                    if output_command.positions is None:
                        raise ValueError("JoyLo arm follower supplied no target angles")
                    value = (
                        output_command.impedance(j) if impedance else output_command.positions[j]
                    )
                    commands[name] = (value, output_command.mode, follower.name)
            limited = limiter.apply(commands, feedback, state.dt)
            values = source.command({n: (v, mode) for n, (v, mode, _) in limited.items()})
            (packet,) = source.command_messages(values, now)
            fields = (
                dynamic_joint_state_to_values(packet.message)
                if impedance
                else joint_state_to_values(packet.message, (Interface.POSITION,))
            )
            if impedance and not joylo.active and any(k.endswith("/stiffness") for k in fields):
                raise RuntimeError("Paused leader still owns arm gains")
            target[4:] = [fields[f"{n}/position"] for n in ARMS]
            limiter.commit(dict(zip(ARMS, target[4:], strict=True)))
        if i % 4 == 0:
            ref, speed, kp, kd, effort = servo.motor_targets(target, measured, fields, 0.004)
            if soft_position:
                kp[4:] = [profile.params["stiffness"][n] for n in ARMS]
                kd[4:] = [profile.params["damping"][n] for n in ARMS]
        spring = kp[4:] * (ref[4:] - q)
        # MuJoCo integrates the measured-velocity damping implicitly at the physics tick.
        physics.dof_damping[vidx] = friction + kd[4:]
        data.qfrc_applied[vidx] = spring + kd[4:] * speed[4:] + effort[4:]
        if 11 <= now < 12:
            data.qfrc_applied[vidx[list(ELBOWS)]] += 1.0
        trajectory[i] = (now, phase, q[3], q[10], spring[3], spring[10])
        mujoco.mj_step(physics, data)
    if not np.isfinite(trajectory).all():
        raise RuntimeError("Non-finite arm simulation")
    return trajectory


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--urdf", type=Path, help="2025 Galaxea R1 Pro URDF")
    parser.add_argument("--plot", type=Path, help="Save the paired elbow traces as a PDF or PNG")
    args = parser.parse_args()
    urdf = args.urdf or Path(VENDOR_MODEL_PATH)
    print(
        "Simulated elbow pair | tracking RMS (deg) | tracking lag (ms) | peak deflection at 1 Nm (deg)"
    )
    traces = {}
    rows = []
    for label, impedance, soft in (
        ("Position", False, False),
        ("Position, soft gains", False, True),
        ("Impedance", True, False),
    ):
        trace = traces[label] = replay(urdf, impedance, soft_position=soft)
        motion = trace[2500:8500]  # Three full cycles after the first cycle settles.
        error = np.rad2deg(np.sqrt(np.mean((motion[:, 2:4] - motion[:, 1:2]) ** 2)))
        harmonic = np.exp(-1j * np.pi * motion[:, 0])
        # Fundamental phase delay at 0.5 Hz, averaged over both elbows.
        lag = round(
            -1000
            * np.angle((motion[:, 2:4].mean(axis=1) @ harmonic) / (motion[:, 1] @ harmonic))
            / np.pi
        )
        displacement = np.rad2deg(np.max(np.abs(trace[11000:12000, 2:4] - trace[10999, 2:4])))
        rows.append([label, f"{error:.2f}°", f"{lag:+d} ms", f"{displacement:.2f}°"])
        print(f"{label:20s} | {error:18.2f} | {lag:17d} | {displacement:28.2f}")
    if args.plot:
        import matplotlib.pyplot as plt

        fig, axes = plt.subplots(
            3,
            1,
            figsize=(9, 8),
            constrained_layout=True,
            gridspec_kw={"height_ratios": [2, 2, 0.9]},
        )
        for axis, window, title in zip(
            axes[:2],
            (slice(500, 8500), slice(10600, 12400)),
            ("Following a 0.5 Hz leader motion", "Response to a 1 Nm external elbow torque"),
            strict=True,
        ):
            axis.plot(trace[window, 0], np.rad2deg(trace[window, 1]), "k--", label="Leader")
            for label, trial in traces.items():
                axis.plot(trial[window, 0], np.rad2deg(trial[window, 2]), label=label)
            axis.set(title=title, xlabel="Time (s)", ylabel="Left elbow (deg)")
            axis.grid(alpha=0.2)
        axes[1].axvspan(11, 12, color="grey", alpha=0.15)
        axes[0].legend(ncol=4)
        axes[2].axis("off")
        table = axes[2].table(
            cellText=rows,
            colLabels=["Controller", "Tracking RMS", "Phase delay", "Peak deflection"],
            cellLoc="center",
            loc="center",
        )
        table.auto_set_font_size(False)
        table.set_fontsize(10)
        table.scale(1, 1.3)
        axes[2].set_title("Both elbows  ·  0.5 Hz tracking  ·  1 Nm load")
        fig.suptitle("R1 Pro JoyLo · fixed-torso simulation")
        fig.savefig(args.plot)
        plt.close(fig)


if __name__ == "__main__":
    main()
