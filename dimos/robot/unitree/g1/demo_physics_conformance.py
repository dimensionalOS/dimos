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

"""Run the same real G1 policy and command checks through either physics engine.

    python -m dimos.robot.unitree.g1.demo_physics_conformance --engine mujoco
    python -m dimos.robot.unitree.g1.demo_physics_conformance --engine genesis

Requires the selected engine and the existing G1 LFS mesh/policy assets. No
renderer or robot hardware is started. JSON includes measured values and any
failed acceptance checks; failure exits nonzero. This checks the learned stock
surrogate, external motor commands and the real GR00T coordinator/connection
workers. No vendor firmware is emulated.
"""

from __future__ import annotations

import argparse
from contextlib import ExitStack
from dataclasses import asdict, replace
from importlib import import_module
import json
import os
from pathlib import Path
from tempfile import TemporaryDirectory
import threading
import time
from typing import Any, cast
from uuid import uuid4

import numpy as np

from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.core.global_config import global_config
from dimos.core.transport import PubSubTransport
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.sensor_msgs.MotorCommandArray import MotorCommandArray
from dimos.robot.unitree.g1.connection import G1ConnectionConfig
from dimos.robot.unitree.g1.groot_config import G1_GROOT_HOME, G1_GROOT_KD, G1_GROOT_KP
from dimos.robot.unitree.g1.model import describe_simulated_robot
from dimos.sim2.backends.factory import create_backend
from dimos.sim2.module import SimulationModule
from dimos.sim2.physics import BodyWrench, CommandBatch, WorldState
from dimos.sim2.spec import RobotInstanceSpec
from dimos.sim2.world.runtime import SimulationRuntime


def _robots(*, motors: bool = False) -> dict[str, RobotInstanceSpec]:
    config = G1ConnectionConfig(
        control_mode="motors" if motors else "stock",
        initial_qpos=tuple(G1_GROOT_HOME) if motors else (),
        startup_kp=tuple(G1_GROOT_KP) if motors else (),
        startup_kd=tuple(G1_GROOT_KD) if motors else (),
    )
    robot = describe_simulated_robot(config)
    return {
        name: replace(robot, robot_id=name, position=(x, 0.0, 0.0))
        for name, x in (("left", 0.0), ("right", 2.0))
    }


def _state_errors(state: WorldState, robots: dict[str, RobotInstanceSpec]) -> list[str]:
    errors = []
    if set(state.robots) != set(robots):
        errors.append("robot identities changed")
    for name, robot in state.robots.items():
        if robot.joint_names != robots[name].joint_names:
            errors.append(f"{name}: declared motor ordering changed")
        if len(robot.position) != 29 or len(robot.velocity) != 29 or len(robot.effort) != 29:
            errors.append(f"{name}: expected 29 motor observations")
        if robot.imu is None:
            errors.append(f"{name}: missing control IMU")
        if not np.isfinite(
            (*robot.position, *robot.velocity, *robot.effort, *robot.base.position)
        ).all():
            errors.append(f"{name}: nonfinite robot state")
    return errors


def stock_conformance(engine: str, scene: Path) -> dict[str, Any]:
    robots = _robots()
    errors: list[str] = []
    with SimulationRuntime(robots=robots, backend=create_backend(engine, robots, scene)) as runtime:
        initial = runtime.last_state
        errors.extend(_state_errors(initial, robots))
        # Compare identical one-step trials, with and without a world-frame push.
        runtime.step()
        baseline = runtime.last_state.robots["left"].base.linear_velocity[0]
        runtime.reset()
        force = BodyWrench(runtime.backend.description.robot_roots["left"], force=(100.0, 0.0, 0.0))
        report = runtime.submit(CommandBatch("push", runtime.episode, (force,)))
        if report.status != "accepted":
            errors.append(f"body force rejected: {report.reason}")
        runtime.step()
        pushed = runtime.last_state.robots["left"].base.linear_velocity[0]
        impulse_response = float(pushed - baseline)
        if impulse_response <= 0.001:
            errors.append("positive world force did not increase forward velocity")

        runtime.reset()
        for _ in range(round(1.5 / runtime.step_seconds)):
            runtime.step()
        settled = runtime.last_state
        for _ in range(round(4.5 / runtime.step_seconds)):
            # Refresh command ownership independently of each engine's wall speed.
            if not runtime.accept_velocity("left", Twist(linear=(0.15, 0.0, 0.0))):
                raise RuntimeError("stock connection rejected its velocity command")
            if not runtime.accept_velocity("right", Twist()):
                raise RuntimeError("stock connection rejected zero velocity")
            runtime.step()
        walked = runtime.last_state
        errors.extend(_state_errors(walked, robots))
        displacement = {
            name: np.subtract(robot.base.position, settled.robots[name].base.position).tolist()
            for name, robot in walked.robots.items()
        }
        heights = {name: float(robot.base.position[2]) for name, robot in walked.robots.items()}
        if displacement["left"][0] < 0.15:
            errors.append("forward stock command did not produce at least 0.15 m of progress")
        if np.linalg.norm(displacement["right"][:2]) > 0.4:
            errors.append("zero-command stock robot drifted more than 0.4 m")
        if any(height < 0.5 for height in heights.values()):
            errors.append("a stock robot fell below 0.5 m pelvis height")
        runtime.reset()
        reset_error = max(
            float(
                np.max(
                    np.abs(
                        np.subtract(
                            (*robot.position, *robot.base.position),
                            (*initial.robots[name].position, *initial.robots[name].base.position),
                        )
                    )
                )
            )
            for name, robot in runtime.last_state.robots.items()
        )
        if reset_error > 1e-6:
            errors.append("reset did not restore the authored robot poses")
        return {
            "engine": engine,
            "scenario": "stock",
            "sim_seconds": 6.0,
            "walk_displacement_m": displacement,
            "final_pelvis_height_m": heights,
            "force_velocity_delta_m_s": impulse_response,
            "reset_max_error": reset_error,
            "errors": errors,
        }


def motor_conformance(engine: str, scene: Path) -> dict[str, Any]:
    robots = _robots(motors=True)
    with SimulationRuntime(robots=robots, backend=create_backend(engine, robots, scene)) as runtime:
        initial = runtime.last_state
        errors = _state_errors(initial, robots)
        target = list(G1_GROOT_HOME)
        target[-1] += 0.2  # Right wrist yaw; the other robot remains at its home target.
        for _ in range(round(0.2 / runtime.step_seconds)):
            for name, q in (("left", target), ("right", G1_GROOT_HOME)):
                command = MotorCommandArray(q=q, kp=G1_GROOT_KP, kd=G1_GROOT_KD)
                if not runtime.accept_command(name, command):
                    raise RuntimeError(f"{name}: external motor command was rejected")
            runtime.step()
        state = runtime.last_state
        errors.extend(_state_errors(state, robots))
        response = float(state.robots["left"].position[-1] - state.robots["right"].position[-1])
        if response < 0.02:
            errors.append("independent wrist target did not move the commanded robot's wrist")
        return {
            "engine": engine,
            "scenario": "motors",
            "sim_seconds": 0.2,
            "commanded_wrist_delta_rad": 0.2,
            "measured_wrist_difference_rad": response,
            "errors": errors,
        }


def groot_conformance(engine: str, scene: Path) -> dict[str, Any]:
    """Exercise the real coordinator, connection and SimRobotClient in workers."""
    domain = f"g1-physics-{uuid4().hex[:10]}"
    bucket = int(uuid4().hex[:4], 16) % 5000
    os.environ.update(
        SIMULATION=engine,
        VIEWER="none",
        ZENOH_DOMAIN=domain,
        ZENOH_SCOUT_ADDR=f"224.0.0.224:{37000 + bucket}",
        LCM_DEFAULT_URL=f"udpm://239.255.76.67:{27000 + bucket}?ttl=0",
    )
    global_config.simulation = engine
    global_config.viewer = "none"
    global_config.transport = "zenoh"
    global_config.zenoh_domain = domain
    global_config.zenoh_scout_addr = os.environ["ZENOH_SCOUT_ADDR"]
    global_config.scene_package = str(scene)
    # The blueprint selects its hardware/simulation arming profile at import.
    g1 = import_module("dimos.robot.unitree.g1.blueprints.basic.unitree_g1_groot_wbc")
    blueprint = g1.unitree_g1_groot_control
    parsed = BlueprintConfigParser(blueprint).parse(
        [],
        environ={},
        global_overrides={
            "simulation": engine,
            "viewer": "none",
            "transport": "zenoh",
            "zenoh_domain": domain,
            "zenoh_scout_addr": global_config.zenoh_scout_addr,
            "scene_package": str(scene),
        },
    )
    with ExitStack() as resources:
        app = ModuleCoordinator.build(blueprint, parsed)
        resources.callback(app.stop)
        commands = cast("PubSubTransport[Twist]", app.transports["cmd_vel", Twist])
        commands.start()
        resources.callback(commands.stop)
        world = app.get_instance(SimulationModule)
        coordinator = app.get_instance("ControlCoordinator")
        connection = app.get_instance(g1.G1Connection)
        start = world.status()
        initial = np.asarray(start["robots"]["g1"], dtype=float)
        initial_ticks = coordinator.get_tick_count()
        deadline = time.monotonic() + 90.0
        pace = threading.Event()
        while True:
            state = world.status()
            if state["error"]:
                raise RuntimeError(state)
            elapsed = state["sim_time"] - start["sim_time"]
            commands.publish(Twist(linear=(0.15 if elapsed > 1.5 else 0.0, 0.0, 0.0)))
            if elapsed >= 10.0:
                break
            if time.monotonic() > deadline:
                raise TimeoutError(f"GR00T did not complete ten simulation seconds: {state}")
            pace.wait(0.05)
        commands.publish(Twist())
        walked = world.status()
        connected = connection.status()
        ticks = coordinator.get_tick_count() - initial_ticks
        displacement = np.asarray(walked["robots"]["g1"], dtype=float) - initial
        errors = []
        if displacement[0] < 0.15:
            errors.append("GR00T forward command did not produce 0.15 m of progress")
        if walked["robots"]["g1"][2] < 0.5:
            errors.append("GR00T robot fell below 0.5 m pelvis height")
        if walked["rejected_commands"]:
            errors.append("simulation rejected coordinator motor commands")
        if (
            not connected.connected
            or connected.deadman_fired
            or connected.last_error
            or connected.last_rejection
        ):
            errors.append("G1 connection reports unhealthy control")
        if ticks < 200:
            errors.append("GR00T coordinator did not sustain its control ticks")
        world.set_paused(True)
        world.reset()
        reset = coordinator.reset_runtime_state(reactivate=True)
        world.set_paused(False)
        if not reset or not all(reset.values()):
            errors.append("GR00T coordinator reset failed")
        return {
            "engine": engine,
            "scenario": "groot",
            "sim_seconds": walked["sim_time"] - start["sim_time"],
            "walk_displacement_m": displacement.tolist(),
            "final_pelvis_height_m": float(walked["robots"]["g1"][2]),
            "coordinator_ticks": ticks,
            "connection": asdict(connected),
            "reset_tasks": reset,
            "errors": errors,
        }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--engine", choices=("mujoco", "genesis"), required=True)
    parser.add_argument("--scenario", choices=("stock", "motors", "groot", "all"), default="all")
    args = parser.parse_args()
    with TemporaryDirectory(prefix="g1-physics-conformance-") as directory:
        scene = Path(directory) / "floor.xml"
        scene.write_text(
            '<mujoco><worldbody><geom name="floor" type="plane" '
            'size="10 10 .1" friction="1 .005 .0001"/></worldbody></mujoco>'
        )
        reports = []
        if args.scenario in ("stock", "all"):
            reports.append(stock_conformance(args.engine, scene))
        if args.scenario in ("motors", "all"):
            reports.append(motor_conformance(args.engine, scene))
        if args.scenario in ("groot", "all"):
            reports.append(groot_conformance(args.engine, scene))
    print(json.dumps(reports, indent=2))
    if any(report["errors"] for report in reports):
        raise SystemExit(1)


if __name__ == "__main__":
    main()
