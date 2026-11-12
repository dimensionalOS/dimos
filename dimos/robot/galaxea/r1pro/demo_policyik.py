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

"""Run the frozen paired-hand PolicyIK controller in native Sim2 with a browser UI."""

import argparse
from contextlib import ExitStack
from datetime import datetime, timezone
import json
from pathlib import Path
from queue import Empty, Full, Queue
import signal
import threading
import time
from typing import Any
import uuid

import numpy as np
import viser
from viser.extras import ViserUrdf

from dimos.constants import STATE_DIR
from dimos.control.policyik.controller import DT_NS, PERIOD_NS
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.core.global_config import global_config
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.galaxea.r1pro.policyik_sim2 import PolicyIKSimModule
from dimos.sim2.module import SimModule
from dimos.sim2.physics.mujoco import MujocoBackend
from dimos.sim2.sensors.encoder.module import EncoderConfig, SimJointEncoderModule


def make_blueprint(bundle: Path, scene: Path, domain: str) -> Any:
    manifest = json.loads((bundle / "manifest.json").read_text())
    joints = tuple(manifest["joints"])
    return autoconnect(
        SimModule.blueprint(
            instance_name="policyik_simulation",
            scene=scene,
            backend=MujocoBackend,
            dt_ns=DT_NS,
            observation_hz=1e9 / DT_NS,
            joint_ids=joints,
        ),
        SimJointEncoderModule.blueprint(
            sensor=EncoderConfig(joints, hz=1e9 / PERIOD_NS),
        ),
        PolicyIKSimModule.blueprint(bundle=bundle),
    ).global_config(viewer="none", transport="zenoh", n_workers=3, zenoh_domain=domain)


def run(args: argparse.Namespace) -> None:
    bundle, scene = args.bundle.resolve(), args.scene.resolve()
    manifest = json.loads((bundle / "manifest.json").read_text())
    with np.load(bundle / "development_cases.npz", allow_pickle=False) as data:
        goals = np.concatenate((data["goal_pos"], data["goal_quat"]), axis=-1)
    output = args.output or STATE_DIR / "policyik" / datetime.now(timezone.utc).strftime(
        "%Y%m%dT%H%M%S%fZ"
    )
    output.mkdir(parents=True, exist_ok=False)
    domain = "policyik-demo-" + uuid.uuid4().hex
    (output / "configuration.json").write_text(
        json.dumps(
            {
                **vars(args),
                "bundle": str(bundle),
                "scene": str(scene),
                "output": str(output),
                "zenoh_domain": domain,
                "checkpoint_sha256": manifest["checkpoint_sha256"],
                "physics_dt_ns": DT_NS,
                "control_period_ns": PERIOD_NS,
            },
            default=str,
            indent=2,
        )
        + "\n"
    )
    done = threading.Event()
    signal.signal(signal.SIGTERM, lambda *_: done.set())
    signal.signal(signal.SIGINT, lambda *_: done.set())
    samples: Queue[JointState] = Queue(1)
    started = time.monotonic()
    final_status: dict[str, Any] = {}
    with ExitStack() as cleanup:
        deployment = ModuleCoordinator.build(make_blueprint(bundle, scene, domain))
        cleanup.callback(deployment.stop)
        sim = deployment.get_instance(SimModule)
        policy = deployment.get_instance(PolicyIKSimModule)
        transport = deployment.transports[("joint_state", JointState)]

        def capture(sample: JointState) -> None:
            try:
                samples.put_nowait(sample)
            except Full:
                try:
                    samples.get_nowait()
                except Empty:
                    pass
                samples.put_nowait(sample)

        cleanup.callback(transport.subscribe(capture))
        policy.set_goals(goals[args.case].tolist(), manifest["frame"])
        viewer = None
        controls = []
        status_widget = None
        if not args.no_viewer:
            server = viser.ViserServer(
                host=args.host, port=args.port, label="R1 Pro · PolicyIK / Sim2"
            )
            cleanup.callback(server.stop)
            server.scene.set_up_direction("+z")
            server.scene.add_grid("/floor", width=4, height=4, plane="xy")
            viewer = ViserUrdf(server, bundle / "robot.urdf", root_node_name="/r1pro")
            server.gui.add_markdown(
                "## R1 Pro · coordinated PolicyIK\n"
                "One learned controller moves **both arms and the torso**. "
                "Drag either target frame, then **Apply paired targets**. "
                "The solid robot shows measured encoder feedback.\n\n"
                "Fixed chassis · grippers fixed · simulation only. "
                "Targets are in the chassis frame; arbitrary dragged goals may be unreachable.\n\n"
                "**Engine transfer preview:** this policy was trained in PhysX. "
                "MuJoCo does not enforce the training plant's hard joint-speed caps. "
                "Do not interpret this preview as hardware or native-benchmark qualification. "
                "Pause freezes the last joint references with the servos active; "
                "clearing Pause resumes the current paired goals."
            )
            for hand, goal in zip(("left", "right"), goals[args.case], strict=True):
                controls.append(
                    server.scene.add_transform_controls(
                        f"/targets/{hand}",
                        position=goal[:3],
                        wxyz=goal[3:],
                        scale=0.13,
                    )
                )
                server.scene.add_label(f"/targets/{hand}/label", text=hand.capitalize() + " goal")
            status_widget = server.gui.add_markdown("Waiting for encoder feedback…")
            apply = server.gui.add_button("Apply paired targets")
            case = server.gui.add_slider(
                "Development target pair",
                min=0,
                max=len(goals) - 1,
                step=1,
                initial_value=args.case,
            )
            load_case = server.gui.add_button("Load selected target pair")
            paused = server.gui.add_checkbox("Pause policy / hold references", initial_value=False)

            @apply.on_click
            def apply_targets(_: Any) -> None:
                values = [list(c.position) + list(c.wxyz) for c in controls]
                policy.set_goals(values, manifest["frame"])

            @load_case.on_click
            def load_targets(_: Any) -> None:
                values = goals[int(case.value)]
                with server.atomic():
                    for control, goal in zip(controls, values, strict=True):
                        control.position, control.wxyz = goal[:3], goal[3:]
                policy.set_goals(values.tolist(), manifest["frame"])

            @paused.on_update
            def pause_policy(_: Any) -> None:
                policy.set_paused(bool(paused.value))

            @server.on_client_connect
            def connect(client: viser.ClientHandle) -> None:
                client.camera.position = (2.5, 2.2, 1.8)
                client.camera.look_at = (0.0, 0.0, 0.9)
                client.camera.up_direction = (0.0, 0.0, 1.0)

        trace = cleanup.enter_context((output / "telemetry.jsonl").open("w"))
        print(
            json.dumps(
                {
                    "viewer": None if args.no_viewer else f"http://{args.host}:{args.port}",
                    "output": str(output),
                    "zenoh_domain": domain,
                }
            ),
            flush=True,
        )
        started = time.monotonic()
        deadline = started
        while not done.is_set() and (
            args.duration is None or time.monotonic() - started < args.duration
        ):
            try:
                sample = samples.get(timeout=0.2)
            except Empty:
                state = sim.status()
                if state["status"] == "fault":
                    raise RuntimeError(state)
                continue
            if viewer is not None:
                positions = dict(zip(sample.name, sample.position, strict=True))
                viewer.update_cfg(
                    np.array([positions[n] for n in viewer.get_actuated_joint_limits()])
                )
            now = time.monotonic()
            if now >= deadline:
                state, plant = policy.status(), sim.status()
                final_status = {
                    "policy": state,
                    "simulation": plant,
                    "encoder_time": sample.ts,
                    "joint_positions": sample.position,
                    "wall_seconds": now - started,
                }
                trace.write(json.dumps(final_status) + "\n")
                trace.flush()
                (output / "latest.json").write_text(json.dumps(final_status, indent=2) + "\n")
                if plant["status"] == "fault":
                    raise RuntimeError(plant)
                if status_widget is not None and "position_error_m" in state:
                    p = np.asarray(state["position_error_m"]) * 1000
                    r = state["orientation_error_deg"]
                    status_widget.content = (
                        f"**{'Last inference error (paused)' if state.get('paused') else 'Measured error'}**\n\n"
                        f"| | Left | Right |\n|---|---:|---:|\n"
                        f"| Position | {p[0]:.2f} mm | {p[1]:.2f} mm |\n"
                        f"| Rotation | {r[0]:.2f}° | {r[1]:.2f}° |\n\n"
                        f"Policy ticks: {state['inferences']} · "
                        f"{'PAUSED' if state.get('paused') else 'RUNNING'} · "
                        f"Simulation: {plant['real_time_factor']:.2f}x real time\n\n"
                        "The training criterion was both hands below 1 mm / 0.2° "
                        "over the final second, excluding constraint failures. "
                        "This live error display does not certify collision safety."
                    )
                deadline = now + 0.2
        policy.set_estop(True)
    (output / "summary.json").write_text(json.dumps(final_status, indent=2) + "\n")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bundle", type=Path, required=True)
    parser.add_argument("--scene", type=Path, required=True)
    parser.add_argument("--case", type=int, default=0)
    parser.add_argument("--host", default=global_config.listen_host)
    parser.add_argument("--port", type=int, default=8875)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--duration", type=float)
    parser.add_argument("--no-viewer", action="store_true")
    args = parser.parse_args()
    run(args)


if __name__ == "__main__":
    main()
