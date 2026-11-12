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

"""Closed-loop evidence for the OpenYAM policy blueprint (``demo_`` = manual run).

This runs the *shipped* blueprint - ``openyam_policy_sim_replay`` - so the
evidence comes from the same entry point an operator launches: scripted WebXR
input module -> coordinator with ``OpenYamPolicyTask`` at 30 Hz -> exported ONNX
actor -> ``sim_mujoco`` adapter -> the trained MuJoCo plant -> SHM joint
feedback. Nothing here computes an action on the side.

While it runs, it samples the plant's own shared-memory joint state and the
task's ``state_snapshot`` at the policy rate, then:

* writes ``demo.mp4``, rendered offscreen from the recorded closed-loop joint
  trajectory of this run (the same q the plant reached, not a re-simulation);
* writes ``closed_loop.npz`` with every sampled tick;
* prints a JSON summary with the actor rate, joint tracking error and the three
  TCP quantities kept apart on purpose: *requested* (operator target in the
  deployed URDF tool frame), *achievable* (IK projection of that target) and
  *executed* (plant ``arm_tcp`` body, the frame the actor was trained on).

Usage::

    MUJOCO_GL=egl PYTHONPATH=... python -m \\
      dimos.robot.manipulators.openyam.policy.demo_policy_sim \\
      --model-dir <dir> --output-dir <dir>
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import time
from typing import Any

import numpy as np
from numpy.typing import NDArray

from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.robot.manipulators.openyam.blueprints.policy import (
    POLICY_TASK_NAME,
    openyam_policy_sim_replay,
    resolve_manifest_path,
    resolve_plant_path,
)
from dimos.robot.manipulators.openyam.policy.contract import JOINT_ORDER
from dimos.robot.manipulators.openyam.policy.tcp import MjcfTcpFk
from dimos.simulation.engines.mujoco_shm import ManipShmReader, shm_key_from_path

#: Sampling rate for the log; the policy itself still runs at its trained 30 Hz.
SAMPLE_HZ = 60.0


def _as_array(value: Any, width: int) -> NDArray[np.float64]:
    if value is None:
        return np.full(width, np.nan)
    array = np.asarray(value, dtype=np.float64).reshape(-1)
    return array if array.shape == (width,) else np.full(width, np.nan)


def _render_video(
    plant: Path,
    q_track: NDArray[np.float64],
    goal_track: NDArray[np.float64],
    output: Path,
    *,
    fps: float,
    width: int = 640,
    height: int = 360,
) -> str:
    """Render the recorded closed-loop trajectory offscreen into an mp4."""
    import cv2
    import mujoco

    model = mujoco.MjModel.from_xml_path(str(plant))
    data = mujoco.MjData(model)
    qpos_adr = [
        int(model.joint(name).qposadr[0]) for name in (f"arm_joint_{i}" for i in range(1, 7))
    ]
    tcp_body = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "arm_tcp"))
    camera = mujoco.MjvCamera()
    camera.lookat[:] = (0.0, 0.0, 0.35)
    camera.distance = 1.5
    camera.azimuth = 140.0
    camera.elevation = -15.0
    writer = cv2.VideoWriter(str(output), cv2.VideoWriter_fourcc(*"mp4v"), fps, (width, height))
    if not writer.isOpened():
        raise RuntimeError(f"could not open video writer for {output}")
    try:
        with mujoco.Renderer(model, height=height, width=width) as renderer:
            for index, q in enumerate(q_track):
                if not np.isfinite(q).all():
                    continue
                data.qpos[qpos_adr] = q
                mujoco.mj_forward(model, data)
                renderer.update_scene(data, camera=camera)
                frame = renderer.render()
                image = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                goal = goal_track[index]
                if np.isfinite(goal).all():
                    error = float(np.linalg.norm(np.asarray(data.xpos[tcp_body]) - goal))
                    label = f"t={index / fps:5.2f}s  goal-tcp err={error * 1000:5.1f} mm"
                else:
                    label = f"t={index / fps:5.2f}s  (session disengaged)"
                cv2.putText(image, label, (12, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (20, 20, 20), 3)
                cv2.putText(
                    image, label, (12, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (245, 245, 245), 1
                )
                writer.write(image)
    finally:
        writer.release()
    return str(output)


def run(args: argparse.Namespace) -> dict[str, Any]:
    """Build the replay blueprint, sample the closed loop, write evidence."""
    manifest = resolve_manifest_path(args.model_dir, args.manifest_path)
    model_dir = Path(args.model_dir) if args.model_dir else manifest.parent
    plant = resolve_plant_path(model_dir, manifest)
    replay_config: dict[str, Any] = {}
    if args.freeze_after_s is not None:
        replay_config["freeze_after_s"] = args.freeze_after_s
    if args.restart_after_s is not None:
        replay_config["restart_after_s"] = args.restart_after_s
    if args.release_after_s is not None:
        replay_config["release_after_s"] = args.release_after_s
    blueprint = openyam_policy_sim_replay(
        model_dir=model_dir,
        manifest_path=manifest,
        headless=True,
        replay_config=replay_config or None,
    )

    coordinator = ModuleCoordinator.build(blueprint)
    control = coordinator.get_instance("ControlCoordinator")
    reader = ManipShmReader.wait_for_live(
        shm_key_from_path(str(plant)),
        attach_timeout_s=60.0,
        ready_timeout_s=60.0,
        poll_s=0.1,
    )
    if reader is None:
        coordinator.stop()
        raise RuntimeError("the MuJoCo sim never published joint state")

    kinematics = MjcfTcpFk(plant)
    samples: dict[str, list[float]] = {
        name: []
        for name in (
            "t",
            "q0",
            "q1",
            "q2",
            "q3",
            "q4",
            "q5",
            "dq0",
            "dq1",
            "dq2",
            "dq3",
            "dq4",
            "dq5",
            "cmd0",
            "cmd1",
            "cmd2",
            "cmd3",
            "cmd4",
            "cmd5",
            "goal_x",
            "goal_y",
            "goal_z",
            "exec_x",
            "exec_y",
            "exec_z",
            "req_x",
            "req_y",
            "req_z",
            "ach_x",
            "ach_y",
            "ach_z",
            "dt_s",
            "projection_m",
            "projection_rad",
            "active",
            "policy_ticks",
            "epoch",
            "shm_seq",
            "clip_rad",
            "deadman",
            "fault_code",
        )
    }
    faults: list[str] = []
    events: list[dict[str, Any]] = []
    previous_epoch: float | None = None
    previous_session: str | None = None
    started = time.monotonic()
    period = 1.0 / SAMPLE_HZ
    deadline = started
    estop_done = False
    while time.monotonic() - started < args.duration:
        tick_started = time.monotonic()
        elapsed = tick_started - started
        if args.estop_at is not None and not estop_done and elapsed >= args.estop_at:
            estop_done = True
            try:
                control.task_invoke(
                    task_name=POLICY_TASK_NAME,
                    method="set_estop",
                    kwargs={"estopped": True},
                )
                events.append({"t": round(elapsed, 3), "event": "estop", "applied": True})
            except Exception as exc:
                events.append({"t": round(elapsed, 3), "event": "estop", "error": repr(exc)})
        q = np.asarray(reader.read_positions(len(JOINT_ORDER)), dtype=np.float64)
        dq = np.asarray(reader.read_velocities(len(JOINT_ORDER)), dtype=np.float64)
        seq = int(reader.position_sequence())
        snapshot: dict[str, Any] = {}
        try:
            snapshot = (
                control.task_invoke(task_name=POLICY_TASK_NAME, method="state_snapshot", kwargs={})
                or {}
            )
        except Exception as exc:
            faults.append(f"state_snapshot: {type(exc).__name__}: {exc}")
        executed, _ = kinematics(q)
        reference = snapshot.get("reference") or {}
        values = {
            "t": [time.monotonic() - started],
            **{f"q{i}": [q[i]] for i in range(6)},
            **{f"dq{i}": [dq[i]] for i in range(6)},
            **{
                f"cmd{i}": [[float(x) for x in _as_array(snapshot.get("last_command"), 6)][i]]
                for i in range(6)
            },
            "goal_x": [(snapshot.get("goal_position") or [np.nan] * 3)[0]],
            "goal_y": [(snapshot.get("goal_position") or [np.nan] * 3)[1]],
            "goal_z": [(snapshot.get("goal_position") or [np.nan] * 3)[2]],
            "exec_x": [executed[0]],
            "exec_y": [executed[1]],
            "exec_z": [executed[2]],
            "req_x": [(reference.get("requested_position") or [np.nan] * 3)[0]],
            "req_y": [(reference.get("requested_position") or [np.nan] * 3)[1]],
            "req_z": [(reference.get("requested_position") or [np.nan] * 3)[2]],
            "ach_x": [(reference.get("achievable_position") or [np.nan] * 3)[0]],
            "ach_y": [(reference.get("achievable_position") or [np.nan] * 3)[1]],
            "ach_z": [(reference.get("achievable_position") or [np.nan] * 3)[2]],
            "dt_s": [float(reference.get("dt_s", np.nan))],
            "projection_m": [float(reference.get("projection_error_m", np.nan))],
            "projection_rad": [float(reference.get("projection_error_rad", np.nan))],
            "active": [1.0 if snapshot.get("session") == "active" else 0.0],
            "policy_ticks": [float(snapshot.get("policy_ticks", np.nan))],
            "epoch": [float(snapshot.get("epoch", np.nan))],
            "shm_seq": [float(seq)],
            "clip_rad": [float(reference.get("envelope_clip_rad", np.nan))],
            "deadman": [1.0 if snapshot.get("deadman") else 0.0],
            "fault_code": [
                float(
                    [
                        "none",
                        "feedback",
                        "intent",
                        "reference",
                        "inference",
                        "preempted",
                        "estop",
                        "released",
                        "stopped",
                        "reset",
                        "episode",
                    ].index(str(snapshot.get("fault", "none")))
                )
            ],
        }
        for key, value in values.items():
            samples[key].extend(value)
        session_now = str(snapshot.get("session"))
        epoch_now = snapshot.get("epoch")
        if session_now != previous_session or epoch_now != previous_epoch:
            events.append(
                {
                    "t": round(values["t"][0], 3),
                    "event": "session",
                    "session": session_now,
                    "epoch": epoch_now,
                    "fault": snapshot.get("fault"),
                    "policy_ticks": snapshot.get("policy_ticks"),
                }
            )
            previous_session = session_now
            previous_epoch = epoch_now if epoch_now is None else float(epoch_now)
        fault = snapshot.get("fault")
        if fault and fault not in ("none",):
            faults.append(str(fault))
        deadline += period
        sleep_s = deadline - time.monotonic()
        if sleep_s > 0:
            time.sleep(sleep_s)
        else:
            deadline = max(deadline, time.monotonic())
        del tick_started

    coordinator.stop()

    arrays = {key: np.asarray(values, dtype=np.float64) for key, values in samples.items()}
    output = Path(args.output_dir)
    output.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(output / "closed_loop.npz", **arrays)

    q_track = np.stack([arrays[f"q{i}"] for i in range(6)], axis=1)
    goal_track = np.stack([arrays["goal_x"], arrays["goal_y"], arrays["goal_z"]], axis=1)
    exec_track = np.stack([arrays["exec_x"], arrays["exec_y"], arrays["exec_z"]], axis=1)
    active = arrays["active"] > 0.5

    duration = float(arrays["t"][-1] - arrays["t"][0]) if len(arrays["t"]) > 1 else 0.0
    ticks = arrays["policy_ticks"][active]
    ticks = ticks[np.isfinite(ticks)]
    ticks_first = float(ticks.min()) if ticks.size else 0.0
    ticks_last = float(ticks.max()) if ticks.size else 0.0
    summary: dict[str, Any] = {
        "input_source": "input_replay (scripted WebXR controller, not a headset session)",
        "plant_model": str(plant),
        "manifest": str(manifest),
        "samples": len(arrays["t"]),
        "wall_duration_s": round(duration, 3),
        "session_active_samples": int(active.sum()),
        "policy_ticks_observed": int(ticks_last - ticks_first) if ticks_last >= ticks_first else 0,
        "faults": sorted(set(faults)),
        "events": events,
        "physics": {},
        "estop_at_s": args.estop_at,
    }
    seq = arrays["shm_seq"]
    if len(seq) > 1 and duration > 0:
        steps = float(seq[-1] - seq[0])
        summary["physics"] = {
            "shm_state_sequence_delta": steps,
            "implied_hz": steps / duration,
            "note": "MuJoCo publishes its state once per physics step; configured "
            "timestep is pinned by the trained plant xml.",
        }
    if active.any():
        dt = arrays["dt_s"][active]
        dt = dt[np.isfinite(dt)]
        joint_track = np.linalg.norm(
            np.stack([arrays[f"cmd{i}"] for i in range(6)], axis=1) - q_track, axis=1
        )[active]
        goal_error = np.linalg.norm(goal_track - exec_track, axis=1)[active]
        projection = arrays["projection_m"][active]
        summary["command_tracking"] = {
            "joint_rmse_rad": float(np.sqrt(np.nanmean(joint_track**2))),
            "joint_max_rad": float(np.nanmax(joint_track)),
        }
        summary["tcp_goal_vs_executed"] = {
            "mean_mm": float(np.nanmean(goal_error) * 1000.0),
            "max_mm": float(np.nanmax(goal_error) * 1000.0),
        }
        summary["requested_vs_achievable"] = {
            "mean_mm": float(np.nanmean(projection) * 1000.0),
            "max_mm": float(np.nanmax(projection) * 1000.0),
            "max_rad": float(np.nanmax(arrays["projection_rad"][active])),
        }
        summary["reference_dt"] = {
            "samples": int(dt.size),
            "mean_s": float(dt.mean()),
            "min_s": float(dt.min()),
            "max_s": float(dt.max()),
            "implied_hz": float(1.0 / dt.mean()) if dt.mean() > 0 else math.inf,
        }
        # Actor cadence measured per session: ticks are counted from the
        # reference generator, which restarts at every new session.
        epochs = arrays["epoch"][active]
        tick_values = arrays["policy_ticks"][active]
        times = arrays["t"][active]
        per_epoch_ticks = 0
        per_epoch_time = 0.0
        for epoch in np.unique(epochs[np.isfinite(epochs)]):
            mask = (epochs == epoch) & np.isfinite(tick_values)
            if not mask.any():
                continue
            per_epoch_ticks += int(np.nanmax(tick_values[mask]) - np.nanmin(tick_values[mask]) + 1)
            session_t = times[mask]
            if session_t.size > 1:
                per_epoch_time += float(session_t[-1] - session_t[0])
        if per_epoch_time > 0 and per_epoch_ticks:
            summary["actor_cadence"] = {
                "policy_ticks": per_epoch_ticks,
                "active_time_s": per_epoch_time,
                "measured_hz": per_epoch_ticks / per_epoch_time,
            }
        clip = arrays["clip_rad"][active]
        clip = clip[np.isfinite(clip)]
        summary["envelope_clip"] = {
            "samples": int(clip.size),
            "max_rad": float(clip.max()) if clip.size else 0.0,
            "note": "IK solution clipped into the trained joint envelope "
            "(sim_lower+0.015 .. sim_upper-0.015)",
        }
        moved = float(np.max(np.linalg.norm(exec_track[active] - exec_track[active][0], axis=1)))
        summary["executed_tcp_travel_m"] = moved

    if not args.no_video:
        summary["video"] = _render_video(
            plant, q_track, goal_track, output / "demo.mp4", fps=SAMPLE_HZ
        )
    (output / "closed_loop.json").write_text(json.dumps(summary, indent=2) + "\n")
    return summary


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model-dir", default=None)
    parser.add_argument("--manifest-path", default=None)
    parser.add_argument("--output-dir", default="artifacts/openyam_policy/evidence")
    parser.add_argument("--duration", type=float, default=9.0)
    parser.add_argument(
        "--estop-at", type=float, default=None, help="send the e-stop this many seconds in"
    )
    parser.add_argument(
        "--freeze-after-s", type=float, default=None, help="replay stops publishing here"
    )
    parser.add_argument(
        "--release-after-s", type=float, default=None, help="replay releases the deadman here"
    )
    parser.add_argument(
        "--restart-after-s", type=float, default=None, help="replay re-presses the deadman here"
    )
    parser.add_argument("--no-video", action="store_true")
    args = parser.parse_args()
    summary = run(args)
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
