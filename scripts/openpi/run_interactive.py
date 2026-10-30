#!/usr/bin/env python3
"""Start the real-arm OpenPI run and control it with t/r keys."""

from __future__ import annotations

import argparse
import math
import os
from pathlib import Path
import socket
import subprocess
import sys
import termios
import time
import tty

from dimos.core.run_registry import get_most_recent
from dimos.msgs.trajectory_msgs.JointTrajectory import JointTrajectory
from dimos.msgs.trajectory_msgs.TrajectoryPoint import TrajectoryPoint
from dimos.msgs.trajectory_msgs.TrajectoryStatus import TrajectoryState
from dimos.porcelain.dimos import Dimos
from dimos.robot.manipulators.dual_openyam.joints import DUAL_OPENYAM_ARM_JOINTS

ROOT = Path(__file__).resolve().parents[2]
DIMOS = ROOT / ".venv/bin/dimos"
POLICY_CONTROL = Path(__file__).with_name("policy_control.py")


def control(command: str) -> bool:
    result = subprocess.run([sys.executable, str(POLICY_CONTROL), command], cwd=ROOT, check=False)
    return result.returncode == 0


def reset_arms_to_zero(*, resume_policy: bool) -> None:
    """Open both grippers and move the arms to zero while the policy is paused."""
    app = Dimos.connect()
    try:
        client = app.OpenPIClient
        coordinator = app.ControlCoordinator
        deadline = time.monotonic() + 10.0
        while not client.is_paused():
            if time.monotonic() >= deadline:
                raise RuntimeError("Policy did not pause; refusing reset")
            time.sleep(0.1)

        if not resume_policy:
            hold = coordinator.task_invoke("operator_hold", "get_status")
            if hold is None or not hold.on or hold.route != "manual":
                raise RuntimeError("Expected a manual operator hold before exit reset")

        positions = coordinator.get_joint_positions()
        joints = list(DUAL_OPENYAM_ARM_JOINTS)
        if any(name not in positions or not math.isfinite(positions[name]) for name in joints):
            raise RuntimeError("Cannot read all twelve arm joints; refusing reset")
        start = [positions[name] for name in joints]
        duration = max(1.0, max(abs(value) for value in start) / 0.25)
        if not resume_policy:
            released = coordinator.task_invoke("operator_hold", "acknowledge")
            if released is None or released.on:
                raise RuntimeError("Could not release operator hold for exit reset")
        for side in ("left", "right"):
            if not coordinator.task_invoke(f"{side}_gripper", "set_normalized", {"values": [1.0]}):
                raise RuntimeError(f"Could not command {side} gripper open")
        deadline = time.monotonic() + 10.0
        while True:
            openings = [
                coordinator.task_invoke(f"{side}_gripper", "get_normalized")
                for side in ("left", "right")
            ]
            if all(
                opening is not None
                and len(opening) == 1
                and math.isfinite(opening[0])
                and opening[0] >= 0.95
                for opening in openings
            ):
                break
            if time.monotonic() >= deadline:
                raise RuntimeError(f"Grippers did not open: measured {openings}")
            time.sleep(0.1)
        print("Both grippers open.")
        trajectory = JointTrajectory(
            joint_names=joints,
            points=[
                TrajectoryPoint(time_from_start=0.0, positions=start),
                TrajectoryPoint(time_from_start=duration, positions=[0.0] * len(joints)),
            ],
        )
        result = coordinator.execute_trajectory(trajectory)
        if result.status.name != "ACCEPTED":
            raise RuntimeError(f"Zero trajectory rejected: {result.status.name}: {result.message}")
        print(f"Moving both arms to zero over approximately {duration:.1f} seconds...")
        deadline = time.monotonic() + duration + 15.0
        while True:
            status = coordinator.task_invoke("joint_trajectory", "get_status", {"t_now": None})
            if status.state == TrajectoryState.COMPLETED:
                break
            if status.state != TrajectoryState.EXECUTING or time.monotonic() >= deadline:
                raise RuntimeError(f"Zero trajectory did not complete: {status.state}")
            time.sleep(0.2)
        measured = coordinator.get_joint_positions()
        worst_error = max(abs(measured[name]) for name in joints)
        if worst_error > 0.05:
            raise RuntimeError(
                f"Zero trajectory completed but measured error is {worst_error:.3f} rad"
            )
        print(f"Both arms reached zero (maximum measured error {worst_error:.3f} rad).")
        if resume_policy:
            if not client.resume():
                raise RuntimeError("Arms reached zero, but policy inference could not start")
            print("Policy inference started.")
        else:
            hold = coordinator.task_invoke(
                "operator_hold",
                "request",
                {"route": "manual", "reason": "OpenPI console closed at zero pose"},
            )
            if hold is None or not hold.on:
                raise RuntimeError("Arms reached zero, but operator hold was not engaged")
            print("Policy remains paused; operator hold engaged at zero pose.")
    except BaseException:
        # Keep the controller and motor torque active; hold the measured pose
        # if initialization or the zero trajectory fails.
        try:
            app.ControlCoordinator.task_invoke(
                "operator_hold", "request", {"route": "manual", "reason": "Reset failed"}
            )
        except Exception:
            print("Could not engage operator hold; inspect the robot immediately.", file=sys.stderr)
        raise
    finally:
        app.stop()


def pause_and_reset() -> bool:
    try:
        if not control("pause"):
            return False
        reset_arms_to_zero(resume_policy=False)
    except KeyboardInterrupt:
        print("Exit reset interrupted; check policy and operator hold state.", file=sys.stderr)
        return False
    except Exception as exc:
        print(f"Exit reset failed: {exc}", file=sys.stderr)
        return False
    return True


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--reset", action="store_true", help="Move both arms to literal zero before inference"
    )
    args = parser.parse_args()
    if not sys.stdin.isatty():
        raise SystemExit("Run this from an interactive terminal.")
    if get_most_recent(alive_only=True) is not None:
        raise SystemExit("A dimOS run is already active. Stop it before starting another.")
    try:
        with socket.create_connection(("127.0.0.1", 8000), timeout=2.0):
            pass
    except OSError as exc:
        raise SystemExit("OpenPI server is not listening on 127.0.0.1:8000.") from exc

    child_env = os.environ.copy()
    child_env["DIMOS_OPENPI_START_PAUSED"] = "1" if args.reset else "0"
    subprocess.run(
        [str(DIMOS), "run", "openpi-dual-openyam", "--daemon"],
        cwd=ROOT,
        env=child_env,
        check=True,
    )
    if args.reset:
        reset_arms_to_zero(resume_policy=True)
    print(
        "\nt = pause after current chunk; r = resume; q or Ctrl-C = pause, reset arms to zero, and leave console"
    )
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        while True:
            key = sys.stdin.read(1).lower()
            if key in ("t", "r", "q"):
                # Restore normal terminal behavior while the RPC runs and prints.
                termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
                try:
                    if key == "t":
                        control("pause")
                    elif key == "r":
                        control("resume")
                    else:
                        if pause_and_reset():
                            print("Console closed. dimOS and the inference server remain running.")
                            return
                        print("Pause or reset failed; console remains open.", file=sys.stderr)
                finally:
                    tty.setcbreak(fd)
    except KeyboardInterrupt:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        if not pause_and_reset():
            print(
                "Pause or reset failed. Check the robot and use another control method.",
                file=sys.stderr,
            )
            raise SystemExit(1) from None
        print("\nReset to zero and held. dimOS and the inference server remain running.")
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)


if __name__ == "__main__":
    main()
