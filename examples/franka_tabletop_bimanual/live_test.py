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

"""Staged live test of dimOS's FrankaConnection on the real FR3.

    read     connect, print joint readings for 3 s, disconnect. No control session.
    hold     start a control session holding the arm where it is for 3 s, then end it.
             Tests session start and end, where the arm moved before; the drift
             guard stops the robot if any joint moves over ~1 deg.
    wrist    turn joint 7 (the wrist) +0.1 rad (~6 deg) at 0.1 rad/s, pause, turn back.
    gripper  close the Hand (grasp, 20 N), then open it.

Run with the dimos-franka venv:
    ~/dimos-franka/.venv/bin/python live_test.py <stage> [--ip 10.42.0.230]
"""

import argparse
import math
import time
from typing import Any

from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.manipulators.franka.config import FR3_FINGER_JOINT, fr3_joint_names
from dimos.robot.manipulators.franka.connection import FrankaConnection


class LocalTransport:
    def __init__(self) -> None:
        self.subscribers: list[Any] = []
        self.latest: dict[tuple[str, ...], JointState] = {}

    def broadcast(self, selfstream: Any, value: Any) -> None:
        self.latest[tuple(value.name)] = value
        for cb in list(self.subscribers):
            cb(value)

    def publish(self, value: Any) -> None:
        self.broadcast(None, value)

    def subscribe(self, cb: Any, selfstream: Any = None) -> Any:
        self.subscribers.append(cb)
        return lambda: self.subscribers.remove(cb)

    def start(self) -> None:
        pass

    def stop(self) -> None:
        pass


JOINTS = fr3_joint_names("arm")


def deg(values: list[float]) -> list[float]:
    return [round(math.degrees(v), 1) for v in values]


def check(module: FrankaConnection) -> None:
    status = module.status()
    if status.last_error:
        raise SystemExit(f"STOPPED: {status.last_error}")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("stage", choices=["read", "hold", "wrist", "gripper"])
    parser.add_argument("--ip", default="10.42.0.230")
    args = parser.parse_args()

    module = FrankaConnection(ip=args.ip, deadman_timeout_s=0.5)
    ports = {name: LocalTransport() for name in {**module.inputs, **module.outputs}}
    for name, transport in ports.items():
        module.set_transport(name, transport)  # type: ignore[arg-type]
    state, command = ports["joint_state"], ports["position_command"]

    def arm() -> list[float]:
        return list(state.latest[tuple(JOINTS)].position)

    def send(**targets: float) -> None:
        command.publish(
            JointState(name=[f"arm/{j}" for j in targets], position=list(targets.values()))
        )

    print("Starting connection...")
    module.start()
    try:
        time.sleep(0.5)
        check(module)
        print("Connected. Joints (deg):", deg(arm()))

        if args.stage == "read":
            for _ in range(6):
                time.sleep(0.5)
                print("  joints (deg):", deg(arm()))
            hand = state.latest.get((f"arm/{FR3_FINGER_JOINT}",))
            if hand:
                print(f"  hand width: {hand.position[0] * 2000:.1f} mm")

        elif args.stage == "hold":
            start = arm()
            for i in range(300):  # 3 s of "stay where you are", every 10 ms
                send(**{j.split("/")[1]: q for j, q in zip(JOINTS, start)})
                time.sleep(0.01)
                if i % 50 == 0:
                    check(module)
                    moved = max(abs(a - b) for a, b in zip(arm(), start))
                    print(f"  largest joint change so far: {math.degrees(moved):.2f} deg")
            check(module)

        elif args.stage == "wrist":
            start = arm()
            q7 = start[6]
            # Out 0.1 rad at 0.1 rad/s, hold 1 s, back, streaming every 10 ms.
            path = (
                [q7 + 0.001 * i for i in range(101)]
                + [q7 + 0.1] * 100
                + [q7 + 0.1 - 0.001 * i for i in range(101)]
                + [q7] * 100
            )
            for i, target in enumerate(path):
                send(fr3_joint7=target)
                time.sleep(0.01)
                if i % 25 == 0:
                    check(module)
                    print(
                        f"  joint7 target {math.degrees(target):6.1f} deg, "
                        f"measured {math.degrees(arm()[6]):6.1f} deg"
                    )
            check(module)
            print("Done. Joints (deg):", deg(arm()))

        elif args.stage == "gripper":
            for label, finger in [("closing", 0.0), ("opening", 0.04)]:
                print(f"  {label}...")
                for _ in range(300):  # 3 s of the held target, as the gripper task does
                    send(**{FR3_FINGER_JOINT: finger})
                    time.sleep(0.01)
                check(module)
                hand = state.latest.get((f"arm/{FR3_FINGER_JOINT}",))
                if hand:
                    print(f"  hand width: {hand.position[0] * 2000:.1f} mm")
    finally:
        before = arm() if tuple(JOINTS) in state.latest else None
        print("Stopping connection.")
        module.stop()
        if before is not None:
            time.sleep(1.0)
            from pylibfranka import RealtimeConfig, Robot

            after = list(Robot(args.ip, RealtimeConfig.kIgnore).read_once().q)
            moved = max(abs(a - b) for a, b in zip(after, before))
            print(f"Change after stopping: {math.degrees(moved):.2f} deg (should be ~0)")


if __name__ == "__main__":
    main()
