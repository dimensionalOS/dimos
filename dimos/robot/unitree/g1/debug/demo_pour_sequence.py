#!/usr/bin/env python3
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

"""Pour into a pot from wherever the robot is standing: the whole sequence.

Given the pot's pose, the robot decides where to stand (the reach map in
``manip_stance``), walks the last stretch itself (``manip_last_mile``),
adopts that stance as its planning base, checks the pour actually plans
from there, pours, and puts the arm back.

The pot's pose arrives on ``/object_pose`` -- privileged body state in sim,
perception on hardware -- so nothing here is simulator-specific except the
ground-truth measurements printed at the end.

Run the sim first::

    dimos --simulation mujoco --viewer none --scene-package office \\
        run unitree-g1-groot-wbc-manip

then, from the repo root::

    .venv/bin/python -m dimos.robot.unitree.g1.debug.demo_pour_sequence
    .venv/bin/python -m dimos.robot.unitree.g1.debug.demo_pour_sequence status
    .venv/bin/python -m dimos.robot.unitree.g1.debug.demo_pour_sequence approach

Ctrl-C stops the robot and hands control back at any point.
"""

from __future__ import annotations

import argparse
import math
import signal
import sys
import threading
import time
from typing import Any

from dimos.core.rpc_client import RPCClient
from dimos.core.transport import LCMTransport
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.robot.unitree.g1.manip_last_mile import Twist2D, arrived, servo_step
from dimos.robot.unitree.g1.manip_stance import (
    POUR_Z,
    TIP_RADIANS,
    PourReachMap,
    pot_in_base_frame,
    select_stance,
    tool_yaw_for,
)

ROBOT = "g1"
GROUP = "g1/right_arm"
SERVO_HZ = 10.0
SERVO_TIMEOUT = 90.0
# The gait keeps stepping after the command stops; let it settle before
# latching a base pose the planner will trust for the rest of the sequence.
SETTLE_SECONDS = 3.0
HOLD_SECONDS = 3.0


class _Sightings:
    """Latest pot pose and robot odometry, latched off their topics.

    The LCM callback thread is shared by every subscription on its instance,
    so it only ever stores the message; the control loop does the work.
    """

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._pot: PoseStamped | None = None
        self._odom: PoseStamped | None = None
        self._pot_transport = LCMTransport("/object_pose", PoseStamped)
        self._odom_transport = LCMTransport("/odom", PoseStamped)
        self._pot_transport.subscribe(self._on_pot)
        self._odom_transport.subscribe(self._on_odom)

    def _on_pot(self, msg: PoseStamped) -> None:
        with self._lock:
            self._pot = msg

    def _on_odom(self, msg: PoseStamped) -> None:
        with self._lock:
            self._odom = msg

    def wait(self, timeout: float = 10.0) -> None:
        deadline = time.time() + timeout
        while time.time() < deadline:
            with self._lock:
                if self._pot is not None and self._odom is not None:
                    return
            time.sleep(0.1)
        raise SystemExit("no pot pose or odometry yet - is the sim running?")

    @property
    def pot_xy(self) -> tuple[float, float]:
        with self._lock:
            assert self._pot is not None
            return float(self._pot.position.x), float(self._pot.position.y)

    @property
    def base(self) -> tuple[float, float, float]:
        with self._lock:
            assert self._odom is not None
            odom = self._odom
        yaw = math.atan2(
            2.0
            * (odom.orientation.w * odom.orientation.z + odom.orientation.x * odom.orientation.y),
            1.0 - 2.0 * (odom.orientation.y**2 + odom.orientation.z**2),
        )
        return float(odom.position.x), float(odom.position.y), yaw

    def seen_offset(self) -> tuple[float, float]:
        """Where the robot currently sees the pot, in its own frame."""
        x, y, yaw = self.base
        return pot_in_base_frame(self.pot_xy, (x, y), yaw)


class _Driver:
    """Publishes twists, and guarantees a stop on the way out."""

    def __init__(self) -> None:
        self._transport = LCMTransport("/cmd_vel", Twist)
        self._transport.start()

    def send(self, twist: Twist2D) -> None:
        self._transport.broadcast(
            None, Twist(linear=[twist.vx, twist.vy, 0.0], angular=[0.0, 0.0, twist.wz])
        )

    def stop(self) -> None:
        # The WBC latches the last command for a second, so a single zero can
        # be missed on a dropped packet; say it a few times.
        for _ in range(5):
            self.send(Twist2D())
            time.sleep(0.05)


def _pose(x: float, y: float, z: float, roll: float, yaw: float) -> PoseStamped:
    return PoseStamped(
        frame_id="world",
        position=Vector3(x, y, z),
        orientation=Quaternion.from_euler(Vector3(roll, 0.0, yaw)),
    )


def approach(
    module: Any, sight: _Sightings, driver: _Driver, reach: PourReachMap
) -> tuple[float, float]:
    """Walk until the pot sits inside the reachable region, then stop."""
    base_x, base_y, _ = sight.base
    pot = sight.pot_xy
    stance = select_stance(
        pot, approach_yaw=math.atan2(pot[1] - base_y, pot[0] - base_x), reach_map=reach
    )
    print(
        f"stance: stand at ({stance.x:+.2f}, {stance.y:+.2f}) facing {math.degrees(stance.yaw):+.0f} deg, "
        f"pot at ({stance.offset[0]:+.2f}, {stance.offset[1]:+.2f}) in the base frame "
        f"({stance.margin_cells} cells inside the region)"
    )

    deadline = time.time() + SERVO_TIMEOUT
    period = 1.0 / SERVO_HZ
    while time.time() < deadline:
        seen = sight.seen_offset()
        twist = servo_step(seen, stance.offset, reach)
        if twist.is_stop:
            driver.stop()
            print(f"arrived: pot at ({seen[0]:+.2f}, {seen[1]:+.2f}) in the base frame")
            break
        driver.send(twist)
        time.sleep(period)
    else:
        driver.stop()
        raise SystemExit("last-mile servo timed out")

    time.sleep(SETTLE_SECONDS)
    return sight.seen_offset()


def latch_and_verify(module: Any, sight: _Sightings, reach: PourReachMap, driver: _Driver) -> bool:
    """Adopt the stance as the planning base, and prove the pour plans from it.

    A latch is only worth having if the arm can actually work from it, so
    the check is a real plan solve rather than a reachability lookup: it is
    the same planner, the same scene, and the same start state the pour
    will use moments later.

    Both pour poses are checked. The tipped one is the harder of the two
    and it is the one the demo would otherwise discover had failed halfway
    through, with the tool already hanging over the pot.
    """
    for attempt in range(3):
        print("latch ->", module.latch_base_pose(), "|", module.describe_base_pose())
        seen = sight.seen_offset()
        _, _, base_yaw = sight.base
        pot = sight.pot_xy
        yaw = tool_yaw_for(seen, base_yaw)
        poses = {
            "upright": _pose(pot[0], pot[1], POUR_Z, 0.0, yaw),
            "tipped": _pose(pot[0], pot[1], POUR_Z, TIP_RADIANS, yaw),
        }
        solved = {name: module.plan_to_pose_targets({GROUP: pose}) for name, pose in poses.items()}
        module.clear_planned_path()
        if all(solved.values()):
            print(f"verified: both pour poses plan from here (attempt {attempt + 1})")
            return True
        module.reset()
        failed = ", ".join(name for name, ok in solved.items() if not ok)

        print(f"verify failed ({failed}): {module.get_error()!r}; nudging and re-latching")
        # A failed plan from a reachable-looking stance means the live
        # stance is at the region's edge in some way the map cannot see:
        # it is sampled from a nominal standing posture, and the real legs
        # and waist are wherever the policy left them. So move and retry.
        nudge = Twist2D(vx=-0.1 if seen[0] < 0.3 else 0.1)
        for _ in range(int(SERVO_HZ)):
            driver.send(nudge)
            time.sleep(1.0 / SERVO_HZ)
        driver.stop()
        time.sleep(SETTLE_SECONDS)
    return False


def pour(module: Any, sight: _Sightings) -> bool:
    """Hold the tool over the pot, tip it, then put the arm back."""
    seen = sight.seen_offset()
    _, _, base_yaw = sight.base
    pot = sight.pot_xy
    yaw = tool_yaw_for(seen, base_yaw)

    over = module.move_to_pose(
        x=pot[0], y=pot[1], z=POUR_Z, roll=0.0, pitch=0.0, yaw=yaw, robot_name=ROBOT, group_id=GROUP
    )
    print("over:", over)
    if not over.is_success():
        module.reset()
        return False
    measure(module, sight, "over the pot")

    # Tipping is commanded at the point that just solved, not at the tool's
    # measured pose: the arm settles a little below its target, and asking
    # for the sag is asking for a lower target than the one that worked.
    tipped = module.move_to_pose(
        x=pot[0],
        y=pot[1],
        z=POUR_Z,
        roll=TIP_RADIANS,
        pitch=0.0,
        yaw=yaw,
        robot_name=ROBOT,
        group_id=GROUP,
    )
    print("tip:", tipped)
    if not tipped.is_success():
        module.reset()
        return False
    measure(module, sight, "tipped")
    time.sleep(HOLD_SECONDS)

    # Levelling through a pose target does not solve from the tipped
    # configuration; go_init is joint-space and brings the tool up on the way.
    print("home:", module.go_init(ROBOT, GROUP))
    print(f"base after pouring: {module.describe_base_pose()}")
    return True


def measure(module: Any, sight: _Sightings, tag: str) -> None:
    """Palm against pot, from the module's own view of where the hand is."""
    palm = module.get_group_ee_pose(GROUP)
    if palm is None:
        print(f"{tag}: no tool pose")
        return
    pot = sight.pot_xy
    error = math.hypot(float(palm.position.x) - pot[0], float(palm.position.y) - pot[1])
    print(
        f"{tag}: palm ({palm.position.x:+.3f}, {palm.position.y:+.3f}, {palm.position.z:+.3f}) "
        f"pot ({pot[0]:+.3f}, {pot[1]:+.3f}) xy error {error * 100:.1f} cm"
    )


def status(module: Any, sight: _Sightings, reach: PourReachMap) -> None:
    x, y, yaw = sight.base
    seen = sight.seen_offset()
    print(f"base:  ({x:+.2f}, {y:+.2f}) facing {math.degrees(yaw):+.0f} deg")
    print(f"pot:   {sight.pot_xy} | seen at ({seen[0]:+.2f}, {seen[1]:+.2f}) in the base frame")
    print(f"reach: margin {reach.margin(seen)} cells, arrived={arrived(seen, reach)}")
    print(f"module: {module.get_state()} | {module.describe_base_pose()}")


def run(command: str) -> int:
    module = RPCClient(None, ManipulationModule)
    sight = _Sightings()
    driver = _Driver()
    reach = PourReachMap.load()
    sight.wait()

    def _bail(*_: object) -> None:
        driver.stop()
        print("\nstopped - control is yours")
        sys.exit(130)

    signal.signal(signal.SIGINT, _bail)

    if command == "status":
        status(module, sight, reach)
        return 0
    if command == "approach":
        approach(module, sight, driver, reach)
        status(module, sight, reach)
        return 0

    module.reset()
    approach(module, sight, driver, reach)
    if not latch_and_verify(module, sight, reach, driver):
        print("could not find a stance the pour plans from")
        return 1
    if not pour(module, sight):
        print("pour failed")
        return 1
    return 0


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "command", nargs="?", default="demo", choices=("demo", "status", "approach")
    )
    raise SystemExit(run(parser.parse_args().command))


if __name__ == "__main__":
    main()
