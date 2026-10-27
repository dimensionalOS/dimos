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

"""Alfred at the bench: the arms, the lift, the whole-body planner and the mast D455, no lidar.

    dimos run alfred-manipulation              # GraspGenX grasps (its own env, synced on first start)
    dimos run alfred-manipulation-heuristic    # one top-down grasp per object, no second environment

The Orin NX runs one of two stacks at a time: this one for manipulation, ``alfred-navigation``
for driving around. Nothing of navigation is here: no Mid-360, no Point-LIO, no planners, no
follower. The base still drives from the Controller (teleop is a claim on the coordinator), and
a whole-body plan's base segment is a motion from where the robot stands, tracked on the
FlowBase's own wheel odometry (``WheelOdometryRelay`` puts it on ``/flowbase/odom`` and tf).
The D455 feeds the planner's obstacle map and, with a GPU, the perception stack's localize
(``localize_boxes``): the Manipulation app's scan.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from dimos.control.components import HardwareComponent
from dimos.core.coordination.blueprints import autoconnect
from dimos.robot.diy.alfred.blueprints.alfred_full import (
    BASE_FRAME,
    ODOM_FRAME,
    alfred_arm_modules,
    alfred_base_transports,
    alfred_coordinator,
)
from dimos.robot.diy.alfred.blueprints.alfred_full_perception import (
    GraspGenerator,
    alfred_perception_modules,
)
from dimos.robot.diy.alfred.effector_high_level import AlfredHighLevel
from dimos.robot.diy.alfred.mount_tf import AlfredMountTf
from dimos.robot.diy.alfred.wheel_odometry_relay import WheelOdometryRelay


@dataclass(frozen=True)
class BenchHardware:
    """Stand-ins for the bench stack's hardware (a simulation's); a None field is the robot's."""

    # Drives the FlowBase off /flowbase/cmd_vel and publishes its wheel_odometry
    # (AlfredHighLevel, over Portal to the FlowBase's Pi).
    flowbase: Any | None = None
    # PillarConnection's port (the Nano's CH340).
    pillar_device_path: str | None = None
    # The coordinator's arms (Damiao on the CAN ports the env names) and base (the transport
    # adapter AlfredHighLevel answers).
    arms: HardwareComponent | None = None
    base: HardwareComponent | None = None
    # The mast D455 and the models behind it: alfred_perception_modules' output.
    perception: tuple[Any, ...] | None = None


def alfred_bench_modules(
    *, grasp: GraspGenerator = "graspgenx", hardware: BenchHardware | None = None
) -> tuple[Any, ...]:
    """The bench stack's modules, with one grasp generator answering the pick pipeline.
    ``hardware`` swaps what drives the robot for stand-ins and leaves the rest as it is."""
    hardware = hardware or BenchHardware()
    return (
        # The mounts rooted at base_link: no lidar owns a frame above it here.
        AlfredMountTf.blueprint(),
        AlfredHighLevel.blueprint() if hardware.flowbase is None else hardware.flowbase,
        WheelOdometryRelay.blueprint(world_frame=ODOM_FRAME, base_frame=BASE_FRAME),
        # No Rerun bridge: the Desktop reads zenoh itself, and the bridge's late-viewer buffer
        # is what grew to 6.4 GB and got this stack OOM-killed (2026-10-25).
        *alfred_arm_modules(pillar_device_path=hardware.pillar_device_path),
        alfred_coordinator(
            arms=True, follower=False, arm_hardware=hardware.arms, base_hardware=hardware.base
        ),
        # The pick pipeline's scene registration only: localize's second OWLv2 + EdgeTAM and
        # its frame memory pushed the Orin into swap beside it.
        *(
            alfred_perception_modules(localize=False, grasp=grasp)
            if hardware.perception is None
            else hardware.perception
        ),
    )


# The registry scans for module-level autoconnect(...) chains: each stack is spelled out.
# GraspGenX proposes the grasps (ranked, angled ones an OpenArm can reach; it runs in its own
# environment, synced on first start, weights downloaded then too); the heuristic's one
# top-down grasp per object is the fallback stack.
alfred_manipulation = (
    autoconnect(*alfred_bench_modules())
    .transports(alfred_base_transports())
    .global_config(n_workers=10, robot_model="alfred")
)
alfred_manipulation_heuristic = (
    autoconnect(*alfred_bench_modules(grasp="heuristic"))
    .transports(alfred_base_transports())
    .global_config(n_workers=10, robot_model="alfred")
)
