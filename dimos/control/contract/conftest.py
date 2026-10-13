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

"""Three example robots for the tests, written out by hand.

  arm      a 7-joint arm with a gripper. It cannot measure how fast its
           joints are turning, so it does not claim to
  g1       a humanoid body whose joints are held in place by stiffness and
           damping, with an IMU in its chest
  chassis  a base that drives around on the floor, in any direction. Its
           speed limits clamp, so a teleop stick pushed too far still drives

They are written out in full rather than built by a shortcut, so that tests
elsewhere can check the shortcuts produce exactly these.
"""

from __future__ import annotations

import pytest

from dimos.control.contract.description import (
    ControlDescription,
    Limits,
    Resource,
    ResourceKind,
)
from dimos.control.contract.keys import Interface, Key, Unit

ARM_JOINTS = tuple(f"joint{i}" for i in range(1, 8))
G1_JOINTS = tuple(f"joint{i}" for i in range(1, 30))


@pytest.fixture
def xarm() -> ControlDescription:
    """An arm that can be told where to go or how fast to move.

    The arm cannot measure how fast its joints are turning, so it does not
    claim to.
    """
    joints = tuple(
        Resource(
            name=name,
            kind=ResourceKind.JOINT,
            # No velocity state: the SDK does not report it, so it is not
            # declared rather than published as a fabricated zero.
            state_interfaces=(Interface.POSITION, Interface.EFFORT),
            command_interfaces=(Interface.POSITION, Interface.VELOCITY),
            units={
                Interface.POSITION: Unit.RAD,
                Interface.VELOCITY: Unit.RAD_PER_S,
                Interface.EFFORT: Unit.NM,
            },
        )
        for name in ARM_JOINTS
    )
    gripper = Resource(
        name="gripper",
        kind=ResourceKind.JOINT,
        state_interfaces=(Interface.POSITION,),
        command_interfaces=(Interface.POSITION,),
        units={Interface.POSITION: Unit.M},
    )
    limits = {Key.of("arm", j, Interface.POSITION): Limits(-3.14, 3.14) for j in ARM_JOINTS}
    limits |= {Key.of("arm", j, Interface.VELOCITY): Limits(-1.0, 1.0) for j in ARM_JOINTS}
    limits[Key.of("arm", "gripper", Interface.POSITION)] = Limits(0.0, 0.085)
    return ControlDescription(
        source="arm",
        resources=(*joints, gripper),
        limits=limits,
        state_rate_hz=100.0,
        deadman_timeout_s=0.1,
    )


@pytest.fixture
def g1() -> ControlDescription:
    """A humanoid whose 29 joints are each held by a stiffness and a damping.

    The IMU only reports: which way up the body is, how fast it is turning,
    and how hard it is being pushed.
    """
    joints = tuple(
        Resource(
            name=name,
            kind=ResourceKind.JOINT,
            state_interfaces=(Interface.POSITION, Interface.VELOCITY, Interface.EFFORT),
            command_interfaces=(
                Interface.POSITION,
                Interface.VELOCITY,
                Interface.EFFORT,
                Interface.KP,
                Interface.KD,
            ),
            units={
                Interface.POSITION: Unit.RAD,
                Interface.VELOCITY: Unit.RAD_PER_S,
                Interface.EFFORT: Unit.NM,
                Interface.KP: Unit.UNITLESS,
                Interface.KD: Unit.UNITLESS,
            },
        )
        for name in G1_JOINTS
    )
    imu = Resource(
        name="imu",
        kind=ResourceKind.SENSOR,
        state_interfaces=(
            Interface.QX,
            Interface.QY,
            Interface.QZ,
            Interface.QW,
            Interface.GX,
            Interface.GY,
            Interface.GZ,
            Interface.AX,
            Interface.AY,
            Interface.AZ,
        ),
        units=dict.fromkeys((Interface.QX, Interface.QY, Interface.QZ, Interface.QW), Unit.UNITLESS)
        | dict.fromkeys((Interface.GX, Interface.GY, Interface.GZ), Unit.RAD_PER_S)
        | dict.fromkeys((Interface.AX, Interface.AY, Interface.AZ), Unit.M_PER_S2),
    )
    limits = {Key.of("g1", j, Interface.POSITION): Limits(-2.0, 2.0) for j in G1_JOINTS}
    limits |= {Key.of("g1", j, Interface.VELOCITY): Limits(-32.0, 32.0) for j in G1_JOINTS}
    limits |= {Key.of("g1", j, Interface.EFFORT): Limits(-88.0, 88.0) for j in G1_JOINTS}
    return ControlDescription(
        source="g1",
        resources=(*joints, imu),
        limits=limits,
        state_rate_hz=500.0,
        deadman_timeout_s=0.05,
    )


@pytest.fixture
def chassis() -> ControlDescription:
    """A base that can drive in any direction, including sideways.

    It is told how fast to move and turn, and reports both that and where it
    has got to.
    """
    base = Resource(
        name="base",
        kind=ResourceKind.BASE,
        state_interfaces=(
            Interface.VX,
            Interface.VY,
            Interface.WZ,
            Interface.X,
            Interface.Y,
            Interface.YAW,
        ),
        command_interfaces=(Interface.VX, Interface.VY, Interface.WZ),
        units={
            Interface.VX: Unit.M_PER_S,
            Interface.VY: Unit.M_PER_S,
            Interface.WZ: Unit.RAD_PER_S,
            Interface.X: Unit.M,
            Interface.Y: Unit.M,
            Interface.YAW: Unit.RAD,
        },
    )
    return ControlDescription(
        source="chassis",
        resources=(base,),
        limits={
            Key.of("chassis", "base", Interface.VX): Limits(-1.5, 1.5, clamp=True),
            Key.of("chassis", "base", Interface.VY): Limits(-1.0, 1.0, clamp=True),
            Key.of("chassis", "base", Interface.WZ): Limits(-2.0, 2.0, clamp=True),
        },
        state_rate_hz=50.0,
        deadman_timeout_s=0.2,
    )
