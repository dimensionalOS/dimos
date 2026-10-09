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
from dimos.control.contract.keys import (
    AX,
    AY,
    AZ,
    EFFORT,
    GX,
    GY,
    GZ,
    KD,
    KP,
    POSITION,
    QW,
    QX,
    QY,
    QZ,
    VELOCITY,
    VX,
    VY,
    WZ,
    YAW,
    Key,
    Unit,
    X,
    Y,
)

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
            state_interfaces=(POSITION, EFFORT),
            command_interfaces=(POSITION, VELOCITY),
            units={POSITION: Unit.RAD, VELOCITY: Unit.RAD_PER_S, EFFORT: Unit.NM},
        )
        for name in ARM_JOINTS
    )
    gripper = Resource(
        name="gripper",
        kind=ResourceKind.JOINT,
        state_interfaces=(POSITION,),
        command_interfaces=(POSITION,),
        units={POSITION: Unit.M},
    )
    limits = {Key.of("arm", j, POSITION): Limits(-3.14, 3.14) for j in ARM_JOINTS}
    limits |= {Key.of("arm", j, VELOCITY): Limits(-1.0, 1.0) for j in ARM_JOINTS}
    limits[Key.of("arm", "gripper", POSITION)] = Limits(0.0, 0.085)
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
            state_interfaces=(POSITION, VELOCITY, EFFORT),
            command_interfaces=(POSITION, VELOCITY, EFFORT, KP, KD),
            units={
                POSITION: Unit.RAD,
                VELOCITY: Unit.RAD_PER_S,
                EFFORT: Unit.NM,
                KP: Unit.UNITLESS,
                KD: Unit.UNITLESS,
            },
        )
        for name in G1_JOINTS
    )
    imu = Resource(
        name="imu",
        kind=ResourceKind.SENSOR,
        state_interfaces=(QX, QY, QZ, QW, GX, GY, GZ, AX, AY, AZ),
        units=dict.fromkeys((QX, QY, QZ, QW), Unit.UNITLESS)
        | dict.fromkeys((GX, GY, GZ), Unit.RAD_PER_S)
        | dict.fromkeys((AX, AY, AZ), Unit.M_PER_S2),
    )
    return ControlDescription(
        source="g1",
        resources=(*joints, imu),
        limits={Key.of("g1", j, POSITION): Limits(-2.0, 2.0) for j in G1_JOINTS},
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
        state_interfaces=(VX, VY, WZ, X, Y, YAW),
        command_interfaces=(VX, VY, WZ),
        units={
            VX: Unit.M_PER_S,
            VY: Unit.M_PER_S,
            WZ: Unit.RAD_PER_S,
            X: Unit.M,
            Y: Unit.M,
            YAW: Unit.RAD,
        },
    )
    return ControlDescription(
        source="chassis",
        resources=(base,),
        limits={
            Key.of("chassis", "base", VX): Limits(-1.5, 1.5, clamp=True),
            Key.of("chassis", "base", VY): Limits(-1.0, 1.0, clamp=True),
            Key.of("chassis", "base", WZ): Limits(-2.0, 2.0, clamp=True),
        },
        state_rate_hz=50.0,
        deadman_timeout_s=0.2,
    )
