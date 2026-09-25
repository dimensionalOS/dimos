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

  arm      a 7-joint arm with a gripper. It can be told where to go or how
           fast to move, but not both at once, and it cannot measure how fast
           its joints are turning
  g1       a humanoid body whose joints are held in place by stiffness and
           damping
  chassis  a base that drives around on the floor, in any direction

They are written out in full rather than built by a shortcut, so that tests
elsewhere can check the shortcuts produce exactly these.
"""

from __future__ import annotations

import pytest

from dimos.control.contract.description import (
    ActivationPolicy,
    ControlDescription,
    Estop,
    EstopKind,
    EstopRecovery,
    LimitPolicy,
    Limits,
    ModeGroup,
    Omission,
    ProcessLoss,
    Resource,
    ResourceKind,
    SafeStop,
    SafeStopKind,
    Timing,
)
from dimos.control.contract.keys import (
    EFFORT,
    KD,
    KP,
    POSITION,
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
    """An arm that can be told where to go or how fast to move, not both.

    Its gripper is separate, so it stays usable either way. The arm cannot
    measure how fast its joints are turning, so it does not claim to.
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
        mode_groups=(
            ModeGroup(name="position", resources=ARM_JOINTS, interfaces=frozenset({POSITION})),
            ModeGroup(name="velocity", resources=ARM_JOINTS, interfaces=frozenset({VELOCITY})),
            # Its own group, and not exclusive: the gripper is commandable
            # alongside whichever arm group is live.
            ModeGroup(
                name="gripper",
                resources=("gripper",),
                interfaces=frozenset({POSITION}),
                exclusive=False,
            ),
        ),
        safe_stop=SafeStop(kind=SafeStopKind.HOLD, stable_state="holds position"),
        estop=Estop(
            kind=EstopKind.VENDOR, recovery=EstopRecovery.CLEAR, stable_state="brakes engage"
        ),
        activation_policy=ActivationPolicy.DIRECT,
        timing=Timing(state_rate_hz=100.0, stale_timeout_s=0.05, watchdog_timeout_s=0.1),
        process_loss=ProcessLoss.UNKNOWN,
    )


@pytest.fixture
def g1() -> ControlDescription:
    """A humanoid whose 29 joints are all driven together.

    Each is held by a stiffness and a damping, which are stored here rather
    than sent with every instruction. Out-of-range values are trimmed rather
    than refused, and it stops by going slack.
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
    return ControlDescription(
        source="g1",
        resources=joints,
        # CLAMP, not REJECT. One instruction covers all 29 joints and is
        # applied all or not at all, so under REJECT a single joint asked to
        # go a hair past its limit throws the whole instruction away and the
        # robot gets nothing. Balancing on two legs, it would fall over.
        # CLAMP trims that one value to the limit and sends the rest.
        limits={Key.of("g1", j, POSITION): Limits(-2.0, 2.0, LimitPolicy.CLAMP) for j in G1_JOINTS},
        mode_groups=(
            ModeGroup(
                name="pd",
                resources=G1_JOINTS,
                interfaces=frozenset({POSITION, VELOCITY, EFFORT, KP, KD}),
            ),
        ),
        # This robot's firmware reads a commanded speed of zero as a real
        # instruction to hold still, not as "no instruction". So when an
        # instruction says nothing about speed, nothing must be sent for it
        # rather than a zero.
        omission={Key.of("g1", j, VELOCITY): Omission.UNSET for j in G1_JOINTS},
        initial_values={Key.of("g1", j, KP): 60.0 for j in G1_JOINTS}
        | {Key.of("g1", j, KD): 1.5 for j in G1_JOINTS},
        safe_stop=SafeStop(
            kind=SafeStopKind.DAMP,
            kd={Key.of("g1", j, KD): 5.0 for j in G1_JOINTS},
            stable_state="sinks to the floor",
        ),
        estop=Estop(
            kind=EstopKind.DISABLE,
            recovery=EstopRecovery.PREPARE_ARM_REQUIRED,
            stable_state="limp",
        ),
        activation_policy=ActivationPolicy.OPERATOR_CONFIRMED,
        timing=Timing(
            state_rate_hz=500.0,
            stale_timeout_s=0.02,
            watchdog_timeout_s=0.05,
            prepare_arm_timeout_s=10.0,
        ),
        process_loss=ProcessLoss.UNPROTECTED,
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
            Key.of("chassis", "base", VX): Limits(-1.5, 1.5),
            Key.of("chassis", "base", VY): Limits(-1.0, 1.0),
            Key.of("chassis", "base", WZ): Limits(-2.0, 2.0),
        },
        mode_groups=(
            ModeGroup(name="twist", resources=("base",), interfaces=frozenset({VX, VY, WZ})),
        ),
        safe_stop=SafeStop(kind=SafeStopKind.ZERO_RAMP, ramp_s=0.3, stable_state="rolls to a stop"),
        estop=Estop(kind=EstopKind.ZERO, recovery=EstopRecovery.CLEAR, stable_state="stops dead"),
        activation_policy=ActivationPolicy.DIRECT,
        timing=Timing(
            state_rate_hz=50.0,
            stale_timeout_s=0.2,
            watchdog_timeout_s=0.2,
        ),
        process_loss=ProcessLoss.EXTERNAL_SUPERVISOR,
        meta={"command_frame": "body", "yaw_convention": "unwrapped"},
    )
