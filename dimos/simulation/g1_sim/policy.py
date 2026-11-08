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

"""The G1's walking actor: 29 joint position targets from proprioception and a velocity command."""

from __future__ import annotations

import copy
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from numpy.typing import NDArray
import onnxruntime as ort

from dimos.utils.data import get_data

JOINTS = 29
# linvel 3 + gyro 3 + gravity 3 + command 3 + angles 29 + velocities 29 + last action 29 + phase 4
OBSERVATIONS = 103
# The actor was trained with a gait clock rather than reading one out of the state.
GAIT_HZ = 1.5


@dataclass(frozen=True)
class G1Proprioception:
    """What the G1 actor senses. Its own, because the quadruped actors are not given linear velocity."""

    linear_velocity: NDArray[np.float64]
    angular_velocity: NDArray[np.float64]
    gravity: NDArray[np.float64]
    joint_position: NDArray[np.float64]
    joint_velocity: NDArray[np.float64]


# The vendor MJCF's `home` keyframe, which is where the actor's zero action sits. It is stated
# here because a USD layer does not carry keyframes -- MuJoCo's own converter says so outright
# ("keys are not supported"). The chassis stands in it and the actor acts from it, so it sits at
# module level rather than on either of them.
DEFAULT_POSE = np.array(
    [
        *[-0.312, 0, 0, 0.669, -0.363, 0],  # left leg
        *[-0.312, 0, 0, 0.669, -0.363, 0],  # right leg
        *[0, 0, 0.073],  # waist
        *[0.2, 0.2, 0, 0.6, 0, 0, 0],  # left arm
        *[0.2, -0.2, 0, 0.6, 0, 0, 0],  # right arm
    ]
)


class G1Policy:
    """The trained G1 actor, stepped at CONTROL_DT with its own gait phase.

    The gait phase and the last action are **per robot**. Three G1s sharing one of these take
    turns overwriting each other's phase and all three fall over; measured, not supposed.
    """

    default_pose = DEFAULT_POSE
    action_scale = 0.5
    # The trained scalings; the actor sees a command twice the commanded planar velocity.
    command_scale = np.array([2.0, 2.0, 1.0])

    def __init__(self, path: Path, control_dt: float) -> None:
        options = ort.SessionOptions()
        options.intra_op_num_threads = 1
        options.inter_op_num_threads = 1
        self._session = ort.InferenceSession(
            str(path.expanduser()), options, providers=["CPUExecutionProvider"]
        )
        (inputs,), (outputs,) = self._session.get_inputs(), self._session.get_outputs()
        if inputs.shape != [1, OBSERVATIONS] or outputs.shape != [1, JOINTS]:
            raise ValueError(
                f"{path} is not a G1 actor with obs[1,{OBSERVATIONS}] and actions[1,{JOINTS}]"
            )
        self._input = inputs.name
        self._phase_dt = 2.0 * np.pi * GAIT_HZ * control_dt
        self.reset()

    @classmethod
    def load(cls, control_dt: float) -> G1Policy:
        """The actor shipped with the sim data."""
        return cls(get_data("mujoco_sim") / "unitree_g1_policy.onnx", control_dt)

    def clone(self) -> G1Policy:
        """A second actor on the same loaded network, with a gait phase of its own.

        The network is read-only and takes a second to load, so copies share it. The phase and
        the last action are not: two robots driven by one of these take turns overwriting each
        other's, and -- measured, three G1s on a flat floor -- all of them fall over within four
        seconds with nothing in the logs to say why.
        """
        twin = copy.copy(self)
        twin.reset()
        return twin

    def reset(self) -> None:
        self._last_action = np.zeros(JOINTS)
        # The two legs start half a cycle apart, which is what makes it walk rather than hop.
        self._phase = np.array([0.0, np.pi])

    def act(self, state: G1Proprioception, command: NDArray[np.float64]) -> NDArray[np.float64]:
        """The joint position targets for this tick."""
        observation = np.hstack(
            [
                state.linear_velocity,
                state.angular_velocity,
                state.gravity,
                command * self.command_scale,
                state.joint_position - self.default_pose,
                state.joint_velocity,
                self._last_action,
                np.cos(self._phase),
                np.sin(self._phase),
            ]
        ).astype(np.float32)
        (action,) = self._session.run(None, {self._input: observation.reshape(1, -1)})
        self._last_action = action.reshape(-1).astype(np.float64)
        self._phase = np.fmod(self._phase + self._phase_dt + np.pi, 2.0 * np.pi) - np.pi
        return self.default_pose + self._last_action * self.action_scale
