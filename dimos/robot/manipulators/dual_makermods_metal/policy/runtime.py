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

"""Pure NumPy runtime for the trained dual Metal bimanual policy.

The runtime owns the training contract's causal state - the six-frame
observation history, the clipped previous action and the low-pass/slew command
state - and talks to a deterministic ONNX actor. It never imports torch, the
training repository or mjlab, so deployment only needs ONNX Runtime and the
pinned model artifact.

Tick order mirrors the training loop exactly:

``act()`` decodes the next joint command from the frames observed *before* it,
then ``observe()`` records the frame the robot reaches during that interval.
That one-period feedback latency is what the policy was trained with; feeding
the newest frame into the same ``act()`` would shift every action by 50 ms.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any

import numpy as np
from numpy.typing import NDArray

from dimos.control.tasks.policy_execution import OnnxActor
from dimos.robot.manipulators.dual_makermods_metal.policy.contract import (
    PolicyContractError,
    PolicyManifest,
    check_feedback,
    check_observation,
    load_normalizer,
)


class ActionDecoder:
    """Training-contract action decode and the command state it mutates.

    Order matters and is copied from the trained controller: clamp the raw
    action, form ``home + scale * action`` and clamp it inside the training
    joint limits, apply a multiplicative low-pass toward that desired target,
    then clamp the per-tick change. The repo's IK output filter must not be
    added on top.
    """

    def __init__(self, manifest: PolicyManifest) -> None:
        self.manifest = manifest
        self._issued = np.asarray(manifest.home, dtype=np.float32).copy()
        self._previous_action = np.zeros(manifest.action_dim, dtype=np.float32)
        self._lower = np.asarray(manifest.lower, dtype=np.float32) + manifest.clamp_margin
        self._upper = np.asarray(manifest.upper, dtype=np.float32) - manifest.clamp_margin
        self._home = np.asarray(manifest.home, dtype=np.float32)

    def reset(self, q_measured: NDArray[np.float32] | NDArray[np.float64] | None = None) -> None:
        """Clear the command state; with a measured pose, start from it (never home)."""
        if q_measured is not None:
            values = np.asarray(q_measured, dtype=np.float32).reshape(-1)
            if values.shape != (self.manifest.action_dim,):
                raise ValueError(
                    f"expected {self.manifest.action_dim} joint angles, got {values.shape}"
                )
            check_feedback(values, np.zeros_like(values), self.manifest)
            self._issued = values.copy()
        self._previous_action = np.zeros(self.manifest.action_dim, dtype=np.float32)

    def seed(self, issued: NDArray[np.float32], previous_action: NDArray[np.float32]) -> None:
        """Restore command state, e.g. when re-anchoring a session."""
        self._issued = np.asarray(issued, dtype=np.float32).reshape(-1).copy()
        self._previous_action = np.asarray(previous_action, dtype=np.float32).reshape(-1).copy()

    @property
    def issued(self) -> NDArray[np.float32]:
        return self._issued.copy()

    @property
    def previous_action(self) -> NDArray[np.float32]:
        return self._previous_action.copy()

    def decode(self, action: NDArray[np.float32]) -> NDArray[np.float32]:
        manifest = self.manifest
        clipped = np.clip(np.asarray(action, dtype=np.float32).reshape(-1), -1.0, 1.0)
        if clipped.shape != (manifest.action_dim,):
            raise PolicyContractError(f"action shape {clipped.shape} != ({manifest.action_dim},)")
        if not np.isfinite(clipped).all():
            raise PolicyContractError("action contains non-finite values")
        desired = np.clip(self._home + manifest.action_scale * clipped, self._lower, self._upper)
        step = np.clip(
            manifest.low_pass_alpha * (desired - self._issued),
            -manifest.slew_limit,
            manifest.slew_limit,
        )
        self._issued = (self._issued + step).astype(np.float32)
        self._previous_action = clipped.astype(np.float32)
        return self._issued.copy()


class MetalArmPolicyRuntime:
    """Deterministic 20 Hz actor plus its training-contract command state."""

    def __init__(
        self,
        manifest: PolicyManifest,
        model_dir: str | Path,
        *,
        kinematics: Any | None = None,
        providers: list[str] | None = None,
    ) -> None:
        self.manifest = manifest
        self._model_dir = Path(model_dir)
        onnx_path, normalizer_path = manifest.verify_artifacts(self._model_dir)
        self._mean, self._std = load_normalizer(normalizer_path)
        self._actor = OnnxActor(
            onnx_path,
            providers=providers or ["CPUExecutionProvider"],
            input_name=manifest.onnx_input,
            output_name=manifest.onnx_output,
        )
        if kinematics is None:
            from dimos.robot.manipulators.dual_makermods_metal.policy.station import (
                MetalStationKinematics,
            )

            kinematics = MetalStationKinematics(manifest.station)
        self._kinematics = kinematics
        self._frames: list[NDArray[np.float32]] = []
        self._decoder = ActionDecoder(manifest)
        self._last_action: NDArray[np.float32] | None = None

    # ------------------------------------------------------------------ state
    def reset(self, q_measured: NDArray[np.float32] | NDArray[np.float64] | None = None) -> None:
        """Clear history and seed the command from measured joints, never from home.

        Passing ``None`` keeps the last issued command; the coordinator always
        passes the measured joints so a mid-pose engagement does not snap home.
        """
        self._frames.clear()
        self._decoder.reset(q_measured)
        self._last_action = None

    @property
    def issued(self) -> NDArray[np.float32]:
        return self._decoder.issued

    @property
    def previous_action(self) -> NDArray[np.float32]:
        return self._decoder.previous_action

    @property
    def decoder(self) -> ActionDecoder:
        return self._decoder

    @property
    def kinematics(self) -> Any:
        """Station FK used for the observation's current-TCP slots."""
        return self._kinematics

    @property
    def history_length(self) -> int:
        return len(self._frames)

    # ------------------------------------------------------------- observation
    def build_frame(
        self,
        q: NDArray[np.float32] | NDArray[np.float64],
        dq: NDArray[np.float32] | NDArray[np.float64],
        goal_pose24: NDArray[np.float32] | NDArray[np.float64],
    ) -> NDArray[np.float32]:
        """Assemble the 96-value observation frame from measured state and the goal."""
        manifest = self.manifest
        q_arr = np.asarray(q, dtype=np.float32).reshape(-1)
        dq_arr = np.asarray(dq, dtype=np.float32).reshape(-1)
        check_feedback(q_arr, dq_arr, manifest)
        goal = np.asarray(goal_pose24, dtype=np.float64).reshape(-1)
        if goal.shape != (24,):
            raise PolicyContractError(f"goal pose must have 24 values, got {goal.shape}")
        if not np.isfinite(goal).all():
            raise PolicyContractError("goal pose contains non-finite values")
        current = np.asarray(self._kinematics.pose24(q_arr), dtype=np.float64)
        if not np.isfinite(current).all():
            raise PolicyContractError("forward kinematics produced a non-finite pose")
        home = np.asarray(manifest.home, dtype=np.float32)
        frame = np.concatenate(
            [
                q_arr - home,
                (dq_arr / manifest.dq_scale).astype(np.float32),
                self._decoder.issued - home,
                self._decoder.previous_action,
                current[:6].astype(np.float32),
                current[6:].astype(np.float32),
                (goal - current).astype(np.float32),
            ]
        ).astype(np.float32)
        if frame.shape != (manifest.frame_dim,):
            raise PolicyContractError(
                f"observation frame has {frame.shape[0]} values, expected {manifest.frame_dim}"
            )
        return np.asarray(frame, dtype=np.float32)

    def observe(
        self,
        q: NDArray[np.float32] | NDArray[np.float64],
        dq: NDArray[np.float32] | NDArray[np.float64],
        goal_pose24: NDArray[np.float32] | NDArray[np.float64],
    ) -> NDArray[np.float32]:
        """Append one frame to the causal history, keeping at most six."""
        frame = self.build_frame(q, dq, goal_pose24)
        self._frames.append(frame)
        if len(self._frames) > self.manifest.history_frames:
            del self._frames[: len(self._frames) - self.manifest.history_frames]
        return frame

    def observation(self) -> NDArray[np.float32]:
        """Flattened actor input; missing history is zero-padded at the front."""
        manifest = self.manifest
        frames = list(self._frames[-(manifest.history_frames) :])
        while len(frames) < manifest.history_frames:
            frames.insert(0, np.zeros(manifest.frame_dim, dtype=np.float32))
        obs = np.concatenate(frames).reshape(1, manifest.obs_dim).astype(np.float32)
        check_observation(obs, manifest)
        return obs

    # ---------------------------------------------------------------- inference
    def infer(self, observation: NDArray[np.float32]) -> NDArray[np.float32]:
        """Run the deterministic actor on an already-built observation.

        The ONNX graph carries the checkpoint's learned observation
        normalization, so the raw 576-value observation is the input.
        """
        check_observation(observation, self.manifest)
        action = self._actor.run(observation)
        if action.shape != (self.manifest.action_dim,):
            raise PolicyContractError(
                f"actor returned {action.shape}, expected ({self.manifest.action_dim},)"
            )
        if not np.isfinite(action).all():
            raise PolicyContractError("actor returned non-finite actions")
        return action

    def decode_action(self, action: NDArray[np.float32]) -> NDArray[np.float32]:
        """Training-contract decode: clip, bounded target, low-pass, slew clamp."""
        command = self._decoder.decode(action)
        self._last_action = self._decoder.previous_action
        return command

    def act(self) -> NDArray[np.float32]:
        """Infer from the current history and return the new joint position command."""
        return self.decode_action(self.infer(self.observation()))

    @property
    def last_action(self) -> NDArray[np.float32] | None:
        return None if self._last_action is None else self._last_action.copy()
