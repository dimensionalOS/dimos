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

"""Pure NumPy runtime for the OpenYAM history policy.

The runtime owns exactly the causal state the trained controller had: a
16-frame sensor history and the command the previous tick issued. It talks to a
deterministic ONNX actor and never imports torch, the training repository or
MuJoCo, so a deployment only needs ONNX Runtime plus the hash-pinned artifacts.

Tick order mirrors the training loop:

``reset()`` seeds sixteen copies of the observed frame, with the validity flag
set only on the newest one - which is what the training reset does, not a
zero-padded history. ``act()`` then decodes ``q_ref + 0.1 * raw`` from the
frames observed *before* it. ``observe()`` records the frame the robot reaches
during the following interval, including the command just issued. That
one-period feedback latency is what the policy was trained with.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any

import numpy as np
from numpy.typing import NDArray

from dimos.control.tasks.policy_execution import OnnxActor
from dimos.robot.manipulators.openyam.policy.contract import (
    PolicyContractError,
    PolicyManifest,
    check_feedback,
    check_goal_motion,
    check_observation,
    load_normalizer,
)
from dimos.robot.manipulators.openyam.policy.reference import ReferenceClip, ReferenceRow


class ActionDecoder:
    """The trained decode law: ``q_des = q_ref + scale * raw``, with no clamp.

    There is deliberately no action clip, no decoded-position clamp, no
    low-pass and no slew limit: the trained plant applied the raw output
    through a 0.1 rad scale and let the PD/motor peak limit the torque. A
    deployment that needs a smaller envelope must enforce it *outside* this
    decoder (see ``contract.hardware_violations``) so the actor mapping stays
    verifiable.
    """

    def __init__(self, manifest: PolicyManifest) -> None:
        self.manifest = manifest
        self._issued: NDArray[np.float32] | None = None

    def reset(self, row: ReferenceRow) -> None:
        """Start the command chain exactly where the trained reset starts it."""
        self._issued = np.asarray(row.q, dtype=np.float32).copy()

    @property
    def issued(self) -> NDArray[np.float32]:
        if self._issued is None:
            raise PolicyContractError("decoder has no command: reset a session first")
        return self._issued.copy()

    def decode(self, raw: NDArray[np.float32], row: ReferenceRow) -> NDArray[np.float32]:
        """Decode one actor output against the reference row of the same tick."""
        manifest = self.manifest
        action = np.asarray(raw, dtype=np.float32).reshape(-1)
        if action.shape != (manifest.model.action_dim,):
            raise PolicyContractError(
                f"action shape {action.shape} != ({manifest.model.action_dim},)"
            )
        if not np.isfinite(action).all():
            raise PolicyContractError("action contains non-finite values")
        reference = np.asarray(row.q, dtype=np.float32).reshape(-1)
        command = reference + manifest.action.action_scale * action
        if not np.isfinite(command).all():
            raise PolicyContractError("decoded command contains non-finite values")
        self._issued = command.astype(np.float32)
        return self._issued.copy()


class OpenYamPolicyRuntime:
    """Deterministic 30 Hz actor plus its training-contract observation state."""

    def __init__(
        self,
        manifest: PolicyManifest,
        model_dir: str | Path,
        *,
        providers: list[str] | None = None,
    ) -> None:
        self.manifest = manifest
        self._model_dir = Path(model_dir)
        onnx_path, normalizer_path = manifest.verify_artifacts(self._model_dir)
        (
            self._obs_mean,
            self._obs_std,
            self._goal_mean,
            self._goal_std,
        ) = load_normalizer(normalizer_path, manifest)
        self._actor = OnnxActor(
            onnx_path,
            providers=providers or ["CPUExecutionProvider"],
            input_name=manifest.model.input,
            output_name=manifest.model.output,
        )
        self._decoder = ActionDecoder(manifest)
        self._frames: list[NDArray[np.float32]] = []
        self._tick: int | None = None
        self._awaiting_observation = False

    # ------------------------------------------------------------------ state
    @property
    def decoder(self) -> ActionDecoder:
        return self._decoder

    @property
    def issued(self) -> NDArray[np.float32]:
        return self._decoder.issued

    @property
    def tick(self) -> int:
        """Index of the newest observed frame, i.e. the reference tick in play."""
        if self._tick is None:
            raise PolicyContractError("runtime has no session: reset it first")
        return self._tick

    @property
    def history(self) -> NDArray[np.float32]:
        """The causal history, oldest frame first."""
        if not self._frames:
            raise PolicyContractError("runtime has no session: reset it first")
        return np.stack(self._frames).astype(np.float32)

    def reset(
        self,
        q_measured: NDArray[np.float32] | NDArray[np.float64],
        dq_measured: NDArray[np.float32] | NDArray[np.float64],
        row: ReferenceRow,
    ) -> None:
        """Seed history and command from the measured pose and the anchor row.

        The training reset starts the plant *on* the reference row, so the
        seeded command is the reference pose. The measured pose is used for the
        sensor slots, and the validity column is zero on the fifteen padded
        frames exactly as the training reset leaves it.
        """
        q = np.asarray(q_measured, dtype=np.float32).reshape(-1)
        dq = np.asarray(dq_measured, dtype=np.float32).reshape(-1)
        check_feedback(q, dq, self.manifest)
        self._decoder.reset(row)
        frame = self._frame(q, dq, self.manifest, issued=row.q, valid=1.0, time_s=0.0)
        self._frames = [frame.copy() for _ in range(self.manifest.observation.history_frames)]
        for padded in self._frames[:-1]:
            padded[-2] = 0.0
        self._tick = 0
        self._awaiting_observation = False

    def observe(
        self,
        q_measured: NDArray[np.float32] | NDArray[np.float64],
        dq_measured: NDArray[np.float32] | NDArray[np.float64],
    ) -> NDArray[np.float32]:
        """Append the frame for the interval that just ended.

        Called once per policy tick *after* ``act`` emitted the command for the
        interval, so the frame carries that command in its ``issued`` slots and
        the tick time of the new frame.
        """
        if self._tick is None:
            raise PolicyContractError("runtime has no session: reset it first")
        if not self._awaiting_observation:
            raise PolicyContractError("observe() without a preceding act(): history would repeat")
        q = np.asarray(q_measured, dtype=np.float32).reshape(-1)
        dq = np.asarray(dq_measured, dtype=np.float32).reshape(-1)
        check_feedback(q, dq, self.manifest)
        self._tick += 1
        frame = self._frame(
            q,
            dq,
            self.manifest,
            issued=self._decoder.issued,
            valid=1.0,
            time_s=self._tick * self.manifest.tick_period_s,
        )
        self._frames.append(frame)
        if len(self._frames) > self.manifest.observation.history_frames:
            del self._frames[: len(self._frames) - self.manifest.observation.history_frames]
        self._awaiting_observation = False
        return frame

    # ------------------------------------------------------------- observation
    @staticmethod
    def _frame(
        q: NDArray[np.float32],
        dq: NDArray[np.float32],
        manifest: PolicyManifest,
        *,
        issued: NDArray[np.float32],
        valid: float,
        time_s: float,
    ) -> NDArray[np.float32]:
        frame = np.concatenate(
            [
                q,
                dq,
                np.asarray(issued, dtype=np.float32).reshape(-1),
                np.asarray([valid], dtype=np.float32),
                np.asarray([time_s], dtype=np.float32),
            ]
        ).astype(np.float32)
        if frame.shape != (manifest.observation.frame_dim,):
            raise PolicyContractError(
                f"observation frame has {frame.shape[0]} values, expected "
                f"{manifest.observation.frame_dim}"
            )
        return frame

    def observation(self, row: ReferenceRow) -> NDArray[np.float32]:
        """Flattened actor input: history (oldest first) followed by the goal.

        The layout is the trained one: ``cat(history.flatten(), goal_motion)``
        where ``goal_motion`` is the recorded TCP pose of this tick followed by
        the reference joint positions and velocities.
        """
        if self._tick is None:
            raise PolicyContractError("runtime has no session: reset it first")
        if len(self._frames) != self.manifest.observation.history_frames:
            raise PolicyContractError(
                f"history has {len(self._frames)} frames, expected "
                f"{self.manifest.observation.history_frames}"
            )
        goal = np.asarray(row.goal_motion, dtype=np.float32).reshape(-1)
        check_goal_motion(goal, self.manifest)
        obs = np.concatenate([np.stack(self._frames).reshape(-1), goal]).reshape(
            1, self.manifest.model.obs_dim
        )
        check_observation(obs.astype(np.float32), self.manifest)
        return np.ascontiguousarray(obs, dtype=np.float32)

    # ---------------------------------------------------------------- inference
    def infer(self, observation: NDArray[np.float32]) -> NDArray[np.float32]:
        """Run the deterministic actor on an already-built observation."""
        check_observation(observation, self.manifest)
        action = self._actor.run(observation)
        if action.shape != (self.manifest.model.action_dim,):
            raise PolicyContractError(
                f"actor returned {action.shape}, expected ({self.manifest.model.action_dim},)"
            )
        if not np.isfinite(action).all():
            raise PolicyContractError("actor returned non-finite actions")
        return action

    def act(self, row: ReferenceRow) -> NDArray[np.float32]:
        """Infer from the current history and return this tick's joint command."""
        command = self._decoder.decode(self.infer(self.observation(row)), row)
        self._tick_guard(row)
        self._awaiting_observation = True
        return command

    def _tick_guard(self, row: ReferenceRow) -> None:
        """The reference row must belong to the tick the history ends on."""
        expected = self.tick * self.manifest.tick_period_s
        if abs(float(row.time_s) - expected) > 1e-4:
            raise PolicyContractError(
                f"reference row time {float(row.time_s):.4f}s does not match tick "
                f"{self.tick} ({expected:.4f}s)"
            )

    @property
    def actor_providers(self) -> list[str]:
        providers: list[str] = self._actor.providers
        return providers

    @property
    def normalizer(self) -> dict[str, Any]:
        """The learned statistics, exposed for parity checks and diagnostics."""
        return {
            "obs_mean": self._obs_mean,
            "obs_std": self._obs_std,
            "goal_mean": self._goal_mean,
            "goal_std": self._goal_std,
        }


def episode_finished(tick: int, manifest: PolicyManifest) -> bool:
    """The trained episode is exactly ``episode_ticks`` commands long."""
    return tick >= manifest.timing.episode_ticks


def clip_for(manifest: PolicyManifest, model_dir: str | Path) -> ReferenceClip:
    """Load the manifest's frozen reference clip."""
    path = manifest.verify_reference(model_dir)
    assert manifest.reference is not None
    return ReferenceClip(path, hz=manifest.reference.hz)
