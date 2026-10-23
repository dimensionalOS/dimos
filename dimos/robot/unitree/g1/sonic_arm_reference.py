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

"""Joint-space arm reference -> SONIC v1.1 sonic78 tokens.

Encoder mode 0 input: ten future frames 5 ticks apart at 50 Hz, absolute joints and velocities in
IsaacLab order, identity anchor, terminal pose held with zero velocity. The baseline is the measured
body pose, so the reference holds the robot where it is. Commissioned on a G1 onboard PC (PC2) at up to
0.5 rad of left shoulder pitch; a planner-frame baseline was rejected (larger side motion in MuJoCo).
"""

from __future__ import annotations

import hashlib
from pathlib import Path
from typing import Any

import numpy as np
from numpy.typing import NDArray

ISAAC_FROM_MOTOR = np.array(
    [
        0,
        6,
        12,
        1,
        7,
        13,
        2,
        8,
        14,
        3,
        9,
        15,
        22,
        4,
        10,
        16,
        23,
        5,
        11,
        17,
        24,
        18,
        25,
        19,
        26,
        20,
        27,
        21,
        28,
    ]
)
LEFT_SHOULDER_PITCH = 15  # Motor order; negative moves the hand forward.
ENCODER_SHA256 = "fb97de22819b2057b41459802128d91723d91a25f0ad73e7bfc41a9cf8365bae"  # SONIC v1.1
RATE = 50
_ARM = [15, 16, 17, 18, 22, 23, 24, 25]
_WRIST = [19, 20, 21, 26, 27, 28]


class SonicEncoder:
    """Hash-checked SONIC v1.1 encoder on one CPU thread (about 1.5 s per 350 rows on the G1 PC2)."""

    def __init__(self, path: Path) -> None:
        import onnxruntime as ort

        if hashlib.sha256(path.read_bytes()).hexdigest() != ENCODER_SHA256:
            raise ValueError(f"{path} is not the SONIC v1.1 encoder")
        options = ort.SessionOptions()  # type: ignore[attr-defined]
        options.intra_op_num_threads = options.inter_op_num_threads = 1
        self._session = ort.InferenceSession(str(path), options, providers=["CPUExecutionProvider"])

    def encode(self, body_motor: NDArray[Any]) -> NDArray[np.float64]:
        """[N,29] motor-order poses at 50 Hz -> [N,64] tokens."""
        body = np.asarray(body_motor, np.float64)[:, ISAAC_FROM_MOTOR]
        vel = np.gradient(body, 1 / RATE, axis=0) if len(body) > 1 else np.zeros_like(body)
        vel[-1] = 0
        body = np.vstack([body, np.repeat(body[-1:], 46, 0)])
        vel = np.vstack([vel, np.zeros((46, 29))])
        rows = np.arange(len(body_motor))[:, None] + np.arange(10)[None] * 5
        x = np.zeros((len(body_motor), 1751), np.float32)
        x[:, 4:294] = body[rows].reshape(-1, 290)
        x[:, 294:584] = vel[rows].reshape(-1, 290)
        x[:, 584:644] = np.tile([1, 0, 0, 1, 0, 0], 10)
        tokens = [self._session.run(None, {"obs_dict": row[None]})[0] for row in x]
        return np.asarray(np.concatenate(tokens), np.float64)


def min_jerk(n: int) -> NDArray[np.float64]:
    s = np.linspace(0, 1, n)
    return np.asarray(10 * s**3 - 15 * s**4 + 6 * s**5, np.float64)


def arm_poses(
    base: NDArray[Any],
    joint: int,
    delta: float,
    *,
    lead_s: float = 1.0,
    move_s: float = 2.0,
    hold_s: float = 1.0,
) -> NDArray[np.float64]:
    """Base pose, lead-in, move by delta, hold, move back, lead-out. [N,29] at 50 Hz."""
    lead, move, hold = (round(s * RATE) for s in (lead_s, move_s, hold_s))
    profile = min_jerk(move)
    offset = np.r_[
        np.zeros(lead), delta * profile, np.full(hold, delta), delta * profile[::-1], np.zeros(lead)
    ]
    poses = np.tile(np.asarray(base, np.float64), (len(offset), 1))
    poses[:, joint] += offset
    return poses


def blend(a: NDArray[Any], b: NDArray[Any], seconds: float) -> NDArray[np.float64]:
    w = np.linspace(0, 1, round(seconds * RATE) + 1)[1:, None]
    return np.asarray((1 - w) * a + w * b, np.float64)


def token_schedule(
    captured: NDArray[Any], motion_tokens: NDArray[Any], blend_s: float = 2.0
) -> NDArray[np.float64]:
    """Planner token -> blend -> motion tokens -> blend -> planner token, at 50 Hz.

    The 2 s blends follow the lerobot finding that an instant switch from the planner token
    unloaded a foot.
    """
    captured = np.asarray(captured, np.float64)
    return np.vstack(
        [
            blend(captured, motion_tokens[0], blend_s),
            motion_tokens,
            blend(motion_tokens[-1], captured, blend_s),
        ]
    )


def group_margins(body: float, arm: float, wrist: float) -> NDArray[np.float64]:
    """Per-joint envelope, motor order: legs and waist `body`, shoulders/elbows `arm`, wrists `wrist`.

    SONIC's decoder moves the wrists by itself (low torque) when an arm moves: 0.05-0.16 rad on a G1.
    """
    margins = np.full(29, body)
    margins[_ARM] = arm
    margins[_WRIST] = wrist
    return margins
