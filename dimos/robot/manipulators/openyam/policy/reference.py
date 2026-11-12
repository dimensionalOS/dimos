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

"""Frozen reference motion the OpenYAM policy is driven from.

The trained controller consumes the reference *bank row* it was trained on:
recorded joint positions and velocities plus the recorded TCP pose, sampled at
30 Hz. This module loads one such row as an immutable clip and answers the two
questions a deployment needs - "which tick may this session start on?" and
"what is the reference at tick ``k``?" - so no controller has to invent a
reference from IK, from an operator pose or from a zero-filled placeholder.

A live retargeting producer does not exist in this repository. When one is
added it must deliver the same five fields at the same timestamp; anything
less (positions only, or a Cartesian goal without ``q_ref``/``dq_ref``) would
change what the actor sees.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np
from numpy.typing import NDArray

from dimos.robot.manipulators.openyam.policy.contract import (
    EPISODE_TICKS,
    PolicyContractError,
    sha256_file,
)


@dataclass(frozen=True)
class ReferenceRow:
    """One 30 Hz reference tick: the reference state and the goal it implies."""

    index: int
    time_s: float
    q: NDArray[np.float32]
    dq: NDArray[np.float32]
    goal_position: NDArray[np.float32]
    goal_quaternion_wxyz: NDArray[np.float32]

    @property
    def goal_motion(self) -> NDArray[np.float32]:
        """The trained 19-value goal: TCP pose, then reference ``q`` and ``dq``."""
        return np.concatenate(
            [
                self.goal_position,
                self.goal_quaternion_wxyz,
                self.q,
                self.dq,
            ]
        ).astype(np.float32)


class ReferenceClip:
    """A hash-checked, read-only reference motion clip.

    The file format is the training bank row format: ``time_s``, ``q``,
    ``dq``, ``tcp_pos`` and ``tcp_quat`` (``wxyz``), all sampled at the policy
    rate. Nothing here synthesises values: a missing or non-finite array is a
    contract error, not a zero fill.
    """

    FIELDS = ("time_s", "q", "dq", "tcp_pos", "tcp_quat")

    def __init__(
        self,
        path: str | Path,
        *,
        hz: float,
        expected_sha256: str | None = None,
        episode_ticks: int = EPISODE_TICKS,
    ) -> None:
        self.path = Path(path)
        if expected_sha256 is not None:
            actual = sha256_file(self.path)
            if actual != expected_sha256:
                raise PolicyContractError(
                    f"reference clip hash mismatch at {self.path}: {actual} != {expected_sha256}"
                )
        with np.load(self.path, allow_pickle=False) as data:
            missing = set(self.FIELDS) - set(data.files)
            if missing:
                raise PolicyContractError(f"reference clip is missing arrays: {sorted(missing)}")
            self._time = self._read(data["time_s"], "time_s")
            self._q = self._read(data["q"], "q")
            self._dq = self._read(data["dq"], "dq")
            self._tcp_pos = self._read(data["tcp_pos"], "tcp_pos")
            self._tcp_quat = self._read(data["tcp_quat"], "tcp_quat")
        self.hz = float(hz)
        self.episode_ticks = int(episode_ticks)
        self._validate()
        for array in (self._time, self._q, self._dq, self._tcp_pos, self._tcp_quat):
            array.setflags(write=False)

    @staticmethod
    def _read(array: NDArray[np.float64], name: str) -> NDArray[np.float32]:
        values = np.asarray(array, dtype=np.float32)
        if not np.isfinite(values).all():
            raise PolicyContractError(f"reference clip {name} contains non-finite values")
        return np.array(values, dtype=np.float32, copy=True)

    def _validate(self) -> None:
        length = self._time.shape[0]
        if length < self.episode_ticks + 1:
            raise PolicyContractError(
                f"reference clip has {length} ticks, needs at least {self.episode_ticks + 1}"
            )
        if self._q.shape != (length, 6) or self._dq.shape != (length, 6):
            raise PolicyContractError("reference clip q/dq must be (ticks, 6)")
        if self._tcp_pos.shape != (length, 3) or self._tcp_quat.shape != (length, 4):
            raise PolicyContractError("reference clip tcp pose must be (ticks, 3) and (ticks, 4)")
        steps = np.diff(self._time)
        if not (np.abs(steps - 1.0 / self.hz) <= 1e-3).all():
            raise PolicyContractError(
                f"reference clip is not uniform at {self.hz} Hz (step "
                f"{float(np.median(steps)) if steps.size else 0.0:.6f}s)"
            )
        if not np.allclose(self._time, np.arange(length, dtype=np.float32) / self.hz, atol=1e-3):
            raise PolicyContractError("reference clip time axis does not start at zero")
        norms = np.linalg.norm(self._tcp_quat, axis=1)
        if not (np.abs(norms - 1.0) <= 1e-3).all():
            raise PolicyContractError("reference clip quaternions are not unit length")

    @property
    def length(self) -> int:
        return int(self._time.shape[0])

    def row(self, index: int) -> ReferenceRow:
        """The reference tick at ``index``; out-of-range ticks are a contract error."""
        if not 0 <= index < self.length:
            raise PolicyContractError(f"reference tick {index} outside 0..{self.length - 1}")
        return ReferenceRow(
            index=index,
            time_s=float(self._time[index]),
            q=self._q[index].copy(),
            dq=self._dq[index].copy(),
            goal_position=self._tcp_pos[index].copy(),
            goal_quaternion_wxyz=self._tcp_quat[index].copy(),
        )

    def anchor(
        self,
        q_measured: NDArray[np.float32] | NDArray[np.float64],
        *,
        tolerance: float,
    ) -> int:
        """Index whose reference pose best matches the measured pose.

        The trained reset starts the plant *on* the reference row, so a session
        may only start where the arm already is. If no candidate is within
        ``tolerance`` the session must not open: starting anyway would feed the
        actor a reference it never saw and jump the arm.
        """
        q = np.asarray(q_measured, dtype=np.float32).reshape(-1)
        if q.shape != (6,):
            raise PolicyContractError(f"measured pose shape {q.shape} != (6,)")
        if not np.isfinite(q).all():
            raise PolicyContractError("measured pose contains non-finite values")
        last = self.length - self.episode_ticks - 1
        if last < 0:
            raise PolicyContractError("reference clip is shorter than one episode")
        distance = np.abs(self._q[: last + 1] - q).max(axis=1)
        best = int(np.argmin(distance))
        if float(distance[best]) > tolerance:
            raise PolicyContractError(
                f"no reference tick within {tolerance:.3f} rad of the measured pose "
                f"(closest {float(distance[best]):.3f} rad at tick {best})"
            )
        return best

    def q_range(self) -> tuple[NDArray[np.float32], NDArray[np.float32]]:
        """Minimum and maximum reference positions per joint, for envelope audits."""
        return self._q.min(axis=0), self._q.max(axis=0)


class ClipReference:
    """Reference source that replays a frozen recorded clip.

    This is evaluation scaffolding, not a deployment input path: it exists so the
    same policy task can be driven from the recorded reference rows the training
    rollout used, which is how the integrated chain is compared against the
    original one. Ticks past the end of the clip are a contract error - the task
    ends the session rather than repeating or padding a row.
    """

    def __init__(
        self,
        clip: ReferenceClip,
        *,
        anchor_index: int | None = None,
        hz: float = 30.0,
    ) -> None:
        self._clip = clip
        self._period_s = 1.0 / float(hz)
        self._anchor = 0 if anchor_index is None else int(anchor_index)
        self._anchored = False

    @property
    def anchored(self) -> bool:
        return self._anchored

    @property
    def anchor_index(self) -> int:
        return self._anchor

    def anchor(self, q_measured: NDArray[np.float32], pose: object, t_now: float) -> ReferenceRow:
        """Anchor at ``anchor_index``; ``pose`` is unused for a recorded clip."""
        del q_measured, pose, t_now
        self._anchored = True
        return self._row(0)

    def update(self, pose: object, t_now: float, *, tick_index: int) -> ReferenceRow:
        del pose, t_now
        if not self._anchored:
            raise PolicyContractError("clip reference is not anchored")
        return self._row(int(tick_index))

    def release(self) -> None:
        self._anchored = False

    def _row(self, tick_index: int) -> ReferenceRow:
        source = self._clip.row(self._anchor + tick_index)
        return ReferenceRow(
            index=tick_index,
            time_s=tick_index * self._period_s,
            q=source.q,
            dq=source.dq,
            goal_position=source.goal_position,
            goal_quaternion_wxyz=source.goal_quaternion_wxyz,
        )
