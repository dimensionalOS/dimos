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

"""Streamed reference-motion merging, ported from SONIC's C++ reference.

Faithful port of StreamedMotionMerger (gear_sonic_deploy
.../input_interface/streamed_motion_merger.hpp) and the protocol-version
handling of the C++ endpoint: incoming pose-message chunks (protocol v1
joint-based, v2 SMPL, v3 both) merge into a sliding-window motion the
policy encoder consumes. Semantics preserved exactly:

- frame_step detected from consecutive frame indices
- sliding window anchored to the playback cursor minus HISTORY_FRAMES
- catch-up reset when the gap exceeds MAX_GAP_FRAMES (+history) with
  catch_up enabled, when incoming data predates the window, or when it
  does not extend it
- old frames re-copied to fill the gap between window start and the
  incoming chunk
- protocol -> encoder mode: v1 -> 0 (g1 joints), v2/v3 -> 2 (SMPL)
- a protocol-version change mid-session is an error (caller falls back)
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from numpy.typing import NDArray

from dimos.utils.logging_config import setup_logger

logger = setup_logger()

HISTORY_FRAMES = 5
MAX_GAP_FRAMES = 200

NUM_JOINTS = 29
NUM_SMPL_JOINTS = 24
NUM_SMPL_POSES = 21


@dataclass
class StreamedMotion:
    """Merged sliding-window motion. Joint data in ONNX/IsaacLab order
    (the wire convention of the pose topic)."""

    joint_pos: NDArray  # [T, 29]
    joint_vel: NDArray  # [T, 29]
    root_quat: NDArray  # [T, 4] (w, x, y, z) - body_quat[:, 0]
    smpl_joints: NDArray | None  # [T, 24, 3]
    smpl_pose: NDArray | None  # [T, 21, 3]
    encode_mode: int = 0
    timesteps: int = 0


@dataclass
class MergeResult:
    motion: StreamedMotion | None = None
    window_start: int = 0
    frame_offset_adjustment: int = 0
    did_catchup_reset: bool = False
    frame_step: int = 1
    protocol_version: int = 0
    error: str | None = None


def infer_protocol_version(fields: dict[str, NDArray]) -> int:
    """v3: SMPL + joints; v2: SMPL only; v1: joints only (upstream protocol rules)."""
    has_smpl = "smpl_joints" in fields and "smpl_pose" in fields
    has_joints = "joint_pos" in fields and "joint_vel" in fields
    if has_smpl and has_joints:
        return 3
    if has_smpl:
        return 2
    if has_joints:
        return 1
    return 0


class StreamedMotionMerger:
    def __init__(self) -> None:
        self.reset()

    def reset(self) -> None:
        self._motion: StreamedMotion | None = None
        self._window_start = 0
        self._active_protocol: int | None = None

    def merge(self, fields: dict[str, NDArray], current_playback_frame: int) -> MergeResult:
        """Merge one decoded pose-topic message. ``fields`` are the raw
        decoded arrays keyed by wire name."""
        result = MergeResult()

        # The protocol version is a property of the sender (which field set it
        # streams: v1 joints, v2 SMPL, v3 both), so it is established once from
        # the first chunk and held until reset(). It only ever "changes" when a
        # different source starts streaming without a reset in between - the
        # C++ endpoint treats that as an unrecoverable error, and so do we.
        incoming = infer_protocol_version(fields)
        if incoming == 0:
            result.error = "pose message has neither joint nor SMPL data"
            return result
        if self._active_protocol is None:
            self._active_protocol = incoming
        elif self._active_protocol != incoming:
            result.error = f"protocol version changed {self._active_protocol} -> {incoming}"
            result.protocol_version = incoming
            return result
        protocol = self._active_protocol

        frame_indices = fields.get("frame_index")
        # The pico teleop server names this field body_quat_w; the reference
        # senders use body_quat. The C++ accepts both.
        body_quat = fields.get("body_quat")
        if body_quat is None:
            body_quat = fields.get("body_quat_w")
        if frame_indices is None or body_quat is None:
            result.error = "missing frame_index or body_quat"
            return result
        frame_indices = np.asarray(frame_indices).ravel().astype(np.int64)
        num_frames = len(frame_indices)
        if num_frames == 0:
            result.error = "empty chunk"
            return result

        joint_pos = fields.get("joint_pos")
        joint_vel = fields.get("joint_vel")
        smpl_joints = fields.get("smpl_joints")
        smpl_pose = fields.get("smpl_pose")
        if protocol in (1, 3) and (joint_pos is None or joint_vel is None):
            result.error = f"protocol v{protocol} missing joint data"
            return result
        if protocol in (2, 3) and (smpl_joints is None or smpl_pose is None):
            result.error = f"protocol v{protocol} missing SMPL data"
            return result

        catch_up = True
        cu = fields.get("catch_up")
        if cu is not None:
            catch_up = bool(np.asarray(cu).ravel()[0])

        # frame step
        if num_frames >= 2:
            step = int(abs(frame_indices[1] - frame_indices[0]))
            frame_step = step if step > 0 else 1
        else:
            frame_step = 1

        incoming_start = int(frame_indices[0])
        incoming_end = int(frame_indices[-1])

        new_window_start, merge_dst, did_catchup = self._sliding_window(
            incoming_start, incoming_end, frame_step, current_playback_frame, catch_up
        )

        total = merge_dst + num_frames
        new = StreamedMotion(
            joint_pos=np.zeros((total, NUM_JOINTS), dtype=np.float32),
            joint_vel=np.zeros((total, NUM_JOINTS), dtype=np.float32),
            root_quat=np.tile(np.array([1, 0, 0, 0], dtype=np.float32), (total, 1)),
            smpl_joints=(
                np.zeros((total, NUM_SMPL_JOINTS, 3), dtype=np.float32)
                if protocol in (2, 3)
                else None
            ),
            smpl_pose=(
                np.zeros((total, NUM_SMPL_POSES, 3), dtype=np.float32)
                if protocol in (2, 3)
                else None
            ),
            encode_mode=0 if protocol == 1 else 2,
            timesteps=total,
        )

        # copy old frames to fill [new_window_start, incoming_start)
        if merge_dst > 0 and self._motion is not None and self._motion.timesteps > 0:
            old = self._motion
            old_start = self._window_start
            old_end = old_start + frame_step * old.timesteps
            need_start, need_end = new_window_start, incoming_start
            ov_start = max(need_start, old_start)
            ov_end = min(need_end, old_end)
            if ov_start < ov_end:
                src0 = (ov_start - old_start) // frame_step
                dst0 = (ov_start - new_window_start) // frame_step
                n = (ov_end - ov_start) // frame_step
                n = min(n, old.timesteps - src0, total - dst0)
                if n > 0:
                    new.joint_pos[dst0 : dst0 + n] = old.joint_pos[src0 : src0 + n]
                    new.joint_vel[dst0 : dst0 + n] = old.joint_vel[src0 : src0 + n]
                    new.root_quat[dst0 : dst0 + n] = old.root_quat[src0 : src0 + n]
                    if new.smpl_joints is not None and old.smpl_joints is not None:
                        new.smpl_joints[dst0 : dst0 + n] = old.smpl_joints[src0 : src0 + n]
                        new.smpl_pose[dst0 : dst0 + n] = old.smpl_pose[src0 : src0 + n]

        # copy incoming
        if joint_pos is not None:
            jp = np.asarray(joint_pos, dtype=np.float32).reshape(num_frames, -1)
            new.joint_pos[merge_dst:, : jp.shape[1]] = jp[:, :NUM_JOINTS]
        if joint_vel is not None:
            jv = np.asarray(joint_vel, dtype=np.float32).reshape(num_frames, -1)
            new.joint_vel[merge_dst:, : jv.shape[1]] = jv[:, :NUM_JOINTS]
        bq = np.asarray(body_quat, dtype=np.float32).reshape(num_frames, -1, 4)
        new.root_quat[merge_dst:] = bq[:, 0, :]
        if new.smpl_joints is not None:
            new.smpl_joints[merge_dst:] = np.asarray(smpl_joints, dtype=np.float32).reshape(
                num_frames, NUM_SMPL_JOINTS, 3
            )
            new.smpl_pose[merge_dst:] = np.asarray(smpl_pose, dtype=np.float32).reshape(
                num_frames, NUM_SMPL_POSES, 3
            )

        old_window_start = self._window_start
        window_shift = (new_window_start - old_window_start) // frame_step if frame_step > 0 else 0

        self._motion = new
        self._window_start = new_window_start

        result.motion = new
        result.window_start = new_window_start
        result.frame_offset_adjustment = 0 if did_catchup else window_shift
        result.did_catchup_reset = did_catchup
        result.frame_step = frame_step
        result.protocol_version = protocol
        return result

    def _sliding_window(
        self,
        incoming_start: int,
        incoming_end: int,
        frame_step: int,
        current_playback_frame: int,
        catch_up_enabled: bool,
    ) -> tuple[int, int, bool]:
        # first packet
        if self._motion is None or self._motion.timesteps <= 0:
            return incoming_start, 0, True

        max_gap = (MAX_GAP_FRAMES + HISTORY_FRAMES) if catch_up_enabled else 2**31
        window_end = self._window_start + frame_step * (self._motion.timesteps - 1)
        global_playback = self._window_start + frame_step * max(
            0, current_playback_frame - HISTORY_FRAMES
        )

        # older than window, or does not extend it -> catch-up
        if incoming_start <= self._window_start or incoming_end <= window_end:
            return incoming_start, 0, True

        tentative_start = min(global_playback, incoming_start)
        merge_dst = (incoming_start - tentative_start) // frame_step if frame_step > 0 else 0
        large_gap = incoming_start > window_end + frame_step

        if merge_dst > max_gap or large_gap:
            return incoming_start, 0, True
        return tentative_start, merge_dst, False
