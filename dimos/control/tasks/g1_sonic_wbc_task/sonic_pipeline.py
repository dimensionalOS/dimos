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

"""SONIC (GEAR-SONIC) inference pipeline, simulator-agnostic.

Planner (10 Hz, background thread) -> Encoder (50 Hz) -> Decoder (50 Hz)
producing 29 joint position targets. Ported from the Matrix project's
parity-verified reimplementation of NVIDIA's C++ reference
(GR00T-WholeBodyControl/gear_sonic_deploy/.../g1_deploy_onnx_ref.cpp);
all observation layouts, joint orderings, gains, and the encoder-injection
rule match that reference. See sonic-notebook/DECISIONS.md D3: upper-body
targets enter ONLY through the encoder observation - never override the
decoder's output.

This module has no DimOS or simulator dependencies: callers feed joint
state (DDS/MuJoCo order), an IMU quaternion (w,x,y,z), and body-frame
angular velocity; ``step()`` returns 29 position targets in DDS order.
"""

from __future__ import annotations

from collections import deque
from concurrent.futures import Future, ThreadPoolExecutor
from dataclasses import dataclass
import math
from pathlib import Path
import time
from typing import Any, Final, Literal, TypeAlias, cast

import numpy as np
from numpy.typing import NDArray
import onnxruntime as ort  # type: ignore[import-untyped]

from dimos.control.tasks.g1_sonic_wbc_task.sonic_onnx_runtime import (
    create_sonic_session,
    prepare_sonic_onnx_runtime,
)
from dimos.control.tasks.g1_sonic_wbc_task.streamed_motion import (
    StreamedMotion,
    StreamedMotionMerger,
)
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

# ---------------------------------------------------------------------------
# Motor constants (policy_parameters.hpp)
# ---------------------------------------------------------------------------

ARMATURE_5020 = 0.003609725
ARMATURE_7520_14 = 0.010177520
ARMATURE_7520_22 = 0.025101925
ARMATURE_4010 = 0.00425

NATURAL_FREQ = 10 * 2 * math.pi
DAMPING_RATIO = 2.0

STIFFNESS_5020 = ARMATURE_5020 * NATURAL_FREQ**2
STIFFNESS_7520_14 = ARMATURE_7520_14 * NATURAL_FREQ**2
STIFFNESS_7520_22 = ARMATURE_7520_22 * NATURAL_FREQ**2
STIFFNESS_4010 = ARMATURE_4010 * NATURAL_FREQ**2

DAMPING_5020 = 2.0 * DAMPING_RATIO * ARMATURE_5020 * NATURAL_FREQ
DAMPING_7520_14 = 2.0 * DAMPING_RATIO * ARMATURE_7520_14 * NATURAL_FREQ
DAMPING_7520_22 = 2.0 * DAMPING_RATIO * ARMATURE_7520_22 * NATURAL_FREQ
DAMPING_4010 = 2.0 * DAMPING_RATIO * ARMATURE_4010 * NATURAL_FREQ

EFFORT_5020 = 25.0
EFFORT_7520_14 = 88.0
EFFORT_7520_22 = 139.0
EFFORT_4010 = 5.0

# PD gains in DDS/MuJoCo joint order, matching the C++ kps/kds arrays
# exactly - including the x2 on ankles and waist roll/pitch. The policy
# was trained against these; the blueprint must pass them as wb_config.
_KP_LEG = [
    STIFFNESS_7520_22,
    STIFFNESS_7520_22,
    STIFFNESS_7520_14,
    STIFFNESS_7520_22,
    2.0 * STIFFNESS_5020,
    2.0 * STIFFNESS_5020,
]
_KD_LEG = [
    DAMPING_7520_22,
    DAMPING_7520_22,
    DAMPING_7520_14,
    DAMPING_7520_22,
    2.0 * DAMPING_5020,
    2.0 * DAMPING_5020,
]
_KP_ARM = [
    STIFFNESS_5020,
    STIFFNESS_5020,
    STIFFNESS_5020,
    STIFFNESS_5020,
    STIFFNESS_5020,
    STIFFNESS_4010,
    STIFFNESS_4010,
]
_KD_ARM = [
    DAMPING_5020,
    DAMPING_5020,
    DAMPING_5020,
    DAMPING_5020,
    DAMPING_5020,
    DAMPING_4010,
    DAMPING_4010,
]
SONIC_KP: list[float] = [
    *_KP_LEG,
    *_KP_LEG,
    STIFFNESS_7520_14,
    2.0 * STIFFNESS_5020,
    2.0 * STIFFNESS_5020,  # waist
    *_KP_ARM,
    *_KP_ARM,
]
SONIC_KD: list[float] = [
    *_KD_LEG,
    *_KD_LEG,
    DAMPING_7520_14,
    2.0 * DAMPING_5020,
    2.0 * DAMPING_5020,  # waist
    *_KD_ARM,
    *_KD_ARM,
]

# ---------------------------------------------------------------------------
# Joint orderings. "DDS order" here equals the MuJoCo order used across
# DimOS G1 code (legs L/R, waist, arms L/R). "ONNX order" is SONIC's
# interleaved left/right BFS training order.
# ---------------------------------------------------------------------------

NUM_JOINTS = 29
HISTORY_LEN = 10
ENCODER_REFERENCE_FRAMES = 10

SonicTeleopPipeline: TypeAlias = Literal["sonic-v1.1", "sonic-low-latency"]
SONIC_V1_1_PIPELINE: Final[SonicTeleopPipeline] = "sonic-v1.1"
SONIC_LOW_LATENCY_PIPELINE: Final[SonicTeleopPipeline] = "sonic-low-latency"


@dataclass(frozen=True)
class SonicModelProfile:
    """One indivisible NVIDIA SONIC model and observation-layout contract."""

    name: SonicTeleopPipeline
    model_subdir: str
    encoder_obs_dim: int
    smpl_frames: int
    g1_frame_stride: int
    heading_normalized: bool
    encoder_sha256: str
    decoder_sha256: str

    @property
    def smpl_anchor_offset(self) -> int:
        return SMPL_JOINTS_OFFSET + self.smpl_frames * 72

    @property
    def wrists_offset(self) -> int:
        return self.smpl_anchor_offset + self.smpl_frames * 6


SONIC_MODEL_PROFILES: Final[dict[SonicTeleopPipeline, SonicModelProfile]] = {
    SONIC_V1_1_PIPELINE: SonicModelProfile(
        name=SONIC_V1_1_PIPELINE,
        model_subdir="sonic_v1_1",
        encoder_obs_dim=1751,
        smpl_frames=10,
        g1_frame_stride=5,
        heading_normalized=True,
        encoder_sha256="fb97de22819b2057b41459802128d91723d91a25f0ad73e7bfc41a9cf8365bae",
        decoder_sha256="34bae8570d4a4421a5391a5c2befd745d4a02d182ec539e5f9da44c091c67509",
    ),
    SONIC_LOW_LATENCY_PIPELINE: SonicModelProfile(
        name=SONIC_LOW_LATENCY_PIPELINE,
        model_subdir="low_latency",
        encoder_obs_dim=1247,
        smpl_frames=4,
        g1_frame_stride=1,
        heading_normalized=False,
        encoder_sha256="60be43157f57d812f38bdbb740a5de5d5d070e8840d9edc16f02a91a6d06255b",
        decoder_sha256="c4ac2e74045e7cbfb568f15e6bf47ea7ce023df7a94322af50be223e0a628bab",
    ),
}


def sonic_model_profile(name: SonicTeleopPipeline) -> SonicModelProfile:
    """Return the exact released model contract selected by the CLI."""
    return SONIC_MODEL_PROFILES[name]


# ONNX index -> DDS index (isaaclab_to_mujoco in the C++)
ONNX_TO_DDS = np.array(
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
    ],
    dtype=np.intp,
)
# DDS index -> ONNX index (mujoco_to_isaaclab in the C++)
DDS_TO_ONNX = np.array(
    [
        0,
        3,
        6,
        9,
        13,
        17,
        1,
        4,
        7,
        10,
        14,
        18,
        2,
        5,
        8,
        11,
        15,
        19,
        21,
        23,
        25,
        27,
        12,
        16,
        20,
        22,
        24,
        26,
        28,
    ],
    dtype=np.intp,
)

DEFAULT_ANGLES_DDS = np.array(
    [
        -0.312,
        0.0,
        0.0,
        0.669,
        -0.363,
        0.0,
        -0.312,
        0.0,
        0.0,
        0.669,
        -0.363,
        0.0,
        0.0,
        0.0,
        0.0,
        0.2,
        0.2,
        0.0,
        0.6,
        0.0,
        0.0,
        0.0,
        0.2,
        -0.2,
        0.0,
        0.6,
        0.0,
        0.0,
        0.0,
    ],
    dtype=np.float32,
)

_SCALE_LEG = [
    0.25 * EFFORT_7520_22 / STIFFNESS_7520_22,
    0.25 * EFFORT_7520_22 / STIFFNESS_7520_22,
    0.25 * EFFORT_7520_14 / STIFFNESS_7520_14,
    0.25 * EFFORT_7520_22 / STIFFNESS_7520_22,
    0.25 * EFFORT_5020 / STIFFNESS_5020,
    0.25 * EFFORT_5020 / STIFFNESS_5020,
]
_SCALE_ARM = [
    0.25 * EFFORT_5020 / STIFFNESS_5020,
    0.25 * EFFORT_5020 / STIFFNESS_5020,
    0.25 * EFFORT_5020 / STIFFNESS_5020,
    0.25 * EFFORT_5020 / STIFFNESS_5020,
    0.25 * EFFORT_5020 / STIFFNESS_5020,
    0.25 * EFFORT_4010 / STIFFNESS_4010,
    0.25 * EFFORT_4010 / STIFFNESS_4010,
]
ACTION_SCALE_DDS = np.array(
    [
        *_SCALE_LEG,
        *_SCALE_LEG,
        0.25 * EFFORT_7520_14 / STIFFNESS_7520_14,
        0.25 * EFFORT_5020 / STIFFNESS_5020,
        0.25 * EFFORT_5020 / STIFFNESS_5020,
        *_SCALE_ARM,
        *_SCALE_ARM,
    ],
    dtype=np.float32,
)

DEFAULT_ANGLES_ONNX = DEFAULT_ANGLES_DDS[ONNX_TO_DDS]
ACTION_SCALE_ONNX = ACTION_SCALE_DDS[ONNX_TO_DDS]

# 6 wrist joints in ONNX order (wrist_joint_isaaclab_order_in_isaaclab_index)
WRIST_ONNX_INDICES = np.array([23, 24, 25, 26, 27, 28], dtype=np.intp)

# Encoder observation offsets for the SMPL (mode 2) fields
# Teleop (encoder mode 1) fields. Lowerbody gather uses MUJOCO-order indices
# into the IsaacLab-order joint array (policy_parameters.hpp
# lower_body_joint_mujoco_order_in_isaaclab_index) - NOT the sorted variant.
LOWER_BODY_MJC_IN_ONNX = np.array([0, 3, 6, 9, 13, 17, 1, 4, 7, 10, 14, 18], dtype=np.intp)
VR_STALE_SEC = 0.5  # hold-last window; stale -> revert to planner obs (mode 0)


# 17 upper-body joints (waist + arms) in ONNX-order indices, matching the
# C++ upper_body_joint_isaaclab_order_in_isaaclab_index.
UPPER_BODY_ONNX_INDICES = np.array(
    [2, 5, 8, 11, 12, 15, 16, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28],
    dtype=np.intp,
)

# ---------------------------------------------------------------------------
# Encoder observation layout - SONIC v1.1 (sonic_v1_1/observation_config.yaml;
# offsets verified against the C++ observation registry). 1751 = 4 (mode) +
# 290 (joint pos) + 290 (joint vel) + 60 (anchor hist) + 6 (anchor single) +
# 120 (lowerbody pos) + 120 (lowerbody vel) + 9 (vr pos) + 12 (vr orn) +
# 720 (smpl joints) + 60 (smpl anchor) + 60 (wrists). Anchor orientations
# are heading-normalized (C++ orientation_mode 1 - left quat is the robot's
# heading, not the full base quat).
# ---------------------------------------------------------------------------

ENCODER_TOKEN_DIM = 64
DECODER_OBS_DIM = 994

ANCHOR_HIST_OFFSET = 584  # motion_anchor_orientation_heading_10frame_step5: 60
ANCHOR_SINGLE_OFFSET = 644  # motion_anchor_orientation_heading: 6
LOWERBODY_POS_OFFSET = 650  # motion_joint_positions_lowerbody_10frame_step5: 120
LOWERBODY_VEL_OFFSET = 770  # motion_joint_velocities_lowerbody_10frame_step5: 120
VR_POS_OFFSET = 890  # vr_3point_local_target: 9
VR_ORN_OFFSET = 899  # vr_3point_local_orn_target: 12
SMPL_JOINTS_OFFSET = 911  # smpl_joints_10frame_step1: 720

DEFAULT_HEIGHT = 0.788740
POLICY_DT = 0.02
REPLAN_INTERVAL_DEFAULT = 1.0
REPLAN_INTERVAL_RUNNING = 0.1
BLEND_FRAMES = 8
LOOK_AHEAD_FRAMES = 2

_IDENTITY_6D = np.array([1.0, 0.0, 0.0, 1.0, 0.0, 0.0], dtype=np.float32)

# LocomotionMode (localmotion_kplanner.hpp) - the full 27.
LOCOMOTION_MODES: dict[str, int] = {
    "IDLE": 0,
    "SLOW_WALK": 1,
    "WALK": 2,
    "RUN": 3,
    "IDEL_SQUAT": 4,
    "IDEL_KNEEL_TWO_LEGS": 5,
    "IDEL_KNEEL": 6,
    "IDEL_LYING_FACE_DOWN": 7,
    "CRAWLING": 8,
    "IDEL_BOXING": 9,
    "WALK_BOXING": 10,
    "LEFT_PUNCH": 11,
    "RIGHT_PUNCH": 12,
    "RANDOM_PUNCH": 13,
    "ELBOW_CRAWLING": 14,
    "LEFT_HOOK": 15,
    "RIGHT_HOOK": 16,
    "FORWARD_JUMP": 17,
    "STEALTH_WALK": 18,
    "INJURED_WALK": 19,
    "LEDGE_WALKING": 20,
    "OBJECT_CARRYING": 21,
    "STEALTH_WALK_2": 22,
    "HAPPY_DANCE_WALK": 23,
    "ZOMBIE_WALK": 24,
    "GUN_WALK": 25,
    "SCARE_WALK": 26,
}
STATIC_MODES = {0, 4, 5, 6, 7, 9}

# Per-mode planner speed/height (gamepad_manager.hpp applySpeedAndHeight).
# Kneel/squat/crawl NEED the height command - with the -1 default the
# planner emits a floor-collapse descent instead of a supported kneel.
MODE_PLANNER_PARAMS: dict[int, tuple[float, float]] = {
    1: (0.4, -1.0),  # SLOW_WALK
    3: (1.5, -1.0),  # RUN
    4: (-1.0, 0.4),  # IDEL_SQUAT
    5: (-1.0, 0.4),  # IDEL_KNEEL_TWO_LEGS
    6: (-1.0, 0.4),  # IDEL_KNEEL
    8: (0.7, 0.4),  # CRAWLING
    9: (0.7, -1.0),  # IDEL_BOXING
    10: (0.7, -1.0),  # WALK_BOXING
    11: (0.7, -1.0),  # LEFT_PUNCH
    12: (0.7, -1.0),  # RIGHT_PUNCH
    13: (0.7, -1.0),  # RANDOM_PUNCH
    14: (0.7, 0.3),  # ELBOW_CRAWLING
    15: (0.7, -1.0),  # LEFT_HOOK
    16: (0.7, -1.0),  # RIGHT_HOOK
}

# Floor-posture ladders (C++ gamepad_manager staging): every deep posture is
# reached through KNEEL_TWO_LEGS, one rung per TRANSITION_DWELL_SEC.
TRANSITION_DWELL_SEC = 2.0
_KNEEL2, _KNEEL, _LYING, _CRAWL, _ELBOW = 5, 6, 7, 8, 14
_FLOOR_CHAINS: dict[int, list[int]] = {
    _KNEEL2: [_KNEEL2],
    _KNEEL: [_KNEEL2, _KNEEL],
    _LYING: [_KNEEL2, _KNEEL, _LYING],
    _CRAWL: [_KNEEL2, _CRAWL],
    _ELBOW: [_KNEEL2, _CRAWL, _ELBOW],
}


def _transition_stages(current: int | None, target: int | None) -> list[int | None]:
    """Mode sequence from ``current`` to ``target`` (target included last).

    Mirrors gamepad_manager.hpp: entering a floor posture descends the
    ladder (stand -> kneel -> crawl -> elbow), leaving one ascends it, and
    switching floor branches goes back through the shared rungs. Non-floor
    to non-floor transitions are direct, exactly like the C++.
    """
    cur_chain = _FLOOR_CHAINS.get(current) if current is not None else None
    tgt_chain = _FLOOR_CHAINS.get(target) if target is not None else None
    if cur_chain is None and tgt_chain is None:
        return [target]
    if cur_chain is None:
        assert tgt_chain is not None
        return list(tgt_chain)
    if tgt_chain is None:
        up = list(reversed(cur_chain[:-1]))
        return [*up, target]
    common = 0
    for a, b in zip(cur_chain, tgt_chain, strict=False):
        if a != b:
            break
        common += 1
    up = list(reversed(cur_chain[common:-1]))
    down = tgt_chain[common:]
    stages: list[int | None] = [*up, *down]
    return stages if stages else [target]


# ---------------------------------------------------------------------------
# Quaternion helpers ([w, x, y, z] convention throughout)
# ---------------------------------------------------------------------------


def _quat_conjugate(q: NDArray[Any]) -> NDArray[Any]:
    return np.array([q[0], -q[1], -q[2], -q[3]], dtype=np.float64)


def _quat_multiply(q1: NDArray[Any], q2: NDArray[Any]) -> NDArray[Any]:
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array(
        [
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        ],
        dtype=np.float64,
    )


def _quat_to_rotmat(q: NDArray[Any]) -> NDArray[Any]:
    w, x, y, z = np.asarray(q, dtype=np.float64)
    n = math.sqrt(w * w + x * x + y * y + z * z)
    if n > 1e-10:
        w, x, y, z = w / n, x / n, y / n, z / n
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def _rotmat_to_6d(rot: NDArray[Any]) -> NDArray[Any]:
    return np.array(
        [rot[0, 0], rot[0, 1], rot[1, 0], rot[1, 1], rot[2, 0], rot[2, 1]],
        dtype=np.float32,
    )


def _quat_lerp(q0: NDArray[Any], q1: NDArray[Any], t: float) -> NDArray[Any]:
    q0 = np.asarray(q0, dtype=np.float64)
    q1 = np.asarray(q1, dtype=np.float64)
    if np.dot(q0, q1) < 0:
        q1 = -q1
    q = (1.0 - t) * q0 + t * q1
    n = np.linalg.norm(q)
    return (q / n if n > 1e-10 else q0).astype(np.float32)


def _yaw_from_quat(q: NDArray[Any]) -> float:
    w, x, y, z = q
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _calc_heading_quat(q: NDArray[Any]) -> NDArray[Any]:
    half = _yaw_from_quat(q) / 2.0
    return np.array([math.cos(half), 0.0, 0.0, math.sin(half)], dtype=np.float64)


def _calc_heading_quat_inv(q: NDArray[Any]) -> NDArray[Any]:
    half = -_yaw_from_quat(q) / 2.0
    return np.array([math.cos(half), 0.0, 0.0, math.sin(half)], dtype=np.float64)


class _Trajectory:
    """50 Hz reference motion (joint data stored in ONNX order)."""

    __slots__ = ("joint_pos", "joint_vel", "num_frames", "root_pos", "root_quat")

    def __init__(self, max_frames: int) -> None:
        self.joint_pos = np.zeros((max_frames, NUM_JOINTS), dtype=np.float32)
        self.joint_vel = np.zeros((max_frames, NUM_JOINTS), dtype=np.float32)
        self.root_pos = np.zeros((max_frames, 3), dtype=np.float32)
        self.root_quat = np.zeros((max_frames, 4), dtype=np.float32)
        self.root_quat[:, 0] = 1.0
        self.num_frames = 0


class SonicPipeline:
    """Planner -> encoder -> decoder pipeline over ONNX Runtime.

    Callers drive it at 50 Hz via :meth:`step`. The planner runs on a
    single background worker so ``step()`` never blocks on the 774 MB
    planner model.
    """

    def __init__(
        self,
        encoder_path: str | Path,
        decoder_path: str | Path,
        planner_path: str | Path,
        profile: SonicTeleopPipeline = SONIC_V1_1_PIPELINE,
    ) -> None:
        self._profile = sonic_model_profile(profile)
        prepare_sonic_onnx_runtime()
        self._encoder = create_sonic_session("encoder", encoder_path, allow_cpu_shape_ops=False)
        self._decoder = create_sonic_session("decoder", decoder_path, allow_cpu_shape_ops=False)
        # The released planner contains a small set of shape/index operators
        # unsupported by ORT 1.20's CUDA EP. sonic-doctor profiles and audits
        # that partition before hardware use; the neural planner remains CUDA.
        self._planner = create_sonic_session("planner", planner_path, allow_cpu_shape_ops=True)
        self._encoder_input = self._encoder.get_inputs()[0].name
        self._decoder_input = self._decoder.get_inputs()[0].name
        # Fail loudly on a mismatched checkpoint (e.g. the pre-v1.1 release,
        # whose encoder takes 1762 floats and a different field layout).
        enc_dim = int(cast("int", self._encoder.get_inputs()[0].shape[-1]))
        if enc_dim != self._profile.encoder_obs_dim:
            raise ValueError(
                f"SONIC {profile} encoder obs dim {enc_dim} != "
                f"{self._profile.encoder_obs_dim}; use the matching NVIDIA "
                f"{self._profile.model_subdir}/ encoder, decoder, and observation config"
            )
        decoder_dim = int(cast("int", self._decoder.get_inputs()[0].shape[-1]))
        if decoder_dim != DECODER_OBS_DIM:
            raise ValueError(f"SONIC {profile} decoder obs dim {decoder_dim} != {DECODER_OBS_DIM}")
        logger.info(
            "SonicPipeline models loaded",
            sonic_pipeline=profile,
            onnxruntime_version=getattr(ort, "__version__", "unknown"),
            encoder_providers=self._encoder.get_providers(),
            decoder_providers=self._decoder.get_providers(),
            planner_providers=self._planner.get_providers(),
        )

        self._standing_token = self._build_standing_token()

        self._his_ang_vel = np.zeros((HISTORY_LEN, 3), dtype=np.float32)
        self._his_joint_pos = np.zeros((HISTORY_LEN, NUM_JOINTS), dtype=np.float32)
        self._his_joint_vel = np.zeros((HISTORY_LEN, NUM_JOINTS), dtype=np.float32)
        self._his_action = np.zeros((HISTORY_LEN, NUM_JOINTS), dtype=np.float32)
        self._his_gravity = np.zeros((HISTORY_LEN, 3), dtype=np.float32)
        self._history_ptr = 0
        self._last_action = np.zeros(NUM_JOINTS, dtype=np.float32)
        self._obs_buffer = np.zeros(DECODER_OBS_DIM, dtype=np.float32)
        self._encoder_durations_ms: deque[float] = deque(maxlen=250)
        self._decoder_durations_ms: deque[float] = deque(maxlen=250)
        self._planner_durations_ms: deque[float] = deque(maxlen=50)

        self._trajectory: _Trajectory | None = None
        self._traj_frame = 0
        self._heading_delta_quat = np.array([1, 0, 0, 0], dtype=np.float64)
        self._heading_initialized = False

        self._planner_executor = ThreadPoolExecutor(
            max_workers=1, thread_name_prefix="sonic-planner"
        )
        self._planner_future: Future[list[Any]] | None = None
        self._planner_started_at: float | None = None
        self._replan_timer = 0.0
        self._needs_replan = True
        self._step_count = 0

        # Commands
        self._vx = 0.0
        self._vy = 0.0
        self._yaw_rate = 0.0
        self._height_cmd = -1.0  # -1 = mode default
        self._mode_override: int | None = None
        self._mode_queue: list[int | None] = []
        self._mode_dwell = 0.0
        self._upper_targets_dds = DEFAULT_ANGLES_DDS[15:].copy()

        # Latest robot state fed by step() (for planner input building)
        self._cur_quat = np.array([1, 0, 0, 0], dtype=np.float64)
        self._cur_q_dds = DEFAULT_ANGLES_DDS.copy()
        self._nan_reported = 0
        self._last_targets_dds = DEFAULT_ANGLES_DDS.copy()
        self._last_reference_token: NDArray[Any] | None = None
        self._last_token_was_stream = False

        # Streamed reference motion (ZMQ pose topic)
        self._merger = StreamedMotionMerger()
        self._streamed: StreamedMotion | None = None
        self._streamed_frame = 0
        self._use_stream = False
        self._reference_transition_start_token: NDArray[Any] | None = None
        self._reference_transition_step = 0
        self._reference_transition_steps = 0
        # Direct planner command (ZMQ planner topic); None -> twist-derived
        self._planner_cmd: dict[str, Any] | None = None
        self._upper_vel_dds: NDArray[Any] | None = None
        # Wire-order (17: waist + arms) upper-body buffers; take precedence
        # over the DDS-14 arm API when set
        self._ub17_pos: NDArray[Any] | None = None
        self._ub17_vel: NDArray[Any] | None = None
        # VR 3-point teleop (encoder mode 1). Root-relative, sender-normalized:
        # positions [L wrist, R wrist, head] xyz; orientations 3x quat wxyz.
        self._vr_pos: NDArray[Any] | None = None
        self._vr_orn: NDArray[Any] | None = None
        self._vr_time = 0.0

    # -- commands ---------------------------------------------------------

    @property
    def target_mode(self) -> int | None:
        """Final mode after any pending staged transition."""
        return self._mode_queue[-1] if self._mode_queue else self._mode_override

    def set_velocity(self, vx: float, vy: float, wz: float) -> None:
        if abs(vx - self._vx) > 0.05 or abs(vy - self._vy) > 0.05 or abs(wz - self._yaw_rate) > 0.1:
            self._needs_replan = True
        self._vx, self._vy, self._yaw_rate = vx, vy, wz

    def set_mode(self, mode: int | str | None) -> int | None:
        """Force a LocomotionMode (int or name); None returns to speed-auto.

        Floor postures are STAGED like the C++ gamepad manager
        (gamepad_manager.hpp): entering crawling kneels first, elbow
        crawling passes through crawling, and exits reverse the ladder -
        each stage holding TRANSITION_DWELL_SEC before the next. Jumping
        straight from a standing/walking context into a deep floor mode
        makes the planner emit a violent drop that the policy tracks into
        a crash. The staged target applies immediately; the remaining
        stages advance from step()."""
        if isinstance(mode, str):
            mode = LOCOMOTION_MODES[mode.upper()]
        if mode is not None and not 0 <= int(mode) <= 26:
            raise ValueError(f"locomotion mode out of range: {mode}")
        target = None if mode is None else int(mode)
        stages = _transition_stages(self._mode_override, target)
        self._mode_queue = stages[1:]
        self._mode_dwell = 0.0
        first = stages[0]
        if first != self._mode_override:
            self._needs_replan = True
        self._mode_override = first
        return target

    def set_base_height(self, height: float) -> None:
        if abs(height - self._height_cmd) > 0.01:
            self._needs_replan = True
        self._height_cmd = float(height)

    def set_upper_body(
        self, targets_dds_14: NDArray[Any], velocities_dds_14: NDArray[Any] | None = None
    ) -> None:
        self._upper_targets_dds = np.asarray(targets_dds_14, dtype=np.float32).flatten()[:14]
        self._upper_vel_dds = (
            None
            if velocities_dds_14 is None
            else np.asarray(velocities_dds_14, dtype=np.float32).flatten()[:14]
        )

    def set_upper_body_wire17(
        self, positions_17: NDArray[Any] | None, velocities_17: NDArray[Any] | None
    ) -> None:
        """Upper-body targets in ZMQ wire order (17: waist + arms). None clears."""
        self._ub17_pos = (
            None if positions_17 is None else np.asarray(positions_17, dtype=np.float32).reshape(17)
        )
        self._ub17_vel = (
            None
            if velocities_17 is None
            else np.asarray(velocities_17, dtype=np.float32).reshape(17)
        )

    def set_vr_3point(
        self, positions_9: NDArray[Any], orientations_12: NDArray[Any], t_now: float | None = None
    ) -> None:
        """VR 3-point teleop targets (encoder mode 1).

        Frame convention (matches C++ GatherVR3Point buffered path - values are
        copied into the encoder obs verbatim): point order left wrist, right
        wrist, head; positions root-relative (p_world - root_pos rotated into
        the root frame); orientations quat wxyz, root-relative
        (quat_mul(quat_inv(root_quat), q_world)); wrist offsets
        [0.18, -/+0.025, 0] and head offset [0, 0, 0.35] already applied by
        the sender. While fresh (< VR_STALE_SEC) the encoder runs in teleop
        mode; stale data reverts to planner obs.
        """
        self._vr_pos = np.asarray(positions_9, dtype=np.float32).reshape(9)
        self._vr_orn = np.asarray(orientations_12, dtype=np.float32).reshape(12)
        self._vr_time = time.perf_counter() if t_now is None else t_now

    def clear_vr_3point(self) -> None:
        self._vr_pos = None
        self._vr_orn = None
        self._vr_time = 0.0

    def _vr_active(self) -> bool:
        return self._vr_pos is not None and (time.perf_counter() - self._vr_time) < VR_STALE_SEC

    def set_source_stream(self, use_stream: bool) -> None:
        """Command-topic planner-flag inverse: True -> pose-topic motion."""
        self._clear_reference_transition()
        if use_stream != self._use_stream:
            self._needs_replan = not use_stream
            # Motion-source switch = heading re-anchor (C++ sets
            # reinitialize_heading_ on every motion switch). Without this the
            # next source keeps the previous source's heading delta - after a
            # clip, planner trajectories would stay anchored to the clip's
            # mocap heading and the policy turns instead of tracking.
            self._reset_heading_alignment()
        self._use_stream = bool(use_stream)

    @property
    def reference_transition_active(self) -> bool:
        """Whether an encoder-token source blend is in progress."""
        return self._reference_transition_start_token is not None

    @property
    def reference_transition_progress(self) -> float:
        """Completed fraction of the active encoder-token source blend."""
        if not self.reference_transition_active or self._reference_transition_steps <= 0:
            return 0.0
        return min(
            1.0,
            self._reference_transition_step / self._reference_transition_steps,
        )

    def begin_stream_transition(self, duration_seconds: float) -> bool:
        """Blend from the last planner token to the live streamed reference.

        The streamed motion must already be loaded. Returns ``False`` until
        at least one planner policy step has produced a reference token.
        """
        self._validate_reference_transition_duration(duration_seconds)
        if (
            self._use_stream
            or self._streamed is None
            or self._streamed.timesteps <= 0
            or self._last_reference_token is None
            or self._last_token_was_stream
        ):
            return False

        start_token = self._last_reference_token.copy()
        self.set_source_stream(True)
        self._start_reference_transition(start_token, duration_seconds)
        return True

    def begin_planner_transition(self, duration_seconds: float) -> bool:
        """Blend from the last streamed token to the balancing planner.

        Returns ``False`` when the policy has not produced a streamed token,
        in which case planner is already the last decoder reference.
        """
        self._validate_reference_transition_duration(duration_seconds)
        if (
            not self._use_stream
            or self._last_reference_token is None
            or not self._last_token_was_stream
        ):
            return False

        start_token = self._last_reference_token.copy()
        self.stop_clip()
        self._start_reference_transition(start_token, duration_seconds)
        return True

    @staticmethod
    def _validate_reference_transition_duration(duration_seconds: float) -> None:
        if not math.isfinite(duration_seconds) or duration_seconds <= 0.0:
            raise ValueError("reference transition duration must be positive and finite")

    def _start_reference_transition(
        self,
        start_token: NDArray[Any],
        duration_seconds: float,
    ) -> None:
        self._reference_transition_start_token = start_token
        self._reference_transition_step = 0
        self._reference_transition_steps = max(1, math.ceil(duration_seconds / POLICY_DT))

    def _clear_reference_transition(self) -> None:
        self._reference_transition_start_token = None
        self._reference_transition_step = 0
        self._reference_transition_steps = 0

    def _blend_reference_token(self, target_token: NDArray[Any]) -> NDArray[Any]:
        start_token = self._reference_transition_start_token
        if start_token is None:
            return target_token

        self._reference_transition_step = min(
            self._reference_transition_step + 1,
            self._reference_transition_steps,
        )
        linear = self._reference_transition_step / self._reference_transition_steps
        alpha = linear * linear * (3.0 - 2.0 * linear)
        return ((1.0 - alpha) * start_token + alpha * target_token).astype(np.float32)

    def _reset_heading_alignment(self) -> None:
        self._heading_delta_quat = np.array([1, 0, 0, 0], dtype=np.float64)
        self._heading_initialized = False

    def apply_heading_increment(self, increment_rad: float) -> None:
        """Operator yaw adjustment (pose-topic heading_increment field, pico
        joystick). C++ accumulates it into HeadingState.delta_heading, applied
        left of the ref-alignment quat; folding it into _heading_delta_quat is
        equivalent and resets with it on re-anchor."""
        if increment_rad == 0.0:
            return
        half = 0.5 * float(increment_rad)
        yaw_quat = np.array([math.cos(half), 0.0, 0.0, math.sin(half)], dtype=np.float64)
        self._heading_delta_quat = _quat_multiply(yaw_quat, self._heading_delta_quat)

    def set_planner_command(
        self,
        mode: int,
        movement: NDArray[Any],
        facing: NDArray[Any],
        speed: float = -1.0,
        height: float = -1.0,
    ) -> None:
        """Direct planner command (ZMQ planner topic); overrides twist mapping."""
        self._planner_cmd = {
            "mode": int(mode),
            "movement": np.asarray(movement, dtype=np.float32).reshape(3),
            "facing": np.asarray(facing, dtype=np.float32).reshape(3),
            "speed": float(speed),
            "height": float(height),
        }
        self._needs_replan = True

    def clear_planner_command(self) -> None:
        self._planner_cmd = None

    def play_clip(self, motion: StreamedMotion) -> None:
        """Play a disk reference clip through the streamed-motion path.

        Resets heading alignment so the clip is re-anchored to the robot's
        current heading (mirrors the C++ reference-motion switch)."""
        self._streamed = motion
        self._streamed_frame = 0
        self._clear_reference_transition()
        self._use_stream = True
        self._reset_heading_alignment()

    def stop_clip(self) -> None:
        """Back to planner-driven locomotion (heading re-anchors on the next
        planner trajectory - see set_source_stream)."""
        self._use_stream = False
        self._clear_reference_transition()
        self._streamed = None
        self._streamed_frame = 0
        self._merger.reset()
        self._needs_replan = True
        self._reset_heading_alignment()

    def apply_pose_message(self, fields: dict[str, NDArray[Any]]) -> dict[str, Any]:
        """Merge one decoded pose-topic chunk; returns a merge summary."""
        res = self._merger.merge(fields, self._streamed_frame)
        if res.error:
            logger.warning("SonicPipeline pose merge rejected", error=res.error)
            return {"error": res.error}
        self._streamed = res.motion
        if res.did_catchup_reset:
            self._streamed_frame = 0
        else:
            self._streamed_frame = max(0, self._streamed_frame - res.frame_offset_adjustment)
        return {
            "frames": res.motion.timesteps if res.motion else 0,
            "encode_mode": res.motion.encode_mode if res.motion else -1,
            "catchup": res.did_catchup_reset,
        }

    def reset(self) -> None:
        self._his_ang_vel[:] = 0.0
        self._his_joint_pos[:] = 0.0
        self._his_joint_vel[:] = 0.0
        self._his_action[:] = 0.0
        self._his_gravity[:] = 0.0
        self._history_ptr = 0
        self._last_action[:] = 0.0
        self._obs_buffer[:] = 0.0
        self._trajectory = None
        self._traj_frame = 0
        self._replan_timer = 0.0
        self._step_count = 0
        self._needs_replan = True
        self._heading_delta_quat = np.array([1, 0, 0, 0], dtype=np.float64)
        self._heading_initialized = False
        if self._planner_future is not None and not self._planner_future.done():
            self._planner_future.cancel()
        self._planner_future = None
        self._upper_targets_dds = DEFAULT_ANGLES_DDS[15:].copy()
        self._mode_override = None
        self._mode_queue = []
        self._mode_dwell = 0.0
        self._merger.reset()
        self._streamed = None
        self._streamed_frame = 0
        self._use_stream = False
        self._clear_reference_transition()
        self._last_reference_token = None
        self._last_token_was_stream = False
        self._planner_cmd = None
        self._upper_vel_dds = None
        self._ub17_pos = None
        self._ub17_vel = None

    # -- encoder ----------------------------------------------------------

    def _build_standing_token(self) -> NDArray[Any]:
        enc_obs = np.zeros(self._profile.encoder_obs_dim, dtype=np.float32)
        for i in range(ENCODER_REFERENCE_FRAMES):
            enc_obs[4 + i * NUM_JOINTS : 4 + (i + 1) * NUM_JOINTS] = DEFAULT_ANGLES_ONNX
            enc_obs[ANCHOR_HIST_OFFSET + i * 6 : ANCHOR_HIST_OFFSET + (i + 1) * 6] = _IDENTITY_6D
        out = self._encoder.run(None, {self._encoder_input: enc_obs.reshape(1, -1)})
        return out[0].squeeze().astype(np.float32)

    def _has_upper_body_targets(self) -> bool:
        if self._ub17_pos is not None:
            return True
        return not np.allclose(self._upper_targets_dds, DEFAULT_ANGLES_DDS[15:], atol=1e-6)

    def _upper_body_17_onnx(self) -> NDArray[Any]:
        if self._ub17_pos is not None:
            return self._ub17_pos
        full = DEFAULT_ANGLES_ONNX.copy()
        for dds_i in range(15, 29):
            full[DDS_TO_ONNX[dds_i]] = self._upper_targets_dds[dds_i - 15]
        return full[UPPER_BODY_ONNX_INDICES]

    def _upper_body_vel_17_onnx(self) -> NDArray[Any]:
        if self._ub17_vel is not None:
            return self._ub17_vel
        full = np.zeros(NUM_JOINTS, dtype=np.float32)
        if self._upper_vel_dds is not None:
            for dds_i in range(15, 29):
                full[DDS_TO_ONNX[dds_i]] = self._upper_vel_dds[dds_i - 15]
        return full[UPPER_BODY_ONNX_INDICES]

    def _inject_upper_body(self, enc_obs: NDArray[Any]) -> None:
        """Encoder-observation injection (D3): positions replaced; velocities
        replaced with provided upper-body velocities (zero when absent) for
        the 17 upper-body joints across all 10 frames."""
        upper_vals = self._upper_body_17_onnx()
        upper_vels = self._upper_body_vel_17_onnx()
        for i in range(ENCODER_REFERENCE_FRAMES):
            pos = 4 + i * NUM_JOINTS
            vel = 294 + i * NUM_JOINTS
            for k, idx in enumerate(UPPER_BODY_ONNX_INDICES):
                enc_obs[pos + idx] = upper_vals[k]
                enc_obs[vel + idx] = upper_vels[k]

    def _build_encoder_obs(self, base_quat: NDArray[Any]) -> NDArray[Any]:
        enc_obs = np.zeros(self._profile.encoder_obs_dim, dtype=np.float32)
        traj = self._trajectory
        assert traj is not None
        f_curr = min(self._traj_frame, traj.num_frames - 1)

        for i in range(ENCODER_REFERENCE_FRAMES):
            f = min(f_curr + i * self._profile.g1_frame_stride, traj.num_frames - 1)
            enc_obs[4 + i * NUM_JOINTS : 4 + (i + 1) * NUM_JOINTS] = traj.joint_pos[f]
            enc_obs[294 + i * NUM_JOINTS : 294 + (i + 1) * NUM_JOINTS] = traj.joint_vel[f]

        if self._has_upper_body_targets():
            self._inject_upper_body(enc_obs)

        # The selected bundle defines heading-normalized or body-frame anchors.
        q_left_inv = self._reference_orientation_inverse(base_quat)
        for i in range(ENCODER_REFERENCE_FRAMES):
            f = min(f_curr + i * self._profile.g1_frame_stride, traj.num_frames - 1)
            q_aligned = _quat_multiply(
                self._heading_delta_quat, traj.root_quat[f].astype(np.float64)
            )
            q_rel = _quat_multiply(q_left_inv, q_aligned)
            enc_obs[ANCHOR_HIST_OFFSET + i * 6 : ANCHOR_HIST_OFFSET + (i + 1) * 6] = _rotmat_to_6d(
                _quat_to_rotmat(q_rel)
            )
        return enc_obs

    def _build_teleop_encoder_obs(self, base_quat: NDArray[Any]) -> NDArray[Any]:
        """Encoder obs for teleop mode (1): mode scalar, lowerbody joint
        pos/vel history from the planner trajectory, single-frame anchor
        orientation, VR 3-point blocks. All other fields stay zero - the C++
        gathers ONLY the active mode's required observations into a zeroed
        buffer (GatherEncoderObservations)."""
        enc_obs = np.zeros(self._profile.encoder_obs_dim, dtype=np.float32)
        enc_obs[0] = 1.0  # encoder_mode_4: scalar mode id, rest zeros
        traj = self._trajectory
        assert traj is not None
        f_curr = min(self._traj_frame, traj.num_frames - 1)

        for i in range(ENCODER_REFERENCE_FRAMES):
            f = min(f_curr + i * self._profile.g1_frame_stride, traj.num_frames - 1)
            enc_obs[LOWERBODY_POS_OFFSET + i * 12 : LOWERBODY_POS_OFFSET + (i + 1) * 12] = (
                traj.joint_pos[f][LOWER_BODY_MJC_IN_ONNX]
            )
            enc_obs[LOWERBODY_VEL_OFFSET + i * 12 : LOWERBODY_VEL_OFFSET + (i + 1) * 12] = (
                traj.joint_vel[f][LOWER_BODY_MJC_IN_ONNX]
            )

        q_left_inv = self._reference_orientation_inverse(base_quat)
        q_aligned = _quat_multiply(
            self._heading_delta_quat, traj.root_quat[f_curr].astype(np.float64)
        )
        q_rel = _quat_multiply(q_left_inv, q_aligned)
        enc_obs[ANCHOR_SINGLE_OFFSET : ANCHOR_SINGLE_OFFSET + 6] = _rotmat_to_6d(
            _quat_to_rotmat(q_rel)
        )

        enc_obs[VR_POS_OFFSET : VR_POS_OFFSET + 9] = self._vr_pos
        enc_obs[VR_ORN_OFFSET : VR_ORN_OFFSET + 12] = self._vr_orn
        return enc_obs

    def _reference_orientation_inverse(self, base_quat: NDArray[Any]) -> NDArray[Any]:
        if self._profile.heading_normalized:
            return _calc_heading_quat_inv(base_quat)
        return _quat_conjugate(np.asarray(base_quat, dtype=np.float64))

    # -- planner ----------------------------------------------------------

    def _auto_mode(self, speed: float) -> int:
        if speed < 0.05:
            return 0
        if speed < 0.4:
            return 1
        if speed < 1.2:
            return 2
        return 3

    def _build_planner_context(self) -> NDArray[Any]:
        context = np.zeros((4, 36), dtype=np.float32)
        if self._trajectory is not None and self._trajectory.num_frames > 4:
            traj = self._trajectory
            start = min(self._traj_frame + LOOK_AHEAD_FRAMES, traj.num_frames - 1)
            for n in range(4):
                f = min(round(start + n * (50.0 / 30.0)), traj.num_frames - 1)
                context[n, 0:3] = traj.root_pos[f]
                context[n, 3:7] = traj.root_quat[f]
                context[n, 7:36] = traj.joint_pos[f][DDS_TO_ONNX]
        else:
            root_pos = np.array([0.0, 0.0, DEFAULT_HEIGHT], dtype=np.float32)
            for n in range(4):
                context[n, 0:3] = root_pos
                context[n, 3:7] = self._cur_quat
                context[n, 7:36] = self._cur_q_dds
        return context

    def _build_planner_inputs(self) -> dict[str, NDArray[Any]]:
        if self._planner_cmd is not None:
            # ZMQ planner topic: mode/movement/facing given directly
            c = self._planner_cmd
            return self._planner_inputs_dict(
                c["mode"], c["movement"], c["facing"], c["speed"], c["height"]
            )
        speed = math.hypot(self._vx, self._vy)
        yaw = _yaw_from_quat(self._cur_quat)
        cos_h, sin_h = math.cos(yaw), math.sin(yaw)
        world_vx = self._vx * cos_h - self._vy * sin_h
        world_vy = self._vx * sin_h + self._vy * cos_h

        mode = self._mode_override if self._mode_override is not None else self._auto_mode(speed)

        if speed > 0.05 and mode not in STATIC_MODES:
            move_dir = np.array([world_vx / speed, world_vy / speed, 0.0], dtype=np.float32)
        else:
            move_dir = np.zeros(3, dtype=np.float32)

        target_yaw = yaw + self._yaw_rate * 1.0
        face_dir = np.array([math.cos(target_yaw), math.sin(target_yaw), 0.0], dtype=np.float32)

        if mode == 1:
            target_vel = max(0.2, min(speed, 0.8))
        elif mode == 3:
            target_vel = max(1.5, min(speed, 3.0))
        else:
            target_vel = -1.0

        # Per-mode planner params (C++ applySpeedAndHeight): forced modes get
        # their canonical speed/height; an explicit set_base_height wins.
        params = MODE_PLANNER_PARAMS.get(mode) if self._mode_override is not None else None
        height = self._height_cmd
        if params is not None:
            mode_speed, mode_height = params
            if mode_speed > 0 and mode not in (1, 3):
                target_vel = mode_speed
            if mode_height > 0 and height < 0:
                height = mode_height

        return self._planner_inputs_dict(mode, move_dir, face_dir, target_vel, height)

    def _planner_inputs_dict(
        self,
        mode: int,
        move_dir: NDArray[Any],
        face_dir: NDArray[Any],
        target_vel: float,
        height: float,
    ) -> dict[str, NDArray[Any]]:
        return {
            "context_mujoco_qpos": self._build_planner_context().reshape(1, 4, 36),
            "target_vel": np.array([target_vel], dtype=np.float32),
            "mode": np.array([mode], dtype=np.int64),
            "movement_direction": np.asarray(move_dir, dtype=np.float32).reshape(1, 3),
            "facing_direction": np.asarray(face_dir, dtype=np.float32).reshape(1, 3),
            "random_seed": np.array([42], dtype=np.int64),
            "has_specific_target": np.zeros((1, 1), dtype=np.int64),
            "specific_target_positions": np.zeros((1, 4, 3), dtype=np.float32),
            "specific_target_headings": np.zeros((1, 4), dtype=np.float32),
            "allowed_pred_num_tokens": np.array(
                [[1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0]], dtype=np.int64
            ),
            "height": np.array([height], dtype=np.float32),
        }

    def _submit_planner(self) -> None:
        if self._planner_future is not None and not self._planner_future.done():
            return
        try:
            inputs = self._build_planner_inputs()
        except Exception as exc:
            logger.warning("SonicPipeline planner input build failed", error=repr(exc))
            return
        self._planner_started_at = time.perf_counter()
        self._planner_future = self._planner_executor.submit(self._planner.run, None, inputs)

    def _check_planner_result(self) -> None:
        if self._planner_future is None or not self._planner_future.done():
            return
        try:
            self._apply_planner_result(self._planner_future.result())
        except Exception as exc:
            logger.warning("SonicPipeline planner inference failed", error=repr(exc))
        if self._planner_started_at is not None:
            self._planner_durations_ms.append(
                (time.perf_counter() - self._planner_started_at) * 1000.0
            )
        self._planner_started_at = None
        self._planner_future = None

    def _apply_planner_result(self, result: list[Any]) -> None:
        qpos_30hz = result[0].squeeze()
        num_frames = int(result[1].item())
        if num_frames < 2:
            return
        if self._nan_check("planner_qpos", qpos_30hz[:num_frames]):
            return
        new_traj = self._resample_to_50hz(qpos_30hz, num_frames)

        if self._trajectory is not None and self._trajectory.num_frames > 0:
            old, old_f = self._trajectory, self._traj_frame
            blend = min(BLEND_FRAMES, new_traj.num_frames)
            for f in range(blend):
                of = min(old_f + f, old.num_frames - 1)
                w_new = (f + 1) / (blend + 1)
                w_old = 1.0 - w_new
                new_traj.joint_pos[f] = w_old * old.joint_pos[of] + w_new * new_traj.joint_pos[f]
                new_traj.root_pos[f] = w_old * old.root_pos[of] + w_new * new_traj.root_pos[f]
                new_traj.root_quat[f] = _quat_lerp(old.root_quat[of], new_traj.root_quat[f], w_new)
            for f in range(min(blend, new_traj.num_frames - 1)):
                new_traj.joint_vel[f] = (new_traj.joint_pos[f + 1] - new_traj.joint_pos[f]) * 50.0

        if not self._heading_initialized and new_traj.num_frames > 0:
            init_heading = _calc_heading_quat(self._cur_quat)
            init_ref_inv = _calc_heading_quat_inv(new_traj.root_quat[0])
            self._heading_delta_quat = _quat_multiply(init_heading, init_ref_inv)
            self._heading_initialized = True

        self._trajectory = new_traj
        self._traj_frame = 0

    def _resample_to_50hz(self, qpos_30hz: NDArray[Any], n30: int) -> _Trajectory:
        n50 = max(2, int(n30 / 30.0 * 50.0))
        traj = _Trajectory(n50)
        for f in range(n50):
            f30 = f / 50.0 * 30.0
            f0 = min(int(f30), n30 - 1)
            f1 = min(f0 + 1, n30 - 1)
            alpha = (f30 - f0) if f0 < n30 - 1 else 0.0
            traj.root_pos[f] = (1 - alpha) * qpos_30hz[f0, 0:3] + alpha * qpos_30hz[f1, 0:3]
            traj.root_quat[f] = _quat_lerp(qpos_30hz[f0, 3:7], qpos_30hz[f1, 3:7], alpha)
            raw = (1 - alpha) * qpos_30hz[f0, 7:36] + alpha * qpos_30hz[f1, 7:36]
            traj.joint_pos[f] = raw[ONNX_TO_DDS]
        for f in range(n50 - 1):
            traj.joint_vel[f] = (traj.joint_pos[f + 1] - traj.joint_pos[f]) * 50.0
        if n50 > 1:
            traj.joint_vel[-1] = traj.joint_vel[-2]
        traj.num_frames = n50
        return traj

    # -- step -------------------------------------------------------------

    def _nan_check(self, name: str, arr: NDArray[Any]) -> bool:
        if np.isnan(arr).any() or np.isinf(arr).any():
            if self._nan_reported < 10:
                logger.warning(
                    "SonicPipeline non-finite tensor",
                    tensor=name,
                    step=self._step_count,
                    sample=np.asarray(arr).ravel()[:8].tolist(),
                )
                self._nan_reported += 1
            return True
        return False

    def step(
        self,
        q_dds: NDArray[Any],
        dq_dds: NDArray[Any],
        base_quat_wxyz: NDArray[Any],
        gyro_body: NDArray[Any],
        gravity_body: NDArray[Any],
    ) -> NDArray[Any]:
        """One 50 Hz policy step. Returns 29 position targets, DDS order."""
        self._step_count += 1

        # Input sentries: a non-finite or degenerate input poisons the
        # heading math and the planner. Hold the previous targets instead.
        bad = (
            self._nan_check("q_dds", np.asarray(q_dds))
            or self._nan_check("dq_dds", np.asarray(dq_dds))
            or self._nan_check("base_quat", np.asarray(base_quat_wxyz))
            or self._nan_check("gyro", np.asarray(gyro_body))
            or self._nan_check("gravity", np.asarray(gravity_body))
        )
        qn = float(np.linalg.norm(np.asarray(base_quat_wxyz, dtype=np.float64)))
        if qn < 0.5:
            if self._nan_reported < 10:
                logger.warning(
                    "SonicPipeline degenerate base quaternion",
                    norm=qn,
                    step=self._step_count,
                )
                self._nan_reported += 1
            bad = True
        if bad:
            return self._last_targets_dds.copy()
        self._cur_quat = np.asarray(base_quat_wxyz, dtype=np.float64)
        self._cur_q_dds = np.asarray(q_dds, dtype=np.float32)

        self._check_planner_result()

        # Staged floor transitions: hold each ladder rung for the dwell,
        # then advance (gamepad_manager.hpp transition timers).
        if self._mode_queue:
            self._mode_dwell += POLICY_DT
            if self._mode_dwell >= TRANSITION_DWELL_SEC:
                self._mode_override = self._mode_queue.pop(0)
                self._mode_dwell = 0.0
                self._needs_replan = True

        self._replan_timer += POLICY_DT
        speed = math.hypot(self._vx, self._vy)
        mode = self._mode_override if self._mode_override is not None else self._auto_mode(speed)
        moving = speed > 0.05 or (self._mode_override is not None and mode not in STATIC_MODES)
        interval = REPLAN_INTERVAL_RUNNING if speed >= 1.2 else REPLAN_INTERVAL_DEFAULT
        traj_low = (
            self._trajectory is not None
            and self._traj_frame > self._trajectory.num_frames - 20
            and moving
        )
        # A forced non-static mode needs planner output even at zero twist.
        mode_needs_traj = (
            self._mode_override is not None
            and mode not in STATIC_MODES
            and self._replan_timer >= interval
        )
        if not self._use_stream and (
            self._needs_replan
            or (self._replan_timer >= interval and moving)
            or traj_low
            or mode_needs_traj
        ):
            self._submit_planner()
            self._replan_timer = 0.0
            self._needs_replan = False

        # Encoder token
        if self._use_stream and self._streamed is not None and self._streamed.timesteps > 0:
            if not self._heading_initialized:
                init_heading = _calc_heading_quat(self._cur_quat)
                init_ref_inv = _calc_heading_quat_inv(
                    self._streamed.root_quat[0].astype(np.float64)
                )
                self._heading_delta_quat = _quat_multiply(init_heading, init_ref_inv)
                self._heading_initialized = True
            token = self._run_encoder(self._build_streamed_encoder_obs(self._cur_quat))
        elif self._vr_active() and self._trajectory is not None and self._trajectory.num_frames > 0:
            token = self._run_encoder(self._build_teleop_encoder_obs(self._cur_quat))
        elif self._trajectory is not None and self._trajectory.num_frames > 0:
            token = self._run_encoder(self._build_encoder_obs(self._cur_quat))
        elif self._has_upper_body_targets():
            enc_obs = np.zeros(self._profile.encoder_obs_dim, dtype=np.float32)
            for i in range(ENCODER_REFERENCE_FRAMES):
                enc_obs[4 + i * NUM_JOINTS : 4 + (i + 1) * NUM_JOINTS] = DEFAULT_ANGLES_ONNX
                enc_obs[ANCHOR_HIST_OFFSET + i * 6 : ANCHOR_HIST_OFFSET + (i + 1) * 6] = (
                    _IDENTITY_6D
                )
            self._inject_upper_body(enc_obs)
            token = self._run_encoder(enc_obs)
        else:
            token = self._standing_token
        token = self._blend_reference_token(token)

        # Proprio history (ONNX order)
        q_onnx = self._cur_q_dds[ONNX_TO_DDS]
        dq_onnx = np.asarray(dq_dds, dtype=np.float32)[ONNX_TO_DDS]
        ptr = self._history_ptr
        self._his_ang_vel[ptr] = np.asarray(gyro_body, dtype=np.float32)
        self._his_joint_pos[ptr] = q_onnx - DEFAULT_ANGLES_ONNX
        self._his_joint_vel[ptr] = dq_onnx
        self._his_action[ptr] = self._last_action
        self._his_gravity[ptr] = np.asarray(gravity_body, dtype=np.float32)
        self._history_ptr = (ptr + 1) % HISTORY_LEN

        obs = self._obs_buffer
        obs[0:ENCODER_TOKEN_DIM] = token
        order = np.array(
            [(self._history_ptr + j) % HISTORY_LEN for j in range(HISTORY_LEN)],
            dtype=np.intp,
        )
        obs[64:94] = self._his_ang_vel[order].ravel()
        obs[94:384] = self._his_joint_pos[order].ravel()
        obs[384:674] = self._his_joint_vel[order].ravel()
        obs[674:964] = self._his_action[order].ravel()
        obs[964:994] = self._his_gravity[order].ravel()

        self._nan_check("token", token)
        self._nan_check("decoder_obs", obs)
        decoder_started = time.perf_counter()
        out = self._decoder.run(None, {self._decoder_input: obs.reshape(1, -1)})
        self._decoder_durations_ms.append((time.perf_counter() - decoder_started) * 1000.0)
        actions = out[0].squeeze()[:NUM_JOINTS].astype(np.float32)
        if self._nan_check("actions", actions):
            return self._last_targets_dds.copy()
        self._last_reference_token = token.copy()
        self._last_token_was_stream = self._use_stream
        if (
            self.reference_transition_active
            and self._reference_transition_step >= self._reference_transition_steps
        ):
            self._clear_reference_transition()
        self._last_action = actions.copy()

        # All 29 decoder actions applied directly - no post-decoder override
        # (D3; matches C++ CreatePolicyCommand).
        targets_onnx = DEFAULT_ANGLES_ONNX + actions * ACTION_SCALE_ONNX
        self._last_targets_dds = targets_onnx[DDS_TO_ONNX].copy()

        if self._use_stream and self._streamed is not None:
            self._streamed_frame = min(self._streamed_frame + 1, self._streamed.timesteps - 1)
        elif self._trajectory is not None:
            self._traj_frame = min(self._traj_frame + 1, self._trajectory.num_frames - 1)

        return targets_onnx[DDS_TO_ONNX]

    def _build_streamed_encoder_obs(self, base_quat: NDArray[Any]) -> NDArray[Any]:
        """Encoder obs from the streamed motion (pose topic).

        Mode 0 (protocol v1): joint fields step5, like a planner trajectory.
        Mode 2 (v2/v3): SMPL fields step1 + wrist positions step1, matching
        the C++ observation registry offsets.
        """
        motion = self._streamed
        assert motion is not None
        enc_obs = np.zeros(self._profile.encoder_obs_dim, dtype=np.float32)
        enc_obs[0] = float(motion.encode_mode)
        f_curr = min(self._streamed_frame, motion.timesteps - 1)
        q_left_inv = self._reference_orientation_inverse(base_quat)

        if motion.encode_mode == 0:
            for i in range(ENCODER_REFERENCE_FRAMES):
                f = min(f_curr + i * self._profile.g1_frame_stride, motion.timesteps - 1)
                enc_obs[4 + i * NUM_JOINTS : 4 + (i + 1) * NUM_JOINTS] = motion.joint_pos[f]
                enc_obs[294 + i * NUM_JOINTS : 294 + (i + 1) * NUM_JOINTS] = motion.joint_vel[f]
                q_aligned = _quat_multiply(
                    self._heading_delta_quat, motion.root_quat[f].astype(np.float64)
                )
                q_rel = _quat_multiply(q_left_inv, q_aligned)
                enc_obs[ANCHOR_HIST_OFFSET + i * 6 : ANCHOR_HIST_OFFSET + (i + 1) * 6] = (
                    _rotmat_to_6d(_quat_to_rotmat(q_rel))
                )
            if self._has_upper_body_targets():
                self._inject_upper_body(enc_obs)
        else:
            assert motion.smpl_joints is not None
            for i in range(self._profile.smpl_frames):
                f = min(f_curr + i, motion.timesteps - 1)
                o = SMPL_JOINTS_OFFSET + i * 72
                enc_obs[o : o + 72] = motion.smpl_joints[f].ravel()
                q_aligned = _quat_multiply(
                    self._heading_delta_quat, motion.root_quat[f].astype(np.float64)
                )
                q_rel = _quat_multiply(q_left_inv, q_aligned)
                ao = self._profile.smpl_anchor_offset + i * 6
                enc_obs[ao : ao + 6] = _rotmat_to_6d(_quat_to_rotmat(q_rel))
                wo = self._profile.wrists_offset + i * 6
                enc_obs[wo : wo + 6] = motion.joint_pos[f][WRIST_ONNX_INDICES]
        return enc_obs

    def _run_encoder(self, enc_obs: NDArray[Any]) -> NDArray[Any]:
        if enc_obs.shape != (self._profile.encoder_obs_dim,):
            raise ValueError(
                f"SONIC {self._profile.name} encoder observation has shape "
                f"{enc_obs.shape}, expected ({self._profile.encoder_obs_dim},)"
            )
        started = time.perf_counter()
        out = self._encoder.run(None, {self._encoder_input: enc_obs.reshape(1, -1)})
        self._encoder_durations_ms.append((time.perf_counter() - started) * 1000.0)
        return out[0].squeeze().astype(np.float32)

    # -- telemetry --------------------------------------------------------

    def snapshot(self) -> dict[str, Any]:
        speed = math.hypot(self._vx, self._vy)
        mode = self._mode_override if self._mode_override is not None else self._auto_mode(speed)
        return {
            "sonic_pipeline": self._profile.name,
            "encoder_obs_dim": self._profile.encoder_obs_dim,
            "smpl_reference_frames": self._profile.smpl_frames,
            "mode": mode,
            "mode_override": self._mode_override,
            "mode_queue": list(self._mode_queue),
            "speed": speed,
            "trajectory": self._trajectory is not None,
            "traj_frame": self._traj_frame,
            "traj_frames_total": (self._trajectory.num_frames if self._trajectory else 0),
            "action_norm": float(np.linalg.norm(self._last_action)),
            "upper_body_active": self._has_upper_body_targets(),
            "stream_active": self._use_stream,
            "stream_frames": self._streamed.timesteps if self._streamed else 0,
            "stream_frame": self._streamed_frame,
            "stream_encode_mode": self._streamed.encode_mode if self._streamed else -1,
            "reference_transition_active": self.reference_transition_active,
            "reference_transition_progress": self.reference_transition_progress,
            "vr_active": self._vr_active(),
            "vr_age_sec": (
                round(time.perf_counter() - self._vr_time, 3) if self._vr_pos is not None else -1.0
            ),
            "encoder_timing_ms": _timing_summary(self._encoder_durations_ms),
            "decoder_timing_ms": _timing_summary(self._decoder_durations_ms),
            "planner_timing_ms": _timing_summary(self._planner_durations_ms),
        }


def _timing_summary(samples: deque[float]) -> dict[str, float | int]:
    if not samples:
        return {"samples": 0, "mean": 0.0, "p95": 0.0, "p99": 0.0, "max": 0.0}
    values = np.asarray(samples, dtype=np.float64)
    return {
        "samples": len(samples),
        "mean": round(float(np.mean(values)), 3),
        "p95": round(float(np.percentile(values, 95)), 3),
        "p99": round(float(np.percentile(values, 99)), 3),
        "max": round(float(np.max(values)), 3),
    }
