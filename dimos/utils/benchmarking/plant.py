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

"""Sim plant + per-robot profile for the twist-base tuning tools.

Per-channel FOPDT velocity tracking + unicycle kinematics (robot-agnostic;
the ``(vx, vy, wz)`` twist-base contract). Tick-based: each call to
:meth:`TwistBasePlantSim.step` advances one control period.

The bottom of this module holds the per-robot plant + control config
(``RobotPlantProfile`` + ``ROBOT_PLANT_PROFILES``). The vendored Go2 fit
(``GO2_PLANT_FITTED``) is the Go2 profile's ground truth — it keeps its
``GO2_`` name because it is genuinely Go2-measured data, not generic.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field
import math


@dataclass
class FopdtChannelParams:
    """First-order-plus-dead-time params for a single velocity channel.

    Symbols match the characterization fitter:
      K   - steady-state gain (output / commanded)
      tau - first-order time constant (s)
      L   - pure dead-time (s)
    """

    K: float
    tau: float
    L: float


@dataclass
class AmplitudeFit:
    """One FOPDT fit at a specific commanded amplitude. Used to record the
    full K(amp), tau(amp), L(amp) tables alongside the canonical single
    fit, so future lookup-based controllers can interpolate without re-
    collecting data."""

    amp: float
    K: float
    tau: float
    L: float
    r2: float
    source: str = "sweep"  # "sweep" | "ceiling_probe"


@dataclass
class FloorProbeResult:
    """Pass/fail of the D3 floor-detection AND-test at one probe amplitude.

    A sample passes when |body_vel| > motion_threshold AND > frac_threshold *
    |amp|. Floor = lowest amp where the test is sustained for N consecutive
    samples within the step window."""

    amp: float
    motion_detected: bool
    sustained_samples: int  # longest contiguous run of passing samples
    net_displacement: float = 0.0  # signed body-frame displacement in cmd dir


@dataclass
class ChannelEnvelope:
    """Measured speed envelope for a single channel.

    ``ceiling`` is the operational ``min(max(K·amp), envelope_cap)`` value
    that DERIVE feeds into RG's v_max / ω_max. ``saturating_at_amp`` is a
    forensic-only diagnostic (lowest amp where K dropped 15% below the
    linear-regime K) and is NOT used as the operational cap."""

    floor: float
    ceiling: float
    floor_not_found: bool = False
    ceiling_not_found: bool = False
    saturating_at_amp: float | None = None


@dataclass
class VelocityEnvelope:
    """Per-channel measured floor/ceiling — m/s for vx/vy, rad/s for wz.

    Saturation is not a dynamics parameter so this is a separate section
    in the artifact (not folded into the FOPDT plant)."""

    vx: ChannelEnvelope
    vy: ChannelEnvelope
    wz: ChannelEnvelope


class FOPDTChannel:
    """First-order lag + dead-time for one velocity axis.

    Tick-based: feed one commanded value per :meth:`step` call, get the
    delayed/lagged actual velocity back.
    """

    def __init__(self, params: FopdtChannelParams) -> None:
        self.params = params
        self._delay_buf: deque[float] = deque()
        self._delay_samples = 0
        self._y = 0.0

    def reset(self, dt: float) -> None:
        # step() appends before reading the head, so a buffer of N slots
        # delays by N-1 ticks; size for int(L/dt) ticks of dead time.
        self._delay_samples = max(1, int(self.params.L / dt) + 1)
        self._delay_buf = deque([0.0] * self._delay_samples, maxlen=self._delay_samples)
        self._y = 0.0

    def step(self, u: float, dt: float) -> float:
        self._delay_buf.append(u)
        u_delayed = self._delay_buf[0]
        alpha = dt / (self.params.tau + dt)
        self._y += alpha * (self.params.K * u_delayed - self._y)
        return self._y


@dataclass
class TwistBasePlantParams:
    """FOPDT params for the three twist-base velocity channels."""

    vx: FopdtChannelParams
    vy: FopdtChannelParams
    wz: FopdtChannelParams


class CommandLimiter:
    """Ruckig-style per-axis velocity + acceleration limiter in COMMAND units.

    Mirrors the limiter that lives inside the FlowBase firmware, applied to
    the commanded twist BEFORE the plant dynamics. Trajectories planned
    within the physical margins never engage it; the sim reproduces the
    saturation behavior when they do.
    """

    def __init__(self, max_vel: tuple[float, float, float], max_acc: tuple[float, float, float]):
        self.max_vel = max_vel
        self.max_acc = max_acc
        self._prev = [0.0, 0.0, 0.0]

    def reset(self) -> None:
        self._prev = [0.0, 0.0, 0.0]

    def step(self, cmds: tuple[float, float, float], dt: float) -> tuple[float, float, float]:
        out = []
        for i, cmd in enumerate(cmds):
            target = max(-self.max_vel[i], min(self.max_vel[i], cmd))
            dv_max = self.max_acc[i] * dt
            dv = max(-dv_max, min(dv_max, target - self._prev[i]))
            self._prev[i] += dv
            out.append(self._prev[i])
        return out[0], out[1], out[2]


class TwistBasePlantSim:
    """Unicycle kinematic sim with FOPDT velocity response per channel.

    Body-frame velocities `(vx, vy, wz)` are commanded; the plant produces
    actual velocities (filtered + delayed) that drive a unicycle integrator
    in the world frame. An optional ``limiter`` clamps the commands first
    (command units), reproducing a firmware-side rate limiter.
    """

    def __init__(self, params: TwistBasePlantParams, limiter: CommandLimiter | None = None) -> None:
        self.params = params
        self.limiter = limiter
        self.ch_vx = FOPDTChannel(params.vx)
        self.ch_vy = FOPDTChannel(params.vy)
        self.ch_wz = FOPDTChannel(params.wz)
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.wz = 0.0

    def reset(self, x: float, y: float, yaw: float, dt: float) -> None:
        self.x, self.y, self.yaw = x, y, yaw
        self.vx = self.vy = self.wz = 0.0
        for ch in (self.ch_vx, self.ch_vy, self.ch_wz):
            ch.reset(dt)
        if self.limiter is not None:
            self.limiter.reset()

    def step(self, cmd_vx: float, cmd_vy: float, cmd_wz: float, dt: float) -> None:
        if self.limiter is not None:
            cmd_vx, cmd_vy, cmd_wz = self.limiter.step((cmd_vx, cmd_vy, cmd_wz), dt)
        self.vx = self.ch_vx.step(cmd_vx, dt)
        self.vy = self.ch_vy.step(cmd_vy, dt)
        self.wz = self.ch_wz.step(cmd_wz, dt)

        self.x += (self.vx * math.cos(self.yaw) - self.vy * math.sin(self.yaw)) * dt
        self.y += (self.vx * math.sin(self.yaw) + self.vy * math.cos(self.yaw)) * dt
        self.yaw = (self.yaw + self.wz * dt + math.pi) % (2 * math.pi) - math.pi


# --- Vendored fitted FOPDT plant for the Go2 base ------------------------
#
# Source: concrete surface, normal/default mode, data collected
# 2026-05-07, fitted by the characterization pipeline. RISE tau/L
# corrected 2026-05-16: an earlier pooled fit was degenerate (tau pinned
# at the solver lower bound with all lag collapsed into L); a per-run
# re-fit of the same raw E1/E2 data (median over converged forward
# trials, fresh-fit r2=0.92 vx / 0.82 wz) gives the true structure —
# small dead-time (L ~ 0.05-0.07 s), larger tau (vx ~ 0.40,
# wz ~ 0.55-0.60 s). K is unchanged (independently validated).
#
# This is the Go2 profile's sim ground truth (self-test recovers it; the
# sim adapter / DERIVE fallback use it). It keeps its GO2_ name because
# it is genuinely Go2-measured data. vy is a placeholder copy of vx until
# vy is characterized on hardware — the Go2 DOES strafe (it just needs a
# higher floor amplitude than vx to start), so vy is now excited in the
# sweep; this sim FOPDT just has no measured lateral model yet.

GO2_VX_RISE = FopdtChannelParams(K=0.922, tau=0.395, L=0.065)
GO2_WZ_RISE = FopdtChannelParams(K=2.453, tau=0.596, L=0.052)

GO2_PLANT_FITTED = TwistBasePlantParams(
    vx=GO2_VX_RISE,
    vy=GO2_VX_RISE,  # placeholder; Go2 doesn't strafe in default gait
    wz=GO2_WZ_RISE,
)


# --- Vendored fitted FOPDT plant for the FlowBase ------------------------
#
# Source: concrete surface, default mode, real FlowBase over LCM SI,
# characterized 2026-06-09 (git 704a591f5, methodology v2, 17 fit points).
# Artifact: data/characterization/flowbase/
#   flowbase_config_hw_concrete_2026-06-09_704a591f5.json
#
# K is actuation-side (the robot genuinely moves K x the command; odometry
# is honest) — caused by a kinematic inconsistency in the sealed firmware.
# All compensation lives controller-side.

FLOWBASE_VX_FIT = FopdtChannelParams(K=0.778, tau=0.288, L=0.010)
FLOWBASE_VY_FIT = FopdtChannelParams(K=0.773, tau=0.267, L=0.033)
FLOWBASE_WZ_FIT = FopdtChannelParams(K=2.929, tau=0.607, L=0.017)

FLOWBASE_PLANT_FITTED = TwistBasePlantParams(
    vx=FLOWBASE_VX_FIT,
    vy=FLOWBASE_VY_FIT,
    wz=FLOWBASE_WZ_FIT,
)

# FlowBase firmware Ruckig limiter, COMMAND units (x, y, yaw). Physical
# limits are K x these. max_vel == max_accel in the firmware config.
FLOWBASE_CMD_MAX_VEL: tuple[float, float, float] = (0.8, 0.8, 3.0)
FLOWBASE_CMD_MAX_ACC: tuple[float, float, float] = FLOWBASE_CMD_MAX_VEL


def flowbase_command_limiter() -> CommandLimiter:
    """Limiter matching the FlowBase firmware's Ruckig config."""
    return CommandLimiter(max_vel=FLOWBASE_CMD_MAX_VEL, max_acc=FLOWBASE_CMD_MAX_ACC)


# --- Per-robot profile (single source of truth for robot specifics) -----


@dataclass(frozen=True)
class RobotPlantProfile:
    """Everything the characterization + benchmark tools need to know
    about a specific velocity-commanded twist base: the FOPDT plant and
    the control-loop knobs that surround it. Add a robot by appending
    one instance to ``ROBOT_PLANT_PROFILES``.

    The tuning tools talk to the operator's running coord on two
    well-known LCM topics (the contract every coord blueprint already
    honors):

      * publish Twist on ``/cmd_vel`` (the coord's ``twist_command`` In)
      * subscribe JointState on ``/coordinator/joint_state`` (the
        coord's published Out — joint *positions* carry ``[x, y, yaw]``
        because ``positions = adapter.read_odometry()``; see
        :class:`~dimos.control.hardware_interface.ConnectedTwistBase`).

    ``robot_id`` is provenance/cosmetic. ``joint_prefix`` is what the
    operator coord's hardware uses for joint names — Go2's coord uses
    ``go2/{vx,vy,wz}``, FlowBase's uses ``base/{vx,vy,wz}`` — so the
    tool knows which positions to pick out of joint_state.
    """

    # identity / cosmetic
    name: str
    robot_id: str  # provenance + artifact filename
    # transport / bring-up
    blueprint: str  # the `dimos run <blueprint>` the operator starts (hw)
    sim_blueprint: str  # the `dimos run <blueprint>` for sim
    # joint name prefix the operator coord's hardware uses. Defaults to
    # robot_id; set explicitly when the coord blueprint uses a different
    # prefix (e.g. flowbase coord uses "base/...").
    joint_prefix: str | None = None
    # physical envelope
    vx_max: float = 1.0
    wz_max: float = 1.5
    tick_rate_hz: float = 10.0
    odom_warmup_s: float = 10.0
    odom_stale_s: float = 1.0
    # SI plan / kinematics
    excited_channels: tuple[str, ...] = ("vx", "wz")  # omit vy => non-strafing
    si_amplitudes: dict[str, list[float]] = field(
        default_factory=lambda: {"vx": [0.3, 0.6, 0.9], "vy": [0.2, 0.4], "wz": [0.4, 0.8, 1.2]}
    )
    # Floor / ceiling probe ladders (methodology v2 — densification).
    # Floor probe runs FIRST per channel: tiny amplitudes to find the
    # smallest commanded value that produces actual robot motion (the D3
    # AND-test in characterization.py). Ceiling probe runs LAST:
    # supra-sweep amplitudes to detect the K-sag onset (saturation).
    floor_probe_amplitudes: dict[str, list[float]] = field(
        default_factory=lambda: {
            "vx": [0.02, 0.05, 0.10, 0.15],
            "vy": [0.1, 0.15, 0.20],
            "wz": [0.05, 0.10, 0.20, 0.30],
        }
    )
    # Ascending floor search: when the predefined floor_probe_amplitudes
    # ladder is exhausted without detected motion, keep probing at
    # last_amp + floor_probe_step[channel] until motion is found or the
    # amplitude exceeds floor_probe_max[channel] (safety cap). Only when
    # the cap is reached without motion is floor_not_found set.
    floor_probe_step: dict[str, float] = field(
        default_factory=lambda: {"vx": 0.05, "vy": 0.05, "wz": 0.10}
    )
    floor_probe_max: dict[str, float] = field(
        default_factory=lambda: {"vx": 0.5, "vy": 0.5, "wz": 1.0}
    )
    ceiling_probe_amplitudes: dict[str, list[float]] = field(
        default_factory=lambda: {"vx": [2.5, 3.0], "vy": [1.5, 2.0], "wz": [2.5, 3.0]}
    )
    # Floor D3 thresholds. AND of (absolute motion above noise floor) and
    # (fractional response above tracking noise), sustained for N samples.
    floor_motion_threshold: float = 0.02  # m/s (vx/vy) or rad/s (wz)
    floor_fractional_threshold: float = 0.05  # |v_body| > 5% of |amp|
    floor_sustained_samples: int = 5
    # Minimum NET signed displacement (commanded direction) over the probe
    # window to count as real translation. Rejects net-zero posture wobble
    # whose |v| spikes but whose integral cancels. m for vx/vy, rad for wz.
    floor_displacement_threshold: dict[str, float] = field(
        default_factory=lambda: {"vx": 0.05, "vy": 0.05, "wz": 0.10}
    )
    # Ceiling K-sag: |K(amp)| drops below (1 - sag) * K_linear -> saturated.
    ceiling_k_sag_threshold: float = 0.15
    step_s: float = 8.0
    pre_roll_s: float = 1.0
    max_dist_m: float = 6.0
    # Hard manual floor — if a profile sets this >0, DERIVE will not let
    # the measured floor go below it. Off by default (use measured).
    min_speed_floor: float = 0.0
    # Sim ground truth: drives the sim blueprint's FOPDT plant + the
    # characterization self-test path + DERIVE ceiling fallback.
    sim_plant: TwistBasePlantParams = field(default_factory=lambda: GO2_PLANT_FITTED)

    @property
    def joints_prefix(self) -> str:
        return self.joint_prefix if self.joint_prefix is not None else self.robot_id


GO2_PLANT_PROFILE = RobotPlantProfile(
    name="Go2",
    robot_id="go2",
    blueprint="unitree-go2-webrtc-keyboard-teleop",
    sim_blueprint="coordinator-sim-fopdt",
    joint_prefix="go2",  # unitree_go2_coordinator uses make_twist_base_joints("go2")
    vx_max=1.0,
    wz_max=1.5,
    tick_rate_hz=10.0,
    odom_warmup_s=10.0,
    odom_stale_s=1.0,
    excited_channels=("vx", "vy", "wz"),
    # Densified sweep (methodology v2): unified numeric ladder across
    # channels (vx/vy in m/s, wz in rad/s). vy data on Go2 is expected
    # noisier — Go2 doesn't strafe natively — but we collect it anyway
    # so future work has something to look at.
    si_amplitudes={
        "vx": [0.2, 0.5, 1.0, 1.5, 2.0],
        "vy": [0.2, 0.5, 1.0, 1.5, 2.0],
        "wz": [0.2, 0.5, 1.0, 1.5, 2.0],
    },
    floor_probe_amplitudes={
        "vx": [0.02, 0.05, 0.10, 0.15],
        "vy": [0.02, 0.05, 0.10],
        "wz": [0.05, 0.10, 0.20, 0.30],
    },
    # Go2: vx true floor ~0.2 (ladder tops at 0.15); step past it in 0.05
    # increments up to 0.5 m/s before declaring floor_not_found. wz in
    # 0.10 rad/s increments up to 1.0 rad/s.
    floor_probe_step={"vx": 0.05, "vy": 0.05, "wz": 0.10},
    floor_probe_max={"vx": 0.5, "vy": 0.5, "wz": 1.0},
    ceiling_probe_amplitudes={"vx": [2.5, 3.0], "vy": [1.5, 2.0], "wz": [2.5, 3.0]},
    # Net-displacement floor gate. A genuine step at the true Go2 floor
    # (~0.1 m/s held a couple seconds) covers >=0.2 m. Go2 odom can drift
    # ~0.07 m over a window with NO real translation, so require 0.10 m /
    # 0.10 rad net to count as motion (above odom drift, below real travel).
    floor_displacement_threshold={"vx": 0.20, "vy": 0.10, "wz": 0.10},
    step_s=8.0,
    pre_roll_s=1.0,
    max_dist_m=6.0,
    sim_plant=GO2_PLANT_FITTED,
)

# FlowBase (holonomic Portal-RPC twist base). Operator-side blueprint:
# the existing `coordinator-flowbase-keyboard-teleop` already publishes
# /coordinator/joint_state with positions=[x,y,yaw] from the flowbase
# adapter's read_odometry. No new blueprint, no bridge, no Connection
# module needed — just this profile entry.
#
# The vy channel is excited (FlowBase strafes natively) so vy is in the
# excited_channels tuple. sim_plant is the vendored 2026-06-09 hw fit.
FLOWBASE_PLANT_PROFILE = RobotPlantProfile(
    name="FlowBase",
    robot_id="flowbase",
    blueprint="coordinator-flowbase-keyboard-teleop",
    sim_blueprint="coordinator-sim-fopdt-flowbase",
    joint_prefix="base",  # coordinator_flowbase uses make_twist_base_joints("base")
    vx_max=0.8,
    wz_max=1.2,
    tick_rate_hz=10.0,
    odom_warmup_s=10.0,
    odom_stale_s=1.0,
    excited_channels=("vx", "vy", "wz"),  # holonomic — strafes
    # Densified sweep (2026-06-09): ~5-6 fit points/axis (was the sparse
    # placeholder [0.2,0.4,0.6]/[0.2,0.4]/[0.3,0.6,1.0] = 8 total fits, vs Go2's
    # 15). Stays WITHIN the already-tested envelope — vx/vy ≤0.6 (achieved
    # ceiling ~0.63 m/s), wz ≤1.0 rad/s — so this is denser, NOT faster (same
    # speeds + risk as the prior run). Extending the RANGE to find the real top
    # speed is separate: raise vx_max after the i2rt firmware-limit check.
    si_amplitudes={
        "vx": [0.1, 0.2, 0.3, 0.4, 0.5, 0.6],
        "vy": [0.1, 0.2, 0.3, 0.4, 0.5, 0.6],
        "wz": [0.2, 0.4, 0.6, 0.8, 1.0],
    },
    step_s=6.0,
    pre_roll_s=1.0,
    max_dist_m=4.0,
    sim_plant=FLOWBASE_PLANT_FITTED,
)

# Conservative SHAKEOUT profile for the FIRST on-hardware run of the tall,
# top-heavy FlowBase: low speed caps (0.3 m/s, 0.5 rad/s), gentle amplitudes,
# short 1 m / 5 s steps (wz never completes a full spin). Use
# `--robot flowbase-slow` to validate direction / mast stability / e-stop /
# wiring at a snail's pace. The fit it produces is LOW-SPEED ONLY (truncated
# envelope) — re-run `--robot flowbase` for the real full-envelope fit once the
# bot is trusted. All amplitudes stay below the caps so nothing is clamped
# (clamped steps would corrupt the K fit). Leaves the canonical profile intact.
FLOWBASE_SLOW_PLANT_PROFILE = RobotPlantProfile(
    name="FlowBase (slow shakeout)",
    robot_id="flowbase_slow",
    blueprint="coordinator-flowbase-keyboard-teleop",
    sim_blueprint="coordinator-sim-fopdt-flowbase",
    joint_prefix="base",
    vx_max=0.3,
    wz_max=0.5,
    tick_rate_hz=10.0,
    odom_warmup_s=10.0,
    odom_stale_s=1.0,
    excited_channels=("vx", "vy", "wz"),
    si_amplitudes={"vx": [0.1, 0.2], "vy": [0.1, 0.2], "wz": [0.15, 0.3]},
    floor_probe_amplitudes={"vx": [0.05, 0.1], "vy": [0.1], "wz": [0.1, 0.2]},
    ceiling_probe_amplitudes={"vx": [0.3], "vy": [0.2], "wz": [0.45]},
    step_s=5.0,
    pre_roll_s=1.0,
    max_dist_m=1.0,
    sim_plant=GO2_PLANT_FITTED,
)

ROBOT_PLANT_PROFILES: dict[str, RobotPlantProfile] = {
    "go2": GO2_PLANT_PROFILE,
    "flowbase": FLOWBASE_PLANT_PROFILE,
    "flowbase-slow": FLOWBASE_SLOW_PLANT_PROFILE,
}


__all__ = [
    "FLOWBASE_CMD_MAX_ACC",
    "FLOWBASE_CMD_MAX_VEL",
    "FLOWBASE_PLANT_FITTED",
    "FLOWBASE_PLANT_PROFILE",
    "FLOWBASE_SLOW_PLANT_PROFILE",
    "FLOWBASE_VX_FIT",
    "FLOWBASE_VY_FIT",
    "FLOWBASE_WZ_FIT",
    "GO2_PLANT_FITTED",
    "GO2_PLANT_PROFILE",
    "GO2_VX_RISE",
    "GO2_WZ_RISE",
    "ROBOT_PLANT_PROFILES",
    "AmplitudeFit",
    "ChannelEnvelope",
    "CommandLimiter",
    "FOPDTChannel",
    "FloorProbeResult",
    "FopdtChannelParams",
    "RobotPlantProfile",
    "TwistBasePlantParams",
    "TwistBasePlantSim",
    "VelocityEnvelope",
    "flowbase_command_limiter",
]
