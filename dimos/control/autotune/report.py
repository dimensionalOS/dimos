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

"""Emit the two autotune outputs.

1. The **tuned controller artifact** - a JSON document in the EXACT shape the
   control task config layer consumes (the ``TuningConfig`` contract, read by a
   task's ``from_artifact``). Autotune is the canonical producer; a deployed task
   references the artifact by path. We do not invent a new format.

2. The **characterization report** - a human/machine summary of the measured
   plant properties (FOPDT, bandwidth, asymmetry, coupling, deadzone, stream
   timing). This is durable robot metadata, kept distinct from the tuned config.

Contract notes baked in here (from the consuming side):
  * The contract is keyed to fixed channel slots ``vx/vy/wz``. The new
    channel-LIST profile is projected onto those slots; channels outside
    {vx,vy,wz} have no home in the contract and raise.
  * ``feedforward.K_*`` are PRE-INVERTED (1/K), not raw plant gain.
  * ``valid_for_tuning`` is true only for hardware-sourced data.
  * ``schema_version`` must be 1.
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

from dimos.control.autotune.profile import RobotProfile
from dimos.control.autotune.tune import ChannelTuning

# The contract's fixed channel slots. The consuming task indexes these names.
CONTRACT_SLOTS = ("vx", "vy", "wz")
SCHEMA_VERSION = 1
# Headroom + ride-quality constants from the reference derivation (named, not
# hidden, so a different robot can override).
WZ_HEADROOM_MARGIN = 0.15
A_LAT_MAX = 1.0
DECEL_ACCEL_RATIO = 2.0
DEFAULT_MIN_SPEED = 0.05
DEFAULT_LOOKAHEAD_PTS = 8


def _safe_inv_gain(K: float) -> float:
    """Feedforward gain inversion 1/K, guarding near-zero K (returns 1.0)."""
    return 1.0 / K if abs(K) > 1e-6 else 1.0


def validate_contract_channels(profile: RobotProfile) -> None:
    """The tuned-config contract only has slots vx/vy/wz. Raise if the profile
    declares channels the contract cannot carry."""
    extra = [c for c in profile.channel_names if c not in CONTRACT_SLOTS]
    if extra:
        raise ValueError(
            f"channels {extra} have no slot in the vx/vy/wz tuned-config contract; "
            f"the consuming task cannot read arbitrary channel names"
        )


def _plant_slots(
    profile: RobotProfile, tunings: dict[str, ChannelTuning]
) -> tuple[dict[str, dict[str, float]], list[str]]:
    """Build the vx/vy/wz plant block, filling absent slots with an identity-ish
    placeholder and returning the list of placeholder slots for a caveat."""
    plant: dict[str, dict[str, float]] = {}
    placeholders: list[str] = []
    for slot in CONTRACT_SLOTS:
        if slot in tunings:
            plant[slot] = dict(tunings[slot].plant)
        elif slot in profile.measured.fopdt:
            f = profile.measured.fopdt[slot]
            plant[slot] = {"K": f.K, "tau": f.tau, "L": f.L}
        else:
            plant[slot] = {"K": 1.0, "tau": 0.0, "L": 0.0}
            placeholders.append(slot)
    return plant, placeholders


def _velocity_profile(profile: RobotProfile, plant: dict[str, dict[str, float]]) -> dict[str, Any]:
    """Derive the velocity-profile block from envelope + fitted dynamics."""
    vx_ceiling = profile.channel("vx").vmax if "vx" in profile.channel_names else 1.0
    wz_ceiling = profile.channel("wz").vmax if "wz" in profile.channel_names else 1.5
    tau_vx = plant["vx"]["tau"] or 0.1
    max_linear_accel = vx_ceiling / tau_vx
    return {
        "max_linear_speed": vx_ceiling,
        "max_angular_speed": wz_ceiling * (1.0 - WZ_HEADROOM_MARGIN),
        "max_centripetal_accel": A_LAT_MAX,
        "max_linear_accel": max_linear_accel,
        "max_linear_decel": DECEL_ACCEL_RATIO * max_linear_accel,
        "min_speed": DEFAULT_MIN_SPEED,
        "lookahead_pts": DEFAULT_LOOKAHEAD_PTS,
    }


def build_tuned_artifact(
    profile: RobotProfile,
    tunings: dict[str, ChannelTuning],
    *,
    robot_id: str,
    sim_or_hw: str,
    surface: str = "",
    mode: str = "characterization",
    date: str = "",
    git_sha: str = "",
    session_dir: str = "",
    methodology_version: int = 2,
    extra_caveats: list[str] | None = None,
) -> dict[str, Any]:
    """Build the tuned-config artifact dict in the exact consuming contract.

    ``tunings`` maps channel name -> ChannelTuning (the winning gains + plant).
    ``sim_or_hw`` gates ``valid_for_tuning``: only hardware data is tune-valid."""
    validate_contract_channels(profile)
    plant, placeholders = _plant_slots(profile, tunings)

    feedforward: dict[str, Any] = {}
    for slot in CONTRACT_SLOTS:
        feedforward[f"K_{slot}"] = _safe_inv_gain(plant[slot]["K"])
        vmax = (
            profile.channel(slot).vmax
            if slot in profile.channel_names
            else (1.5 if slot == "wz" else 1.0)
        )
        feedforward[f"output_min_{slot}"] = -vmax
        feedforward[f"output_max_{slot}"] = vmax

    is_hw = sim_or_hw == "hw"
    caveats: list[str] = list(extra_caveats or [])
    if not is_hw:
        caveats.insert(0, "PIPELINE CHECK ONLY - DO NOT TUNE (data is not hardware-sourced)")
    if placeholders:
        caveats.append(
            f"channels {placeholders} were not characterized; emitted as identity placeholders"
        )
    failed = [ch for ch, t in tunings.items() if t.verdict == "fail"]
    if failed:
        caveats.append(f"channels {failed} FAILED the robustness verdict; gains are unsafe")

    return {
        "provenance": {
            "robot_id": robot_id,
            "surface": surface,
            "mode": mode,
            "date": date,
            "git_sha": git_sha,
            "sim_or_hw": sim_or_hw,
            "characterization_session_dir": session_dir,
            "methodology_version": methodology_version,
        },
        "plant": plant,
        "feedforward": feedforward,
        "velocity_profile": _velocity_profile(profile, plant),
        "recommended_controller": {
            "name": "baseline",
            "params": {"k_angular": 0.5},
            "evidence": "lambda-tuned PI per axis from FOPDT fit",
        },
        "caveats": caveats,
        "operating_point_map": None,
        "velocity_envelope": None,
        "dynamics_by_amplitude": None,
        "floor_probe_results": None,
        "valid_for_tuning": is_hw,
        "schema_version": SCHEMA_VERSION,
    }


def write_tuned_artifact(path: str | Path, artifact: dict[str, Any]) -> Path:
    """Write the tuned-config artifact with stable key order for round-trip."""
    p = Path(path)
    p.parent.mkdir(parents=True, exist_ok=True)
    p.write_text(json.dumps(artifact, indent=2, sort_keys=False))
    return p


def build_characterization_report(
    profile: RobotProfile, tunings: dict[str, ChannelTuning] | None = None
) -> dict[str, Any]:
    """Summarize measured plant properties as durable robot metadata. Distinct
    from the tuned artifact: this is the characterization record, not gains."""
    m = profile.measured
    channels: dict[str, Any] = {}
    for name in profile.channel_names:
        f = m.fopdt.get(name)
        ch: dict[str, Any] = {
            "fopdt": {"K": f.K, "tau": f.tau, "L": f.L} if f else None,
            "bandwidth_hz": m.bandwidth_hz.get(name),
            "deadzone": f.deadzone if f else None,
            "direction_asymmetric": (
                f.K_forward is not None and f.K_reverse is not None if f else None
            ),
            "cross_axis_coupling": m.cross_axis_coupling.get(name),
            "samples_per_tau": m.samples_per_tau.get(name),
        }
        if tunings and name in tunings:
            ch["verdict"] = tunings[name].verdict
        channels[name] = ch
    return {
        "robot": profile.name,
        "odom_type": profile.odom_type,
        "fitter": profile.fitter,
        "channels": channels,
        "streams": {
            n: {"rate_hz": s.rate_hz, "jitter_s": s.jitter_s, "noise_floor": s.noise_floor}
            for n, s in m.streams.items()
        },
    }


def write_characterization_report(path: str | Path, report: dict[str, Any]) -> Path:
    p = Path(path)
    p.parent.mkdir(parents=True, exist_ok=True)
    p.write_text(json.dumps(report, indent=2, sort_keys=False))
    return p
