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

"""Integration test for the autotune module.

Drives one full autotune run against a known synthetic plant - declare a
profile, probe the streams, build and drive the excitation battery, fit, tune,
and emit the artifact - asserting at each stage. Every external API the module
binds to (Twist/Vector3, EpisodeStatus) is exercised as part of the flow, so
drift in those surfaces fails here rather than on the robot.

In-process and synthetic: no coordinator is started and no LCM traffic is
exchanged. The process-level run against a booted blueprint belongs in
``dimos/e2e_tests/`` and needs the live drive wiring first.
"""

from __future__ import annotations

import numpy as np
import pytest

from dimos.control.autotune.drive import run_battery
from dimos.control.autotune.excitation import step_battery
from dimos.control.autotune.fit.synth import MeasurementModel, synth_step
from dimos.control.autotune.live import make_sinks
from dimos.control.autotune.probe import advise, compute_timing
from dimos.control.autotune.profile import BatteryConfig, Channel, RobotProfile
from dimos.control.autotune.runner import autotune_offline

# The plant the synthetic robot actually has; autotune must recover it.
K_TRUE, TAU_TRUE, L_TRUE = 0.9, 0.35, 0.10
ODOM_HZ = 50.0


class _FakeClock:
    def __init__(self) -> None:
        self.t = 0.0

    def now(self) -> float:
        return self.t

    def sleep(self, seconds: float) -> None:
        self.t += max(0.0, seconds)


def _recorded_segments(vmax: float) -> list:
    """Step segments a real run would have recorded, from the known plant."""
    segments = []
    for fraction in (0.25, 0.5, 0.75):
        for direction in (1, -1):
            amp = direction * fraction * vmax
            t, velocity, _ = synth_step(
                K_TRUE,
                TAU_TRUE,
                L_TRUE,
                amp=amp,
                duration_s=4.0,
                model=MeasurementModel(rate_hz=ODOM_HZ, noise_std=0.0),
            )
            segments.append((t, velocity, amp))
    return segments


def test_autotune_recovers_known_plant_and_emits_gains() -> None:
    # 1. Declare the robot.
    profile = RobotProfile(
        name="synth-base",
        command_interface="twist",
        odom_type="velocity",
        channels=[Channel("vx", vmax=1.0), Channel("wz", vmax=1.5)],
        fitter="velocity",
        controller_form="velocity_pi",
        command_stream="/cmd_vel",
        feedback_stream="/odom",
        expected_tau_s=TAU_TRUE,
        battery=BatteryConfig(amplitude_fractions=(0.5,), repeats=1),
    )

    # 2. Passive probe: stream timing, and the advisory that informs (never
    #    forces) the fitter choice.
    timing = compute_timing("odom", np.arange(0, 2.0, 1.0 / ODOM_HZ))
    advice = advise(timing, profile.expected_tau_s)
    assert timing.rate_hz == pytest.approx(ODOM_HZ, rel=1e-6)
    assert advice.samples_per_tau == pytest.approx(ODOM_HZ * TAU_TRUE, rel=1e-3)
    assert profile.fitter == "velocity"  # advice did not mutate the choice

    # 3. Drive the battery through the live sinks (Twist + EpisodeStatus APIs),
    #    capturing what a coordinator and recorder would have seen.
    twists: list = []
    statuses: list = []
    command_sink, episode_sink, _ = make_sinks(profile, twists.append, statuses.append)
    runs = step_battery(profile)  # 2 channels * 1 amplitude * 2 directions
    played = run_battery(runs, command_sink, episode_sink, _FakeClock(), tick_hz=10.0, settle_s=0.1)

    assert played == len(runs) == 4
    # one episode per run, and the base is commanded to zero at each run's end
    assert sum(s.last_event == "start" for s in statuses) == 4
    assert sum(s.last_event == "save" for s in statuses) == 4
    assert statuses[-1].episodes_saved == 4
    assert twists[-1].linear.x == 0.0 and twists[-1].angular.z == 0.0
    # commands landed on the right twist axes
    assert any(t.linear.x != 0.0 for t in twists)
    assert any(t.angular.z != 0.0 for t in twists)

    # 4. Fit, derive bandwidth, tune, and emit - from the recorded episodes.
    segments = {
        name: _recorded_segments(profile.channel(name).vmax) for name in profile.channel_names
    }
    outputs = autotune_offline(profile, segments, robot_id="synth", sim_or_hw="sim")

    # recovered the plant it was driven with
    vx = outputs.profile.measured.fopdt["vx"]
    assert vx.K == pytest.approx(K_TRUE, rel=0.05)
    assert vx.tau == pytest.approx(TAU_TRUE, rel=0.15)
    assert vx.L == pytest.approx(L_TRUE, abs=0.03)
    assert outputs.profile.measured.bandwidth_hz["vx"] == pytest.approx(
        1.0 / (2.0 * np.pi * vx.tau), rel=1e-6
    )
    assert set(outputs.tunings) == {"vx", "wz"}
    assert outputs.tunings["vx"].gains.Kp > 0

    # 5. The artifact a control task loads, in the shape it expects.
    artifact = outputs.artifact
    assert artifact["schema_version"] == 1
    assert set(artifact["plant"]) == {"vx", "vy", "wz"}
    assert artifact["feedforward"]["K_vx"] == pytest.approx(1.0 / vx.K, rel=1e-6)
    assert artifact["valid_for_tuning"] is False  # synthetic data is never tune-valid
    assert "DO NOT TUNE" in artifact["caveats"][0]

    # 6. The characterization report is separate metadata, not gains.
    assert outputs.report["robot"] == "synth-base"
    assert outputs.report["channels"]["vx"]["verdict"] in ("pass", "marginal", "fail")
    assert "feedforward" not in outputs.report
