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

"""Tests for the operator run-profile contract."""

import math

import pytest

from dimos.navigation.dannav.controllers import command_envelope_overrides_for_profile
from dimos.navigation.holonomic_trajectory_controller.trajectory_run_profiles import (
    GO2_RUN_PROFILES,
    RunProfile,
    RunProfileError,
    RunProfileRegistry,
)


def _profile(**overrides: object) -> RunProfile:
    base: dict[str, object] = dict(
        name="probe",
        requested_planner_speed_m_s=1.0,
        max_tangent_accel_m_s2=1.0,
        max_normal_accel_m_s2=0.6,
        goal_decel_m_s2=1.0,
        max_planar_cmd_accel_m_s2=5.0,
        max_yaw_rate_rad_s=1.0,
        max_yaw_accel_rad_s2=5.0,
        required_locomotion_mode="default",
    )
    base.update(overrides)
    return RunProfile(**base)  # type: ignore[arg-type]


def test_go2_profiles_step_the_speed_envelope_up() -> None:
    speeds = [
        GO2_RUN_PROFILES.get(name).requested_planner_speed_m_s
        for name in ("walk", "trot", "run_conservative", "run_verified")
    ]
    assert speeds == sorted(speeds)
    assert len(set(speeds)) == len(speeds)


@pytest.mark.parametrize(
    "field_name",
    [
        "requested_planner_speed_m_s",
        "max_tangent_accel_m_s2",
        "max_normal_accel_m_s2",
        "goal_decel_m_s2",
        "max_planar_cmd_accel_m_s2",
        "max_yaw_rate_rad_s",
        "max_yaw_accel_rad_s2",
    ],
)
@pytest.mark.parametrize("bad_value", [0.0, -1.0, math.nan, math.inf])
def test_envelope_fields_reject_non_positive_or_non_finite(
    field_name: str, bad_value: float
) -> None:
    with pytest.raises(RunProfileError, match=field_name):
        _profile(**{field_name: bad_value})


def test_empty_name_rejected() -> None:
    with pytest.raises(RunProfileError, match="name"):
        _profile(name="   ")


def test_empty_locomotion_mode_rejected() -> None:
    with pytest.raises(RunProfileError, match="required_locomotion_mode"):
        _profile(required_locomotion_mode="")


def test_profile_limit_helpers_match_fields() -> None:
    profile = GO2_RUN_PROFILES.get("run_conservative")
    limits = profile.command_limits()
    assert limits.max_planar_speed_m_s == pytest.approx(profile.requested_planner_speed_m_s)
    assert limits.max_yaw_rate_rad_s == pytest.approx(profile.max_yaw_rate_rad_s)

    path_limits = profile.path_speed_profile_limits_at(1.25)
    assert path_limits.max_speed_m_s == pytest.approx(1.25)
    assert path_limits.max_tangent_accel_m_s2 == pytest.approx(profile.max_tangent_accel_m_s2)

    overrides = command_envelope_overrides_for_profile(profile)
    assert overrides.max_yaw_rate_rad_s == pytest.approx(profile.max_yaw_rate_rad_s)
    assert overrides.max_planar_cmd_accel_m_s2 == pytest.approx(profile.max_planar_cmd_accel_m_s2)


def test_registry_rejects_key_name_mismatch() -> None:
    walk = GO2_RUN_PROFILES.get("walk")
    with pytest.raises(RunProfileError, match="does not match"):
        RunProfileRegistry(
            profiles={"stroll": walk},
            default_profile_name="stroll",
        )


def test_registry_rejects_missing_default() -> None:
    walk = GO2_RUN_PROFILES.get("walk")
    with pytest.raises(RunProfileError, match="default profile"):
        RunProfileRegistry(
            profiles={"walk": walk},
            default_profile_name="run_verified",
        )


def test_get_unknown_name_raises_with_known_names() -> None:
    with pytest.raises(RunProfileError) as excinfo:
        GO2_RUN_PROFILES.get("sprint")
    message = str(excinfo.value)
    assert "sprint" in message
    for known in ("walk", "trot", "run_conservative", "run_verified"):
        assert known in message
