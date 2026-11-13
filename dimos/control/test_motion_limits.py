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

"""Motion policy is shared across sources, with explicit dispatch progress."""

import math

import pytest

from dimos.control.motion_limits import MotionGroup, MotionLimiter
from dimos.control.task import ControlMode


@pytest.fixture
def limiter():
    return MotionLimiter({"arm": MotionGroup(joints=("joint",), rate=0.5, ceiling=1)})


@pytest.mark.parametrize("owner", ["leader", "browser", "webxr_ik", "policy"])
def test_all_sources_share_slew_and_unaccepted_commands_do_not_advance(limiter, owner):
    commands = {"joint": (1.0, ControlMode.SERVO_POSITION, owner)}
    first = limiter.apply(commands, {"joint": 0.0}, 0.01)
    assert first["joint"][0] == pytest.approx(0.005)
    assert limiter.apply(commands, {"joint": 0.0}, 0.01) == first
    limiter.commit({"joint": first["joint"][0]})
    assert limiter.apply(commands, {"joint": 0.0}, 0.01)["joint"][0] == pytest.approx(0.01)


def test_handoff_uses_last_dispatch_and_missing_feedback_requires_new_anchor(limiter):
    limiter.commit({"joint": 0.3})
    assert limiter.apply({"joint": (0.9, ControlMode.POSITION, "ik")}, {"joint": 0.1}, 0.01)[
        "joint"
    ][0] == pytest.approx(0.305)
    assert limiter.snapshot({}) == {"joint": None}
    assert not limiter.apply({"joint": (1.0, ControlMode.POSITION, "ik")}, {}, 0.01)
    assert limiter.apply({"joint": (1.0, ControlMode.POSITION, "ik")}, {"joint": 0.7}, 0.01)[
        "joint"
    ][0] == pytest.approx(0.705)


@pytest.mark.parametrize("invalid", [-1, 1.1, math.nan, math.inf])
def test_settings_are_atomic_and_bounded(limiter, invalid):
    with pytest.raises(ValueError):
        limiter.set_rates({"arm": invalid})
    assert limiter.settings()["arm"].rate == 0.5
    with pytest.raises(ValueError):
        limiter.set_rates({"arm": 0, "unknown": 0.2})
    assert limiter.settings()["arm"].rate == 0.5


def test_zero_rate_holds_position_and_caps_velocity(limiter):
    limiter.set_rates({"arm": 0})
    assert (
        limiter.apply({"joint": (1.0, ControlMode.POSITION, "policy")}, {"joint": 0.4}, 1)["joint"][
            0
        ]
        == 0.4
    )
    assert limiter.apply({"joint": (-2.0, ControlMode.VELOCITY, "policy")}, {}, 1)["joint"][0] == 0


def test_coordinated_group_preserves_relative_joint_step():
    limiter = MotionLimiter(
        {"torso": MotionGroup(joints=("a", "b"), rate=1, ceiling=1, coordinated=True)}
    )
    assert limiter.apply({}, {}, 0.01) == {}
    out = limiter.apply(
        {"a": (1.0, ControlMode.POSITION, "joylo"), "b": (-2.0, ControlMode.POSITION, "joylo")},
        {"a": 0.0, "b": 0.0},
        0.01,
    )
    assert out["a"][0] == pytest.approx(0.005)
    assert out["b"][0] == pytest.approx(-0.01)
