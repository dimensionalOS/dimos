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

from __future__ import annotations

import numpy as np
import pytest

from dimos.control.safety.guardrail_policy import (
    GuardrailHealth,
    GuardrailState,
    OpticalFlowMagnitudeGuardrailPolicy,
    OpticalFlowMagnitudePolicyConfig,
)
from dimos.control.safety.test_utils import (
    _textured_gray_image,
)
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat


def _policy_config(
    *,
    caution_frame_count: int = 2,
    stop_frame_count: int = 2,
    clear_frame_count: int = 3,
    stop_release_frame_count: int = 2,
) -> OpticalFlowMagnitudePolicyConfig:
    return OpticalFlowMagnitudePolicyConfig(
        forward_motion_deadband_mps=0.05,
        clamp_forward_speed_mps=0.1,
        flow_downsample_width_px=160,
        forward_roi_top_fraction=0.45,
        forward_roi_bottom_fraction=0.95,
        forward_roi_width_fraction=0.5,
        low_texture_variance_threshold=150.0,
        occlusion_dark_pixel_threshold=20,
        occlusion_bright_pixel_threshold=235,
        occlusion_extreme_fraction_threshold=0.9,
        caution_flow_magnitude_threshold=0.8,
        stop_flow_magnitude_threshold=1.5,
        caution_frame_count=caution_frame_count,
        stop_frame_count=stop_frame_count,
        clear_frame_count=clear_frame_count,
        stop_release_frame_count=stop_release_frame_count,
    )


def _forward_cmd(
    x: float = 0.4,
    *,
    linear_y: float = 0.0,
    linear_z: float = 0.0,
    angular_z: float = 0.2,
) -> Twist:
    return Twist(
        linear=[x, linear_y, linear_z],
        angular=[0.0, 0.0, angular_z],
    )


def _fresh_health(
    *,
    has_previous_frame: bool = True,
    image_fresh: bool = True,
    frame_pair_fresh: bool = True,
) -> GuardrailHealth:
    return GuardrailHealth(
        has_previous_frame=has_previous_frame,
        image_fresh=image_fresh,
        cmd_fresh=True,
        risk_fresh=True,
        frame_pair_fresh=frame_pair_fresh,
    )


def _uniform_gray_image(value: int, *, width: int = 160, height: int = 120) -> Image:
    return Image.from_numpy(
        np.full((height, width), value, dtype=np.uint8),
        format=ImageFormat.GRAY,
    )


@pytest.fixture
def image_pair() -> tuple[Image, Image]:
    return (_textured_gray_image(), _textured_gray_image(shift_x=3))


@pytest.mark.parametrize(
    "cmd",
    [
        pytest.param(_forward_cmd(0.03, angular_z=0.35), id="below_forward_deadband"),
        pytest.param(_forward_cmd(-0.2, linear_y=0.1, angular_z=0.4), id="reverse_motion"),
        pytest.param(
            Twist(linear=[0.0, 0.0, 0.0], angular=[0.0, 0.0, 0.6]),
            id="pure_yaw",
        ),
    ],
)
def test_forward_guard_inactive_passthrough(image_pair: tuple[Image, Image], cmd: Twist) -> None:
    policy = OpticalFlowMagnitudeGuardrailPolicy(_policy_config())

    decision = policy.evaluate(
        previous_image=image_pair[0],
        current_image=image_pair[1],
        incoming_cmd_vel=cmd,
        health=_fresh_health(),
    )

    assert decision.state == GuardrailState.PASS
    assert decision.reason == "forward_guard_inactive"
    assert decision.cmd_vel == cmd


def test_missing_previous_frame_returns_init_zero(image_pair: tuple[Image, Image]) -> None:
    policy = OpticalFlowMagnitudeGuardrailPolicy(_policy_config())

    decision = policy.evaluate(
        previous_image=image_pair[0],
        current_image=image_pair[1],
        incoming_cmd_vel=_forward_cmd(),
        health=_fresh_health(has_previous_frame=False),
    )

    assert decision.state == GuardrailState.INIT
    assert decision.reason == "missing_previous_frame"
    assert decision.cmd_vel == Twist.zero()


def test_stale_image_health_degrades_to_zero(image_pair: tuple[Image, Image]) -> None:
    policy = OpticalFlowMagnitudeGuardrailPolicy(_policy_config())

    decision = policy.evaluate(
        previous_image=image_pair[0],
        current_image=image_pair[1],
        incoming_cmd_vel=_forward_cmd(),
        health=_fresh_health(image_fresh=False),
    )

    assert decision.state == GuardrailState.SENSOR_DEGRADED
    assert decision.reason == "image_not_fresh"
    assert decision.cmd_vel == Twist.zero()
    assert decision.publish_immediately is True


@pytest.mark.parametrize(
    ("bad_frame_position", "bad_frame_kind", "expected_reason"),
    [
        pytest.param("previous", "black", "previous_roi_occluded", id="previous_black_occluded"),
        pytest.param("current", "black", "current_roi_occluded", id="current_black_occluded"),
        pytest.param("previous", "white", "previous_roi_occluded", id="previous_white_occluded"),
        pytest.param("current", "white", "current_roi_occluded", id="current_white_occluded"),
        pytest.param(
            "previous",
            "uniform_gray",
            "previous_roi_low_texture",
            id="previous_low_texture",
        ),
        pytest.param(
            "current",
            "uniform_gray",
            "current_roi_low_texture",
            id="current_low_texture",
        ),
    ],
)
def test_bad_previous_or_current_roi_fail_closes(
    image_pair: tuple[Image, Image],
    bad_frame_position: str,
    bad_frame_kind: str,
    expected_reason: str,
) -> None:
    policy = OpticalFlowMagnitudeGuardrailPolicy(_policy_config())

    if bad_frame_kind == "black":
        bad_image = _uniform_gray_image(0)
    elif bad_frame_kind == "white":
        bad_image = _uniform_gray_image(255)
    else:
        bad_image = _uniform_gray_image(127)

    previous_image, current_image = image_pair
    if bad_frame_position == "previous":
        previous_image = bad_image
    else:
        current_image = bad_image

    decision = policy.evaluate(
        previous_image=previous_image,
        current_image=current_image,
        incoming_cmd_vel=_forward_cmd(),
        health=_fresh_health(),
    )

    assert decision.state == GuardrailState.SENSOR_DEGRADED
    assert decision.reason == expected_reason
    assert decision.cmd_vel == Twist.zero()
    assert decision.publish_immediately is True


def test_caution_hysteresis_reaches_clamp(image_pair: tuple[Image, Image], mocker) -> None:
    policy = OpticalFlowMagnitudeGuardrailPolicy(_policy_config())
    mocker.patch.object(
        policy,
        "_mean_flow_magnitude",
        side_effect=[0.9, 0.9],
    )

    first = policy.evaluate(
        previous_image=image_pair[0],
        current_image=image_pair[1],
        incoming_cmd_vel=_forward_cmd(),
        health=_fresh_health(),
    )
    second = policy.evaluate(
        previous_image=image_pair[0],
        current_image=image_pair[1],
        incoming_cmd_vel=_forward_cmd(),
        health=_fresh_health(),
    )

    assert first.state == GuardrailState.PASS
    assert second.state == GuardrailState.CLAMP
    assert second.reason == "forward_flow_clamp"
    assert second.cmd_vel.linear.x == pytest.approx(0.1)


def test_first_stop_strength_frame_clamps_immediately(
    image_pair: tuple[Image, Image], mocker
) -> None:
    policy = OpticalFlowMagnitudeGuardrailPolicy(_policy_config())
    cmd = _forward_cmd(0.45, angular_z=0.55)

    mocker.patch.object(policy, "_mean_flow_magnitude", return_value=1.8)

    decision = policy.evaluate(
        previous_image=image_pair[0],
        current_image=image_pair[1],
        incoming_cmd_vel=cmd,
        health=_fresh_health(),
    )

    assert decision.state == GuardrailState.CLAMP
    assert decision.reason == "forward_flow_clamp"
    assert decision.cmd_vel.linear.x == pytest.approx(0.1)
    assert decision.cmd_vel.angular.z == pytest.approx(cmd.angular.z)


def test_repeated_stop_strength_frames_reach_stop_latched(
    image_pair: tuple[Image, Image], mocker
) -> None:
    policy = OpticalFlowMagnitudeGuardrailPolicy(_policy_config())
    cmd = _forward_cmd(0.45, angular_z=0.55)

    mocker.patch.object(
        policy,
        "_mean_flow_magnitude",
        side_effect=[1.8, 1.8],
    )

    first = policy.evaluate(
        previous_image=image_pair[0],
        current_image=image_pair[1],
        incoming_cmd_vel=cmd,
        health=_fresh_health(),
    )
    second = policy.evaluate(
        previous_image=image_pair[0],
        current_image=image_pair[1],
        incoming_cmd_vel=cmd,
        health=_fresh_health(),
    )

    assert first.state == GuardrailState.CLAMP
    assert first.reason == "forward_flow_clamp"
    assert second.state == GuardrailState.STOP_LATCHED
    assert second.reason == "forward_flow_stop"
    assert second.cmd_vel.linear.x == pytest.approx(0.0)
    assert second.cmd_vel.angular.z == pytest.approx(cmd.angular.z)


def test_stop_latched_does_not_release_on_first_clear_frame(
    image_pair: tuple[Image, Image], mocker
) -> None:
    policy = OpticalFlowMagnitudeGuardrailPolicy(_policy_config())

    mocker.patch.object(
        policy,
        "_mean_flow_magnitude",
        side_effect=[1.8, 1.8, 0.0],
    )

    states = [
        policy.evaluate(
            previous_image=image_pair[0],
            current_image=image_pair[1],
            incoming_cmd_vel=_forward_cmd(),
            health=_fresh_health(),
        ).state
        for _ in range(3)
    ]

    assert states == [
        GuardrailState.CLAMP,
        GuardrailState.STOP_LATCHED,
        GuardrailState.STOP_LATCHED,
    ]


def test_stop_latched_recovery_requires_clear_frames(
    image_pair: tuple[Image, Image], mocker
) -> None:
    policy = OpticalFlowMagnitudeGuardrailPolicy(_policy_config(clear_frame_count=3))

    mocker.patch.object(
        policy,
        "_mean_flow_magnitude",
        side_effect=[1.8, 1.8, 0.0, 0.0, 0.0],
    )

    states = [
        policy.evaluate(
            previous_image=image_pair[0],
            current_image=image_pair[1],
            incoming_cmd_vel=_forward_cmd(),
            health=_fresh_health(),
        ).state
        for _ in range(5)
    ]

    assert states[0] == GuardrailState.CLAMP
    assert states[1] == GuardrailState.STOP_LATCHED
    assert states[2] != GuardrailState.PASS
    assert states[3] != GuardrailState.PASS
    assert states[4] == GuardrailState.PASS


def test_recovery_after_clear_frames_returns_to_pass(
    image_pair: tuple[Image, Image], mocker
) -> None:
    policy = OpticalFlowMagnitudeGuardrailPolicy(_policy_config())

    mocker.patch.object(
        policy,
        "_mean_flow_magnitude",
        side_effect=[0.9, 0.9, 0.0, 0.0, 0.0],
    )

    states = [
        policy.evaluate(
            previous_image=image_pair[0],
            current_image=image_pair[1],
            incoming_cmd_vel=_forward_cmd(),
            health=_fresh_health(),
        ).state
        for _ in range(5)
    ]

    assert states == [
        GuardrailState.PASS,
        GuardrailState.CLAMP,
        GuardrailState.CLAMP,
        GuardrailState.CLAMP,
        GuardrailState.PASS,
    ]


def test_forward_deadband_does_not_reset_hysteresis(
    image_pair: tuple[Image, Image], mocker
) -> None:
    policy = OpticalFlowMagnitudeGuardrailPolicy(_policy_config(caution_frame_count=2))

    mocker.patch.object(
        policy,
        "_mean_flow_magnitude",
        side_effect=[0.9, 0.9],
    )

    first = policy.evaluate(
        previous_image=image_pair[0],
        current_image=image_pair[1],
        incoming_cmd_vel=_forward_cmd(0.4),
        health=_fresh_health(),
    )
    inactive = policy.evaluate(
        previous_image=image_pair[0],
        current_image=image_pair[1],
        incoming_cmd_vel=_forward_cmd(0.03),
        health=_fresh_health(),
    )
    third = policy.evaluate(
        previous_image=image_pair[0],
        current_image=image_pair[1],
        incoming_cmd_vel=_forward_cmd(0.4),
        health=_fresh_health(),
    )

    assert first.state == GuardrailState.PASS
    assert inactive.state == GuardrailState.PASS
    assert inactive.reason == "forward_guard_inactive"
    assert third.state == GuardrailState.CLAMP
    assert third.reason == "forward_flow_clamp"


def test_clamp_preserves_angular_terms(image_pair: tuple[Image, Image], mocker) -> None:
    policy = OpticalFlowMagnitudeGuardrailPolicy(_policy_config(caution_frame_count=1))
    mocker.patch.object(policy, "_mean_flow_magnitude", return_value=0.9)
    cmd = _forward_cmd(0.4, linear_y=0.15, linear_z=-0.1, angular_z=0.65)

    decision = policy.evaluate(
        previous_image=image_pair[0],
        current_image=image_pair[1],
        incoming_cmd_vel=cmd,
        health=_fresh_health(),
    )

    assert decision.state == GuardrailState.CLAMP
    assert decision.cmd_vel.linear.x == pytest.approx(0.1)
    assert decision.cmd_vel.linear.y == pytest.approx(cmd.linear.y)
    assert decision.cmd_vel.linear.z == pytest.approx(cmd.linear.z)
    assert decision.cmd_vel.angular.z == pytest.approx(cmd.angular.z)


def test_stop_zeroes_only_linear_x(image_pair: tuple[Image, Image], mocker) -> None:
    policy = OpticalFlowMagnitudeGuardrailPolicy(_policy_config(stop_frame_count=1))
    mocker.patch.object(policy, "_mean_flow_magnitude", return_value=1.8)
    cmd = _forward_cmd(0.45, linear_y=0.2, linear_z=-0.1, angular_z=0.75)

    decision = policy.evaluate(
        previous_image=image_pair[0],
        current_image=image_pair[1],
        incoming_cmd_vel=cmd,
        health=_fresh_health(),
    )

    assert decision.state == GuardrailState.STOP_LATCHED
    assert decision.cmd_vel.linear.x == pytest.approx(0.0)
    assert decision.cmd_vel.linear.y == pytest.approx(cmd.linear.y)
    assert decision.cmd_vel.linear.z == pytest.approx(cmd.linear.z)
    assert decision.cmd_vel.angular.z == pytest.approx(cmd.angular.z)
