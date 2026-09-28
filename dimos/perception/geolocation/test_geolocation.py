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

"""Camera rays, line of sight, ground intersection and the target filter."""

from __future__ import annotations

from dataclasses import replace
import math

import numpy as np
import pytest

from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.perception.geolocation.estimators import (
    EstimatorConfig,
    GimbalGeo,
    LosEstimator,
    Observation,
    TargetEstimator,
    VehicleGeo,
)
from dimos.perception.geolocation.geometry import (
    CameraModel,
    GimbalFrameConfig,
    LOSSolver,
    intersect_ground,
    ned_to_az_el,
)

# 1280x720 over an 81 deg horizontal field of view.
_F = 640.0 / math.tan(math.radians(81.0) / 2)
INFO = CameraInfo.from_intrinsics(_F, _F, 640.0, 360.0, 1280, 720, "camera_optical")
PERSON = Observation(1, "person", (620.0, 320.0, 40.0, 80.0))  # centred on (640, 360)
PITCH, YAW = -20.0, 30.0  # gimbal


def test_camera_model_from_camera_info() -> None:
    cam = CameraModel(INFO)
    assert cam.fx == pytest.approx(749.3, abs=0.2)
    assert (cam.cx, cam.cy) == (640.0, 360.0)
    assert cam.pixel_to_ray(640.0, 360.0).tolist() == pytest.approx([1.0, 0.0, 0.0])
    right_edge = cam.pixel_to_ray(1280.0, 360.0)  # half the 81 deg field of view
    assert math.degrees(math.atan2(right_edge[1], right_edge[0])) == pytest.approx(40.5, abs=0.1)
    assert right_edge[2] == pytest.approx(0.0)


def test_line_of_sight_adds_gimbal_yaw_to_vehicle_heading() -> None:
    r = LOSSolver(CameraModel(INFO)).solve(640.0, 360.0, PITCH, YAW, vehicle_yaw_deg=10.0)
    assert r.azimuth_deg == pytest.approx(40.0, abs=1e-6)
    assert r.elevation_deg == pytest.approx(-20.0, abs=1e-6)
    assert r.gimbal_yaw_body_deg == pytest.approx(30.0)
    assert ned_to_az_el(np.array(r.los_ned)) == pytest.approx((40.0, -20.0), abs=1e-6)


def test_line_of_sight_literal_values() -> None:
    # Off-centre pixel, all three angles set: pins the rotation order and the signs.
    r = LOSSolver(CameraModel(INFO)).solve(900.0, 500.0, -35.0, 30.0, vehicle_yaw_deg=100.0)
    assert r.los_ned == pytest.approx((-0.6730763759, 0.2999390691, 0.6760212623), abs=1e-9)
    assert r.azimuth_deg == pytest.approx(155.9810941519, abs=1e-8)
    assert r.elevation_deg == pytest.approx(-42.5335073808, abs=1e-8)
    assert r.gimbal_yaw_body_deg == pytest.approx(30.0)


def test_earth_frame_gimbal_yaw_is_not_added_to_the_heading() -> None:
    solver = LOSSolver(CameraModel(INFO), GimbalFrameConfig(yaw_frame="earth"))
    r = solver.solve(640.0, 360.0, PITCH, YAW, vehicle_yaw_deg=170.0)
    assert r.azimuth_deg == pytest.approx(30.0, abs=1e-6)
    assert r.gimbal_yaw_body_deg == pytest.approx(-140.0)
    r = LOSSolver(CameraModel(INFO)).solve(640.0, 360.0, PITCH, YAW, vehicle_yaw_deg=170.0)
    assert r.azimuth_deg == pytest.approx(200.0, abs=1e-6)
    assert r.gimbal_yaw_body_deg == pytest.approx(30.0)


def test_ground_intersection_range_and_rejections() -> None:
    down20 = (math.cos(math.radians(20)), 0.0, math.sin(math.radians(20)))
    pt, rng = intersect_ground((0.0, 0.0, -10.0), down20, 0.0)
    assert pt is not None and isinstance(rng, float)
    assert pt[0] == pytest.approx(10.0 / math.tan(math.radians(20)), abs=1e-6)
    assert rng == pytest.approx(10.0 / math.sin(math.radians(20)), abs=1e-6)
    assert intersect_ground((0.0, 0.0, -10.0), (1.0, 0.0, 0.01), 0.0)[0] is None  # too shallow
    assert intersect_ground((0.0, 0.0, 5.0), (0.7, 0.0, 0.7), 0.0)[0] is None  # below the plane


def test_los_estimator_rejects_unselected_and_stale_then_solves() -> None:
    est = LosEstimator(LOSSolver(CameraModel(INFO)), EstimatorConfig())
    veh = VehicleGeo(yaw_deg=10.0, attitude_age_s=0.1, n=0.0, e=0.0, d=-10.0, rel_alt=10.0)
    gim = GimbalGeo(PITCH, YAW, age_s=0.2)
    assert est.process([PERSON], None, 100.1, veh, gim).reason == "no selection"
    lost = est.process([PERSON], 9, 100.1, veh, gim)
    assert not lost.valid and lost.reason == "selected target not visible"
    stale = GimbalGeo(PITCH, YAW, age_s=5.0)
    assert est.process([PERSON], 1, 100.1, veh, stale).reason == "gimbal attitude stale"
    old = VehicleGeo(yaw_deg=10.0, attitude_age_s=5.0)
    assert est.process([PERSON], 1, 100.1, old, gim).reason == "vehicle attitude stale"
    los = est.process([PERSON], 1, 100.1, veh, gim)
    assert los.valid and los.track_id == 1 and los.class_name == "person"
    assert los.azimuth_deg == pytest.approx(40.0, abs=1e-6)
    assert los.gimbal_yaw_body_deg == pytest.approx(30.0)


def test_target_estimator_intersects_ground_and_filters() -> None:
    cfg = EstimatorConfig()
    est = LosEstimator(LOSSolver(CameraModel(INFO)), cfg)
    target = TargetEstimator(cfg)
    veh = VehicleGeo(yaw_deg=0.0, attitude_age_s=0.1, n=0.0, e=0.0, d=-10.0, rel_alt=10.0)
    gim = GimbalGeo(PITCH, YAW, age_s=0.2)
    state = None
    for i in range(6):
        t = 100.0 + i / 25
        state = target.process(est.process([PERSON], 1, t, veh, gim), veh, t + 0.01)
    assert state is not None
    assert state.valid, state.reason
    # 10 m up, aiming 1.0 m up a person: 9.0 m to the aim plane.
    horizontal = 9.0 / math.tan(math.radians(20.0))
    assert state.n == pytest.approx(horizontal * math.cos(math.radians(30)), abs=0.05)
    assert state.e == pytest.approx(horizontal * math.sin(math.radians(30)), abs=0.05)
    assert state.bearing_deg == pytest.approx(30.0, abs=0.05)
    assert state.range_m == pytest.approx(horizontal, abs=0.05)
    # Without a line of sight the filter coasts, goes invalid, then drops the state.
    lost = target.process(None, veh, 101.5)
    assert not lost.valid and lost.n is not None
    assert target.process(None, veh, 100.24 + cfg.drop_after_s + 1.0).n is None


def test_target_estimator_needs_a_height() -> None:
    cfg = EstimatorConfig()
    est = LosEstimator(LOSSolver(CameraModel(INFO)), cfg)
    gim = GimbalGeo(PITCH, YAW, age_s=0.2)
    veh = VehicleGeo(yaw_deg=0.0, attitude_age_s=0.1, n=0.0, e=0.0, d=-10.0)
    los = est.process([PERSON], 1, 100.0, veh, gim)
    state = TargetEstimator(cfg).process(los, veh, 100.01)
    assert not state.valid and state.reason == "AGL: relative_alt unavailable"
    fixed = TargetEstimator(EstimatorConfig(agl_source="fixed", fixed_agl_m=10.0))
    assert fixed.process(los, veh, 100.01).valid


def test_camera_offset_and_unlisted_class_height() -> None:
    # Camera 0.1 m below the altitude reference; a class with no entry, in a table without it.
    cfg = EstimatorConfig(camera_below_ref_m=0.1, aim_height_m={"person": 1.0})
    veh = VehicleGeo(yaw_deg=0.0, attitude_age_s=0.1, n=0.0, e=0.0, d=-10.0, rel_alt=10.0)
    gim = GimbalGeo(PITCH, 0.0, age_s=0.2)
    cat = Observation(1, "cat", PERSON.bbox)
    los = LosEstimator(LOSSolver(CameraModel(INFO)), cfg).process([cat], 1, 100.0, veh, gim)
    state = TargetEstimator(cfg).process(los, veh, 100.01)
    assert state.valid, state.reason
    drop = 10.0 - 0.1 - cfg.default_aim_height_m
    assert state.n == pytest.approx(drop / math.tan(math.radians(20.0)), abs=1e-6)


def test_non_finite_input_never_enters_the_filter() -> None:
    cfg = EstimatorConfig()
    est = LosEstimator(LOSSolver(CameraModel(INFO)), cfg)
    target = TargetEstimator(cfg)
    veh = VehicleGeo(yaw_deg=0.0, attitude_age_s=0.1, n=0.0, e=0.0, d=-10.0, rel_alt=10.0)
    gim = GimbalGeo(PITCH, YAW, age_s=0.2)
    assert target.process(est.process([PERSON], 1, 100.0, veh, gim), veh, 100.01).valid
    # One bad vehicle position on the same track id, then good data again.
    bad = target.process(
        est.process([PERSON], 1, 100.04, veh, gim), replace(veh, n=math.nan), 100.05
    )
    assert bad.reason == "non-finite measurement"
    state = target.process(est.process([PERSON], 1, 100.08, veh, gim), veh, 100.09)
    assert state.valid and state.reason == "ok"
    assert state.n is not None and math.isfinite(state.n)
    # A NaN gimbal angle is caught a stage earlier.
    los = est.process([PERSON], 1, 100.12, veh, GimbalGeo(math.nan, YAW, age_s=0.2))
    assert not los.valid and los.reason == "non-finite line of sight"
    # An earth-frame gimbal yaw keeps the ray finite under a NaN heading; the body yaw is not.
    earth = LosEstimator(LOSSolver(CameraModel(INFO), GimbalFrameConfig(yaw_frame="earth")), cfg)
    los = earth.process([PERSON], 1, 100.16, replace(veh, yaw_deg=math.nan), gim)
    assert not los.valid and los.reason == "non-finite line of sight"
