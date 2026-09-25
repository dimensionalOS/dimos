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

"""Run with: pytest experimental/tars_sdk/tests"""

from collections.abc import Iterator
import math

import pytest
from tars_sdk import TarsClient
from tars_sdk.kinematics import SlabGeometry, foot_fk, foot_ik
from tars_sdk.model.params import Params


@pytest.mark.parametrize("dx", [-0.5, -0.2, 0.0, 0.1, 0.45])
def test_foot_ik_fk_roundtrip(dx: float) -> None:
    g = SlabGeometry.from_params(Params().resolved())
    h = 1.45 * Params().scale
    theta, slide = foot_ik(g, dx * Params().scale, h)
    edge_x, edge_z, center_x = foot_fk(g, theta, slide)
    assert center_x == pytest.approx(dx * Params().scale, abs=1e-9)
    assert edge_z == pytest.approx(-h, abs=1e-9)


@pytest.fixture
def client() -> Iterator[TarsClient]:
    c = TarsClient(realtime=False, cmd_timeout=None)
    c.connect()
    c.stand()
    assert c.wait_for_mode("ready", timeout=5)
    yield c
    c.disconnect()


def test_stands_at_walking_height(client: TarsClient) -> None:
    s = client.get_state()
    assert s.odom_gt is not None
    s_ = Params().scale
    assert s.odom_gt.z == pytest.approx(1.45 * s_, abs=0.01)
    assert s.measurement.foot_force.sum() == pytest.approx(60 * s_**3 * 9.81, rel=0.05)


def test_walks_forward_with_accurate_odometry(client: TarsClient) -> None:
    client.move(0.3, 0.0)
    client.step(20.0)
    s = client.get_state()
    assert s.odom_gt is not None
    assert s.odom_gt.x > 4.0
    assert abs(s.odom.x - s.odom_gt.x) < 0.05 * s.odom_gt.x
    assert abs(s.odom_gt.y) < 0.05


def test_timeout_stops_motion() -> None:
    c = TarsClient(realtime=False, cmd_timeout=0.5)
    c.connect()
    c.stand()
    c.wait_for_mode("ready", timeout=5)
    c.move(0.15, 0.0)  # never resent
    c.step(10.0)
    assert c.get_state().mode == "ready"
    assert c.get_odometry(ground_truth=True).x < 1.0 * Params().scale  # finishes one step, stops
    c.disconnect()


def test_turns_in_place(client: TarsClient) -> None:
    client.move(0.0, 0.2)
    client.step(10.0)
    gt = client.get_odometry(ground_truth=True)
    assert gt.yaw == pytest.approx(2.0, abs=0.3)
    assert math.hypot(gt.x, gt.y) < 0.05 * Params().scale


def test_camera_renders_rgb_and_depth(client: TarsClient) -> None:
    f = client.get_camera(160, 120)
    assert f.rgb.shape == (120, 160, 3)
    assert f.depth is not None and f.depth.shape == (120, 160)
    assert f.depth.min() > 0.5


def test_rolls_and_folds_back_up(client: TarsClient) -> None:
    client.set_mode("roll")
    client.move(1.0, 0.0)
    client.step(12.0)
    assert client.locomotion == "roll"
    gt = client.get_odometry(ground_truth=True)
    assert gt.x > 6.0 * Params().scale ** 0.5 * Params().scale ** 0.5, (
        "roll mode should keep rolling"
    )
    assert gt.z > 1.0 * Params().scale

    client.set_mode("walk")
    client.move(0.0, 0.0)
    assert client.wait_for_mode("ready", timeout=15.0)
    assert client.get_odometry(ground_truth=True).z == pytest.approx(
        1.45 * Params().scale, abs=0.03
    )
