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

"""m20-rust builds without hardware: no vendor bridge, the drivers placed on AOS, and m20's navigation and relocalization."""

import pytest

from dimos.core.coordination.blueprints import Blueprint
from dimos.core.coordination.module_coordinator import stream_name_types, verify_no_name_conflicts
from dimos.hosted.fragment_compiler import compile_fragments, resolve_hosted_assignments
from dimos.hosted.protocol import HostDescriptor
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.robot.deeprobotics.m20.blueprints._nav import m20_nav_stack, m20_relocalization
from dimos.robot.deeprobotics.m20.blueprints.m20 import m20
from dimos.robot.deeprobotics.m20.blueprints.m20_rust import m20_rust
from dimos.robot.deeprobotics.m20.connection import M20Connection

_REVISION = "revision-1"
_HOSTS = (
    HostDescriptor(host_id="gos", name="gos", tags={}, revision=_REVISION, active_run_ids=()),
    HostDescriptor(
        host_id="aos", name="aos", tags={"board": "aos"}, revision=_REVISION, active_run_ids=()
    ),
    HostDescriptor(
        host_id="nos", name="nos", tags={"board": "nos"}, revision=_REVISION, active_run_ids=()
    ),
)
_AOS_DRIVERS = {"front_airy", "rear_airy", "m20lidarmounttf", "m20imu", "m20camerarelay"}


def _solid(**choices: str) -> Blueprint:
    blueprint = m20_rust.solidify(choices=choices)
    verify_no_name_conflicts(blueprint)
    return blueprint


def _placement(blueprint: Blueprint) -> dict[str, str]:
    return resolve_hosted_assignments(blueprint, _HOSTS, local_host_id="gos", revision=_REVISION)


def test_no_vendor_bridge() -> None:
    for blueprint in (_solid(), _solid(M20Joints="ecan", M20Cameras="off")):
        assert M20Connection not in {atom.module for atom in blueprint.active_blueprints}


@pytest.mark.parametrize(
    ("choices", "on_aos"),
    [
        ({}, _AOS_DRIVERS),
        ({"M20Joints": "ecan"}, _AOS_DRIVERS | {"m20ecan"}),
        ({"M20Cameras": "off"}, _AOS_DRIVERS - {"m20camerarelay"}),
        ({"M20Cameras": "off", "M20BodyImu": "off"}, _AOS_DRIVERS - {"m20camerarelay", "m20imu"}),
    ],
)
def test_drivers_run_on_aos_and_the_rest_where_the_run_starts(
    choices: dict[str, str], on_aos: set[str]
) -> None:
    placement = _placement(_solid(**choices))
    assert {name for name, host in placement.items() if host == "aos"} == on_aos
    assert {host for name, host in placement.items() if name not in on_aos} == {"gos"}
    assert len(compile_fragments(_solid(**choices), placement, run_id="r", revision=_REVISION)) == 2


def test_each_airy_is_its_own_stream_and_every_consumer_takes_both() -> None:
    blueprint = _solid()

    def topic(module: str, stream: str) -> object:
        return blueprint.remapping_map.get((module, stream), stream)

    assert (topic("front_airy", "lidar"), topic("front_airy", "imu")) == (
        "front_lidar",
        "front_lidar_imu",
    )
    assert (topic("rear_airy", "lidar"), topic("rear_airy", "imu")) == (
        "rear_lidar",
        "rear_lidar_imu",
    )
    for consumer in ("dimslamlio", "raytracingvoxelmap", "premapmodule"):
        assert (topic(consumer, "lidar"), topic(consumer, "second_lidar")) == (
            "front_lidar",
            "rear_lidar",
        )
    assert topic("dimslamlio", "imu") == "front_lidar_imu"
    streams = stream_name_types(blueprint)
    assert {
        ("front_lidar", PointCloud2),
        ("rear_lidar", PointCloud2),
        ("front_lidar_imu", Imu),
    } <= streams
    assert not {name for name, _ in streams} & {"slam_body_points", "lidar", "second_lidar"}


def test_navigation_and_relocalization_match_m20() -> None:
    rust = {atom.name: atom for atom in _solid().active_blueprints}
    vendor = {atom.name: atom for atom in m20.solidify().active_blueprints}
    shared = {atom.name for atom in m20_nav_stack().solidify().active_blueprints} | {
        atom.name for atom in m20_relocalization().active_blueprints
    }
    for name in shared:
        assert (rust[name].module, rust[name].kwargs) == (vendor[name].module, vendor[name].kwargs)
