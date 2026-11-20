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
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.robot.deeprobotics.m20.blueprints._nav import m20_nav_stack, m20_relocalization
from dimos.robot.deeprobotics.m20.blueprints.m20 import m20
from dimos.robot.deeprobotics.m20.blueprints.m20_rust import m20_rust
from dimos.robot.deeprobotics.m20.blueprints.m20_slam import m20_dim_slam
from dimos.robot.deeprobotics.m20.connection import M20Connection
from dimos.robot.deeprobotics.m20.model import M20StandingJointStateConfig

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
_AOS_DRIVERS = {"front_lidar/m20lidar", "rear_lidar/m20lidar", "m20imu", "m20camerarelay"}


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


def _topic(blueprint: Blueprint, module: str, stream: str) -> object:
    return blueprint.remapping_map.get((module, stream), stream)


def test_raw_lidar_goes_only_into_the_body_filter_and_everything_else_takes_the_filtered_cloud() -> (
    None
):
    blueprint = _solid()
    atoms = {atom.name: atom for atom in blueprint.active_blueprints}
    for lidar in ("front_lidar/m20lidar", "rear_lidar/m20lidar"):
        assert (_topic(blueprint, lidar, "lidar"), _topic(blueprint, lidar, "imu")) == (
            "lidar",
            "lidar_imu",
        )
        assert _topic(blueprint, lidar, "tf") == "tf"
    assert {
        atoms["front_lidar/m20lidar"].kwargs["frame_id"],
        atoms["rear_lidar/m20lidar"].kwargs["frame_id"],
    } == {
        "front_lidar",
        "rear_lidar",
    }
    readers_of_lidar = {
        atom.name
        for atom in blueprint.active_blueprints
        for stream in atom.streams
        if stream.direction == "in" and _topic(blueprint, atom.name, stream.name) == "lidar"
    }
    assert readers_of_lidar == {"urdfmanagerpointclearing"}
    assert _topic(blueprint, "urdfmanagerpointclearing", "filtered_pointcloud") == "lidar_filtered"
    for consumer in ("dimslamlio", "raytracingvoxelmap", "premapmodule"):
        assert _topic(blueprint, consumer, "lidar") == "lidar_filtered"
    assert _topic(blueprint, "dimslamlio", "imu") == "lidar_imu"
    streams = stream_name_types(blueprint)
    assert {
        ("lidar", PointCloud2),
        ("lidar_filtered", PointCloud2),
        ("lidar_imu", Imu),
        ("tf", TFMessage),
    } <= streams
    assert not any(name.startswith(("front_lidar/", "rear_lidar/")) for name, _ in streams)


def test_the_only_thinning_is_the_lidars_voxel_at_the_ray_tracers_voxel() -> None:
    atoms = {atom.name: atom for atom in _solid().active_blueprints}
    voxel = atoms["raytracingvoxelmap"].kwargs["voxel_size"]
    for lidar in ("front_lidar/m20lidar", "rear_lidar/m20lidar"):
        assert atoms[lidar].kwargs["voxel"] == voxel
    assert (
        atoms["raytracingvoxelmap"].kwargs["pre_trace_subsample"],
        atoms["raytracingvoxelmap"].kwargs["ray_subsample"],
    ) == (1, 1)


def test_dimslam_keeps_its_own_lidar_and_imu_frames() -> None:
    """The shared streams carry every frame; dimSLAM keeps the ones m20_dim_slam lists, which the lidars publish."""
    atoms = {atom.name: atom for atom in _solid().active_blueprints}
    slam = atoms["dimslamlio"].kwargs
    expected = m20_dim_slam().blueprints[0].kwargs
    assert (slam["lidars"], slam["imus"]) == (expected["lidars"], expected["imus"])
    lidar_frames = {
        atoms[name].kwargs["imu_frame_id"]
        for name in ("front_lidar/m20lidar", "rear_lidar/m20lidar")
    }
    assert {imu.frame_id for imu in slam["imus"]} <= lidar_frames
    assert {lidar.frame_id for lidar in slam["lidars"]} == lidar_frames


def test_navigation_and_relocalization_match_m20() -> None:
    rust = {atom.name: atom for atom in _solid().active_blueprints}
    vendor = {atom.name: atom for atom in m20.solidify().active_blueprints}
    shared = {atom.name for atom in m20_nav_stack().solidify().active_blueprints} | {
        atom.name for atom in m20_relocalization().active_blueprints
    }
    thinning = {"pre_trace_subsample", "ray_subsample"}
    for name in shared:
        rust_kwargs = {k: v for k, v in rust[name].kwargs.items() if k not in thinning}
        vendor_kwargs = {k: v for k, v in vendor[name].kwargs.items() if k not in thinning}
        assert (rust[name].module, rust_kwargs) == (vendor[name].module, vendor_kwargs)


def test_the_body_filter_matches_the_joint_source_it_is_given() -> None:
    """A fixed pose republished every period is matched within that period; ecan's 1 kHz state keeps the default."""
    standing = {a.name: a for a in _solid().active_blueprints}["urdfmanagerpointclearing"].kwargs
    ecan = {a.name: a for a in _solid(M20Joints="ecan").active_blueprints}[
        "urdfmanagerpointclearing"
    ].kwargs
    assert standing["state_tolerance_s"] == M20StandingJointStateConfig().publish_period_s
    assert "state_tolerance_s" not in ecan
