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
from dimos.robot.deeprobotics.m20.constants import GOS_SLAM_CPUS, NOS_SLAM_CPUS
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
_AOS_DRIVERS = {"front_lidar/m20lidar", "rear_lidar/m20lidar", "m20bodyimusniff", "m20camerarelay"}
_NOS_SLAM = {"front_lidar_slices/m20lidar", "rear_lidar_slices/m20lidar", "dimslamlio"}


def _solid(**choices: str) -> Blueprint:
    blueprint = m20_rust.solidify(choices=choices)
    verify_no_name_conflicts(blueprint)
    return blueprint


def _placement(blueprint: Blueprint) -> dict[str, str]:
    return resolve_hosted_assignments(blueprint, _HOSTS, local_host_id="gos", revision=_REVISION)


def test_no_vendor_bridge() -> None:
    for blueprint in (
        _solid(),
        _solid(M20Joints="ecan", M20Cameras="off"),
        _solid(M20BodyFilter="python"),
    ):
        assert M20Connection not in {atom.module for atom in blueprint.active_blueprints}


@pytest.mark.parametrize(
    ("choices", "on_aos"),
    [
        ({}, _AOS_DRIVERS),
        ({"M20Joints": "ecan"}, _AOS_DRIVERS | {"m20ecan"}),
        ({"M20Cameras": "off"}, _AOS_DRIVERS - {"m20camerarelay"}),
        (
            {"M20Cameras": "off", "M20BodyImu": "serial"},
            _AOS_DRIVERS - {"m20camerarelay", "m20bodyimusniff"} | {"m20imu"},
        ),
        ({"M20BodyImu": "off"}, _AOS_DRIVERS - {"m20bodyimusniff"}),
        ({"M20BodyFilter": "python"}, _AOS_DRIVERS),
        ({"M20Joints": "ecan", "M20BodyFilter": "python"}, _AOS_DRIVERS | {"m20ecan"}),
    ],
)
def test_drivers_run_on_aos_and_the_rest_where_the_run_starts(
    choices: dict[str, str], on_aos: set[str]
) -> None:
    placement = _placement(_solid(**choices))
    assert {name for name, host in placement.items() if host == "aos"} == on_aos
    assert {name for name, host in placement.items() if host == "nos"} == _NOS_SLAM
    assert {host for name, host in placement.items() if name not in on_aos | _NOS_SLAM} == {"gos"}
    assert len(compile_fragments(_solid(**choices), placement, run_id="r", revision=_REVISION)) == 3


def test_dimslam_on_gos_stays_selectable() -> None:
    blueprint = _solid(M20Slam="gos")
    placement = _placement(blueprint)
    assert placement["dimslamlio"] == "gos"
    assert "nos" not in placement.values()
    assert _readers(blueprint, "lidar") == {"dimslamlio", "raytracingvoxelmap", "premapmodule"}
    assert _topic(blueprint, "dimslamlio", "imu") == "lidar_imu"
    atoms = {atom.name: atom for atom in blueprint.active_blueprints}
    assert atoms["dimslamlio"].kwargs["cpus"] == list(GOS_SLAM_CPUS)


def test_dimslam_on_nos_reads_only_its_own_slices_of_the_lidars() -> None:
    blueprint = _solid()
    atoms = {atom.name: atom for atom in blueprint.active_blueprints}
    slices = ("front_lidar_slices/m20lidar", "rear_lidar_slices/m20lidar")
    for name in slices:
        kwargs = atoms[name].kwargs
        assert (_topic(blueprint, name, "lidar"), _topic(blueprint, name, "imu")) == (
            "lidar_slices",
            "lidar_slices_imu",
        )
        # raw slices, ~10 ms of the 10 Hz sweep each, at the same voxel as the full sweeps
        assert kwargs["split_blocks"] == 179
        assert kwargs["body_filter"] is False
        assert kwargs["voxel"] == atoms["front_lidar/m20lidar"].kwargs["voxel"]
    # each lidar is the same frame in both pairs: dimSLAM's lidar and IMU frames are the slices'
    assert {atoms[n].kwargs["frame_id"] for n in slices} == {"front_lidar", "rear_lidar"}
    assert _readers(blueprint, "lidar_slices") == {"dimslamlio"}
    assert _readers(blueprint, "lidar_slices_imu") == {"dimslamlio"}
    assert _readers(blueprint, "lidar") == {"raytracingvoxelmap", "premapmodule"}
    assert _topic(blueprint, "dimslamlio", "odometry") == "slam_odom"
    slam = atoms["dimslamlio"].kwargs
    assert slam["cpus"] == list(NOS_SLAM_CPUS)
    assert slam["publish_every"] >= 1


@pytest.mark.parametrize("joints", ["standing", "ecan"])
def test_every_lidar_mount_has_one_publisher(joints: str) -> None:
    """Both pairs read the same two Airys; only AOS's publishes base_link -> lidar, so tf has one writer per edge."""
    atoms = {atom.name: atom for atom in _solid(M20Joints=joints).active_blueprints}
    publishers = [
        atoms[name].kwargs["frame_id"]
        for name in atoms
        if name.endswith("m20lidar") and atoms[name].kwargs.get("mount_parent_frame_id")
    ]
    assert sorted(publishers) == ["front_lidar", "rear_lidar"]
    for name in ("front_lidar/m20lidar", "rear_lidar/m20lidar"):
        assert atoms[name].kwargs["mount_parent_frame_id"] == "base_link"


def _topic(blueprint: Blueprint, module: str, stream: str) -> object:
    return blueprint.remapping_map.get((module, stream), stream)


def _readers(blueprint: Blueprint, topic: str) -> set[str]:
    return {
        atom.name
        for atom in blueprint.active_blueprints
        for stream in atom.streams
        if stream.direction == "in" and _topic(blueprint, atom.name, stream.name) == topic
    }


@pytest.mark.parametrize("joints", ["standing", "ecan"])
def test_the_lidars_cut_the_body_and_every_consumer_takes_their_stream(joints: str) -> None:
    blueprint = _solid(M20Joints=joints)
    atoms = {atom.name: atom for atom in blueprint.active_blueprints}
    for lidar in ("front_lidar/m20lidar", "rear_lidar/m20lidar"):
        assert (_topic(blueprint, lidar, "lidar"), _topic(blueprint, lidar, "imu")) == (
            "lidar",
            "lidar_imu",
        )
        assert _topic(blueprint, lidar, "tf") == "tf"
        assert atoms[lidar].kwargs["body_filter_joint_source"] == (
            "fixed" if joints == "standing" else "joint_state"
        )
        assert _topic(blueprint, lidar, "joint_state") == "coordinator_joint_state"
    assert {
        atoms["front_lidar/m20lidar"].kwargs["frame_id"],
        atoms["rear_lidar/m20lidar"].kwargs["frame_id"],
    } == {
        "front_lidar",
        "rear_lidar",
    }
    # no separate filter: dimSLAM, the ray tracer and relocalization read the lidars' own stream
    assert "urdfmanagerpointclearing" not in atoms
    assert _readers(blueprint, "lidar") == {"raytracingvoxelmap", "premapmodule"}
    streams = stream_name_types(blueprint)
    assert {("lidar", PointCloud2), ("lidar_imu", Imu), ("tf", TFMessage)} <= streams
    assert not any(name.startswith(("front_lidar", "rear_lidar")) for name, _ in streams)
    assert "lidar_raw" not in {name for name, _ in streams}


@pytest.mark.parametrize("joints", ["standing", "ecan"])
def test_the_python_filter_stays_selectable_between_lidar_raw_and_lidar(joints: str) -> None:
    blueprint = _solid(M20Joints=joints, M20BodyFilter="python")
    atoms = {atom.name: atom for atom in blueprint.active_blueprints}
    for lidar in ("front_lidar/m20lidar", "rear_lidar/m20lidar"):
        assert _topic(blueprint, lidar, "lidar") == "lidar_raw"
        assert atoms[lidar].kwargs["body_filter"] is False
    assert _readers(blueprint, "lidar_raw") == {"urdfmanagerpointclearing"}
    assert _topic(blueprint, "urdfmanagerpointclearing", "filtered_pointcloud") == "lidar"
    assert _readers(blueprint, "lidar") == {"raytracingvoxelmap", "premapmodule"}


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
        for name in ("front_lidar_slices/m20lidar", "rear_lidar_slices/m20lidar")
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


def test_the_python_filter_matches_the_joint_source_it_is_given() -> None:
    """A fixed pose republished every period is matched within that period; ecan's 1 kHz state keeps the default."""
    standing = {a.name: a for a in _solid(M20BodyFilter="python").active_blueprints}[
        "urdfmanagerpointclearing"
    ].kwargs
    ecan = {a.name: a for a in _solid(M20Joints="ecan", M20BodyFilter="python").active_blueprints}[
        "urdfmanagerpointclearing"
    ].kwargs
    assert standing["state_tolerance_s"] == M20StandingJointStateConfig().publish_period_s
    assert "state_tolerance_s" not in ecan


def test_dimslam_runs_on_nos_a76_cores() -> None:
    atoms = {atom.name: atom for atom in _solid().active_blueprints}
    assert atoms["dimslamlio"].kwargs["cpus"] == list(NOS_SLAM_CPUS)
    assert all(4 <= cpu <= 7 for cpu in NOS_SLAM_CPUS)
