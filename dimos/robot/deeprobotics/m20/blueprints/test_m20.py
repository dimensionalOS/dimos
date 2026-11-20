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


"""``m20`` builds without hardware in each mode: real (no vendor bridge, the drivers placed on AOS and NOS, and the
navigation and relocalization m20 ran on the vendor's sensors), sim (the same navigation) and sim-bench."""

from dataclasses import replace
import math
import typing

import pytest

from dimos.core.coordination.blueprints import Blueprint, BlueprintAtom, autoconnect
from dimos.core.coordination.module_coordinator import stream_name_types, verify_no_name_conflicts
from dimos.core.module import ModuleBase
from dimos.hardware.sensors.camera.rockchip_h264.module import RockchipH264CamerasConfig
from dimos.hardware.sensors.imu.yesense.module import YesenseImuConfig
from dimos.hosted.fragment_compiler import compile_fragments, resolve_hosted_assignments
from dimos.hosted.protocol import HostDescriptor
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.navigation.embodiment.m20 import M20
from dimos.navigation.global_planner.mls_planner.mls_planner_native import MLSPlannerNative
from dimos.navigation.movement_manager.movement_manager import MovementManager
from dimos.robot.deeprobotics.m20.blueprints.m20 import (
    m20,
    m20_dim_slam,
    m20_nav_stack,
    m20_relocalization,
)
from dimos.robot.deeprobotics.m20.connection import M20Connection
from dimos.robot.deeprobotics.m20.constants import GOS_SLAM_CPUS, NOS_SLAM_CPUS
from dimos.robot.deeprobotics.m20.model import M20StandingJointStateConfig
from dimos.robot.deeprobotics.m20.rerun import m20_nav_overrides, m20_nav_static
from dimos.simulation.m20_sim.world import SIM_M20, STEP_LIMIT
from dimos.spec.utils import Spec
from dimos.visualization.rerun.bridge import RerunBridgeModule

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
_AOS_DRIVERS = {"m20bodyimusniff", "rockchiph264cameras"}
# both lidar pairs read the multicast on NOS: the full scans (body cut) for the map, the slices for dimSLAM
_NOS_LIDARS = {"front_lidar/airy", "rear_lidar/airy"}
_NOS_SLAM = {"front_lidar_slices/airy", "rear_lidar_slices/airy", "dimslam"}


# m20 on the vendor's sensors was these with the vendor bridge; the real mode keeps their settings
_VENDOR_NAV = autoconnect(m20_nav_stack(), m20_relocalization())


def _solid(**choices: str) -> Blueprint:
    blueprint = m20.solidify(choices={"M20Mode": "real", **choices})
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
        ({"M20Cameras": "off"}, _AOS_DRIVERS - {"rockchiph264cameras"}),
        (
            {"M20Cameras": "off", "M20BodyImu": "serial"},
            _AOS_DRIVERS - {"rockchiph264cameras", "m20bodyimusniff"} | {"yesenseimu"},
        ),
        ({"M20BodyImu": "off"}, _AOS_DRIVERS - {"m20bodyimusniff"}),
        ({"M20BodyFilter": "python"}, _AOS_DRIVERS),
        ({"M20Joints": "ecan", "M20BodyFilter": "python"}, _AOS_DRIVERS | {"m20ecan"}),
    ],
)
def test_drivers_on_aos_lidars_and_dimslam_on_nos_and_the_rest_where_the_run_starts(
    choices: dict[str, str], on_aos: set[str]
) -> None:
    placement = _placement(_solid(**choices))
    assert {name for name, host in placement.items() if host == "aos"} == on_aos
    assert {name for name, host in placement.items() if host == "nos"} == _NOS_LIDARS | _NOS_SLAM
    assert {
        host for name, host in placement.items() if name not in on_aos | _NOS_LIDARS | _NOS_SLAM
    } == {"gos"}
    assert len(compile_fragments(_solid(**choices), placement, run_id="r", revision=_REVISION)) == 3


def test_dimslam_on_gos_stays_selectable() -> None:
    blueprint = _solid(M20Slam="gos")
    placement = _placement(blueprint)
    assert placement["dimslam"] == "gos"
    assert {name for name, host in placement.items() if host == "nos"} == _NOS_LIDARS
    assert _readers(blueprint, "lidar") == {"dimslam", "raytracingvoxelmap", "premapmodule"}
    assert _topic(blueprint, "dimslam", "imu") == "lidar_imu"
    atoms = {atom.name: atom for atom in blueprint.active_blueprints}
    assert atoms["dimslam"].kwargs["cpus"] == list(GOS_SLAM_CPUS)


def test_dimslam_on_nos_reads_only_its_own_slices_of_the_lidars() -> None:
    blueprint = _solid()
    atoms = {atom.name: atom for atom in blueprint.active_blueprints}
    slices = ("front_lidar_slices/airy", "rear_lidar_slices/airy")
    for name in slices:
        kwargs = atoms[name].kwargs
        assert (_topic(blueprint, name, "lidar"), _topic(blueprint, name, "imu")) == (
            "lidar_slices",
            "lidar_slices_imu",
        )
        # raw slices, ~10 ms of the 10 Hz sweep each, at the same voxel as the full sweeps
        assert kwargs["split_blocks"] == 179
        assert kwargs["body_filter"] is False
        assert kwargs["voxel"] == atoms["front_lidar/airy"].kwargs["voxel"]
    # each lidar is the same frame in both pairs: dimSLAM's lidar and IMU frames are the slices'
    assert {atoms[n].kwargs["frame_id"] for n in slices} == {"front_lidar", "rear_lidar"}
    assert _readers(blueprint, "lidar_slices") == {"dimslam"}
    assert _readers(blueprint, "lidar_slices_imu") == {"dimslam"}
    assert _readers(blueprint, "lidar") == {"raytracingvoxelmap", "premapmodule"}
    assert _topic(blueprint, "dimslam", "odometry") == "slam_odom"
    slam = atoms["dimslam"].kwargs
    assert slam["cpus"] == list(NOS_SLAM_CPUS)
    assert slam["publish_every"] >= 1


@pytest.mark.parametrize("joints", ["standing", "ecan"])
def test_every_lidar_mount_has_one_publisher(joints: str) -> None:
    """Both pairs read the same two Airys; only AOS's publishes base_link -> lidar, so tf has one writer per edge."""
    atoms = {atom.name: atom for atom in _solid(M20Joints=joints).active_blueprints}
    publishers = [
        atoms[name].kwargs["frame_id"]
        for name in atoms
        if name.endswith("/airy") and atoms[name].kwargs.get("mount_parent_frame_id")
    ]
    assert sorted(publishers) == ["front_lidar", "rear_lidar"]
    for name in ("front_lidar/airy", "rear_lidar/airy"):
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
    for lidar in ("front_lidar/airy", "rear_lidar/airy"):
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
        atoms["front_lidar/airy"].kwargs["frame_id"],
        atoms["rear_lidar/airy"].kwargs["frame_id"],
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
    for lidar in ("front_lidar/airy", "rear_lidar/airy"):
        assert _topic(blueprint, lidar, "lidar") == "lidar_raw"
        assert atoms[lidar].kwargs["body_filter"] is False
    assert _readers(blueprint, "lidar_raw") == {"urdfmanagerpointclearing"}
    assert _topic(blueprint, "urdfmanagerpointclearing", "filtered_pointcloud") == "lidar"
    assert _readers(blueprint, "lidar") == {"raytracingvoxelmap", "premapmodule"}


def test_the_lidars_thin_at_or_above_the_ray_tracers_voxel_and_it_subsamples_as_m20() -> None:
    """The lidars thin their own scans (no finer than the map's voxel) and send only x, y, z and offset_time; the ray
    tracer then subsamples them as m20's does the vendor cloud."""
    atoms = {atom.name: atom for atom in _solid().active_blueprints}
    m20_atoms = {atom.name: atom for atom in _VENDOR_NAV.solidify().active_blueprints}
    voxel = atoms["raytracingvoxelmap"].kwargs["voxel_size"]
    for lidar in ("front_lidar/airy", "rear_lidar/airy"):
        assert atoms[lidar].kwargs["voxel"] >= voxel
        assert atoms[lidar].kwargs["point_format"] == "minimal"
    for key in ("pre_trace_subsample", "ray_subsample"):
        assert (
            atoms["raytracingvoxelmap"].kwargs[key] == m20_atoms["raytracingvoxelmap"].kwargs[key]
        )


def test_dimslams_slices_keep_every_field_at_the_ray_tracers_voxel() -> None:
    atoms = {atom.name: atom for atom in _solid().active_blueprints}
    for lidar in ("front_lidar_slices/airy", "rear_lidar_slices/airy"):
        assert atoms[lidar].kwargs["voxel"] == atoms["raytracingvoxelmap"].kwargs["voxel_size"]
        assert "point_format" not in atoms[lidar].kwargs


def test_dimslam_keeps_its_own_lidar_and_imu_frames() -> None:
    """The shared streams carry every frame; dimSLAM keeps the ones m20_dim_slam lists, which the lidars publish."""
    atoms = {atom.name: atom for atom in _solid().active_blueprints}
    slam = atoms["dimslam"].kwargs
    expected = m20_dim_slam().blueprints[0].kwargs
    assert (slam["lidars"], slam["imus"]) == (expected["lidars"], expected["imus"])
    lidar_frames = {
        atoms[name].kwargs["imu_frame_id"]
        for name in ("front_lidar_slices/airy", "rear_lidar_slices/airy")
    }
    assert {imu.frame_id for imu in slam["imus"]} <= lidar_frames
    assert {lidar.frame_id for lidar in slam["lidars"]} == lidar_frames


def test_navigation_and_relocalization_match_m20() -> None:
    rust = {atom.name: atom for atom in _solid().active_blueprints}
    vendor = {atom.name: atom for atom in _VENDOR_NAV.solidify().active_blueprints}
    shared = {atom.name for atom in m20_nav_stack().solidify().active_blueprints} | {
        atom.name for atom in m20_relocalization().active_blueprints
    }
    # only the per-sweep pacing of two lidars differs (checked in its own test)
    thinning = {"emit_every", "attempt_after_frames", "attempt_every_sweeps"}
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
    assert atoms["dimslam"].kwargs["cpus"] == list(NOS_SLAM_CPUS)
    assert all(4 <= cpu <= 7 for cpu in NOS_SLAM_CPUS)


@pytest.mark.parametrize(
    "choices",
    [
        {},
        {"M20Slam": "gos"},
        {"M20Joints": "ecan"},
        {"M20Joints": "ecan", "M20BodyFilter": "python"},
    ],
)
def test_every_module_config_m20_rust_builds_validates_as_its_board_will_parse_it(
    choices: dict[str, str],
) -> None:
    """A Host validates each module's config when it starts the fragment; nothing the blueprint sets may fail there."""
    for atom in _solid(**choices).active_blueprints:
        config_type = typing.get_type_hints(atom.module)["config"]
        config_type(**{key: value for key, value in atom.kwargs.items() if key != "instance_name"})


def test_one_local_map_and_one_relocalization_per_sweep_of_both_lidars() -> None:
    """Each Airy publishes its own cloud per sweep, so the ray tracer emits every 2nd and the relocalizer waits for both."""
    atoms = {atom.name: atom for atom in _solid().active_blueprints}
    m20_atoms = {atom.name: atom for atom in _VENDOR_NAV.solidify().active_blueprints}
    assert (
        atoms["raytracingvoxelmap"].kwargs["emit_every"]
        == 2 * m20_atoms["raytracingvoxelmap"].kwargs["emit_every"]
    )
    lidar_frames = {
        atoms[name].kwargs["frame_id"] for name in ("front_lidar/airy", "rear_lidar/airy")
    }
    assert set(atoms["premapmodule"].kwargs["attempt_after_frames"]) == lidar_frames
    assert atoms["premapmodule"].kwargs["attempt_every_sweeps"] == 10
    assert m20_atoms["premapmodule"].kwargs["attempt_after_frames"] == []


def test_the_cameras_get_the_m20s_devices_mounts_and_focal_length() -> None:
    (atom,) = (a for a in _solid().active_blueprints if a.name == "rockchiph264cameras")
    config = RockchipH264CamerasConfig(
        **{k: v for k, v in atom.kwargs.items() if k != "instance_name"}
    )
    assert (config.front_device, config.rear_device) == ("/dev/video0", "/dev/video2")
    assert (config.front_camera_xyz, config.rear_camera_xyz) == (
        (0.37646, 0.0, 0.03738),
        (-0.37646, 0.0, 0.03738),
    )
    assert (config.front_camera_yaw_rad, config.rear_camera_yaw_rad, config.focal_length_px) == (
        0.0,
        math.pi,
        607.0,
    )
    assert (config.width, config.height, config.fps, config.bitrate_bps, config.deskew) == (
        1920,
        1080,
        30,
        9_000_000,
        False,
    )
    # the 1280-wide focal length, which the driver scales to the stream's width: x1.5 at 1080p
    assert config.focal_length_px * config.width / config.focal_length_width_px == 607.0 * 1.5


def test_the_serial_body_imu_gets_the_m20s_uart_and_mount() -> None:
    (atom,) = (a for a in _solid(M20BodyImu="serial").active_blueprints if a.name == "yesenseimu")
    config = YesenseImuConfig(**{k: v for k, v in atom.kwargs.items() if k != "instance_name"})
    assert (config.device, config.frame_id, config.imu_xyz) == (
        "/dev/ttyS9",
        "imu_link",
        (0.0632, -0.0268, -0.0435),
    )
    assert config.mount_rpy_rad == (math.pi / 2, 0.0, 0.0)


# --- sim and sim-bench ---

# The robot's connection gates nav velocity, the sim world takes it directly.
_SIM = m20.solidify(choices={"M20Mode": "sim"})
_SIM_BENCH = m20.solidify(choices={"M20Mode": "sim-bench"})
_REAL = m20.solidify(choices={"M20Mode": "real"})


def _atoms(blueprint: Blueprint) -> dict[str, BlueprintAtom]:
    return {atom.name: atom for atom in blueprint.active_blueprints}


def _remappings(
    blueprint: Blueprint, name: str
) -> dict[tuple[str, str], str | type[ModuleBase] | type[Spec]]:
    return {key: target for key, target in blueprint.remapping_map.items() if key[0] == name}


def test_sim_composes_the_navigation_the_robot_runs() -> None:
    sim, sim_stack, robot_stack = (
        _atoms(_SIM),
        _atoms(m20_nav_stack(SIM_M20).solidify()),
        _atoms(m20_nav_stack().solidify()),
    )
    assert sim_stack.keys() == robot_stack.keys()
    for name in robot_stack:
        assert (sim[name].module, sim[name].kwargs) == (
            sim_stack[name].module,
            sim_stack[name].kwargs,
        )
        assert _remappings(_SIM, name) == _remappings(m20_nav_stack().solidify(), name)


def test_the_sim_plans_only_steps_its_policy_climbs() -> None:
    assert SIM_M20.steppable == STEP_LIMIT
    assert replace(SIM_M20, steppable=M20.steppable) == M20


def test_the_world_feeds_the_streams_the_vendor_lio_and_the_urdf_manager_read() -> None:
    assert _remappings(_SIM, "simm20world") == {
        ("simm20world", "lidar"): "slam_body_points",
        ("simm20world", "odometry"): "slam_odom",
        ("simm20world", "joint_state"): "coordinator_joint_state",
    }


@pytest.mark.parametrize("blueprint", [_REAL, _SIM], ids=["real", "sim"])
def test_teleop_cancel_reaches_the_planner(blueprint: Blueprint) -> None:
    def goal_topic(module: type[ModuleBase]) -> tuple[str | type[ModuleBase] | type[Spec], type]:
        (atom,) = (a for a in blueprint.active_blueprints if a.module is module)
        (stream,) = (s for s in atom.streams if s.name == "goal")
        return blueprint.remapping_map.get((atom.name, "goal"), "goal"), stream.type

    assert goal_topic(MLSPlannerNative) == goal_topic(MovementManager)


def test_nothing_on_the_real_robot_takes_velocity() -> None:
    """Real mode has no motor controller: nothing reads what the navigation sends on cmd_vel."""
    readers = [
        atom.module
        for atom in _REAL.active_blueprints
        for stream in atom.streams
        if stream.type is Twist
        and stream.direction == "in"
        and _REAL.remapping_map.get((atom.name, stream.name), stream.name) == "cmd_vel"
    ]
    assert readers == []


def test_one_movement_manager() -> None:
    assert [a.module for a in _SIM.active_blueprints].count(MovementManager) == 1


def test_viewer_gets_the_navigation_config() -> None:
    (bridge,) = (a for a in _SIM.active_blueprints if a.module is RerunBridgeModule)
    assert m20_nav_overrides().keys() <= bridge.kwargs["visual_override"].keys()
    assert m20_nav_static().keys() <= bridge.kwargs["static"].keys()


@pytest.mark.parametrize("blueprint", [_SIM, _SIM_BENCH], ids=["sim", "sim-bench"])
def test_one_worker_per_module_with_gossip_off(blueprint: Blueprint) -> None:
    overrides = blueprint.global_config_overrides
    assert overrides["n_workers"] == len(blueprint.active_blueprints)
    assert overrides["zenoh_gossip"] is False


def test_real_is_the_default_mode() -> None:
    assert [a.name for a in m20.solidify().active_blueprints] == [
        a.name for a in _REAL.active_blueprints
    ]


def test_sim_bench_is_sim_with_an_episode_driver() -> None:
    assert _atoms(_SIM_BENCH).keys() == _atoms(_SIM).keys() | {"episodedriver"}
